#include "locomotion_system.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "state_controller.h"
#include "walk_controller.h"
#include "workspace_analyzer.h"
#include <algorithm>
#include <cmath>
#include <vector>
#ifndef ARDUINO
#include <chrono>
#endif

namespace {
double clampAndStep(double input, double min_value, double max_value, double step) {
    double clamped = math_utils::clamp(input, min_value, max_value);
    if (step <= 0.0) {
        return clamped;
    }
    double stepped = std::round((clamped - min_value) / step) * step + min_value;
    return math_utils::clamp(stepped, min_value, max_value);
}
} // namespace

/** Constructor. */
LocomotionSystem::LocomotionSystem(const Parameters &params)
    : params(params), imu_interface(nullptr), fsr_interface(nullptr), servo_interface(nullptr),
      body_position(0.0f, 0.0f, params.standing_height), body_orientation(0.0f, 0.0f, 0.0f),
      model(params),
      legs{Leg(0, model), Leg(1, model), Leg(2, model), Leg(3, model), Leg(4, model), Leg(5, model)},
      system_enabled(false), velocity_controller(nullptr), last_error(NO_ERROR),
      body_pose_ctrl(nullptr), walk_ctrl(nullptr), admittance_ctrl(nullptr),
      system_state(SYSTEM_UNKNOWN), startup_in_progress(false), shutdown_in_progress(false) {

    for (int i = 0; i < NUM_LEGS; ++i) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            last_joint_command_deg_[i][j] = 0.0;
            last_joint_command_valid_[i][j] = false;
            desired_joint_command_state_[i][j] = DesiredJointCommandState();
        }
    }

    /** Initialize last logged phases for FSR debug debouncing (testing only). */
#ifdef TESTING_ENABLED
    for (int i = 0; i < NUM_LEGS; ++i) {
        /** Arbitrary default. */
        last_logged_leg_phase_[i] = SWING_PHASE;
    }
    last_logged_initialized_ = true;
#endif
}

/** Destructor. */
LocomotionSystem::~LocomotionSystem() {
    system_enabled = false;
    /** Destroy state controller before its dependencies. */
    state_controller_.reset();
    delete body_pose_ctrl;
    delete walk_ctrl;
    delete admittance_ctrl;
    delete velocity_controller;
}

/** Attempt to read initial joint positions from all servos (OpenSHC ACQUISTION_TIME equivalent). */
bool LocomotionSystem::attemptJointAcquisition() {
    /** Number of polling cycles based on acquisition timeout and control frequency. */
    const int max_cycles = static_cast<int>(ACQUISITION_TIMEOUT_S / params.time_delta);

    for (int cycle = 0; cycle < max_cycles; cycle++) {
        bool all_valid = true;
        for (int leg = 0; leg < NUM_LEGS && all_valid; leg++) {
            for (int joint = 0; joint < DOF_PER_LEG && all_valid; joint++) {
                double angle = servo_interface->getJointAngle(leg, joint);
                if (!std::isfinite(angle)) {
                    all_valid = false;
                }
            }
        }

        if (all_valid) {
            return true;
        }

        /** Delay one control cycle before retrying. */
        /** On Arduino use delay(); elsewhere use chrono. */
#ifdef ARDUINO
        delay(static_cast<unsigned long>(params.time_delta * 1000.0));
#else
        {
            auto dur = std::chrono::duration<double>(params.time_delta);
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < dur) {
                /** Busy-wait (short cycle). */
            }
        }
#endif
    }

    /** Timed out; will use default joint positions. */
    return false;
}

/** System initialization. */
bool LocomotionSystem::initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo, const BodyPoseConfiguration &pose_config) {
    if (!imu || !fsr || !servo) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    imu_interface = imu;
    fsr_interface = fsr;
    servo_interface = servo;

    /** Initialize interfaces. */
    if (!imu_interface->initialize()) {
        last_error = IMU_ERROR;
        return false;
    }

    if (!fsr_interface->initialize()) {
        last_error = FSR_ERROR;
        return false;
    }

    if (!servo_interface->initialize()) {
        last_error = SERVO_ERROR;
        return false;
    }

    /**
     * @brief Attempt to acquire initial joint positions from servos (OpenSHC ACQUISTION_TIME equivalent).
     *
     * On failure, continue with default positions (mirrors OpenSHC's fallback behavior).
     */
    joint_positions_initialised_ = attemptJointAcquisition();

    /** Initialize controllers with proper architecture. */
    body_pose_ctrl = new BodyPoseController(model, pose_config);
    walk_ctrl = new WalkController(model, legs, pose_config);
    admittance_ctrl = new AdmittanceController(params);
    velocity_controller = new CartesianVelocityController(model);

    walk_ctrl->setBodyPoseController(body_pose_ctrl);

    /** Enable IMU pose compensation by default if configured. */
    body_pose_ctrl->setIMUPoseEnabled(params.body_comp.enable);

    /** Initialize LegPosers in BodyPoseController. */
    body_pose_ctrl->initializeLegPosers(legs);

    /** Validate parameters. */
    if (!validateParameters()) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    /** Initialize legs with default stance position. */
    /** This happens after DH parameters are initialized in the constructor. */
    Pose default_stance(Point3D(0, 0, -params.standing_height), Eigen::Vector3d(0, 0, 0));
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].initialize(default_stance);
    }

    system_enabled = true;

    /** Create and initialize the state controller (always present; no fallback mode).
     *  This mirrors OpenSHC where StateController is always the orchestrator. */
    state_controller_ = std::make_unique<StateController>(*this);
    if (!state_controller_->initialize(pose_config)) {
        last_error = STATE_ERROR;
        return false;
    }

    return true;
}

/** System status check. */
bool LocomotionSystem::isSystemEnabled() const {
    return system_enabled;
}

RobotState LocomotionSystem::getRobotState() const {
    if (!state_controller_) {
        return ROBOT_UNKNOWN;
    }
    return state_controller_->getRobotState();
}

Pose LocomotionSystem::getCurrentBodyPose() const {
    if (body_pose_ctrl) {
        return body_pose_ctrl->getCurrentBodyPose();
    }
    return Pose::Identity();
}

bool LocomotionSystem::legsBearingLoad() const {
    if (body_pose_ctrl) {
        return body_pose_ctrl->legsBearingLoad(legs);
    }
    return false;
}

bool LocomotionSystem::updateDefaultConfiguration() {
    /** OpenSHC parity: capture achieved leg joint configuration as new defaults. */
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].updateDefaultConfiguration();
    }

    /** Keep WalkController default tip references aligned with current achieved tips. */
    if (walk_ctrl) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            auto leg_stepper = walk_ctrl->getLegStepper(i);
            if (leg_stepper) {
                Pose current_tip_state = leg_stepper->getCurrentTipPoseState();
                leg_stepper->setDefaultTipPose(current_tip_state);
            }
        }

        /**
         * Regenerate walkspace after default update so WorkspaceAnalyzer receives
         * updated default/identity tip references via WalkController::generateWalkspace().
         */
        walk_ctrl->generateWalkspace();
    }

    return true;
}

/** System calibration. */
bool LocomotionSystem::calibrateSystem() {
    /** Calibrate IMU. */
    if (!imu_interface->calibrate()) {
        last_error = IMU_ERROR;
        return false;
    }

    /** Calibrate FSRs. */
    for (int i = 0; i < NUM_LEGS; i++) {
        if (!fsr_interface->calibrateFSR(i)) {
            last_error = FSR_ERROR;
            return false;
        }
    }

    /** Set initial pose. */
    setStandingPose();

    return true;
}

/** Inverse kinematics using an optimized geometric method. */
JointAngles LocomotionSystem::calculateInverseKinematics(int leg,
                                                         const Point3D &p_target) {
    /** Use current joint angles as starting point for IK. */
    JointAngles current_angles = legs[leg].getJointAngles();
    return model.inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, p_target);
}

bool LocomotionSystem::isTargetReachable(int leg_index, const Point3D &target) {
    /** OpenSHC parity: reachability inferred via makeReachable() geometric projection. */
    Point3D reachable = model.getWorkspaceAnalyzer().makeReachable(leg_index, target);
    return math_utils::distance(reachable, target) <= IK_TOLERANCE;
}

Point3D LocomotionSystem::constrainToWorkspace(int leg_index, const Point3D &target) {
    /** Use geometric workspace constraint (OpenSHC-style makeReachable path). */
    return model.getWorkspaceAnalyzer().makeReachable(leg_index, target);
}

double LocomotionSystem::getJointLimitProximity(int leg_index, const JointAngles &angles) {
    /** OpenSHC-style joint limit proximity calculation (radian limits). */
    double min_proximity = 1.0f;

    /** Check each joint proximity to limits (radians). */
    double joints[3] = {angles.coxa, angles.femur, angles.tibia};
    double limits[3][2] = {
        {model.getCoxaAngleLimitRad(0), model.getCoxaAngleLimitRad(1)},
        {model.getFemurAngleLimitRad(0), model.getFemurAngleLimitRad(1)},
        {model.getTibiaAngleLimitRad(0), model.getTibiaAngleLimitRad(1)}};

    for (int j = 0; j < 3; ++j) {
        double range = limits[j][1] - limits[j][0];
        if (range > 0.0) {
            double half_range = range * 0.5;
            double center = (limits[j][1] + limits[j][0]) * 0.5;
            double distance_from_center = std::abs(joints[j] - center);
            double proximity = math_utils::clamp<double>((half_range - distance_from_center) / half_range, 0.0, 1.0);
            min_proximity = std::min(min_proximity, proximity);
        }
    }

    return min_proximity;
}

/** Store angles both in RAM and servos. */
bool LocomotionSystem::setLegJointAngles(int leg, const JointAngles &q) {
    if (!servo_interface)
        return false;

    /** Verify that all servos in this leg are ready for movement. */
    for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
        if (servo_interface->hasBlockingStatusFlags(leg, joint)) {
            /** Servo is blocked; cannot move this leg. */
            last_error = SERVO_BLOCKED_ERROR;
            return false;
        }
    }

    /** Clamp angles to joint limits (radians, OpenSHC parity). */
    JointAngles clamped_angles = q;
    clamped_angles.coxa = math_utils::clamp(q.coxa, model.getCoxaAngleLimitRad(0), model.getCoxaAngleLimitRad(1));
    clamped_angles.femur = math_utils::clamp(q.femur, model.getFemurAngleLimitRad(0), model.getFemurAngleLimitRad(1));
    clamped_angles.tibia = math_utils::clamp(q.tibia, model.getTibiaAngleLimitRad(0), model.getTibiaAngleLimitRad(1));

    /** Use velocity controller to get appropriate servo speeds. */
    double coxa_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 0) : params.default_servo_speed;
    double femur_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 1) : params.default_servo_speed;
    double tibia_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 2) : params.default_servo_speed;

    syncDesiredJointStateFromInternalCommand(leg, clamped_angles, JointAngles(coxa_speed, femur_speed, tibia_speed));

    /** Apply sign inversion/preservation per servo using params.angle_sign_*. */
    double servo_coxa = math_utils::radiansToDegrees(clamped_angles.coxa * params.angle_sign_coxa);
    double servo_femur = math_utils::radiansToDegrees(clamped_angles.femur * params.angle_sign_femur);
    double servo_tibia = math_utils::radiansToDegrees(clamped_angles.tibia * params.angle_sign_tibia);

    servo_coxa = applyJointOutputCalibration(leg, 0, servo_coxa);
    servo_femur = applyJointOutputCalibration(leg, 1, servo_femur);
    servo_tibia = applyJointOutputCalibration(leg, 2, servo_tibia);

    /** Enable coxa movement based for test mode. */
    /** This allows us to gate coxa servo output during test. */
    /** If coxa movement is disabled, freeze coxa at 0 degrees angle. */
    if (coxa_movement_enabled_) {
        /** Normal behavior: command coxa as computed. */
        servo_coxa = limitJointAngularSpeedCommand(leg, 0, servo_coxa);
        servo_interface->setJointAngleAndSpeed(leg, 0, servo_coxa, coxa_speed);
    } else {
        /** Test mode: freeze coxa at 0 degrees angle. */
        last_joint_command_deg_[leg][0] = 0.0;
        last_joint_command_valid_[leg][0] = true;
        servo_interface->setJointAngleAndSpeed(leg, 0, 0, coxa_speed);
    }

    /** Apply the computed angles to the servos. */
    servo_femur = limitJointAngularSpeedCommand(leg, 1, servo_femur);
    servo_tibia = limitJointAngularSpeedCommand(leg, 2, servo_tibia);
    servo_interface->setJointAngleAndSpeed(leg, 1, servo_femur, femur_speed);
    servo_interface->setJointAngleAndSpeed(leg, 2, servo_tibia, tibia_speed);
    return true;
}

/** Gait planner. */
bool LocomotionSystem::setGaitConfiguration(const GaitConfiguration &gait_config) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    bool result = walk_ctrl->setGait(gait_config);

    if (result) {
        walk_ctrl->generateWalkspace();
    }

    /** Update BodyPoseController with current gait type for startup sequence selection. */
    if (result && body_pose_ctrl) {
        body_pose_ctrl->setCurrentGaitType(gait_config.gait_type);
        body_pose_ctrl->setGaitPhaseParams(gait_config.phase_config.stance_phase,
                                           gait_config.phase_config.swing_phase,
                                           gait_config.phase_config.phase_offset);
        body_pose_ctrl->refreshAutoPoseParameters();
    }

    return result;
}

/** Gait sequence planning - use WalkController with LegStepper (OpenSHC pattern). */
bool LocomotionSystem::planGaitSequence(double velocity_x, double velocity_y, double angular_velocity) {
    /** Apply body velocity scaler (OpenSHC: body_velocity_scaler applied in bodyVelocityCallback). */
    const double scaler = params.body_velocity_scaler;
    commanded_linear_velocity_x_ = velocity_x * scaler;
    commanded_linear_velocity_y_ = velocity_y * scaler;
    commanded_angular_velocity_ = angular_velocity * scaler;
    /** Defer stride recomputation to per-cycle update() after ramp/limit applied. */
    return true;
}

bool LocomotionSystem::canApplyStepFrequency(double step_frequency) const {
    if (!walk_ctrl) {
        return false;
    }

    GaitConfiguration proposed = walk_ctrl->getCurrentGaitConfig();
    proposed.step_frequency = step_frequency;

    std::map<int, double> new_linear_limits;
    std::map<int, double> new_angular_limits;
    std::map<int, double> unused_linear_accel;
    std::map<int, double> unused_angular_accel;
    walk_ctrl->computeLimitsForConfig(proposed,
                                      new_linear_limits,
                                      new_angular_limits,
                                      unused_linear_accel,
                                      unused_angular_accel);

    if (new_linear_limits.empty() || new_angular_limits.empty()) {
        return false;
    }

    Eigen::Vector2d current_linear(walk_ctrl->getDesiredLinearVelocity().x,
                                   walk_ctrl->getDesiredLinearVelocity().y);
    double current_angular = walk_ctrl->getDesiredAngularVelocity();

    double max_linear = walk_ctrl->getLimit(current_linear, current_angular, new_linear_limits);
    double max_angular = walk_ctrl->getLimit(current_linear, current_angular, new_angular_limits);

    return (current_linear.norm() <= max_linear && std::abs(current_angular) <= std::abs(max_angular));
}

bool LocomotionSystem::applyStepFrequency(double step_frequency) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    GaitConfiguration updated = walk_ctrl->getCurrentGaitConfig();
    updated.step_frequency = step_frequency;
    params.step_frequency = step_frequency;
    model.setStepFrequency(step_frequency);
    return setGaitConfiguration(updated);
}

bool LocomotionSystem::tryApplyPendingStepFrequency() {
    if (!step_frequency_adjust_pending_) {
        return true;
    }
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }
    if (!canApplyStepFrequency(pending_step_frequency_)) {
        return true;
    }
    if (!applyStepFrequency(pending_step_frequency_)) {
        return false;
    }
    step_frequency_adjust_pending_ = false;
    pending_step_frequency_ = 0.0;
    return true;
}

bool LocomotionSystem::setStepFrequency(double value) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    double new_value = clampAndStep(value, 0.001, 2.0, 0.1);
    if (canApplyStepFrequency(new_value)) {
        step_frequency_adjust_pending_ = false;
        pending_step_frequency_ = 0.0;
        return applyStepFrequency(new_value);
    }

    step_frequency_adjust_pending_ = true;
    pending_step_frequency_ = new_value;
    return true;
}

bool LocomotionSystem::setSwingHeight(double value) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    GaitConfiguration updated = walk_ctrl->getCurrentGaitConfig();
    updated.swing_height = clampAndStep(value, 10.0, 80.0, 5.0);
    return setGaitConfiguration(updated);
}

bool LocomotionSystem::setStanceSpanModifier(double value) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    GaitConfiguration updated = walk_ctrl->getCurrentGaitConfig();
    updated.stance_span_modifier = clampAndStep(value, -1.0, 1.0, 0.1);
    return setGaitConfiguration(updated);
}

bool LocomotionSystem::setSwingWidth(double value) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    GaitConfiguration updated = walk_ctrl->getCurrentGaitConfig();
    updated.swing_width = clampAndStep(value, 0.0, 200.0, 5.0);
    return setGaitConfiguration(updated);
}

bool LocomotionSystem::setStepDepth(double value) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    GaitConfiguration updated = walk_ctrl->getCurrentGaitConfig();
    updated.step_depth = clampAndStep(value, -100.0, 100.0, 5.0);
    return setGaitConfiguration(updated);
}

bool LocomotionSystem::setVirtualMass(double value) {
    params.admittance.virtual_mass = clampAndStep(value, 0.01, 10.0, 0.01);
    return true;
}

bool LocomotionSystem::setVirtualStiffness(double value) {
    params.admittance.virtual_stiffness = clampAndStep(value, 0.0, 100.0, 1.0);
    return true;
}

bool LocomotionSystem::setVirtualDamping(double value) {
    params.admittance.virtual_damping_ratio = clampAndStep(value, 0.0, 50.0, 0.5);
    return true;
}

bool LocomotionSystem::setForceGain(double value) {
    params.admittance.force_gain = clampAndStep(value, 0.0, 10.0, 0.1);
    return true;
}

/** Forward locomotion control. */
bool LocomotionSystem::walkForward(double velocity) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    setDesiredVelocity(velocity, 0.0, 0.0);
    return true;
}

/** Backward locomotion control. */
bool LocomotionSystem::walkBackward(double velocity) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    setDesiredVelocity(-velocity, 0.0, 0.0);
    return true;
}

/** In-place turning control. */
bool LocomotionSystem::turnInPlace(double angular_velocity) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    setDesiredVelocity(0.0, 0.0, angular_velocity);
    return true;
}

/** Sideways locomotion control. */
bool LocomotionSystem::walkSideways(double velocity, bool right_direction) {
    /** Framework convention: +Y = left, -Y = right. */
    double lateral_velocity = right_direction ? -velocity : velocity;
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    setDesiredVelocity(0.0, lateral_velocity, 0.0);
    return true;
}

bool LocomotionSystem::executeStartupSequence() {
    if (!body_pose_ctrl || !walk_ctrl) {
        last_error = STATE_ERROR;
        return false;
    }

    /** Execute the body pose controller startup sequence (OpenSHC executeSequence). */
    int progress = body_pose_ctrl->executeSequence(START_UP, legs);
    last_startup_progress_ = (progress < 0 ? 0 : progress);
    bool startup_complete = (progress == PROGRESS_COMPLETE);
    startup_in_progress = !startup_complete;

    if (startup_complete) {
        /** Initialize walk controller for RUNNING state (OpenSHC pattern). */
        walk_ctrl->init(body_position, body_orientation);

        /** OpenSHC parity: preserve achieved configuration as new default. */
        if (!updateDefaultConfiguration()) {
            last_error = KINEMATICS_ERROR;
            return false;
        }

        /** OpenSHC pattern: do not apply velocities during startup. */
        /** Velocities will be applied in update() after startup completes. */
        /** This prevents the velocity_y loss bug that occurred here. */

        /** Initialize leg phases based on gait pattern (without velocity application). */
        for (int i = 0; i < NUM_LEGS; i++) {
            auto leg_stepper = walk_ctrl->getLegStepper(i);
            if (leg_stepper) {
                auto step_state = leg_stepper->getStepState();
                if (step_state == STEP_STANCE || step_state == STEP_FORCE_STANCE) {
                    legs[i].setStepPhase(STANCE_PHASE);
                } else {
                    legs[i].setStepPhase(SWING_PHASE);
                }
            }
        }

        system_state = OPERATIONAL;

        /** Clear flag here instead of requiring external code to modify private member. */
        startup_in_progress = false;
    }

    return startup_complete;
}

bool LocomotionSystem::activateRunningState() {
    if (!walk_ctrl || !body_pose_ctrl) {
        last_error = STATE_ERROR;
        return false;
    }

    /** Initialize walk controller for RUNNING state (same as executeStartupSequence completion). */
    walk_ctrl->init(body_position, body_orientation);

    /** OpenSHC direct mode parity: preserve achieved configuration as new default. */
    if (!updateDefaultConfiguration()) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    /** Initialize leg phases based on gait pattern. */
    for (int i = 0; i < NUM_LEGS; i++) {
        auto leg_stepper = walk_ctrl->getLegStepper(i);
        if (leg_stepper) {
            auto step_state = leg_stepper->getStepState();
            if (step_state == STEP_STANCE || step_state == STEP_FORCE_STANCE) {
                legs[i].setStepPhase(STANCE_PHASE);
            } else {
                legs[i].setStepPhase(SWING_PHASE);
            }
        }
    }

    system_state = OPERATIONAL;
    startup_in_progress = false;
    return true;
}

bool LocomotionSystem::executeShutdownSequence() {

    if (!body_pose_ctrl || !walk_ctrl) {
        last_error = STATE_ERROR;
        return false;
    }

    shutdown_in_progress = true;

    /** Update walk controller one final time with zero velocity. */
    walk_ctrl->updateWalk(Point3D(0.0, 0.0, 0.0), 0.0,
                          Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));

    /** OpenSHC: force all legs to STANCE according to shutdown protocol. */
    for (int i = 0; i < NUM_LEGS; i++) {
        auto leg_stepper = walk_ctrl->getLegStepper(i);
        if (leg_stepper) {
            leg_stepper->setStepState(STEP_FORCE_STOP);
            /** Force to stance position. */
            leg_stepper->setPhase(0.0);

            /** Immediately update tip position to identity (stance) position. */
            leg_stepper->updateTipPositionIterative(0, params.time_delta, false, false);

            /** Get the forced stance position and apply to leg. */
            legs[i].setStepPhase(STANCE_PHASE);
            Point3D stance_position = leg_stepper->getCurrentTipPose();
            legs[i].setCurrentTipPositionGlobal(stance_position);

            /** Apply IK and servo commands immediately. */
            if (legs[i].applyAdvancedIK(stance_position)) {
                JointAngles target_angles = legs[i].getJointAngles();
                setLegJointAngles(i, target_angles);
            }
        }
    }

    /** Execute the body pose controller shutdown sequence (OpenSHC executeSequence). */
    int sd_progress = body_pose_ctrl->executeSequence(SHUT_DOWN, legs);
    bool shutdown_complete = (sd_progress == PROGRESS_COMPLETE);

    if (shutdown_complete) {
        system_state = OPERATIONAL;

        /** Clear internal flag upon completion to simplify caller logic. */
        shutdown_in_progress = false;

        /** Reset body pose controller sequence states so next startup re-learns transition. */
        if (body_pose_ctrl) {
            body_pose_ctrl->resetSequenceStates();
        }
    }

    return shutdown_complete;
}

/** Body pose control. */
/** Set standing pose using BodyPoseController with LegPoser. */
bool LocomotionSystem::setStandingPose() {
    /** Set standing pose: apply configured joint angles to each leg (moved from BPC). */
    if (!body_pose_ctrl->getLegPoser(0)) {
        body_pose_ctrl->initializeLegPosers(legs);
    }
    for (int i = 0; i < NUM_LEGS; ++i) {
        const auto &standing_joints = body_pose_ctrl->getBodyPoseConfig().standing_pose_joints[i];
        JointAngles angles;
        angles.coxa = standing_joints.coxa;
        angles.femur = standing_joints.femur;
        angles.tibia = standing_joints.tibia;
        legs[i].setJointAngles(angles);
        Point3D pos = model.forwardKinematicsGlobalCoordinates(i, angles);
        legs[i].setCurrentTipPositionGlobal(pos);
    }
    bool success = true;

    if (success) {
        /** Ensure all legs are in STANCE_PHASE for standing pose. */
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setStepPhase(STANCE_PHASE);
        }

        /** Apply the calculated joint angles to servos. */
        for (int i = 0; i < NUM_LEGS; i++) {
            JointAngles angles = legs[i].getJointAngles();
            if (!setLegJointAngles(i, angles)) {
                last_error = KINEMATICS_ERROR;
                return false;
            }
        }

        /** Update body position from average leg tip height. */
        double total_z = 0.0;
        for (int i = 0; i < NUM_LEGS; i++) {
            total_z += legs[i].getCurrentTipPositionGlobal().z;
        }
        body_position = Eigen::Vector3d(0.0, 0.0, total_z / NUM_LEGS);

        /** Set system state to READY (OpenSHC equivalent). */
        system_state = OPERATIONAL;
        startup_in_progress = false;
        shutdown_in_progress = false;

        /** Notify state controller that robot is in READY state so that
         *  ROBOT_UNKNOWN detection resolves correctly without heuristics. */
        if (state_controller_) {
            state_controller_->notifyRobotReady();
        }

        return true;
    } else {
        last_error = KINEMATICS_ERROR;
        return false;
    }
}

bool LocomotionSystem::setRobotJointAngles(const JointAngles target_angles[NUM_LEGS]) {
    if (!servo_interface) {
        last_error = SERVO_ERROR;
        return false;
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        const JointAngles &angles = target_angles[i];
        if (!setLegJointAngles(i, angles)) {
            last_error = KINEMATICS_ERROR;
            return false;
        }

        legs[i].setJointAngles(angles);
        Point3D tip_position = model.forwardKinematicsGlobalCoordinates(i, angles);
        legs[i].setCurrentTipPositionGlobal(tip_position);
        legs[i].setDesiredTipPosition(tip_position);
        legs[i].setStepPhase(STANCE_PHASE);
    }

    if (body_pose_ctrl) {
        double total_z = 0.0;
        for (int i = 0; i < NUM_LEGS; i++) {
            total_z += legs[i].getCurrentTipPositionGlobal().z;
        }
        body_position = Eigen::Vector3d(0.0, 0.0, total_z / NUM_LEGS);
    }

    return true;
}

bool LocomotionSystem::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    /** Orientation expected in radians (roll, pitch, yaw). */

    /** Apply body pose via IK to all legs (moved from BPC). */
    Eigen::Quaterniond body_rotation = math_utils::eulerAnglesToQuaterniond(orientation);
    Pose body_pose(Point3D(position.x(), position.y(), position.z()), body_rotation);

    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D default_tip = legs[i].getCurrentTipPositionGlobal();
        Point3D posed_tip = body_pose.inverseTransformVector(default_tip);
        if (!legs[i].applyIK(posed_tip)) {
            last_error = KINEMATICS_ERROR;
            return false;
        }
    }

    /** Apply joint angles to servos. */
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles angles = legs[i].getJointAngles();
        if (!setLegJointAngles(i, angles)) {
            last_error = KINEMATICS_ERROR;
            return false;
        }
    }

    body_position = position;
    body_orientation = orientation;
    return true;
}

bool LocomotionSystem::setManualBodyPoseInput(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!body_pose_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    body_pose_ctrl->setManualPoseInput(Eigen::Vector3d(position.x(), position.y(), position.z()),
                                       Eigen::Vector3d(orientation.x(), orientation.y(), orientation.z()));
    body_pose_ctrl->setManualPoseEnabled(true);
    return true;
}

bool LocomotionSystem::applyManualLegInputs(int primary_leg_index,
                                            const Eigen::Vector3d &primary_tip_velocity,
                                            int secondary_leg_index,
                                            const Eigen::Vector3d &secondary_tip_velocity,
                                            bool primary_pose_valid,
                                            const Point3D &primary_tip_pose,
                                            bool secondary_pose_valid,
                                            const Point3D &secondary_tip_pose) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    // Cache inputs so manual update is applied after walk update in the same pipeline tick
    // (OpenSHC parity: updateWalk() then updateManual()).
    pending_primary_leg_index_ = primary_leg_index;
    pending_secondary_leg_index_ = secondary_leg_index;
    pending_primary_tip_velocity_ = primary_tip_velocity;
    pending_secondary_tip_velocity_ = secondary_tip_velocity;
    pending_primary_pose_valid_ = primary_pose_valid;
    pending_secondary_pose_valid_ = secondary_pose_valid;
    pending_primary_tip_pose_ = primary_tip_pose;
    pending_secondary_tip_pose_ = secondary_tip_pose;

    return true;
}

/**
 * @brief Main system update — delegates to StateController.
 *
 * StateController is the orchestrator (OpenSHC-equivalent role without ROS).
 * It manages state transitions, velocity/pose control, and calls back into
 * LocomotionSystem::runControlPipelineStep() through the context interface
 * for sensor reading, walk update, IK and servo output.
 */
bool LocomotionSystem::update() {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    state_controller_->update(params.time_delta);
    return true;
}

bool LocomotionSystem::runControlPipelineStep() {

    const bool robot_running = state_controller_ &&
                               state_controller_->getRobotState() == RobotState::ROBOT_RUNNING;

    /** Update sensors in parallel for optimal performance. */
    if (!updateSensorsParallel()) {
        last_error = SENSOR_ERROR;
        return false;
    }

    updateLegJointTelemetry();

    if (walk_ctrl) {
        walk_ctrl->updateTerrainAdaptation(fsr_interface, imu_interface);
    }

    if (body_pose_ctrl && imu_interface) {
        body_pose_ctrl->setIMUData(imu_interface->readIMU());
    }

    /** Update FSR contact history early (for swing adaptation inside LegStepper) without final phase assignment. */
    /** Phase (STANCE/SWING) will be finalized after walk controller updates trajectories. */
    if (params.use_fsr_contact) {
        /** Early call: updates in_contact_ and histories; phase may be overwritten later. */
        updateLegStates();
    }

    /**
     * @brief Body pose composition (OpenSHC 1:1 pipeline position).
     *
     * In OpenSHC loop(), updateCurrentPose() runs BEFORE admittance for any
     * robot_state != UNKNOWN. Placing it here ensures the composed body pose
     * is available when the admittance ODE integrates tip forces.
     */
    if (body_pose_ctrl && state_controller_ &&
        state_controller_->getRobotState() != RobotState::ROBOT_UNKNOWN) {
        int robot_state = static_cast<int>(state_controller_->getRobotState());
        body_pose_ctrl->updateCurrentPose(robot_state, legs);
    }

    /**
     * @brief Admittance control (OpenSHC 1:1 pipeline position).
     *
     * In OpenSHC this runs in StateController::loop() after updateCurrentPose
     * but before the state machine, as long as robot_state_ != UNKNOWN.
     */
    if (params.admittance.enable && admittance_ctrl) {
        /** Dynamic stiffness: only when actively walking and dynamic_stiffness enabled. */
        if (walk_ctrl && walk_ctrl->getWalkState() != WALK_STOPPED &&
            params.admittance.dynamic_stiffness) {
            admittance_ctrl->updateStiffness(legs, walk_ctrl);
        }
        /** Integrate admittance ODE for all legs. */
        admittance_ctrl->updateAdmittance(legs, fsr_interface);
    }

    /** Handle initial standing pose transition (non-blocking) using directStartup. */
    if (initial_standing_active_) {
        if (!stepInitialStandingPose()) {
            /** Error already set. */
            return false;
        }
    }

    if (!robot_running && step_frequency_adjust_pending_) {
        if (!applyStepFrequency(pending_step_frequency_)) {
            last_error = PARAMETER_ERROR;
            return false;
        }
        step_frequency_adjust_pending_ = false;
        pending_step_frequency_ = 0.0;
    }

    /** Only update leg trajectories if robot state is RUNNING. */
    if (walk_ctrl && robot_running) {
        /** Optional kinematic integration of body pose (test / simulation). */
        if (params.enable_body_translation) {
            /** Integrate translation (mm) and yaw (degrees) from commanded velocities. */
            /** mm/s * s. */
            body_position[0] += commanded_linear_velocity_x_ * params.time_delta;
            body_position[1] += commanded_linear_velocity_y_ * params.time_delta;
            /** deg/s * s. */
            body_orientation[2] += commanded_angular_velocity_ * params.time_delta;
            /** Wrap yaw to [-180,180]. */
            if (body_orientation[2] > 180.0)
                body_orientation[2] -= 360.0;
            else if (body_orientation[2] < -180.0)
                body_orientation[2] += 360.0;
        }

        /** Step 1: update walk controller (it performs its own limiting and ramping via VelocityLimits). */
        Point3D applied_linear_velocity(commanded_linear_velocity_x_, commanded_linear_velocity_y_, 0.0);
        walk_ctrl->updateWalk(applied_linear_velocity,
                              commanded_angular_velocity_,
                              body_position, body_orientation);

        if (!tryApplyPendingStepFrequency()) {
            last_error = PARAMETER_ERROR;
            return false;
        }

        /** Step 1b: apply manual-leg updates after walk (OpenSHC update order parity). */
        walk_ctrl->updateManual(pending_primary_leg_index_,
                                pending_primary_tip_velocity_,
                                pending_secondary_leg_index_,
                                pending_secondary_tip_velocity_);
        if (pending_primary_pose_valid_ || pending_secondary_pose_valid_) {
            walk_ctrl->updateManual(pending_primary_leg_index_,
                                    pending_primary_tip_pose_,
                                    pending_secondary_leg_index_,
                                    pending_secondary_tip_pose_);
        }

        /** Step 2: collect desired positions from Bezier trajectories (= OpenSHC::setDesiredTipPose). */
        for (int i = 0; i < NUM_LEGS; i++) {
            auto leg_stepper = walk_ctrl->getLegStepper(i);
            if (leg_stepper) {
                Point3D desired_tip_position = leg_stepper->getCurrentTipPose();
                legs[i].setDesiredTipPosition(desired_tip_position);
            }
        }

        /** Step 2a: update stance with composed body pose (OpenSHC PoseController::updateStance equivalent). */
        if (body_pose_ctrl) {
            body_pose_ctrl->updateStance(legs);
        }

        /** Step 2b: finalize leg phases (FSR or pure kinematic) after trajectories computed. */
        updateLegStates();

        /** Step 3: apply IK to all legs at once (= OpenSHC::Model::updateModel). */
        applyInverseKinematicsToAllLegs();

        /** Step 4: publish all joint angles to servos (= OpenSHC::publishDesiredJointState). */
        publishJointAnglesToServos();

#ifdef TESTING_ENABLED
        /** Record telemetry sample after applying servos and IK (final cycle state). */
        if (telemetry_enabled_) {
            telemetry_time_accumulator_ += params.time_delta;
            recordCoxaTelemetrySample();
        }
#endif
    } else {
        /** When robot is not RUNNING, keep all legs in stance phase. */
        /** This prevents update() calls from overriding the shutdown-forced STANCE states. */
        /** Do not read leg_stepper states; preserve the shutdown state. */
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setStepPhase(STANCE_PHASE);
        }
    }

    return true;
}

void LocomotionSystem::updateLegJointTelemetry() {
    if (!servo_interface) {
        return;
    }

    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles velocities;
        velocities.coxa = servo_interface->getJointVelocity(i, 0);
        velocities.femur = servo_interface->getJointVelocity(i, 1);
        velocities.tibia = servo_interface->getJointVelocity(i, 2);
        legs[i].setCurrentJointVelocity(velocities);

        JointAngles efforts;
        efforts.coxa = servo_interface->getJointEffort(i, 0);
        efforts.femur = servo_interface->getJointEffort(i, 1);
        efforts.tibia = servo_interface->getJointEffort(i, 2);
        legs[i].setCurrentJointEffort(efforts);
        legs[i].calculateTipForce();

        if (std::isfinite(efforts.coxa)) {
            desired_joint_command_state_[i][0].desired_effort = efforts.coxa;
        }
        if (std::isfinite(efforts.femur)) {
            desired_joint_command_state_[i][1].desired_effort = efforts.femur;
        }
        if (std::isfinite(efforts.tibia)) {
            desired_joint_command_state_[i][2].desired_effort = efforts.tibia;
        }
    }
}

/** Error handling. */
String LocomotionSystem::getErrorMessage(ErrorCode error) {
    switch (error) {
    case NO_ERROR:
        return "No errors";
    case IMU_ERROR:
        return "IMU error";
    case FSR_ERROR:
        return "FSR sensor error";
    case SERVO_ERROR:
        return "Servo error";
    case KINEMATICS_ERROR:
        return "Kinematics error";
    case STABILITY_ERROR:
        return "Stability error";
    case PARAMETER_ERROR:
        return "Parameter error";
    case SENSOR_ERROR:
        return "Sensor communication error";
    case SERVO_BLOCKED_ERROR:
        return "Servo blocked by status flags";
    default:
        return "Unknown error";
    }
}

bool LocomotionSystem::handleError(ErrorCode error) {
    last_error = error;

    switch (error) {
    case IMU_ERROR:
        /** Try to reinitialize IMU. */
        if (imu_interface) {
            return imu_interface->initialize();
        }
        break;

    case FSR_ERROR:
        /** Try to recalibrate FSRs. */
        if (fsr_interface) {
            for (int i = 0; i < NUM_LEGS; i++) {
                fsr_interface->calibrateFSR(i);
            }
            return true;
        }
        break;

    case SERVO_ERROR:
        /** Try to reinitialize servos. */
        if (servo_interface) {
            return servo_interface->initialize();
        }
        break;

    case STABILITY_ERROR:
        /** Adopt a more stable pose. */
        return setStandingPose();

    case KINEMATICS_ERROR:
        /** Return to a safe pose. */
        return setStandingPose();

    case SERVO_BLOCKED_ERROR:
        /** Cannot recover from blocked servos automatically; requires manual intervention. */
        /** The error provides diagnostic information about which servos are blocked. */
        return false;

    default:
        return false;
    }

    return false;
}

void LocomotionSystem::updateLegStates() {
    /** Two-stage logic. */
    /** 1) Update contact histories if FSR available. */
    /** 2) Decide final STANCE/SWING phase using filtered contact or kinematic StepState. */

    bool fsr_enabled = params.use_fsr_contact && fsr_interface;

    if (fsr_enabled) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            FSRData fsr = fsr_interface->readFSR(i);
            legs[i].updateFSRHistory(fsr.in_contact, fsr.pressure);
        }
    }

    for (int i = 0; i < NUM_LEGS; ++i) {
        auto leg_stepper = walk_ctrl ? walk_ctrl->getLegStepper(i) : nullptr;

        if (fsr_enabled) {
            bool filtered_contact = legs[i].getFilteredContactState(params.fsr_touchdown_threshold, params.fsr_liftoff_threshold);
            StepPhase current_state = legs[i].getStepPhase();

            /** Hysteresis transitions with optional debug logging. */
            if (current_state == SWING_PHASE && filtered_contact) {
                legs[i].setStepPhase(STANCE_PHASE);

#ifdef TESTING_ENABLED
                if (params.debug_fsr_transitions) {
#ifdef TESTING_ENABLED
                    if (!last_logged_initialized_ || last_logged_leg_phase_[i] != STANCE_PHASE) {
                        std::cout << "[FSR] leg=" << i << " SWING->STANCE filtered_contact=1 avg>=contact_th pressure="
                                  << legs[i].getContactForce() << std::endl;
                        last_logged_leg_phase_[i] = STANCE_PHASE;
                    }
#endif
                }
#endif

            } else if (current_state == STANCE_PHASE && !filtered_contact) {
                legs[i].setStepPhase(SWING_PHASE);

#ifdef TESTING_ENABLED
                if (params.debug_fsr_transitions) {
#ifdef TESTING_ENABLED
                    if (!last_logged_initialized_ || last_logged_leg_phase_[i] != SWING_PHASE) {
                        std::cout << "[FSR] leg=" << i << " STANCE->SWING filtered_contact=0 avg<release_th pressure="
                                  << legs[i].getContactForce() << std::endl;
                        last_logged_leg_phase_[i] = SWING_PHASE;
                    }
#endif
                }
#endif
            }

            /** Safety: revert if kinematic expectation indicates swing but pressure is low. */
            if (leg_stepper) {
                double phase_fraction = static_cast<double>(leg_stepper->getPhase()) /
                                        static_cast<double>(walk_ctrl->getStepCycle().period_);
                bool should_be_swing = legs[i].shouldBeInSwing(phase_fraction, walk_ctrl->getStanceDuration());
                if (should_be_swing && legs[i].getStepPhase() == STANCE_PHASE && legs[i].getContactForce() < params.fsr_min_pressure) {
                    legs[i].setStepPhase(SWING_PHASE);

#ifdef TESTING_ENABLED
                    if (params.debug_fsr_transitions) {
                        /** Revert implies we force SWING; guard against duplicate logs. */
                        if (!last_logged_initialized_ || last_logged_leg_phase_[i] != SWING_PHASE) {
                            std::cout << "[FSR] leg=" << i << " REVERT STANCE->SWING low_pressure=" << legs[i].getContactForce()
                                      << " < fsr_min_pressure phase_guard" << std::endl;
                            last_logged_leg_phase_[i] = SWING_PHASE;
                        }
                    }
#endif
                }
            }
        } else {
            /** No FSR: derive directly from LegStepper StepState (more direct than approximate phase math). */
            if (leg_stepper) {
                StepState ss = leg_stepper->getStepState();
                legs[i].setStepPhase((ss == STEP_SWING) ? SWING_PHASE : STANCE_PHASE);
            }
        }
    }
}

bool LocomotionSystem::checkJointLimits(int leg_index, const JointAngles &angles) {
    return model.checkJointLimits(leg_index, angles);
}

double LocomotionSystem::constrainAngle(double angle, double min_angle, double max_angle) {
    return model.constrainAngle(angle, min_angle, max_angle);
}

bool LocomotionSystem::validateParameters() {
    return model.validate();
}

bool LocomotionSystem::isValidJointAddress(int leg_index, int joint_index) const {
    return leg_index >= 0 && leg_index < NUM_LEGS && joint_index >= 0 && joint_index < DOF_PER_LEG;
}

/**
 * @brief Mirror internal command updates into OpenSHC-like desired/prev desired state.
 *
 * Called from internal command paths (single-joint and sync publish) so that
 * external software can inspect a coherent desired_* / prev_desired_* history.
 */
void LocomotionSystem::syncDesiredJointStateFromInternalCommand(int leg_index, const JointAngles &desired_positions,
                                                                const JointAngles &desired_velocities) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return;
    }

    const double positions[DOF_PER_LEG] = {desired_positions.coxa, desired_positions.femur, desired_positions.tibia};
    const double velocities[DOF_PER_LEG] = {desired_velocities.coxa, desired_velocities.femur, desired_velocities.tibia};

    for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
        DesiredJointCommandState &state = desired_joint_command_state_[leg_index][joint];
        state.prev_desired_position_rad = state.desired_position_rad;
        state.prev_desired_velocity_rad_s = state.desired_velocity_rad_s;
        state.prev_desired_effort = state.desired_effort;

        state.desired_position_rad = positions[joint];
        state.desired_velocity_rad_s = velocities[joint];
    }
}

/**
 * @brief Advance desired-state cycle markers (desired_* -> prev_desired_*).
 *
 * This method intentionally does not change current desired values; it only shifts
 * previous snapshots, matching OpenSHC-style cycle bookkeeping expectations.
 */
void LocomotionSystem::beginDesiredJointCommandCycle() {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
            DesiredJointCommandState &state = desired_joint_command_state_[leg][joint];
            state.prev_desired_position_rad = state.desired_position_rad;
            state.prev_desired_velocity_rad_s = state.desired_velocity_rad_s;
            state.prev_desired_effort = state.desired_effort;
        }
    }
}

/**
 * @brief Set desired command tuple for one physical joint.
 *
 * The prev_desired_* fields are not modified here; use beginDesiredJointCommandCycle()
 * to advance cycle history explicitly.
 */
bool LocomotionSystem::setDesiredJointCommandState(int leg_index, int joint_index,
                                                   double desired_position_rad,
                                                   double desired_velocity_rad_s,
                                                   double desired_effort) {
    if (!isValidJointAddress(leg_index, joint_index)) {
        return false;
    }

    DesiredJointCommandState &state = desired_joint_command_state_[leg_index][joint_index];
    state.desired_position_rad = desired_position_rad;
    state.desired_velocity_rad_s = desired_velocity_rad_s;
    state.desired_effort = desired_effort;
    return true;
}

/**
 * @brief Update only desired effort for one physical joint.
 *
 * Useful when an external estimator/controller computes effort independently from
 * position/velocity targets.
 */
bool LocomotionSystem::setDesiredJointEffort(int leg_index, int joint_index, double desired_effort) {
    if (!isValidJointAddress(leg_index, joint_index)) {
        return false;
    }
    desired_joint_command_state_[leg_index][joint_index].desired_effort = desired_effort;
    return true;
}

/**
 * @brief Retrieve desired/prev desired state snapshot for one joint.
 */
bool LocomotionSystem::getDesiredJointCommandState(int leg_index, int joint_index, DesiredJointCommandState &state) const {
    if (!isValidJointAddress(leg_index, joint_index)) {
        return false;
    }
    state = desired_joint_command_state_[leg_index][joint_index];
    return true;
}

double LocomotionSystem::applyJointOutputCalibration(int leg_index, int joint_index, double commanded_angle_deg) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
        return commanded_angle_deg;
    }
    return commanded_angle_deg + params.joint_angle_offset_deg[leg_index][joint_index];
}

double LocomotionSystem::limitJointAngularSpeedCommand(int leg_index, int joint_index, double target_angle_deg) {
    if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
        return target_angle_deg;
    }

    const double configured_max_speed = params.joint_max_angular_speed_deg_s[leg_index][joint_index];
    if (!std::isfinite(configured_max_speed) || configured_max_speed <= 0.0 || !std::isfinite(params.time_delta) || params.time_delta <= 0.0) {
        last_joint_command_deg_[leg_index][joint_index] = target_angle_deg;
        last_joint_command_valid_[leg_index][joint_index] = true;
        return target_angle_deg;
    }

    if (!last_joint_command_valid_[leg_index][joint_index] || !std::isfinite(last_joint_command_deg_[leg_index][joint_index])) {
        last_joint_command_deg_[leg_index][joint_index] = target_angle_deg;
        last_joint_command_valid_[leg_index][joint_index] = true;
        return target_angle_deg;
    }

    const double previous_command = last_joint_command_deg_[leg_index][joint_index];
    const double max_delta = configured_max_speed * params.time_delta;
    const double min_allowed = previous_command - max_delta;
    const double max_allowed = previous_command + max_delta;
    const double limited_command = math_utils::clamp(target_angle_deg, min_allowed, max_allowed);

    last_joint_command_deg_[leg_index][joint_index] = limited_command;
    last_joint_command_valid_[leg_index][joint_index] = true;
    return limited_command;
}

void LocomotionSystem::updateAdmittanceStiffness(int leg_index, double scale_reference) {
    if (admittance_ctrl && params.admittance.enable && params.admittance.dynamic_stiffness) {
        admittance_ctrl->updateStiffness(legs, leg_index, scale_reference);
    }
}

/** Parallel sensor update implementation. */
bool LocomotionSystem::updateSensorsParallel() {
    bool fsr_updated = false;
    bool imu_updated = false;

    /** Start parallel sensor updates. */
    /** FSR: AdvancedAnalog DMA for simultaneous ADC reading. */
    /** Only update if FSR usage is enabled in parameters to avoid blocking latency otherwise. */
    if (fsr_interface && params.use_fsr_contact) {
        fsr_updated = fsr_interface->update();
    }

    /** IMU: non-blocking sensor update (parallel with FSR). */
    if (imu_interface && imu_interface->isConnected()) {
        imu_updated = imu_interface->update();
    }

    /** Validate both sensors updated successfully. */
    if (fsr_interface && params.use_fsr_contact && !fsr_updated) {
        last_error = FSR_ERROR;
        return false;
    }

    if (imu_interface && imu_interface->isConnected() && !imu_updated) {
        last_error = IMU_ERROR;
        return false;
    }

    return true;
}

/** Cartesian velocity control methods. */
bool LocomotionSystem::setVelocityControlEnabled(bool enable) {
    if (velocity_controller) {
        velocity_controller->setVelocityControlEnabled(enable);
        return true;
    }
    return false;
}

bool LocomotionSystem::setVelocityScaling(const CartesianVelocityController::VelocityScaling &scaling) {
    if (velocity_controller) {
        velocity_controller->setVelocityScaling(scaling);
        return true;
    }
    return false;
}

bool LocomotionSystem::setGaitSpeedModifiers(const CartesianVelocityController::GaitSpeedModifiers &modifiers) {
    if (velocity_controller) {
        velocity_controller->setGaitSpeedModifiers(modifiers);
        return true;
    }
    return false;
}

double LocomotionSystem::getCurrentServoSpeed(int leg_index, int joint_index) const {
    if (velocity_controller) {
        return velocity_controller->getServoSpeed(leg_index, joint_index);
    }
    return params.default_servo_speed;
}

/** Start walking (delegates to setRobotState for RUNNING transition). */
bool LocomotionSystem::startWalking() {
    return setRobotState(RobotState::ROBOT_RUNNING);
}

/** Stop walking without shutdown; ensure all feet on ground. Mode controls stance behavior. */
bool LocomotionSystem::stopWalking(StopMode mode) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    /** Zero velocity lets WalkController transition STOPPING → STOPPED naturally. */
    setDesiredVelocity(0.0, 0.0, 0.0);
    if (mode == STOP_UNIFORM) {
        /** Also request RUNNING → READY transition for a full stop. */
        setRobotState(RobotState::ROBOT_READY);
    }
    return true;
}

/** OpenSHC-style IK batch processing. */

void LocomotionSystem::applyInverseKinematicsToAllLegs() {
    /** Following OpenSHC::Model::updateModel() pattern exactly. */
    /** Apply IK to all legs at once using applyAdvancedIK (equivalent to OpenSHC::solveIK). */

    for (int i = 0; i < NUM_LEGS; i++) {
        /** Get desired position from Bezier trajectory (pure, no IK applied yet). */
        Point3D desired_tip_position = legs[i].getDesiredTipPosition();

        /**
         * OpenSHC parity: if desired position is unset (zero),
         * fall back to current tip position (equivalent to
         * Leg::setDesiredTipPose(Pose::Undefined()) → poser->getCurrentTipPose()).
         */
        if (desired_tip_position.x == 0.0 && desired_tip_position.y == 0.0 &&
            desired_tip_position.z == 0.0) {
            desired_tip_position = legs[i].getCurrentTipPositionGlobal();
        }

        /**
         * Apply admittance delta (OpenSHC: done inside Leg::setDesiredTipPose).
         * Skip legs in MANUAL state (OpenSHC pattern).
         */
        if (params.admittance.enable && legs[i].getLegState() != LEG_MANUAL) {
            Eigen::Vector3d delta = legs[i].getAdmittanceDelta();
            desired_tip_position.x += delta[0];
            desired_tip_position.y += delta[1];
            desired_tip_position.z += delta[2];
        }

        /** Get current state for delta calculation. */
        JointAngles current_angles = legs[i].getJointAngles();
        Point3D current_tip_position = legs[i].getCurrentTipPositionGlobal();

        /** Apply applyAdvancedIK (= OpenSHC::solveIK with DLS + cost gradients). */
        /** Apply IK for leg index, current tip pose, desired tip pose, and current joint angles. */
        JointAngles new_angles = model.applyAdvancedIK(
            i,
            current_tip_position,
            desired_tip_position,
            current_angles,
            params.time_delta);

        /** Update joint angles in Leg object. */
        /** Keep FK-derived tip pose as runtime truth (OpenSHC parity). */
        legs[i].setJointAngles(new_angles);
    }
}

void LocomotionSystem::publishJointAnglesToServos() {

    /** Refresh a small slice of health state per tick (non-blocking cache) if supported. */
    if (servo_interface) {
        /** ~3 servos/tick => full scan ~6 ticks. */
        servo_interface->refreshHealthSlice(3);
    }

    /** Prefer a single synchronous write when supported by the driver to remove per-joint bus latency. */
    if (servo_interface) {
        double angles_deg[NUM_LEGS][DOF_PER_LEG];
        double speeds[NUM_LEGS][DOF_PER_LEG];
        JointAngles desired_positions[NUM_LEGS];

        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles a = legs[i].getJointAngles();
            desired_positions[i] = a;

            /** Convert to output degrees with sign. */
            /** Respect runtime coxa gating in the fast sync path as well. */
            if (coxa_movement_enabled_) {
                angles_deg[i][0] = math_utils::radiansToDegrees(a.coxa * params.angle_sign_coxa);
                angles_deg[i][0] = applyJointOutputCalibration(i, 0, angles_deg[i][0]);
                angles_deg[i][0] = limitJointAngularSpeedCommand(i, 0, angles_deg[i][0]);
            } else {
                /** Freeze coxa at 0 degrees when disabled (same behavior as per-leg path). */
                angles_deg[i][0] = 0.0;
                last_joint_command_deg_[i][0] = 0.0;
                last_joint_command_valid_[i][0] = true;
            }

            angles_deg[i][1] = math_utils::radiansToDegrees(a.femur * params.angle_sign_femur);
            angles_deg[i][2] = math_utils::radiansToDegrees(a.tibia * params.angle_sign_tibia);
            angles_deg[i][1] = applyJointOutputCalibration(i, 1, angles_deg[i][1]);
            angles_deg[i][2] = applyJointOutputCalibration(i, 2, angles_deg[i][2]);
            angles_deg[i][1] = limitJointAngularSpeedCommand(i, 1, angles_deg[i][1]);
            angles_deg[i][2] = limitJointAngularSpeedCommand(i, 2, angles_deg[i][2]);

            /** Per-joint speeds. */
            speeds[i][0] = velocity_controller ? velocity_controller->getServoSpeed(i, 0) : params.default_servo_speed;
            speeds[i][1] = velocity_controller ? velocity_controller->getServoSpeed(i, 1) : params.default_servo_speed;
            speeds[i][2] = velocity_controller ? velocity_controller->getServoSpeed(i, 2) : params.default_servo_speed;

            legs[i].setDesiredJointVelocity(JointAngles(speeds[i][0], speeds[i][1], speeds[i][2]));
        }

        if (servo_interface->syncSetAllJointAnglesAndSpeeds(angles_deg, speeds)) {
            for (int li = 0; li < NUM_LEGS; ++li) {
                syncDesiredJointStateFromInternalCommand(li, desired_positions[li],
                                                         JointAngles(speeds[li][0], speeds[li][1], speeds[li][2]));
            }
            /** Cache last servo commands (degrees) for telemetry. */
#ifdef TESTING_ENABLED
            for (int li = 0; li < NUM_LEGS; ++li) {
                for (int j = 0; j < DOF_PER_LEG; ++j) {
                    last_servo_cmd_deg_[li][j] = angles_deg[li][j];
                }
            }
#endif
            /** Done. */
            return;
        }
    }

    /** Fallback: per-leg commands. */
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles angles = legs[i].getJointAngles();

        double coxa_speed = velocity_controller ? velocity_controller->getServoSpeed(i, 0) : params.default_servo_speed;
        double femur_speed = velocity_controller ? velocity_controller->getServoSpeed(i, 1) : params.default_servo_speed;
        double tibia_speed = velocity_controller ? velocity_controller->getServoSpeed(i, 2) : params.default_servo_speed;
        legs[i].setDesiredJointVelocity(JointAngles(coxa_speed, femur_speed, tibia_speed));

        /** Send all angles for this leg to servos. */
        if (!setLegJointAngles(i, angles)) {
            /** Handle servo failure; log error but continue with other legs. */
            last_error = SERVO_ERROR;
            /** Continue processing other legs instead of failing completely. */
        }
        /** Cache per-leg command (degrees). */
#ifdef TESTING_ENABLED
        last_servo_cmd_deg_[i][0] = last_joint_command_valid_[i][0] ? last_joint_command_deg_[i][0]
                                                                    : math_utils::radiansToDegrees(angles.coxa * params.angle_sign_coxa);
        last_servo_cmd_deg_[i][1] = last_joint_command_valid_[i][1] ? last_joint_command_deg_[i][1]
                                                                    : math_utils::radiansToDegrees(angles.femur * params.angle_sign_femur);
        last_servo_cmd_deg_[i][2] = last_joint_command_valid_[i][2] ? last_joint_command_deg_[i][2]
                                                                    : math_utils::radiansToDegrees(angles.tibia * params.angle_sign_tibia);
#endif
    }
}

/** Initial standing pose establishment using directStartup (OpenSHC parity). */
bool LocomotionSystem::establishInitialStandingPose() {
    if (!servo_interface || !body_pose_ctrl) {
        last_error = SERVO_ERROR;
        return false;
    }
    if (initial_standing_active_) {
        /** Already in progress; just step. */
        return stepInitialStandingPose();
    }
    /** Refresh leg joint angles from actual servo feedback. */
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles current_angles;
        current_angles.coxa = servo_interface->getJointAngle(i, 0);
        current_angles.femur = servo_interface->getJointAngle(i, 1);
        current_angles.tibia = servo_interface->getJointAngle(i, 2);
        legs[i].setJointAngles(current_angles);
        legs[i].setCurrentTipPositionGlobal(model.forwardKinematicsGlobalCoordinates(i, current_angles));
    }
    /** Use directStartup for joint-space transition to standing pose. */
    if (!body_pose_ctrl->getLegPoser(0)) {
        body_pose_ctrl->initializeLegPosers(legs);
    }
    initial_standing_active_ = true;
    startup_in_progress = true;
    system_state = OPERATIONAL;
    return stepInitialStandingPose();
}

bool LocomotionSystem::stepInitialStandingPose() {
    if (!servo_interface || !body_pose_ctrl) {
        last_error = SERVO_ERROR;
        return false;
    }
    if (!initial_standing_active_) {
        return true;
    }

    /** Use directStartup to transition joints to standing configuration. */
    int progress = body_pose_ctrl->directStartup(legs);

    /** Apply resulting joint angles to servos. */
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles angles = legs[i].getJointAngles();
        setLegJointAngles(i, angles);
    }

    /** If finished, finalize state and compute body position. */
    if (progress == PROGRESS_COMPLETE) {
        initial_standing_active_ = false;
        double total_z = 0.0;
        for (int i = 0; i < NUM_LEGS; i++) {
            total_z += legs[i].getCurrentTipPositionGlobal().z;
        }
        body_position = Eigen::Vector3d(0.0, 0.0, total_z / NUM_LEGS);
        system_state = OPERATIONAL;
        startup_in_progress = false;
        shutdown_in_progress = false;
    }
    return true;
}

/* ===================================================================
 *  ROS-equivalent subscription accessors (input setters).
 *
 *  Each method mirrors one OpenSHC ROS subscriber callback, translating
 *  external commands into the internal control pipeline.
 * =================================================================== */

bool LocomotionSystem::setSystemState(SystemState state) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    return state_controller_->requestSystemState(state);
}

bool LocomotionSystem::setRobotState(RobotState state) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    return state_controller_->requestRobotState(state);
}

void LocomotionSystem::setDesiredVelocity(double linear_x, double linear_y, double angular_z) {
    /** Route through StateController so updateVelocityControl() → planGaitSequence()
     *  applies body_velocity_scaler and writes commanded_* in the pipeline. */
    if (state_controller_) {
        state_controller_->setDesiredVelocity(Eigen::Vector2d(linear_x, linear_y), angular_z);
    }
}

void LocomotionSystem::setDesiredPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        return;
    }
    state_controller_->setDesiredPose(position, orientation);
}

bool LocomotionSystem::setPosingMode(PosingMode mode) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    return state_controller_->setPosingMode(mode);
}

bool LocomotionSystem::setPoseResetMode(PoseResetMode mode) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    return state_controller_->setPoseResetMode(mode);
}

bool LocomotionSystem::selectGait(GaitType gait) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    return state_controller_->changeGait(gait);
}

bool LocomotionSystem::setPrimaryLegSelection(int leg_index) {
    if (leg_index < -1 || leg_index >= NUM_LEGS) {
        return false;
    }
    primary_leg_selection_ = leg_index;
    return true;
}

bool LocomotionSystem::setSecondaryLegSelection(int leg_index) {
    if (leg_index < -1 || leg_index >= NUM_LEGS) {
        return false;
    }
    secondary_leg_selection_ = leg_index;
    return true;
}

bool LocomotionSystem::setPrimaryLegState(LegState state) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    if (primary_leg_selection_ < 0 || primary_leg_selection_ >= NUM_LEGS) {
        return false;
    }
    return state_controller_->setLegState(primary_leg_selection_, state);
}

bool LocomotionSystem::setSecondaryLegState(LegState state) {
    if (!state_controller_ || !state_controller_->isInitialized()) {
        last_error = STATE_ERROR;
        return false;
    }
    if (secondary_leg_selection_ < 0 || secondary_leg_selection_ >= NUM_LEGS) {
        return false;
    }
    return state_controller_->setLegState(secondary_leg_selection_, state);
}

void LocomotionSystem::setPrimaryTipVelocity(const Eigen::Vector3d &velocity) {
    if (!state_controller_ || primary_leg_selection_ < 0) {
        return;
    }
    state_controller_->setLegTipVelocity(primary_leg_selection_, velocity);
}

void LocomotionSystem::setSecondaryTipVelocity(const Eigen::Vector3d &velocity) {
    if (!state_controller_ || secondary_leg_selection_ < 0) {
        return;
    }
    state_controller_->setLegTipVelocity(secondary_leg_selection_, velocity);
}

void LocomotionSystem::setPrimaryTipPose(const Point3D &pose) {
    if (!state_controller_ || primary_leg_selection_ < 0) {
        return;
    }
    state_controller_->setLegTipPose(primary_leg_selection_, pose);
}

void LocomotionSystem::setSecondaryTipPose(const Point3D &pose) {
    if (!state_controller_ || secondary_leg_selection_ < 0) {
        return;
    }
    state_controller_->setLegTipPose(secondary_leg_selection_, pose);
}

/* ===================================================================
 *  ROS-equivalent publication accessors (output getters).
 *
 *  Each method mirrors one OpenSHC ROS publisher topic, exposing
 *  internal state that external software would have obtained through
 *  topic subscription in OpenSHC.
 * =================================================================== */

LocomotionSystem::VelocityCommand LocomotionSystem::getDesiredVelocityCommand() const {
    VelocityCommand cmd;
    cmd.linear_x = commanded_linear_velocity_x_;
    cmd.linear_y = commanded_linear_velocity_y_;
    cmd.angular_z = commanded_angular_velocity_;
    return cmd;
}

LocomotionSystem::BodyPoseCommand LocomotionSystem::getDesiredBodyPoseCommand() const {
    BodyPoseCommand cmd;
    if (body_pose_ctrl) {
        const Pose &current = body_pose_ctrl->getCurrentBodyPose();
        cmd.position_x = current.position.x;
        cmd.position_y = current.position.y;
        cmd.position_z = current.position.z;
        /** Extract Euler angles from the quaternion (roll, pitch, yaw). */
        Eigen::Vector3d euler = current.rotation.toRotationMatrix().canonicalEulerAngles(0, 1, 2);
        cmd.roll = euler.x();
        cmd.pitch = euler.y();
        cmd.yaw = euler.z();
    } else {
        cmd.position_x = body_position.x();
        cmd.position_y = body_position.y();
        cmd.position_z = body_position.z();
        cmd.roll = body_orientation.x();
        cmd.pitch = body_orientation.y();
        cmd.yaw = body_orientation.z();
    }
    return cmd;
}

LocomotionSystem::WalkspaceInfo LocomotionSystem::getWalkspaceInfo() const {
    WalkspaceInfo info;
    info.average_radius = 0.0;
    info.min_radius = 0.0;
    info.max_radius = 0.0;

    if (!walk_ctrl) {
        return info;
    }

    std::map<int, double> walkspace = walk_ctrl->getWalkspace();
    if (walkspace.empty()) {
        return info;
    }

    double sum = 0.0;
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();
    for (const auto &entry : walkspace) {
        sum += entry.second;
        if (entry.second < min_val)
            min_val = entry.second;
        if (entry.second > max_val)
            max_val = entry.second;
    }
    info.average_radius = sum / static_cast<double>(walkspace.size());
    info.min_radius = min_val;
    info.max_radius = max_val;
    return info;
}

LocomotionSystem::RotationPoseError LocomotionSystem::getRotationPoseError() const {
    RotationPoseError err;
    err.absement_error = Eigen::Vector3d::Zero();
    err.position_error = Eigen::Vector3d::Zero();
    err.velocity_error = Eigen::Vector3d::Zero();

    if (body_pose_ctrl) {
        err.absement_error = body_pose_ctrl->getRotationAbsementError();
        err.position_error = body_pose_ctrl->getRotationPositionError();
        err.velocity_error = body_pose_ctrl->getRotationVelocityError();
    }
    return err;
}

bool LocomotionSystem::getDesiredJointStates(JointAngles positions[NUM_LEGS],
                                             JointAngles velocities[NUM_LEGS],
                                             JointAngles efforts[NUM_LEGS]) const {
    if (!system_enabled) {
        return false;
    }

    for (int i = 0; i < NUM_LEGS; ++i) {
        DesiredJointCommandState coxa_state, femur_state, tibia_state;
        getDesiredJointCommandState(i, 0, coxa_state);
        getDesiredJointCommandState(i, 1, femur_state);
        getDesiredJointCommandState(i, 2, tibia_state);

        positions[i].coxa = coxa_state.desired_position_rad;
        positions[i].femur = femur_state.desired_position_rad;
        positions[i].tibia = tibia_state.desired_position_rad;

        velocities[i].coxa = coxa_state.desired_velocity_rad_s;
        velocities[i].femur = femur_state.desired_velocity_rad_s;
        velocities[i].tibia = tibia_state.desired_velocity_rad_s;

        efforts[i].coxa = coxa_state.desired_effort;
        efforts[i].femur = femur_state.desired_effort;
        efforts[i].tibia = tibia_state.desired_effort;
    }
    return true;
}

LocomotionSystem::LegStateInfo LocomotionSystem::getLegStateInfo(int leg_index) const {
    LegStateInfo info;
    info.walker_tip_pose = Point3D();
    info.target_tip_pose = Point3D();
    info.model_tip_pose = Point3D();
    info.joint_positions = JointAngles();
    info.joint_velocities = JointAngles();
    info.joint_efforts = JointAngles();
    info.step_progress = 0.0;
    info.step_phase = STANCE_PHASE;
    info.leg_state = LEG_WALKING;
    info.contact_force = 0.0;
    info.admittance_delta = Eigen::Vector3d::Zero();
    info.virtual_stiffness = 0.0;
    info.tip_force = Eigen::Vector3d::Zero();
    info.stride_vector = Point3D();
    info.tip_velocity = Point3D();

    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return info;
    }

    const Leg &leg = legs[leg_index];

    /** Walker tip pose from LegStepper trajectory. */
    if (walk_ctrl) {
        auto stepper = walk_ctrl->getLegStepper(leg_index);
        if (stepper) {
            info.walker_tip_pose = stepper->getCurrentTipPose();
            info.step_progress = stepper->getStepProgress();
            info.stride_vector = stepper->getStrideVector();
            info.tip_velocity = stepper->getCurrentTipVelocity();
        }
    }

    /** Target tip pose and model (IK-solved) tip pose from Leg. */
    info.target_tip_pose = leg.getDesiredTipPosition();
    info.model_tip_pose = leg.getCurrentTipPositionGlobal();

    /** Joint state. */
    info.joint_positions = leg.getJointAngles();
    info.joint_velocities = leg.getCurrentJointVelocity();
    info.joint_efforts = leg.getCurrentJointEffort();

    /** Phase and state. */
    info.step_phase = leg.getStepPhase();
    info.leg_state = leg.getLegState();

    /** Force and admittance. */
    info.contact_force = leg.getContactForce();
    info.admittance_delta = leg.getAdmittanceDelta();
    info.virtual_stiffness = leg.getVirtualStiffness();
    info.tip_force = leg.getCalculatedTipForce();

    return info;
}

Pose LocomotionSystem::getOdometry() const {
    if (walk_ctrl) {
        return walk_ctrl->getOdometryIdeal();
    }
    return Pose(Point3D(body_position.x(), body_position.y(), body_position.z()),
                Eigen::Quaterniond::Identity());
}

WalkState LocomotionSystem::getWalkState() const {
    if (walk_ctrl) {
        return walk_ctrl->getWalkState();
    }
    return WALK_STOPPED;
}

PosingMode LocomotionSystem::getPosingMode() const {
    if (state_controller_) {
        return state_controller_->getPosingMode();
    }
    return POSING_NONE;
}

PoseResetMode LocomotionSystem::getPoseResetMode() const {
    if (state_controller_) {
        return state_controller_->getPoseResetMode();
    }
    return NO_RESET;
}

GaitType LocomotionSystem::getCurrentGaitType() const {
    if (walk_ctrl) {
        return walk_ctrl->getCurrentGaitConfig().gait_type;
    }
    return NO_GAIT;
}

int LocomotionSystem::getPrimaryLegSelection() const {
    return primary_leg_selection_;
}

int LocomotionSystem::getSecondaryLegSelection() const {
    return secondary_leg_selection_;
}

#ifdef TESTING_ENABLED
/** Testing: coxa telemetry instrumentation. */
void LocomotionSystem::recordCoxaTelemetrySample() {
    if (!telemetry_enabled_)
        return;
    CoxaTelemetrySample sample{};
    sample.time = telemetry_time_accumulator_;
    /** Acquire instantaneous data + stride vectors + servo commands. */
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = legs[i].getJointAngles();
        /** Absolute. */
        double global = a.coxa;
        /** Local (offset removed). */
        double local = global - BASE_THETA_OFFSETS[i];
        sample.global_angle[i] = global;
        sample.local_angle[i] = local;
        sample.phase[i] = legs[i].getStepPhase();
        /**
         * Stride baseline logic: set when entering stance for first time in cycle;
         * keep across swing to observe full leg cycle displacement.
         */
        if (sample.phase[i] == STANCE_PHASE && !stride_start_valid_[i]) {
            stride_start_tip_[i] = legs[i].getCurrentTipPositionGlobal();
            stride_start_valid_[i] = true;
        } else if (sample.phase[i] == SWING_PHASE && stride_start_valid_[i]) {
            /** Reset baseline at swing start so next STANCE establishes a fresh forward reference. */
            /** This avoids accumulating backwards drift across multiple cycles. */
            stride_start_valid_[i] = false;
        }
        Point3D tip = legs[i].getCurrentTipPositionGlobal();
        if (stride_start_valid_[i]) {
            sample.stride_dx[i] = tip.x - stride_start_tip_[i].x;
            sample.stride_dy[i] = tip.y - stride_start_tip_[i].y;
        } else {
            sample.stride_dx[i] = 0.0;
            sample.stride_dy[i] = 0.0;
        }
        /** Store radians. */
        sample.servo_command_coxa[i] = math_utils::degreesToRadians(last_servo_cmd_deg_[i][0]);
    }
    sample.body_vel_x = commanded_linear_velocity_x_;
    sample.body_vel_y = commanded_linear_velocity_y_;
    sample.body_ang_vel = commanded_angular_velocity_;
    /** Derive velocities and accelerations via finite differences. */
    if (prev_valid_) {
        /** Guard. */
        double dt = params.time_delta > 1e-9 ? params.time_delta : 1e-3;
        for (int i = 0; i < NUM_LEGS; ++i) {
            double v_global = (sample.global_angle[i] - prev_coxa_angle_[i]) / dt;
            double v_local = (sample.local_angle[i] - (prev_coxa_angle_[i] - BASE_THETA_OFFSETS[i])) / dt;
            double a_global = (v_global - prev_coxa_velocity_[i]) / dt;
            /** Same prev velocity source since offset constant. */
            double a_local = (v_local - (prev_coxa_velocity_[i])) / dt;
            sample.global_velocity[i] = v_global;
            sample.local_velocity[i] = v_local;
            sample.global_accel[i] = a_global;
            sample.local_accel[i] = a_local;
        }
    } else {
        for (int i = 0; i < NUM_LEGS; ++i) {
            sample.global_velocity[i] = 0.0;
            sample.local_velocity[i] = 0.0;
            sample.global_accel[i] = 0.0;
            sample.local_accel[i] = 0.0;
        }
    }
    /** Update previous. */
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev_coxa_velocity_[i] = sample.global_velocity[i];
        prev_coxa_angle_[i] = sample.global_angle[i];
    }
    prev_valid_ = true;

    if (telemetry_.size() >= kMaxTelemetrySamples_) {
        /** Simple ring buffer behavior: drop oldest by shifting (cost acceptable for test-sized buffer). */
        telemetry_.erase(telemetry_.begin());
    }
    telemetry_.push_back(sample);
}
#endif
