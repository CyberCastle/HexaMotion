#include "locomotion_system.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "walk_controller.h"
#include "workspace_analyzer.h" // Add unified analyzer
#include <algorithm>
#include <cmath>
#include <vector>
#ifndef ARDUINO
#include <chrono>
#endif

// Constructor
LocomotionSystem::LocomotionSystem(const Parameters &params)
    : params(params), imu_interface(nullptr), fsr_interface(nullptr), servo_interface(nullptr),
      body_position(0.0f, 0.0f, params.standing_height), body_orientation(0.0f, 0.0f, 0.0f),
      legs{Leg(0, model), Leg(1, model), Leg(2, model), Leg(3, model), Leg(4, model), Leg(5, model)},
      system_enabled(false), velocity_controller(nullptr), last_error(NO_ERROR),
      model(params), body_pose_ctrl(nullptr), walk_ctrl(nullptr), admittance_ctrl(nullptr),
      system_state(SYSTEM_UNKNOWN), startup_in_progress(false), shutdown_in_progress(false) {

    // Initialize last logged phases for FSR debug debouncing (testing only)
#ifdef TESTING_ENABLED
    for (int i = 0; i < NUM_LEGS; ++i) {
        last_logged_leg_phase_[i] = SWING_PHASE; // arbitrary default
    }
    last_logged_initialized_ = true;
#endif
}

// Destructor
LocomotionSystem::~LocomotionSystem() {
    system_enabled = false;
    delete body_pose_ctrl;
    delete walk_ctrl;
    delete admittance_ctrl;
    delete velocity_controller;
}

// System initialization
bool LocomotionSystem::initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo, const BodyPoseConfiguration &pose_config) {
    if (!imu || !fsr || !servo) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    imu_interface = imu;
    fsr_interface = fsr;
    servo_interface = servo;

    // Initialize interfaces
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

    // Initialize controllers with proper architecture
    body_pose_ctrl = new BodyPoseController(model, pose_config);
    walk_ctrl = new WalkController(model, legs, pose_config);
    admittance_ctrl = new AdmittanceController(model, imu_interface, fsr_interface);
    velocity_controller = new CartesianVelocityController(model);

    // Initialize LegPosers in BodyPoseController
    body_pose_ctrl->initializeLegPosers(legs);

    // Validate parameters
    if (!validateParameters()) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    // Initialize legs with default stance position
    // This happens after DH parameters are initialized in the constructor
    Pose default_stance(Point3D(0, 0, -params.standing_height), Eigen::Vector3d(0, 0, 0));
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].initialize(default_stance);
    }

    system_enabled = true;
    return true;
}

// System status check
bool LocomotionSystem::isSystemEnabled() const {
    return system_enabled;
}

// System calibration
bool LocomotionSystem::calibrateSystem() {
    // Calibrate IMU
    if (!imu_interface->calibrate()) {
        last_error = IMU_ERROR;
        return false;
    }

    // Calibrate FSRs
    for (int i = 0; i < NUM_LEGS; i++) {
        if (!fsr_interface->calibrateFSR(i)) {
            last_error = FSR_ERROR;
            return false;
        }
    }

    // Set initial pose
    setStandingPose();

    return true;
}

// Inverse kinematics using an optimized geometric method
JointAngles LocomotionSystem::calculateInverseKinematics(int leg,
                                                         const Point3D &p_target) {
    // Use current joint angles as starting point for IK
    JointAngles current_angles = legs[leg].getJointAngles();
    return model.inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, p_target);
}

bool LocomotionSystem::isTargetReachable(int leg_index, const Point3D &target) {
    // Use the WorkspaceAnalyzer from RobotModel for consistency and performance
    return model.getWorkspaceAnalyzer().isPositionReachable(leg_index, target);
}

Point3D LocomotionSystem::constrainToWorkspace(int leg_index, const Point3D &target) {
    // Use the WorkspaceAnalyzer from RobotModel for consistency and performance
    Point3D dummy_positions[6]; // Empty positions for basic geometric constraint
    for (int i = 0; i < 6; i++) {
        dummy_positions[i] = Point3D(0, 0, 0);
    }
    return model.getWorkspaceAnalyzer().constrainToValidWorkspace(leg_index, target, dummy_positions);
}

double LocomotionSystem::getJointLimitProximity(int leg_index, const JointAngles &angles) {
    // OpenSHC-style joint limit proximity calculation
    double min_proximity = 1.0f;

    // Check each joint proximity to limits
    double joints[3] = {angles.coxa, angles.femur, angles.tibia};
    double limits[3][2] = {
        {params.coxa_angle_limits[0], params.coxa_angle_limits[1]},
        {params.femur_angle_limits[0], params.femur_angle_limits[1]},
        {params.tibia_angle_limits[0], params.tibia_angle_limits[1]}};

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

/* Store angles both in RAM and servos */
bool LocomotionSystem::setLegJointAngles(int leg, const JointAngles &q) {
    if (!servo_interface)
        return false;

    // Verify that all servos in this leg are ready for movement
    for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
        if (servo_interface->hasBlockingStatusFlags(leg, joint)) {
            // Servo is blocked, cannot move this leg
            last_error = SERVO_BLOCKED_ERROR;
            return false;
        }
    }

    // Clamp angles to joint limits instead of rejecting them
    // This is the OpenSHC approach for handling workspace edge cases
    JointAngles clamped_angles = q;
    clamped_angles.coxa = math_utils::clamp(q.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
    clamped_angles.femur = math_utils::clamp(q.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    clamped_angles.tibia = math_utils::clamp(q.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    // Use velocity controller to get appropriate servo speeds
    double coxa_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 0) : params.default_servo_speed;
    double femur_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 1) : params.default_servo_speed;
    double tibia_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 2) : params.default_servo_speed;

    // Apply sign inversion/preservation per servo using params.angle_sign_* (adjust left/right or above/below servo orientation)
    double servo_coxa = clamped_angles.coxa * params.angle_sign_coxa * RADIANS_TO_DEGREES_FACTOR;
    double servo_femur = clamped_angles.femur * params.angle_sign_femur * RADIANS_TO_DEGREES_FACTOR;
    double servo_tibia = clamped_angles.tibia * params.angle_sign_tibia * RADIANS_TO_DEGREES_FACTOR;

    // Enable coxa movement based for test mode
    // This allows us to gate coxa servo output during test
    // If coxa movement is disabled, freeze coxa at 0º angle
    if (coxa_movement_enabled_) {
        // Normal behavior: command coxa as computed
        servo_interface->setJointAngleAndSpeed(leg, 0, servo_coxa, coxa_speed);
    } else {
        // Test mode: freeze coxa at 0º angle
        servo_interface->setJointAngleAndSpeed(leg, 0, 0, coxa_speed);
    }

    // Apply the computed angles to the servos
    servo_interface->setJointAngleAndSpeed(leg, 1, servo_femur, femur_speed);
    servo_interface->setJointAngleAndSpeed(leg, 2, servo_tibia, tibia_speed);
    return true;
}

// Gait planner
bool LocomotionSystem::setGaitType(GaitType gait) {
    bool result = false;

    switch (gait) {
    case TRIPOD_GAIT:
        result = walk_ctrl->setGait(TRIPOD_GAIT);
        break;
    case WAVE_GAIT:
        result = walk_ctrl->setGait(WAVE_GAIT);
        break;
    case RIPPLE_GAIT:
        result = walk_ctrl->setGait(RIPPLE_GAIT);
        break;
    case METACHRONAL_GAIT:
        result = walk_ctrl->setGait(METACHRONAL_GAIT);
        break;
    default:
        return false;
    }

    // Update BodyPoseController with current gait type for startup sequence selection
    if (result && body_pose_ctrl) {
        body_pose_ctrl->setCurrentGaitType(gait);
    }

    return result;
}

// Gait sequence planning - Use WalkController with LegStepper (OpenSHC pattern)
bool LocomotionSystem::planGaitSequence(double velocity_x, double velocity_y, double angular_velocity) {
    // Store persistent velocities (OpenSHC pattern)
    commanded_linear_velocity_x_ = velocity_x;
    commanded_linear_velocity_y_ = velocity_y; // Now properly stored!
    commanded_angular_velocity_ = angular_velocity;
    // Defer stride recomputation to per-cycle update() after ramp/limit applied
    return true;
}

// Forward locomotion control
bool LocomotionSystem::walkForward(double velocity) {
    return planGaitSequence(velocity, 0.0f, 0.0f);
}

// Backward locomotion control
bool LocomotionSystem::walkBackward(double velocity) {
    return planGaitSequence(-velocity, 0.0f, 0.0f);
}

// In-place turning control
bool LocomotionSystem::turnInPlace(double angular_velocity) {
    return planGaitSequence(0.0f, 0.0f, angular_velocity);
}

// Sideways locomotion control
bool LocomotionSystem::walkSideways(double velocity, bool right_direction) {
    // Framework convention: +Y = left, -Y = right
    double lateral_velocity = right_direction ? -velocity : velocity;
    return planGaitSequence(0.0f, lateral_velocity, 0.0f);
}

bool LocomotionSystem::executeStartupSequence() {

    // Execute the body pose controller startup sequence
    bool startup_complete = body_pose_ctrl->executeStartupSequence(legs);

    if (startup_complete) {
        // Initialize walk controller for RUNNING state (OpenSHC pattern)
        walk_ctrl->init(body_position, body_orientation);
        walk_ctrl->generateWalkspace();

        // OpenSHC PATTERN: DO NOT apply velocities during startup!
        // Velocities will be applied in update() method after startup completes
        // This prevents the velocity_y loss bug that was happening here

        // Initialize leg phases based on gait pattern (without velocity application)
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

        system_state = SYSTEM_RUNNING;

        // Clear flag here instead of requiring external code to modify private member
        startup_in_progress = false;
    }

    return startup_complete;
}

bool LocomotionSystem::executeShutdownSequence() {

    // Update walk controller one final time with zero velocity
    walk_ctrl->updateWalk(Point3D(0.0, 0.0, 0.0), 0.0,
                          Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));

    // OpenSHC: Force all legs to STANCE according to OpenSHC shutdown protocol
    for (int i = 0; i < NUM_LEGS; i++) {
        auto leg_stepper = walk_ctrl->getLegStepper(i);
        if (leg_stepper) {
            leg_stepper->setStepState(STEP_FORCE_STOP);
            leg_stepper->setPhase(0.0); // Force to stance position

            // Immediately update tip position to identity (stance) position
            leg_stepper->updateTipPositionIterative(0, params.time_delta, false, false);

            // Get the forced stance position and apply to leg
            legs[i].setStepPhase(STANCE_PHASE);
            Point3D stance_position = leg_stepper->getCurrentTipPose();
            legs[i].setCurrentTipPositionGlobal(stance_position);

            // Apply IK and servo commands immediately
            if (legs[i].applyAdvancedIK(stance_position)) {
                JointAngles target_angles = legs[i].getJointAngles();
                setLegJointAngles(i, target_angles);
            }
        }
    }

    // Execute the body pose controller shutdown sequence
    bool shutdown_complete = body_pose_ctrl->executeShutdownSequence(legs);

    if (shutdown_complete) {
        system_state = SYSTEM_READY;

        // Clear internal flag upon completion to simplify caller logic
        shutdown_in_progress = false;

        // Reset body pose controller sequence states so next startup re-learns transition
        if (body_pose_ctrl) {
            body_pose_ctrl->resetSequenceStates();
        }
    }

    return shutdown_complete;
}

// Check stability margin
bool LocomotionSystem::checkStabilityMargin() {
    // Create temporary arrays for compatibility with admittance controller
    Point3D temp_leg_positions[NUM_LEGS];
    StepPhase temp_leg_states[NUM_LEGS];

    for (int i = 0; i < NUM_LEGS; i++) {
        temp_leg_positions[i] = legs[i].getCurrentTipPositionGlobal();
        temp_leg_states[i] = legs[i].getStepPhase();
    }

    return admittance_ctrl->checkStability(temp_leg_positions, temp_leg_states);
}

// Compute center of pressure
Eigen::Vector2d LocomotionSystem::calculateCenterOfPressure() {
    Eigen::Vector2d cop(0.0f, 0.0f);
    double total_force = 0.0f;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            FSRData fsr_data = fsr_interface->readFSR(i);
            if (fsr_data.in_contact && fsr_data.pressure > 0) {
                cop[0] += legs[i].getCurrentTipPositionGlobal().x * fsr_data.pressure;
                cop[1] += legs[i].getCurrentTipPositionGlobal().y * fsr_data.pressure;
                total_force += fsr_data.pressure;
            }
        }
    }

    if (total_force > 0) {
        cop /= total_force;
    }

    return cop;
}

// Compute stability index
double LocomotionSystem::calculateStabilityIndex() {
    if (!checkStabilityMargin())
        return 0.0f;

    Eigen::Vector2d cop = calculateCenterOfPressure();
    double stability_index = 1.0f;

    // Enhanced implementation: Calculate stability margin properly
    // Get support polygon from current stance legs
    std::vector<Point3D> support_polygon;
    Point3D stance_positions[NUM_LEGS];
    int stance_count = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            stance_positions[stance_count] = legs[i].getCurrentTipPositionGlobal();
            support_polygon.push_back(legs[i].getCurrentTipPositionGlobal());
            stance_count++;
        }
    }

    if (stance_count < 3) {
        // Insufficient support for stability
        return 0.0f;
    }

    // Calculate minimum distance from COP to support polygon edges
    double min_edge_distance = 1000.0f;

    // For each stance leg, calculate distance from COP
    for (int i = 0; i < stance_count; i++) {
        double distance = sqrt((stance_positions[i].x - cop[0]) * (stance_positions[i].x - cop[0]) +
                               (stance_positions[i].y - cop[1]) * (stance_positions[i].y - cop[1]));
        if (distance < min_edge_distance) {
            min_edge_distance = distance;
        }
    }

    // Normalize stability index based on minimum required margin
    double required_margin = params.stability_margin;
    stability_index = min_edge_distance / (required_margin * 2.0f); // Factor of 2 for good margin

    return math_utils::clamp<double>(stability_index, 0.0, 1.0);
}

// Check static stability
bool LocomotionSystem::isStaticallyStable() {
    return calculateStabilityIndex() > 0.2f; // Minimum stability threshold
}

// Body pose control
// Set standing pose using BodyPoseController with LegPoser
bool LocomotionSystem::setStandingPose() {
    // Use BodyPoseController with LegPoser for pose control
    bool success = body_pose_ctrl->setStandingPose(legs);

    if (success) {
        // Ensure all legs are in STANCE_PHASE for standing pose
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setStepPhase(STANCE_PHASE);
        }

        // Apply the calculated joint angles to servos
        for (int i = 0; i < NUM_LEGS; i++) {
            JointAngles angles = legs[i].getJointAngles();
            if (!setLegJointAngles(i, angles)) {
                last_error = KINEMATICS_ERROR;
                return false;
            }
        }

        // Update body position based on actual leg positions through BodyPoseController
        // The BodyPoseController handles the calculation of body position
        body_position = body_pose_ctrl->calculateBodyPosition(legs);

        // Set system state to READY (OpenSHC equivalent)
        system_state = SYSTEM_READY;
        startup_in_progress = false;
        shutdown_in_progress = false;

        return true;
    } else {
        last_error = KINEMATICS_ERROR;
        return false;
    }
}

bool LocomotionSystem::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    // orientation expected in radians (roll,pitch,yaw)

    // Use BodyPoseController to set the pose
    bool success = body_pose_ctrl->setBodyPose(position, orientation, legs);

    if (success) {
        // Apply the calculated joint angles to servos
        for (int i = 0; i < NUM_LEGS; i++) {
            JointAngles angles = legs[i].getJointAngles();
            if (!setLegJointAngles(i, angles)) {
                last_error = KINEMATICS_ERROR;
                return false;
            }
        }

        // Update body position and orientation
        body_position = position;
        body_orientation = orientation;

        return true;
    } else {
        last_error = KINEMATICS_ERROR;
        return false;
    }
}

bool LocomotionSystem::isSmoothMovementInProgress() const {
    return body_pose_ctrl->isTrajectoryInProgress();
}

void LocomotionSystem::resetSmoothMovement() {
    if (body_pose_ctrl)
        body_pose_ctrl->resetTrajectory();
}

// Main system update - Only calculates leg trajectories based on current phase
/*
 * USAGE PATTERN:
 *
 * while (system_running) {
 *     // Handle startup/shutdown sequences separately
 *     if (locomotion_system.isStartupInProgress()) {
 *         if (locomotion_system.executeStartupSequence()) {
 *             locomotion_system.startup_in_progress = false;
 *         }
 *     } else if (locomotion_system.isShutdownInProgress()) {
 *         if (locomotion_system.executeShutdownSequence()) {
 *             locomotion_system.shutdown_in_progress = false;
 *         }
 *     } else {
 *         // Only update leg trajectories when system is in normal operation
 *         locomotion_system.update();
 *     }
 * }
 */
bool LocomotionSystem::update() {

    // Update sensors in parallel for optimal performance
    if (!updateSensorsParallel()) {
        last_error = SENSOR_ERROR;
        return false;
    }

    // Update FSR contact history early (for swing adaptation inside LegStepper) WITHOUT final phase assignment
    // Phase (STANCE/SWING) will be finalized after walk controller updates trajectories.
    if (params.use_fsr_contact) {
        updateLegStates(); // early call: updates in_contact_ & histories; phase may be overwritten later
    }

    // Handle initial standing pose transition (non-blocking) prior to normal running
    // During initial standing S-curve we temporarily keep system_state as SYSTEM_READY until finished
    if (body_pose_ctrl && body_pose_ctrl->isInitialStandingPoseActive()) {
        if (!stepInitialStandingPose()) {
            return false; // error already set
        }
    }

    // Only update leg trajectories if system is in RUNNING state
    if (walk_ctrl && system_state == SYSTEM_RUNNING) {
        // Optional kinematic integration of body pose (test / simulation)
        if (params.enable_body_translation) {
            // Integrate translation (mm) and yaw (degrees) from commanded velocities
            body_position[0] += commanded_linear_velocity_x_ * params.time_delta; // mm/s * s
            body_position[1] += commanded_linear_velocity_y_ * params.time_delta;
            body_orientation[2] += commanded_angular_velocity_ * params.time_delta; // deg/s * s
            // Wrap yaw to [-180,180]
            if (body_orientation[2] > 180.0)
                body_orientation[2] -= 360.0;
            else if (body_orientation[2] < -180.0)
                body_orientation[2] += 360.0;
        }

        // STEP 1: Update walk controller (it performs its own limiting & ramping via VelocityLimits)
        Point3D applied_linear_velocity(commanded_linear_velocity_x_, commanded_linear_velocity_y_, 0.0);
        walk_ctrl->updateWalk(applied_linear_velocity,
                              commanded_angular_velocity_,
                              body_position, body_orientation);

        // STEP 2: Collect desired positions from Bézier trajectories (= OpenSHC::setDesiredTipPose)
        for (int i = 0; i < NUM_LEGS; i++) {
            auto leg_stepper = walk_ctrl->getLegStepper(i);
            if (leg_stepper) {
                // Do not set StepPhase here; defer to updateLegStates() after considering FSR contact

                // OpenSHC pattern: Use Bézier-calculated position for BOTH swing AND stance phases
                // The LegStepper.updateTipPositionIterative() correctly handles both phases:
                // - SWING: Uses swing Bézier curves for air movement
                // - STANCE: Uses stance Bézier curves for ground movement (coxa movement)
                Point3D desired_tip_position = leg_stepper->getCurrentTipPose();
                legs[i].setDesiredTipPosition(desired_tip_position);
            }
        }

        // STEP 2a: Update body pose (partial OpenSHC PoseController::updateCurrentPose)
        // We only update auto-pose modulation and walk plane pose estimation here.
        if (body_pose_ctrl) {

            // Derive normalized gait phase [0,1) from first leg stepper (consistent across legs in synchronized gaits)
            double gait_phase = 0.0;
            StepCycle sc = walk_ctrl->getStepCycle();
            int period = sc.period_ > 0 ? sc.period_ : 1;
            auto leg0 = walk_ctrl->getLegStepper(0);
            if (leg0) {
                gait_phase = static_cast<double>(leg0->getPhase() % period) / static_cast<double>(period);
            }
            body_pose_ctrl->updateCurrentPose(gait_phase, legs);
        }

        // STEP 2b: Finalize leg phases (FSR or pure kinematic) after trajectories computed
        updateLegStates();

        // STEP 3: Apply IK to ALL legs at once (= OpenSHC::Model::updateModel)
        applyInverseKinematicsToAllLegs();

        // STEP 4: Publish ALL joint angles to servos (= OpenSHC::publishDesiredJointState)
        publishJointAnglesToServos();

#ifdef TESTING_ENABLED
        // Registrar muestra de telemetría tras aplicar servos / IK (estado final del ciclo)
        if (telemetry_enabled_) {
            telemetry_time_accumulator_ += params.time_delta;
            recordCoxaTelemetrySample();
        }
#endif
    } else if (system_state == SYSTEM_READY) {
        // OpenSHC: When system is READY (after shutdown), maintain STANCE_PHASE for ALL legs
        // This prevents sys.update() calls from overriding the shutdown-forced STANCE states
        // Do NOT read leg_stepper states - preserve the shutdown state
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setStepPhase(STANCE_PHASE);
        }
    }

    return true;
}

// Error handling
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
        // Try to reinitialize IMU
        if (imu_interface) {
            return imu_interface->initialize();
        }
        break;

    case FSR_ERROR:
        // Try to recalibrate FSRs
        if (fsr_interface) {
            for (int i = 0; i < NUM_LEGS; i++) {
                fsr_interface->calibrateFSR(i);
            }
            return true;
        }
        break;

    case SERVO_ERROR:
        // Try to reinitialize servos
        if (servo_interface) {
            return servo_interface->initialize();
        }
        break;

    case STABILITY_ERROR:
        // Adopt a more stable pose
        return setStandingPose();

    case KINEMATICS_ERROR:
        // Return to a safe pose
        return setStandingPose();

    case SERVO_BLOCKED_ERROR:
        // Cannot recover from blocked servos automatically - requires manual intervention
        // The error provides diagnostic information about which servos are blocked
        return false;

    default:
        return false;
    }

    return false;
}

void LocomotionSystem::updateLegStates() {
    // Two-stage logic:
    // 1. Update contact histories if FSR available
    // 2. Decide final STANCE/SWING phase using (a) filtered contact + hysteresis or (b) kinematic StepState

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
            bool filtered_contact = legs[i].getFilteredContactState(params.contact_threshold, params.release_threshold);
            StepPhase current_state = legs[i].getStepPhase();

            // Hysteresis transitions with optional debug logging
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

            // Safety: if kinematic expectation strongly indicates swing but we have low pressure (likely false contact), revert
            if (leg_stepper) {
                double phase_fraction = static_cast<double>(leg_stepper->getPhase()) /
                                        static_cast<double>(walk_ctrl->getStepCycle().period_);
                bool should_be_swing = legs[i].shouldBeInSwing(phase_fraction, walk_ctrl->getStanceDuration());
                if (should_be_swing && legs[i].getStepPhase() == STANCE_PHASE && legs[i].getContactForce() < params.min_pressure) {
                    legs[i].setStepPhase(SWING_PHASE);

#ifdef TESTING_ENABLED
                    if (params.debug_fsr_transitions) {
                        // Revert implies we force SWING; guard against duplicate logs
                        if (!last_logged_initialized_ || last_logged_leg_phase_[i] != SWING_PHASE) {
                            std::cout << "[FSR] leg=" << i << " REVERT STANCE->SWING low_pressure=" << legs[i].getContactForce()
                                      << " < min_pressure phase_guard" << std::endl;
                            last_logged_leg_phase_[i] = SWING_PHASE;
                        }
                    }
#endif
                }
            }
        } else {
            // No FSR: derive directly from LegStepper StepState (more direct than approximate phase math)
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

bool LocomotionSystem::setParameters(const Parameters &new_params) {
    // Validate new parameters
    if (new_params.hexagon_radius <= 0 || new_params.coxa_length <= 0 ||
        new_params.femur_length <= 0 || new_params.tibia_length <= 0) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params = new_params;
    return validateParameters();
}

double LocomotionSystem::calculateDynamicStabilityIndex() {
    // Enhanced stability analysis using absolute positioning data
    if (!imu_interface || !imu_interface->isConnected())
        return calculateStabilityIndex(); // Fallback to basic stability

    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return 0.5f; // Neutral stability

    double stability_index = 1.0f;

    // Enhanced stability calculation using absolute positioning
    if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
        // Orientation stability from absolute data
        double orientation_stability = 1.0f;
        double total_tilt = sqrt(
            imu_data.absolute_data.absolute_roll * imu_data.absolute_data.absolute_roll +
            imu_data.absolute_data.absolute_pitch * imu_data.absolute_data.absolute_pitch);

        if (total_tilt > 5.0f) {
            orientation_stability =
                math_utils::clamp<double>(1.0 - (total_tilt - 5.0) / 30.0, 0.2, 1.0);
        }

        // Dynamic motion stability from linear acceleration
        double motion_stability = 1.0f;
        if (imu_data.absolute_data.linear_acceleration_valid) {
            double acceleration_magnitude = sqrt(
                imu_data.absolute_data.linear_accel_x * imu_data.absolute_data.linear_accel_x +
                imu_data.absolute_data.linear_accel_y * imu_data.absolute_data.linear_accel_y +
                imu_data.absolute_data.linear_accel_z * imu_data.absolute_data.linear_accel_z);

            // High acceleration reduces stability
            if (acceleration_magnitude > 1.0f) {
                motion_stability =
                    math_utils::clamp<double>(1.0 - (acceleration_magnitude - 1.0) / 4.0, 0.3, 1.0);
            }
        }

        // Quaternion-based rotational stability
        double rotational_stability = 1.0f;
        if (imu_data.absolute_data.quaternion_valid) {
            double qx = imu_data.absolute_data.quaternion_x;
            double qy = imu_data.absolute_data.quaternion_y;
            double qz = imu_data.absolute_data.quaternion_z;

            // Calculate rotational deviation from level position
            double rotational_deviation = sqrt(qx * qx + qy * qy + qz * qz);
            if (rotational_deviation > 0.2f) {
                rotational_stability =
                    math_utils::clamp<double>(1.0 - (rotational_deviation - 0.2) / 0.6, 0.4, 1.0);
            }
        }

        // Combine stability factors
        stability_index = orientation_stability * motion_stability * rotational_stability;

        // Calibration status affects confidence in stability calculation
        if (imu_data.absolute_data.calibration_status < 2) {
            stability_index = 0.5f * stability_index + 0.5f * calculateStabilityIndex();
        }
    } else {
        // Fallback to basic stability calculation
        stability_index = calculateStabilityIndex();
    }

    // Include FSR-based stability if available
    if (fsr_interface) {
        double fsr_stability = calculateStabilityIndex();                // Basic FSR stability
        stability_index = 0.7f * stability_index + 0.3f * fsr_stability; // Blend IMU and FSR
    }

    return math_utils::clamp<double>(stability_index, 0.0, 1.0);
}

// Parallel sensor update implementation
bool LocomotionSystem::updateSensorsParallel() {
    bool fsr_updated = false;
    bool imu_updated = false;

    // Start parallel sensor updates
    // FSR: AdvancedAnalog DMA for simultaneous ADC reading
    // Only update if FSR usage is enabled in parameters to avoid blocking latency otherwise
    if (fsr_interface && params.use_fsr_contact) {
        fsr_updated = fsr_interface->update();
    }

    // IMU: Non-blocking sensor update (parallel with FSR)
    if (imu_interface && imu_interface->isConnected()) {
        imu_updated = imu_interface->update();
    }

    // Validate both sensors updated successfully
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

// Cartesian velocity control methods
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

// Execute startup sequence (READY -> RUNNING transition)
// Start walking with specified gait type
bool LocomotionSystem::startWalking() {
    if (!system_enabled || !walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }
    if (system_state != SYSTEM_READY) {
        last_error = STATE_ERROR;
        return false;
    }
    // Assumes gait already selected via setGaitType() and velocities set via walkForward/back/sideways/turn methods
    if (resume_from_stop_) {
        // Fast resume: skip body pose startup; re-init walk controller for current pose
        walk_ctrl->init(body_position, body_orientation);
        walk_ctrl->generateWalkspace();

        // Sync leg steppers and phases to current grounded foot poses to avoid pre-walk drop
        for (int i = 0; i < NUM_LEGS; i++) {
            auto leg_stepper = walk_ctrl->getLegStepper(i);
            if (leg_stepper) {
                Point3D current_pose = legs[i].getCurrentTipPositionGlobal();
                leg_stepper->setCurrentTipPose(current_pose);
                leg_stepper->setStepState(STEP_FORCE_STANCE);
            }
            legs[i].setStepPhase(STANCE_PHASE);
        }

        system_state = SYSTEM_RUNNING;
        resume_from_stop_ = false;
        startup_in_progress = false;
        return true;
    }

    // Normal path: run startup sequence next loop
    startup_in_progress = true;
    system_state = SYSTEM_READY;
    return true;
}

// Stop walking without shutdown; ensure all feet on ground. Mode controls stance behavior.
bool LocomotionSystem::stopWalking(StopMode mode) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }
    if (system_state != SYSTEM_RUNNING) {
        last_error = STATE_ERROR;
        return false;
    }

    // Zero velocities
    commanded_linear_velocity_x_ = 0.0;
    commanded_linear_velocity_y_ = 0.0;
    commanded_angular_velocity_ = 0.0;

    // Notify planner that velocity is zero
    walk_ctrl->updateWalk(Point3D(0.0, 0.0, 0.0), 0.0, body_position, body_orientation);

    // Place all feet on ground according to requested mode
    for (int i = 0; i < NUM_LEGS; ++i) {
        auto leg_stepper = walk_ctrl->getLegStepper(i);
        if (leg_stepper) {
            if (mode == STOP_UNIFORM) {
                // Uniform stop: identical stance for all legs
                leg_stepper->setStepState(STEP_FORCE_STOP);
                leg_stepper->setPhase(0.0);
                leg_stepper->updateTipPositionIterative(0, params.time_delta, false, false);
            } else { // STOP_SOFT
                // Soft stop: preserve XY but ensure Z is on stance plane
                leg_stepper->setStepState(STEP_FORCE_STANCE);

                // 1) Capture current (possibly airborne) tip pose
                Point3D current_pose = leg_stepper->getCurrentTipPose();

                // 2) Query identity stance Z by temporarily sampling phase=0
                double saved_phase = leg_stepper->getPhase();
                int saved_iter = leg_stepper->getCurrentIteration();

                leg_stepper->setPhase(0.0);
                leg_stepper->updateTipPositionIterative(0, params.time_delta, false, false);
                Point3D identity_pose = leg_stepper->getCurrentTipPose();

                // 3) Restore phase/iteration continuity for internal state
                leg_stepper->setPhase(saved_phase);
                leg_stepper->updateTipPositionIterative(saved_iter, params.time_delta, false, false);

                // 4) Build target stance pose: keep XY, drop to stance Z
                Point3D projected_pose(current_pose.x, current_pose.y, identity_pose.z);

                // Write projected pose into the stepper's current tip pose so downstream reads are consistent
                // Note: LegStepper doesn't expose a direct setter; we push via the Leg object below.
                // The READY state will freeze advancement until the next start.

                // Use projected pose as the stance target for this leg
                legs[i].setStepPhase(STANCE_PHASE);
                legs[i].setCurrentTipPositionGlobal(projected_pose);

                // Apply IK and servo commands for the projected stance pose, then continue to next leg
                if (legs[i].applyAdvancedIK(projected_pose)) {
                    JointAngles target_angles = legs[i].getJointAngles();
                    setLegJointAngles(i, target_angles);
                }

                // Continue to next leg – skip default stance application below for STOP_SOFT
                continue;
            }

            // Apply stance to leg and servos
            legs[i].setStepPhase(STANCE_PHASE);
            Point3D stance_position = leg_stepper->getCurrentTipPose();
            legs[i].setCurrentTipPositionGlobal(stance_position);
            if (legs[i].applyAdvancedIK(stance_position)) {
                JointAngles target_angles = legs[i].getJointAngles();
                setLegJointAngles(i, target_angles);
            }
        } else {
            legs[i].setStepPhase(STANCE_PHASE);
        }
    }

    // Transition to READY so update() no longer advances walking
    shutdown_in_progress = false;
    system_state = SYSTEM_READY;
    resume_from_stop_ = true; // mark for fast resume without dropping legs again
    return true;
}

// ====================================================================
// OPENSHC-STYLE IK BATCH PROCESSING
// ====================================================================

void LocomotionSystem::applyInverseKinematicsToAllLegs() {
    // Following OpenSHC::Model::updateModel() pattern exactly
    // Apply IK to ALL legs at once using applyAdvancedIK (equivalent to OpenSHC::solveIK)

    for (int i = 0; i < NUM_LEGS; i++) {
        // Get desired position from Bézier trajectory (pure, no IK applied yet)
        Point3D desired_tip_position = legs[i].getDesiredTipPosition();

        // Get current state for delta calculation
        JointAngles current_angles = legs[i].getJointAngles();
        Point3D current_tip_position = legs[i].getCurrentTipPositionGlobal();

        // Apply applyAdvancedIK (= OpenSHC::solveIK with DLS + cost gradients)
        JointAngles new_angles = model.applyAdvancedIK(
            i,                    // leg index
            current_tip_position, // current tip pose
            desired_tip_position, // desired tip pose (from Bézier)
            current_angles,       // current joint angles
            params.time_delta     // unified time delta
        );

        // Update joint angles in Leg object
        legs[i].setJointAngles(new_angles);

        // Update tip position using FK to maintain consistency
        legs[i].updateTipPosition();
    }
}

void LocomotionSystem::publishJointAnglesToServos() {

    // Refresh a small slice of health state per tick (non-blocking cache) if supported
    if (servo_interface) {
        servo_interface->refreshHealthSlice(3); // ~3 servos/tick => full scan ~6 ticks
    }

    // Prefer a single synchronous write when supported by the driver to remove per-joint bus latency
    if (servo_interface) {
        double angles_deg[NUM_LEGS][DOF_PER_LEG];
        double speeds[NUM_LEGS][DOF_PER_LEG];

        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles a = legs[i].getJointAngles();

            // Convert to output degrees with sign
            // Respect runtime coxa gating in the fast sync path as well
            if (coxa_movement_enabled_) {
                angles_deg[i][0] = a.coxa * params.angle_sign_coxa * RADIANS_TO_DEGREES_FACTOR;
            } else {
                angles_deg[i][0] = 0.0; // freeze coxa at 0° when disabled (same behavior as per-leg path)
            }

            angles_deg[i][1] = a.femur * params.angle_sign_femur * RADIANS_TO_DEGREES_FACTOR;
            angles_deg[i][2] = a.tibia * params.angle_sign_tibia * RADIANS_TO_DEGREES_FACTOR;

            // Per-joint speeds
            speeds[i][0] = velocity_controller ? velocity_controller->getServoSpeed(i, 0) : params.default_servo_speed;
            speeds[i][1] = velocity_controller ? velocity_controller->getServoSpeed(i, 1) : params.default_servo_speed;
            speeds[i][2] = velocity_controller ? velocity_controller->getServoSpeed(i, 2) : params.default_servo_speed;
        }

        if (servo_interface->syncSetAllJointAnglesAndSpeeds(angles_deg, speeds)) {
            // Cache last servo commands (degrees) for telemetry
#ifdef TESTING_ENABLED
            for (int li = 0; li < NUM_LEGS; ++li) {
                for (int j = 0; j < DOF_PER_LEG; ++j) {
                    last_servo_cmd_deg_[li][j] = angles_deg[li][j];
                }
            }
#endif
            return; // done
        }
    }

    // Fallback: per-leg commands
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles angles = legs[i].getJointAngles();

        // Send all angles for this leg to servos
        if (!setLegJointAngles(i, angles)) {
            // Handle servo failure - log error but continue with other legs
            last_error = SERVO_ERROR;
            // Continue processing other legs instead of failing completely
        }
        // Cache per-leg command (degrees)
#ifdef TESTING_ENABLED
        last_servo_cmd_deg_[i][0] = angles.coxa * params.angle_sign_coxa * RADIANS_TO_DEGREES_FACTOR;
        last_servo_cmd_deg_[i][1] = angles.femur * params.angle_sign_femur * RADIANS_TO_DEGREES_FACTOR;
        last_servo_cmd_deg_[i][2] = angles.tibia * params.angle_sign_tibia * RADIANS_TO_DEGREES_FACTOR;
#endif
    }
}

// =====================================================================================
// INITIAL STANDING POSE ESTABLISHMENT (jerk-limited smooth transition)
// =====================================================================================
bool LocomotionSystem::establishInitialStandingPose() {
    if (!servo_interface || !body_pose_ctrl) {
        last_error = SERVO_ERROR;
        return false;
    }
    if (body_pose_ctrl->isInitialStandingPoseActive()) {
        // Already in progress; just step
        return stepInitialStandingPose();
    }
    // Initialize controller-side profiles using current leg joint angles
    if (!body_pose_ctrl->beginInitialStandingPoseTransition(legs)) {
        return false; // no change
    }
    startup_in_progress = true;
    system_state = SYSTEM_READY; // use READY as transitional state for startup S-curve
    return stepInitialStandingPose();
}

bool LocomotionSystem::stepInitialStandingPose() {
    if (!servo_interface || !body_pose_ctrl) {
        last_error = SERVO_ERROR;
        return false;
    }
    if (!body_pose_ctrl->isInitialStandingPoseActive()) {
        return true; // nothing to do
    }

    // Use internal parameter set time delta
    double dt = params.time_delta; // time step seconds

    // Allocate temporary buffers for positions, velocities, accelerations per joint
    double positions[NUM_LEGS][3];
    double velocities[NUM_LEGS][3];
    double accelerations[NUM_LEGS][3];

    if (!body_pose_ctrl->stepInitialStandingPoseTransition(legs, dt, positions, velocities, accelerations)) {
        // stepping failed
        last_error = STATE_ERROR;
        return false;
    }

    // Map radian joint state to servo commands each iteration
    // Determine speed multipliers based on normalized velocity relative to heuristic vmax per joint (derived same way as in controller begin)
    // We recompute heuristic vmax in degrees/s to convert to speed multiplier (0..1), then send acceleration when available.
    double angles_deg[NUM_LEGS][3];
    double speeds[NUM_LEGS][3];
    double accels[NUM_LEGS][3];

    // Heuristic max velocity (degrees/s) consistent with earlier logic
    double vmax_deg = params.default_servo_speed; // heuristic max speed (deg/s) for normalization
    double amax_deg = vmax_deg * 4.0;

    // Precompute torque-balanced weight factors (relative) once (lazy static cache)
    static bool weight_factors_initialized = false;
    static double weight_factor[3] = {1.0, 1.0, 1.0};
    if (!weight_factors_initialized) {
        // Quasi-static torque proxy:
        // tau_joint ≈ (m_segment * g * L_segment/2) + (m_distal_total * g * L_segment)
        // If masses are not defined (>0) assume mass proportional to length.
        double Lc = params.coxa_length > 1e-6 ? params.coxa_length : 1.0;
        double Lf = params.femur_length > 1e-6 ? params.femur_length : 1.0;
        double Lt = params.tibia_length > 1e-6 ? params.tibia_length : 1.0;
        double mc = params.coxa_mass > 0.0 ? params.coxa_mass : Lc; // fallback proporcional
        double mf = params.femur_mass > 0.0 ? params.femur_mass : Lf;
        double mt = params.tibia_mass > 0.0 ? params.tibia_mass : Lt;
        // Distal accumulated masses
        double distal_femur = mt;     // femur ve tibia distal
        double distal_coxa = mf + mt; // coxa ve femur + tibia
        double tau_coxa = mc * Lc * 0.5 + distal_coxa * Lc;
        double tau_femur = mf * Lf * 0.5 + distal_femur * Lf;
        double tau_tibia = mt * Lt * 0.5; // tibia solo propia
        // Normalize relative to coxa
        double alpha = params.startup_norm.alpha;
        weight_factor[0] = 1.0; // coxa baseline
        weight_factor[1] = pow(tau_femur / tau_coxa, alpha);
        weight_factor[2] = pow(tau_tibia / tau_coxa, alpha);
        // Safety clamps
        for (int k = 0; k < 3; ++k) {
            if (weight_factor[k] < 1e-3)
                weight_factor[k] = 1e-3;
            if (weight_factor[k] > 10.0)
                weight_factor[k] = 10.0;
        }
        weight_factors_initialized = true;
    }

    bool in_lift_phase = body_pose_ctrl->isInitialStandingPoseActive() && !body_pose_ctrl->isInitialStandingAlignmentPhase();
    bool apply_torque_balanced = in_lift_phase && params.startup_norm.enable_torque_balanced;
    double v_dead = params.startup_norm.speed_deadband;
    double a_dead = params.startup_norm.accel_deadband;

    for (int i = 0; i < NUM_LEGS; ++i) {
        for (int j = 0; j < 3; ++j) {
            double pos_rad = positions[i][j];
            double vel_rad = velocities[i][j];
            double acc_rad = accelerations[i][j];
            double sign = 1.0;
            if (j == 0)
                sign = params.angle_sign_coxa;
            else if (j == 1)
                sign = params.angle_sign_femur;
            else
                sign = params.angle_sign_tibia;
            angles_deg[i][j] = pos_rad * sign * RADIANS_TO_DEGREES_FACTOR;
            double vel_deg = vel_rad * RADIANS_TO_DEGREES_FACTOR;
            double speed_mult = vmax_deg > 1e-6 ? fabs(vel_deg) / vmax_deg : 0.0;
            double acc_deg = acc_rad * RADIANS_TO_DEGREES_FACTOR;
            double accel_mult = amax_deg > 1e-6 ? fabs(acc_deg) / amax_deg : 0.0;
            if (apply_torque_balanced) {
                double wf = weight_factor[j];
                if (wf > 1e-6) {
                    speed_mult /= wf;
                    accel_mult /= wf;
                }
            }
            // Deadbands
            if (speed_mult < v_dead)
                speed_mult = v_dead;
            if (accel_mult < a_dead)
                accel_mult = a_dead;
            if (speed_mult > 1.0)
                speed_mult = 1.0;
            if (accel_mult > 1.0)
                accel_mult = 1.0;
            // Optional tibia speed cap
            if (apply_torque_balanced && j == 2 && params.startup_norm.tibia_speed_cap > 0.0 && speed_mult > params.startup_norm.tibia_speed_cap) {
                speed_mult = params.startup_norm.tibia_speed_cap;
            }
            speeds[i][j] = speed_mult;
            accels[i][j] = accel_mult;
        }
        if (!coxa_movement_enabled_) {
            // Freeze coxa if disabled
            angles_deg[i][0] = 0.0;
            speeds[i][0] = 0.0;
            accels[i][0] = 0.0;
        }
    }

    // Prefer batch interface supporting acceleration if available
    bool batch_ok = false;
    if (servo_interface->syncSetAllJointAnglesSpeedsAccels(angles_deg, speeds, accels)) {
        batch_ok = true; // full accel-capable batch path
    }
    if (!batch_ok) {
        // Fallback: per-joint
        for (int i = 0; i < NUM_LEGS; ++i) {
            for (int j = 0; j < 3; ++j) {
                servo_interface->setJointAngleSpeedAccel(i, j, angles_deg[i][j], speeds[i][j], accels[i][j]);
            }
        }
    }

    // If finished, finalize state and compute body position
    if (!body_pose_ctrl->isInitialStandingPoseActive()) {
        body_position = body_pose_ctrl->calculateBodyPosition(legs);
        system_state = SYSTEM_READY;
        startup_in_progress = false;
        shutdown_in_progress = false;
    }
    return true;
}

#ifdef TESTING_ENABLED
// ====================================================================
// TESTING: Coxa telemetry instrumentation
// ====================================================================
void LocomotionSystem::recordCoxaTelemetrySample() {
    if (!telemetry_enabled_)
        return;
    CoxaTelemetrySample sample{};
    sample.time = telemetry_time_accumulator_;
    // Acquire instantaneous data + stride vectors + servo commands
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = legs[i].getJointAngles();
        double global = a.coxa;                        // absolute
        double local = global - BASE_THETA_OFFSETS[i]; // local (offset removed)
        sample.global_angle[i] = global;
        sample.local_angle[i] = local;
        sample.phase[i] = legs[i].getStepPhase();
        // stride baseline logic: set when entering stance for first time in cycle; keep across swing to observe full leg cycle displacement
        if (sample.phase[i] == STANCE_PHASE && !stride_start_valid_[i]) {
            stride_start_tip_[i] = legs[i].getCurrentTipPositionGlobal();
            stride_start_valid_[i] = true;
        } else if (sample.phase[i] == SWING_PHASE && stride_start_valid_[i]) {
            // Reset baseline at swing start so next STANCE establishes a fresh forward reference.
            // This avoids accumulating backwards drift across multiple cycles which produced
            // negative average dx despite forward command velocity in tests.
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
        sample.servo_command_coxa[i] = last_servo_cmd_deg_[i][0] * DEGREES_TO_RADIANS_FACTOR; // store radians
    }
    sample.body_vel_x = commanded_linear_velocity_x_;
    sample.body_vel_y = commanded_linear_velocity_y_;
    sample.body_ang_vel = commanded_angular_velocity_;
    // Derive velocities / accelerations via finite differences (simple, adequate for test sampling rate)
    if (prev_valid_) {
        double dt = params.time_delta > 1e-9 ? params.time_delta : 1e-3; // guard
        for (int i = 0; i < NUM_LEGS; ++i) {
            double v_global = (sample.global_angle[i] - prev_coxa_angle_[i]) / dt;
            double v_local = (sample.local_angle[i] - (prev_coxa_angle_[i] - BASE_THETA_OFFSETS[i])) / dt;
            double a_global = (v_global - prev_coxa_velocity_[i]) / dt;
            double a_local = (v_local - (prev_coxa_velocity_[i])) / dt; // same prev velocity source since offset constant
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
    // Update previous
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev_coxa_velocity_[i] = sample.global_velocity[i];
        prev_coxa_angle_[i] = sample.global_angle[i];
    }
    prev_valid_ = true;

    if (telemetry_.size() >= kMaxTelemetrySamples_) {
        // Simple ring buffer behavior: drop oldest by shifting (cost acceptable for test-sized buffer)
        telemetry_.erase(telemetry_.begin());
    }
    telemetry_.push_back(sample);
}
#endif
