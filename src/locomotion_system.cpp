/**
 * @file locomotion_system.cpp
 * @brief Implementation of the Locomotion Control System
 * @author BlightHunter Team
 * @version 1.0
 * @date 2024
 *
 * Implements control based on:
 * - Inverse kinematics using Denavit-Hartenberg parameters
 * - Jacobians for velocity control
 * - Gait planner with multiple gaits
 * - Orientation and stability control
 * - Principles of OpenSHC (Open Source Humanoid Control)
 */

#include "locomotion_system.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "body_pose_config_factory.h"
#include "walk_controller.h"
#include "workspace_validator.h" // Add unified validator
#include <algorithm>
#include <cmath>
#include <vector>
#ifndef ARDUINO
#include <chrono>
#endif

// Constructor
LocomotionSystem::LocomotionSystem(const Parameters &params)
    : params(params), imu_interface(nullptr), fsr_interface(nullptr), servo_interface(nullptr),
      body_position(0.0f, 0.0f, params.robot_height), body_orientation(0.0f, 0.0f, 0.0f),
      current_gait(TRIPOD_GAIT), gait_phase(0.0f), step_height(30.0f), step_length(50.0f),
      stance_duration(WORKSPACE_SCALING_FACTOR), swing_duration(WORKSPACE_SCALING_FACTOR), cycle_frequency(ANGULAR_ACCELERATION_FACTOR), // OpenSHC-compatible tripod timing: 50% stance, 50% swing
      system_enabled(false), last_update_time(0), dt(0.02f),
      velocity_controller(nullptr), last_error(NO_ERROR),
      model(params), body_pose_ctrl(nullptr), walk_ctrl(nullptr), admittance_ctrl(nullptr),
      legs{Leg(0, params), Leg(1, params), Leg(2, params), Leg(3, params), Leg(4, params), Leg(5, params)} {

    // Initialize sensor log timestamp to avoid static local in updateSensorsParallel
    last_sensor_log_time = 0;
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
    walk_ctrl = new WalkController(model, legs);
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
    Pose default_stance(Point3D(0, 0, -params.robot_height), Eigen::Vector3d(0, 0, 0));
    for (int i = 0; i < NUM_LEGS; i++) {
        legs[i].initialize(model, default_stance);
    }

    system_enabled = true;
    last_update_time = millis();

    return true;
}

// System status check
bool LocomotionSystem::isSystemEnabled() const {
    return system_enabled;
}

// System calibration
bool LocomotionSystem::calibrateSystem() {
    if (!system_enabled)
        return false;

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
    return model.inverseKinematics(leg, p_target);
}

// Forward kinematics using DH transforms
Point3D LocomotionSystem::calculateForwardKinematics(int leg_index, const JointAngles &angles) {
    return model.forwardKinematics(leg_index, angles);
}

// DH transformation calculation
Eigen::Matrix4d LocomotionSystem::calculateDHTransform(double a, double alpha, double d, double theta) {
    double alpha_rad = math_utils::degreesToRadians(alpha);
    double theta_rad = math_utils::degreesToRadians(theta);
    return math_utils::dhTransform(a, alpha_rad, d, theta_rad);
}

// Complete leg transform
Eigen::Matrix4d LocomotionSystem::calculateLegTransform(int leg_index,
                                                        const JointAngles &q) {
    return model.legTransform(leg_index, q);
}

bool LocomotionSystem::isTargetReachable(int leg_index, const Point3D &target) {
    // Delegate to WorkspaceValidator for consistency
    // Note: This creates a temporary validator. For better performance,
    // consider using a shared validator instance in production code.
    WorkspaceValidator temp_validator(model);
    return temp_validator.isReachable(leg_index, target);
}

Point3D LocomotionSystem::constrainToWorkspace(int leg_index, const Point3D &target) {
    // Delegate to WorkspaceValidator for consistency
    // Note: This creates a temporary validator. For better performance,
    // consider using a shared validator instance in production code.
    WorkspaceValidator temp_validator(model);
    Point3D dummy_positions[6]; // Empty positions for basic geometric constraint
    for (int i = 0; i < 6; i++) {
        dummy_positions[i] = Point3D(0, 0, 0);
    }
    return temp_validator.constrainToValidWorkspace(leg_index, target, dummy_positions);
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
            double proximity = std::clamp<double>((half_range - distance_from_center) / half_range, 0.0, 1.0);
            min_proximity = std::min(min_proximity, proximity);
        }
    }

    return min_proximity;
}

/* Transform world point to body frame = Rᵀ·(p - p0) */
Point3D LocomotionSystem::transformWorldToBody(const Point3D &p_world) const {
    // Vector relative to the body center
    Point3D rel(p_world.x - body_position[0],
                p_world.y - body_position[1],
                p_world.z - body_position[2]);

    // Rotate with negative angles (inverse)
    Eigen::Vector3d neg_rpy(math_utils::degreesToRadians(-body_orientation[0]),
                            math_utils::degreesToRadians(-body_orientation[1]),
                            math_utils::degreesToRadians(-body_orientation[2]));
    return math_utils::rotatePoint(rel, neg_rpy);
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
    clamped_angles.coxa = std::clamp(q.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
    clamped_angles.femur = std::clamp(q.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    clamped_angles.tibia = std::clamp(q.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    // Check if clamping was needed
    bool was_clamped = (clamped_angles.coxa != q.coxa) ||
                       (clamped_angles.femur != q.femur) ||
                       (clamped_angles.tibia != q.tibia);

    if (was_clamped) {
        // Debug output for joint limit clamping
        std::cout << "[setLegJointAngles] Leg " << leg << " angles clamped:" << std::endl;
        std::cout << "  Original: coxa=" << q.coxa << " femur=" << q.femur << " tibia=" << q.tibia << std::endl;
        std::cout << "  Clamped:  coxa=" << clamped_angles.coxa << " femur=" << clamped_angles.femur << " tibia=" << clamped_angles.tibia << std::endl;
        std::cout << "  Limits: coxa=[" << params.coxa_angle_limits[0] << "," << params.coxa_angle_limits[1] << "]";
        std::cout << " femur=[" << params.femur_angle_limits[0] << "," << params.femur_angle_limits[1] << "]";
        std::cout << " tibia=[" << params.tibia_angle_limits[0] << "," << params.tibia_angle_limits[1] << "]" << std::endl;
    }

    // Update both joint angles and leg positions in a single atomic operation
    legs[leg].setJointAngles(clamped_angles);                            // Update leg object
    legs[leg].updateForwardKinematics(model);                            // Update leg position based on new angles

    // Use velocity controller to get appropriate servo speeds
    double coxa_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 0) : params.default_servo_speed;
    double femur_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 1) : params.default_servo_speed;
    double tibia_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 2) : params.default_servo_speed;

    // Apply sign inversion/preservation per servo using params.angle_sign_* (adjust left/right or above/below servo orientation)
    double servo_coxa = clamped_angles.coxa * params.angle_sign_coxa;
    double servo_femur = clamped_angles.femur * params.angle_sign_femur;
    double servo_tibia = clamped_angles.tibia * params.angle_sign_tibia;
    servo_interface->setJointAngleAndSpeed(leg, 0, servo_coxa, coxa_speed);
    servo_interface->setJointAngleAndSpeed(leg, 1, servo_femur, femur_speed);
    servo_interface->setJointAngleAndSpeed(leg, 2, servo_tibia, tibia_speed);
    return true;
}

// Gait planner
bool LocomotionSystem::setGaitType(GaitType gait) {
    if (!walk_ctrl)
        return false;

    // ✅ UPDATED: Delegate gait type setting to WalkController
    bool success = walk_ctrl->setGaitType(gait);

    if (success) {
        current_gait = gait;
        gait_phase = 0.0f;

        // ✅ UPDATED: Get timing parameters from WalkController
        walk_ctrl->getGaitTimingParameters(gait, stance_duration, swing_duration, cycle_frequency);
    }

    return success;
}

// Advanced gait methods
void LocomotionSystem::updateMetachronalPattern() {
    // ✅ UPDATED: Delegate to WalkController
    if (walk_ctrl) {
        walk_ctrl->updateMetachronalPattern();
    }
}

void LocomotionSystem::updateAdaptivePattern() {
    // ✅ UPDATED: Delegate to WalkController
    if (walk_ctrl) {
        walk_ctrl->updateAdaptivePattern();
    }
}

bool LocomotionSystem::shouldAdaptGaitPattern() {
    // ✅ UPDATED: Delegate to WalkController
    if (walk_ctrl) {
        return walk_ctrl->shouldAdaptGaitPattern();
    }
    return false;
}

void LocomotionSystem::calculateAdaptivePhaseOffsets() {
    // ✅ UPDATED: Delegate to WalkController
    if (walk_ctrl) {
        walk_ctrl->calculateAdaptivePhaseOffsets();
    }
}

// Gait sequence planning - ARCHITECTURE: Use WalkController with LegStepper
bool LocomotionSystem::planGaitSequence(double vx, double vy, double omega) {
    if (!walk_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    // Update velocity controller with current velocity commands
    if (velocity_controller) {
        velocity_controller->updateServoSpeeds(vx, vy, omega, current_gait);
    }

    // ARCHITECTURE: Use WalkController with LegStepper for gait planning
    bool success = walk_ctrl->planGaitSequence(vx, vy, omega);

    if (success) {
        // Store velocity commands for gait execution
        // These will be used in the update() method
        step_length = sqrt(vx * vx + vy * vy) / cycle_frequency;

        // Ensure step length is within reasonable bounds
        step_length = std::max(10.0, std::min(100.0, static_cast<double>(step_length)));

        return true;
    } else {
        last_error = KINEMATICS_ERROR;
        return false;
    }
}

// Gait phase update
void LocomotionSystem::updateGaitPhase() {
    if (walk_ctrl) {
        // Use walk controller's phase update which handles frequency correctly
        walk_ctrl->updateGaitPhase(dt);
        // Get the updated phase from walk controller to avoid duplication
        gait_phase = walk_ctrl->getGaitPhase();
    }
}

// Foot trajectory calculation
Point3D LocomotionSystem::calculateFootTrajectory(int leg_index, double phase) {
    if (!walk_ctrl)
        return Point3D();
    // Crea un arreglo temporal de LegState y haz el mapeo desde StepPhase si es necesario
    LegState adv_leg_states[NUM_LEGS];
    double leg_phase_offsets[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        adv_leg_states[i] = LEG_WALKING; // O mapea según lógica si tienes más estados
        leg_phase_offsets[i] = legs[i].getPhaseOffset();
    }
    return walk_ctrl->footTrajectory(leg_index, phase, step_height, step_length,
                                     stance_duration, swing_duration, params.robot_height,
                                     leg_phase_offsets, adv_leg_states, fsr_interface, imu_interface);
}

// Forward locomotion control
bool LocomotionSystem::walkForward(double velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(velocity, 0.0f, 0.0f);
}

// Backward locomotion control
bool LocomotionSystem::walkBackward(double velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(-velocity, 0.0f, 0.0f);
}

// In-place turning control
bool LocomotionSystem::turnInPlace(double angular_velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(0.0f, 0.0f, angular_velocity);
}

// Sideways locomotion control
bool LocomotionSystem::walkSideways(double velocity, bool right_direction) {
    if (!system_enabled)
        return false;

    double lateral_velocity = right_direction ? velocity : -velocity;
    return planGaitSequence(0.0f, lateral_velocity, 0.0f);
}

// Advance for "duration"seconds
bool LocomotionSystem::walkForward(double velocity, double duration) {
    if (!system_enabled)
        return false;

    // Plan gait to move forward along X (velocity m/s)
    planGaitSequence(velocity, 0.0f, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    // Blocking loop: run update() until duration elapsed
    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1) calls
        // delay(1);
    }

    stopMovement();
    return true;
}

// Move backward for "duration"seconds
bool LocomotionSystem::walkBackward(double velocity, double duration) {
    if (!system_enabled)
        return false;

    // Plan gait to move in -X (backwards)
    planGaitSequence(-velocity, 0.0f, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Turn in place for "duration"seconds
bool LocomotionSystem::turnInPlace(double angular_velocity, double duration) {
    if (!system_enabled)
        return false;

    // Plan gait with only angular component
    planGaitSequence(0.0f, 0.0f, angular_velocity);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Walk sideways (right/left) for "duration"seconds
bool LocomotionSystem::walkSideways(double velocity, double duration, bool right_direction) {
    if (!system_enabled)
        return false;

    double lateral_velocity = right_direction ? velocity : -velocity;
    planGaitSequence(0.0f, lateral_velocity, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Stop movement while keeping current pose
bool LocomotionSystem::stopMovement() {
    if (!system_enabled)
        return false;

    // Plan gait with zero velocities to stop the robot
    planGaitSequence(0.0f, 0.0f, 0.0f);
    // Call update() once to resend no movement angles
    update();
    return true;
}

// Orientation control
bool LocomotionSystem::maintainOrientation(const Eigen::Vector3d &target_rpy) {
    if (!system_enabled || !admittance_ctrl)
        return false;
    Point3D target(target_rpy.x(), target_rpy.y(), target_rpy.z());
    Point3D current(body_orientation.x(), body_orientation.y(), body_orientation.z());
    bool result = admittance_ctrl->maintainOrientation(target, current, dt);
    body_orientation = Eigen::Vector3d(current.x, current.y, current.z);

    // Reproject standing feet to maintain contact during orientation changes
    reprojectStandingFeet();

    return result;
}

void LocomotionSystem::reprojectStandingFeet() {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        if (legs[leg].getStepPhase() != STANCE_PHASE)
            continue;

        // Current foot position world -> body
        Point3D tip_body = transformWorldToBody(legs[leg].getTipPosition());

        // IK for the new body orientation
        JointAngles q_new = calculateInverseKinematics(leg, tip_body);

        // Apply angles to servos and RAM - this will update both joint_angles and leg_positions internally
        setLegJointAngles(leg, q_new);
    }
}

// Automatic tilt correction
bool LocomotionSystem::correctBodyTilt() {
    Eigen::Vector3d target_orientation(0.0f, 0.0f, body_orientation[2]);
    return maintainOrientation(target_orientation);
}

// Calculate orientation error
Eigen::Vector3d LocomotionSystem::calculateOrientationError() {
    if (!admittance_ctrl)
        return Eigen::Vector3d::Zero();
    Point3D current(body_orientation.x(), body_orientation.y(), body_orientation.z());
    Point3D error = admittance_ctrl->orientationError(current);
    return Eigen::Vector3d(error.x, error.y, error.z);
}

// Check stability margin
bool LocomotionSystem::checkStabilityMargin() {
    if (!system_enabled || !admittance_ctrl)
        return false;

    // Create temporary arrays for compatibility with admittance controller
    Point3D temp_leg_positions[NUM_LEGS];
    StepPhase temp_leg_states[NUM_LEGS];

    for (int i = 0; i < NUM_LEGS; i++) {
        temp_leg_positions[i] = legs[i].getTipPosition();
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
                cop[0] += legs[i].getTipPosition().x * fsr_data.pressure;
                cop[1] += legs[i].getTipPosition().y * fsr_data.pressure;
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
            stance_positions[stance_count] = legs[i].getTipPosition();
            support_polygon.push_back(legs[i].getTipPosition());
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

    return std::clamp<double>(stability_index, 0.0, 1.0);
}

// Check static stability
bool LocomotionSystem::isStaticallyStable() {
    return calculateStabilityIndex() > 0.2f; // Minimum stability threshold
}

// Body pose control
bool LocomotionSystem::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!system_enabled || !body_pose_ctrl)
        return false;

    // Calculate new pose using pose controller
    if (!body_pose_ctrl->setBodyPose(position, orientation, legs)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    body_position = position;
    body_orientation = orientation;

    // Reproject standing feet to maintain contact during pose changes
    reprojectStandingFeet();

    return true;
}

// Set standing pose using BodyPoseController with LegPoser
bool LocomotionSystem::setStandingPose() {
    if (!body_pose_ctrl) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    // ARCHITECTURE: Use BodyPoseController with LegPoser for pose control
    bool success = body_pose_ctrl->setStandingPose(legs);

    if (success) {
        // Apply the calculated joint angles to servos
        for (int i = 0; i < NUM_LEGS; i++) {
            JointAngles angles = legs[i].getJointAngles();
            if (!setLegJointAngles(i, angles)) {
                last_error = KINEMATICS_ERROR;
                return false;
            }
        }

        // Reset gait state for standing pose
        gait_phase = 0.0f;
        current_gait = NO_GAIT;

        return true;
    } else {
        last_error = KINEMATICS_ERROR;
        return false;
    }
}

// Smooth trajectory configuration methods (OpenSHC-style movement)
bool LocomotionSystem::configureSmoothMovement(bool enable, double interpolation_speed, uint8_t max_steps) {
    if (!body_pose_ctrl)
        return false;

    // Configure the pose controller's smooth trajectory settings
    body_pose_ctrl->configureSmoothTrajectory(enable, interpolation_speed, max_steps);
    return true;
}

// Smooth movement methods for OpenSHC-style pose control
bool LocomotionSystem::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!system_enabled || !body_pose_ctrl)
        return false;

    // Use the smooth trajectory method explicitly
    if (!body_pose_ctrl->setBodyPoseSmooth(position, orientation, legs, servo_interface)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    body_position = position;
    body_orientation = orientation;

    // Reproject standing feet to maintain contact during pose changes
    reprojectStandingFeet();

    return true;
}

bool LocomotionSystem::setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!system_enabled || !body_pose_ctrl)
        return false;

    // Use the immediate (non-smooth) method for compatibility
    if (!body_pose_ctrl->setBodyPoseImmediate(position, orientation, legs)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    body_position = position;
    body_orientation = orientation;

    // Reproject standing feet to maintain contact during pose changes
    reprojectStandingFeet();

    return true;
}

bool LocomotionSystem::isSmoothMovementInProgress() const {
    if (!body_pose_ctrl)
        return false;
    return body_pose_ctrl->isTrajectoryInProgress();
}

void LocomotionSystem::resetSmoothMovement() {
    if (body_pose_ctrl)
        body_pose_ctrl->resetTrajectory();
}

// Main system update
bool LocomotionSystem::update() {
    if (!system_enabled)
        return false;

    unsigned long current_time = millis();
    dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
    last_update_time = current_time;

    // Ensure minimum timestep using control frequency when running faster than real time
    double min_dt = 1.0f / params.control_frequency;
    if (dt <= 0.0f || dt < min_dt)
        dt = min_dt;

    // Limit dt to avoid large jumps
    if (dt > 0.1f)
        dt = 0.1f;

    // PARALLEL SENSOR READING IMPLEMENTATION
    // Update both FSR and IMU sensors simultaneously for optimal performance
    bool sensors_updated = updateSensorsParallel();
    if (!sensors_updated) {
        last_error = SENSOR_ERROR;
        return false;
    }

    // Update leg contact states from FSR sensors
    updateLegStates();

    // Update gait phase
    updateGaitPhase();

    // ✅ UPDATED: Use WalkController for gait pattern management
    if (walk_ctrl) {
        // Update WalkController which manages LegSteppers
        walk_ctrl->updateGaitPhase(dt);

        // ✅ UPDATED: Update advanced gait patterns through WalkController
        if (current_gait == METACHRONAL_GAIT) {
            walk_ctrl->updateMetachronalPattern();
        } else if (current_gait == ADAPTIVE_GAIT) {
            walk_ctrl->updateAdaptivePattern();
        }

        // Get current velocity commands for gait pattern
        Point3D linear_velocity(0, 0, 0);
        double angular_velocity = 0.0;

        // If we have planned gait sequence, use those velocities
        if (current_gait != NO_GAIT) {
            // Calculate velocities based on gait phase and step parameters
            double step_frequency = cycle_frequency;
            double step_length = this->step_length;

            // For forward walking, use positive X velocity
            linear_velocity.x = step_length * step_frequency;
        }

        // Update WalkController with current velocities
        walk_ctrl->updateWalk(linear_velocity, angular_velocity);

        // Apply gait trajectories to legs using LegSteppers
        for (int i = 0; i < NUM_LEGS; i++) {
            auto stepper = walk_ctrl->getLegStepper(i);
            if (stepper) {
                // ✅ CORRECTED: Update LegStepper state with proper parameters
                StepCycle step = walk_ctrl->getStepCycle();
                stepper->updateStepState(step);
                stepper->updatePhase(step);

                // Calculate leg phase based on gait phase and offset
                double leg_phase = legs[i].calculateLegPhase(gait_phase);

                // ✅ CORRECTED: Update LegStepper with current phase, step length, and time delta
                double time_delta = 1.0 / params.control_frequency;
                stepper->updateWithPhase(leg_phase, step_length, time_delta);

                // Get calculated trajectory from LegStepper
                Point3D target_position = stepper->getCurrentTipPose();

                // Calculate joint angles using inverse kinematics
                JointAngles target_angles = calculateInverseKinematics(i, target_position);

                // Apply to leg object
                legs[i].setJointAngles(target_angles);
                legs[i].setTipPosition(target_position);

                // Apply to servos through LocomotionSystem
                if (!setLegJointAngles(i, target_angles)) {
                    last_error = KINEMATICS_ERROR;
                }
            }
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

// System self test
bool LocomotionSystem::performSelfTest() {
#if defined(ENABLE_LOG) && defined(ARDUINO)
    Serial.println("=== Starting self test ===");

    // IMU test
    if (!imu_interface || !imu_interface->isConnected()) {
        Serial.println("X Error: IMU not connected");
        return false;
    }

    IMUData imu_test = imu_interface->readIMU();
    if (!imu_test.is_valid) {
        Serial.println("X Error: invalid IMU data");
        return false;
    }
    Serial.println("OK IMU working correctly");

    // FSR test
    for (int i = 0; i < NUM_LEGS; i++) {
        FSRData fsr_test = fsr_interface->readFSR(i);
        if (fsr_test.pressure < 0) {
            Serial.print("X Error: FSR leg ");
            Serial.print(i);
            Serial.println(" malfunction");
            return false;
        }
    }
    Serial.println("OK All FSRs working");

    // Servo test
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < DOF_PER_LEG; j++) {
            double current_angle = servo_interface->getJointAngle(i, j);
            if (current_angle < -180 || current_angle > 180) {
                Serial.print("X Error: Servo leg ");
                Serial.print(i);
                Serial.print(" joint ");
                Serial.println(j);
                return false;
            }
        }
    }
    Serial.println("OK All servos working");

    // Kinematics test
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D test_point(100, 0, -100);
        JointAngles angles = calculateInverseKinematics(i, test_point);
        Point3D calculated_point = calculateForwardKinematics(i, angles);

        double error = math_utils::distance3D(test_point, calculated_point);
        if (error > 5.0f) { // Error greater than 5mm
            Serial.print("X Error: leg kinematics ");
            Serial.print(i);
            Serial.print(" error=");
            Serial.print(error);
            Serial.println("mm");
            return false;
        }
    }
    Serial.println("OK Kinematics working correctly");

    Serial.println("=== Self test completed successfully ===");
    return true;
#else
    // Self test not available without Arduino environment
    return false;
#endif
}

void LocomotionSystem::updateLegStates() {
    // Si no se usa FSR/contacto, alternar solo por fase de marcha
    if (!params.use_fsr_contact) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (legs[i].shouldBeInStance(gait_phase, stance_duration)) {
                legs[i].setStepPhase(STANCE_PHASE);
            } else {
                legs[i].setStepPhase(SWING_PHASE);
            }
        }
        return;
    }

    if (!fsr_interface)
        return;

    // Iterate over each leg and filter FSR contact
    for (int i = 0; i < NUM_LEGS; ++i) {
        FSRData fsr = fsr_interface->readFSR(i);

        // Update FSR contact history using Leg class methods
        legs[i].updateFSRHistory(fsr.in_contact, fsr.pressure);

        // Get filtered contact state using Leg class methods
        bool filtered_contact = legs[i].getFilteredContactState(0.7f, 0.3f);

        StepPhase current_state = legs[i].getStepPhase();

        // State transition logic with hysteresis
        if (current_state == SWING_PHASE && filtered_contact) {
            legs[i].setStepPhase(STANCE_PHASE);
        } else if (current_state == STANCE_PHASE && !filtered_contact) {
            legs[i].setStepPhase(SWING_PHASE);
        }
        // Otherwise maintain current state

        // Additional validation: Don't allow contact during planned swing phase
        // Only validate during active movement (non-zero gait phase progression)
        if (cycle_frequency > 0.0f) {
            // Check if leg should be in swing based on gait phase
            bool should_be_swinging = legs[i].shouldBeInSwing(gait_phase, stance_duration);

            if (should_be_swinging && legs[i].getStepPhase() == STANCE_PHASE) {
                // Only override if pressure is very low (potential sensor error)
                if (fsr.pressure < 10.0f) { // Low pressure threshold
                    legs[i].setStepPhase(SWING_PHASE);
                }
            }
        }
    }
}

void LocomotionSystem::updateStepParameters() {
    // Calculate leg reach and robot dimensions
    double leg_reach = calculateLegReach(); // Use standardized function
    double robot_height = params.robot_height;

    // Adjust step parameters depending on gait type using parametrizable factors
    switch (current_gait) {
    case NO_GAIT:
        // No gait - robot is stationary
        step_length = 0.0;
        step_height = 0.0;
        break;
    case TRIPOD_GAIT:
        // Longer steps for speed
        step_length = leg_reach * params.gait_factors.tripod_length_factor;
        step_height = robot_height * params.gait_factors.tripod_height_factor;
        break;

    case WAVE_GAIT:
        // Shorter steps for stability
        step_length = leg_reach * params.gait_factors.wave_length_factor;
        step_height = robot_height * params.gait_factors.wave_height_factor;
        break;

    case RIPPLE_GAIT:
        // Medium steps for balance
        step_length = leg_reach * params.gait_factors.ripple_length_factor;
        step_height = robot_height * params.gait_factors.ripple_height_factor;
        break;

    case METACHRONAL_GAIT:
        // Adaptive steps
        step_length = leg_reach * params.gait_factors.metachronal_length_factor;
        step_height = robot_height * params.gait_factors.metachronal_height_factor;
        break;

    case ADAPTIVE_GAIT:
        // They will be adjusted dynamically
        step_length = leg_reach * params.gait_factors.adaptive_length_factor;
        step_height = robot_height * params.gait_factors.adaptive_height_factor;
        break;
    }

    // Calculate dynamic limits based on robot dimensions
    double min_step_length = leg_reach * params.gait_factors.min_length_factor;
    double max_step_length = leg_reach * params.gait_factors.max_length_factor;
    double min_step_height = robot_height * params.gait_factors.min_height_factor;
    double max_step_height = robot_height * params.gait_factors.max_height_factor;

    // Verify that parameters are within dynamic limits
    step_length = constrainAngle(step_length, min_step_length, max_step_length);
    step_height = constrainAngle(step_height, min_step_height, max_step_height);
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

void LocomotionSystem::adaptGaitToTerrain() {
    // Analyze FSR data to adapt gait
    double avg_pressure = 0;
    int contact_count = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        FSRData fsr_data = fsr_interface->readFSR(i);
        if (fsr_data.in_contact) {
            avg_pressure += fsr_data.pressure;
            contact_count++;
        }
    }

    if (contact_count > 0) {
        avg_pressure /= contact_count;

        // If average pressure is high use a more stable gait
        if (avg_pressure > params.fsr_max_pressure * 0.8f) {
            current_gait = WAVE_GAIT;
        } else {
            current_gait = TRIPOD_GAIT;
        }
    }
}

bool LocomotionSystem::setLegPosition(int leg_index, const Point3D &position) {
    if (!system_enabled || leg_index < 0 || leg_index >= NUM_LEGS || !body_pose_ctrl)
        return false;

    if (!body_pose_ctrl->setLegPosition(leg_index, position, legs)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    return true;
}

bool LocomotionSystem::setStepParameters(double height, double length) {
    if (height < 15.0f || height > 50.0f || length < 20.0f || length > 80.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    step_height = height;
    step_length = length;
    return true;
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

bool LocomotionSystem::setControlFrequency(double frequency) {
    if (frequency < 10.0f || frequency > 200.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params.control_frequency = frequency;
    return true;
}

double LocomotionSystem::calculateLegReach() const {
    return params.coxa_length + params.femur_length + params.tibia_length;
}

void LocomotionSystem::adjustStepParameters() {
    // Enhanced step parameter adjustment with absolute positioning support
    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return;

    double total_tilt;
    double stability_factor = 1.0f;

    // Use enhanced data for more precise adjustment
    if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
        // More precise tilt calculation using absolute orientation
        total_tilt = sqrt(
            imu_data.absolute_data.absolute_roll * imu_data.absolute_data.absolute_roll +
            imu_data.absolute_data.absolute_pitch * imu_data.absolute_data.absolute_pitch);

        // Consider dynamic stability using linear acceleration
        if (imu_data.absolute_data.linear_acceleration_valid) {
            double dynamic_instability = sqrt(
                imu_data.absolute_data.linear_accel_x * imu_data.absolute_data.linear_accel_x +
                imu_data.absolute_data.linear_accel_y * imu_data.absolute_data.linear_accel_y);

            // Reduce step parameters during dynamic instability
            if (dynamic_instability > 2.0f) {
                stability_factor =
                    std::clamp<double>(1.0 - (dynamic_instability - 2.0) / 5.0, 0.6, 1.0);
            }
        }

        // Enhanced slope-based adjustment
        if (total_tilt > 10.0f) {
            double slope_factor = std::clamp<double>(1.0 - (total_tilt - 10.0) / 30.0, 0.5, 1.0);
            step_height *= slope_factor * stability_factor;
            step_length *= slope_factor * stability_factor;
        }
    } else {
        // Fallback to basic IMU data
        total_tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);

        // Original logic for basic IMU
        if (total_tilt > 15.0f) {
            step_height *= 0.8f;
            step_length *= 0.7f;
        }
    }

    // Limit parameters
    step_height = constrainAngle(step_height, 15.0f, 50.0f);
    step_length = constrainAngle(step_length, 20.0f, 80.0f);
}

void LocomotionSystem::compensateForSlope() {
    if (!imu_interface)
        return;

    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return;

    double roll_compensation, pitch_compensation;

    // Enhanced slope compensation using absolute positioning data
    if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
        // Use absolute orientation for more precise compensation
        roll_compensation = -imu_data.absolute_data.absolute_roll * 0.6f; // Enhanced compensation
        pitch_compensation = -imu_data.absolute_data.absolute_pitch * 0.6f;

        // Additional quaternion-based compensation for complex terrain
        if (imu_data.absolute_data.quaternion_valid) {
            // Extract more sophisticated orientation information from quaternion
            double qw = imu_data.absolute_data.quaternion_w;
            double qx = imu_data.absolute_data.quaternion_x;
            double qy = imu_data.absolute_data.quaternion_y;
            double qz = imu_data.absolute_data.quaternion_z;

            // Calculate terrain-aligned compensation using quaternion
            double quat_roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RADIANS_TO_DEGREES_FACTOR;
            double quat_pitch = asin(2.0f * (qw * qy - qz * qx)) * RADIANS_TO_DEGREES_FACTOR;

            // Blend quaternion and Euler compensations for robustness
            roll_compensation = 0.7f * roll_compensation + 0.3f * (-quat_roll * 0.6f);
            pitch_compensation = 0.7f * pitch_compensation + 0.3f * (-quat_pitch * 0.6f);
        }

        // Dynamic adjustment based on linear acceleration
        if (imu_data.absolute_data.linear_acceleration_valid) {
            double lateral_accel = sqrt(
                imu_data.absolute_data.linear_accel_x * imu_data.absolute_data.linear_accel_x +
                imu_data.absolute_data.linear_accel_y * imu_data.absolute_data.linear_accel_y);

            // Reduce compensation during high lateral acceleration
            if (lateral_accel > 1.5f) {
                double dynamic_factor = std::clamp<double>(1.0 - (lateral_accel - 1.5) / 3.0, 0.4, 1.0);
                roll_compensation *= dynamic_factor;
                pitch_compensation *= dynamic_factor;
            }
        }
    } else {
        // Fallback to basic IMU compensation
        roll_compensation = -imu_data.roll * 0.5f; // Compensation factor
        pitch_compensation = -imu_data.pitch * 0.5f;
    }

    // Adjust body orientation
    body_orientation[0] += roll_compensation * dt;
    body_orientation[1] += pitch_compensation * dt;

    // Clamp compensation
    body_orientation[0] = constrainAngle(body_orientation[0], -15.0f, 15.0f);
    body_orientation[1] = constrainAngle(body_orientation[1], -15.0f, 15.0f);

    // Reproject standing feet after slope compensation changes body orientation
    reprojectStandingFeet();
}

double LocomotionSystem::getStepLength() const {
    // Base step length according to the current gait
    double base_step_length = step_length;

    // Adjustment factors based on robot parameters
    double leg_reach = calculateLegReach(); // Use standardized function
    double max_safe_step = leg_reach * params.gait_factors.max_length_factor;

    // Stability adjustment - reduce step if stability is low
    double stability_factor = 1.0f;
    if (system_enabled) {
        double stability_index = const_cast<LocomotionSystem *>(this)->calculateStabilityIndex();
        if (stability_index < 0.5f) {
            stability_factor = 0.7f + 0.3f * stability_index; // Reduce up to 70%
        }
    }

    // Enhanced slope adjustment using absolute positioning data
    double terrain_factor = 1.0f;
    if (imu_interface && imu_interface->isConnected()) {
        IMUData imu_data = imu_interface->readIMU();
        if (imu_data.is_valid) {
            double total_tilt;

            // Use enhanced orientation data for more precise terrain assessment
            if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
                total_tilt = sqrt(
                    imu_data.absolute_data.absolute_roll * imu_data.absolute_data.absolute_roll +
                    imu_data.absolute_data.absolute_pitch * imu_data.absolute_data.absolute_pitch);

                // Additional terrain complexity assessment using quaternion
                if (imu_data.absolute_data.quaternion_valid) {
                    // Calculate terrain complexity using quaternion derivatives
                    double qx = imu_data.absolute_data.quaternion_x;
                    double qy = imu_data.absolute_data.quaternion_y;
                    double qz = imu_data.absolute_data.quaternion_z;

                    // Assess terrain complexity based on quaternion non-uniformity
                    double complexity = abs(qx) + abs(qy) + abs(qz);
                    if (complexity > 0.4f) { // Complex terrain indicator
                        terrain_factor *= std::clamp<double>(1.0 - (complexity - 0.4) / 0.6, 0.6, 1.0);
                    }
                }

                // Dynamic motion consideration
                if (imu_data.absolute_data.linear_acceleration_valid) {
                    double motion_magnitude = sqrt(
                        imu_data.absolute_data.linear_accel_x * imu_data.absolute_data.linear_accel_x +
                        imu_data.absolute_data.linear_accel_y * imu_data.absolute_data.linear_accel_y +
                        imu_data.absolute_data.linear_accel_z * imu_data.absolute_data.linear_accel_z);

                    // Reduce step length during high dynamic motion
                    if (motion_magnitude > 2.5f) {
                        terrain_factor *= std::clamp<double>(1.0 - (motion_magnitude - 2.5) / 5.0, 0.5, 1.0);
                    }
                }

                // Enhanced slope calculation
                if (total_tilt > 8.0f) { // Lower threshold for absolute data
                    terrain_factor *= std::clamp<double>(1.0 - (total_tilt - 8.0) / 25.0, 0.5, 1.0);
                }
            } else {
                // Fallback to basic IMU data
                total_tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);

                if (total_tilt > 10.0f) {
                    terrain_factor = std::clamp<double>(1.0 - (total_tilt - 10.0) / 20.0, 0.6, 1.0);
                }
            }
        }
    }

    // Calculate final step length
    double calculated_step_length = base_step_length * stability_factor * terrain_factor;

    // Limit within safe ranges based on parameters
    double min_safe_step = leg_reach * params.gait_factors.min_length_factor;
    calculated_step_length =
        std::clamp<double>(calculated_step_length, min_safe_step, max_safe_step);

    return calculated_step_length;
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
                std::clamp<double>(1.0 - (total_tilt - 5.0) / 30.0, 0.2, 1.0);
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
                    std::clamp<double>(1.0 - (acceleration_magnitude - 1.0) / 4.0, 0.3, 1.0);
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
                    std::clamp<double>(1.0 - (rotational_deviation - 0.2) / 0.6, 0.4, 1.0);
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

    return std::clamp<double>(stability_index, 0.0, 1.0);
}

// Parallel sensor update implementation
bool LocomotionSystem::updateSensorsParallel() {
    if (!system_enabled)
        return false;

    bool fsr_updated = false;
    bool imu_updated = false;
#ifdef ARDUINO
    unsigned long start_time = micros();
#else
    auto start_time = std::chrono::high_resolution_clock::now();
#endif

    // Start parallel sensor updates
    // FSR: AdvancedAnalog DMA for simultaneous ADC reading
    if (fsr_interface) {
        fsr_updated = fsr_interface->update();
    }

    // IMU: Non-blocking sensor update (parallel with FSR)
    if (imu_interface && imu_interface->isConnected()) {
        imu_updated = imu_interface->update();
    }

    // Performance monitoring
#ifdef ARDUINO
    unsigned long update_time = micros() - start_time;
#else
    auto end_time = std::chrono::high_resolution_clock::now();
    unsigned long update_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
#endif

#if defined(ENABLE_LOG) && defined(ARDUINO)
    // Log every 5 seconds without static local variable
    if (millis() - last_sensor_log_time > 5000) {
        last_sensor_log_time = millis();
        // Log update timing for performance analysis
        Serial.print("Parallel sensor update time: ");
        Serial.print(update_time);
        Serial.print("µs, FSR: ");
        Serial.print(fsr_updated ? "OK" : "FAIL");
        Serial.print(", IMU: ");
        Serial.println(imu_updated ? "OK" : "FAIL");
    }
#else
    // Suppress unused variable warning when logging is disabled
    (void)update_time;
#endif

    // Validate both sensors updated successfully
    if (fsr_interface && !fsr_updated) {
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

// ===== PHASE OFFSET MANAGEMENT METHODS =====

void LocomotionSystem::setLegPhaseOffset(int leg_index, double offset) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        legs[leg_index].setPhaseOffset(offset);
    }
}

double LocomotionSystem::getLegPhaseOffset(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return legs[leg_index].getPhaseOffset();
    }
    return 0.0;
}

void LocomotionSystem::setAllLegPhaseOffsets(const double offsets[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].setPhaseOffset(offsets[i]);
    }
}

void LocomotionSystem::configureGaitPhaseOffsets(GaitType gait) {
    // ✅ UPDATED: Delegate to WalkController
    if (walk_ctrl) {
        walk_ctrl->configureGaitPhaseOffsets(gait);
    }
}


