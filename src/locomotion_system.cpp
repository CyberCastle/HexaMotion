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
#include "pose_config_factory.h"
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
      stance_duration(WORKSPACE_SCALING_FACTOR), swing_duration(WORKSPACE_SCALING_FACTOR), cycle_frequency(ANGULAR_ACCELERATION_FACTOR),
      system_enabled(false), last_update_time(0), dt(0.02f),
      velocity_controller(nullptr), last_error(NO_ERROR),
      model(params), pose_ctrl(nullptr), walk_ctrl(nullptr), admittance_ctrl(nullptr) {

    // Initialize leg states and phase offsets for tripod gait
    // Tripod gait requires legs in alternate groups to be half a
    // cycle out of phase. Legs 1, 3 and 5 step together while
    // legs 2, 4 and 6 are offset by 180 degrees.
    static const double tripod_phase_offsets[NUM_LEGS] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states[i] = STANCE_PHASE;
        leg_phase_offsets[i] = tripod_phase_offsets[i];
    }
    // Initialize FSR contact history buffer and index
    fsr_history_index = -1;
    for (int i = 0; i < NUM_LEGS; ++i)
        for (int j = 0; j < 3; ++j)
            fsr_contact_history[i][j] = 0.0f;
    // Initialize sensor log timestamp to avoid static local in updateSensorsParallel
    last_sensor_log_time = 0;
}

// Destructor
LocomotionSystem::~LocomotionSystem() {
    system_enabled = false;
    delete pose_ctrl;
    delete walk_ctrl;
    delete admittance_ctrl;
    delete velocity_controller;
}

// System initialization
bool LocomotionSystem::initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo, const PoseConfiguration &pose_config) {
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

    pose_ctrl = new PoseController(model, servo_interface, pose_config);
    walk_ctrl = new WalkController(model);
    admittance_ctrl = new AdmittanceController(model, imu_interface, fsr_interface);
    velocity_controller = new CartesianVelocityController(model);

    // Validate parameters
    if (!validateParameters()) {
        last_error = PARAMETER_ERROR;
        return false;
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

    // CHECK SERVO STATUS FLAGS BEFORE ATTEMPTING MOVEMENT
    // Verify that all servos in this leg are ready for movement
    for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
        if (servo_interface->hasBlockingStatusFlags(leg, joint)) {
            // Servo is blocked, cannot move this leg
            last_error = SERVO_BLOCKED_ERROR;
#if defined(ENABLE_LOG) && defined(ARDUINO)
            Serial.print("Servo blocked - leg ");
            Serial.print(leg);
            Serial.print(" joint ");
            Serial.println(joint);
#endif
            return false;
        }
    }

    bool within_limits = q.coxa >= params.coxa_angle_limits[0] &&
                         q.coxa <= params.coxa_angle_limits[1] &&
                         q.femur >= params.femur_angle_limits[0] &&
                         q.femur <= params.femur_angle_limits[1] &&
                         q.tibia >= params.tibia_angle_limits[0] &&
                         q.tibia <= params.tibia_angle_limits[1];

    JointAngles clamped;
    clamped.coxa = constrainAngle(q.coxa, params.coxa_angle_limits[0],
                                  params.coxa_angle_limits[1]);
    clamped.femur = constrainAngle(q.femur, params.femur_angle_limits[0],
                                   params.femur_angle_limits[1]);
    clamped.tibia = constrainAngle(q.tibia, params.tibia_angle_limits[0],
                                   params.tibia_angle_limits[1]);

    if (!within_limits)
        return false;

    joint_angles[leg] = clamped; // internal state

    // Use velocity controller to get appropriate servo speeds
    double coxa_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 0) : params.default_servo_speed;
    double femur_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 1) : params.default_servo_speed;
    double tibia_speed = velocity_controller ? velocity_controller->getServoSpeed(leg, 2) : params.default_servo_speed;

    // Apply sign inversion/preservation per servo using params.angle_sign_* (adjust left/right or above/below servo orientation)
    double servo_coxa = clamped.coxa * params.angle_sign_coxa;
    double servo_femur = clamped.femur * params.angle_sign_femur;
    double servo_tibia = clamped.tibia * params.angle_sign_tibia;
    servo_interface->setJointAngleAndSpeed(leg, 0, servo_coxa, coxa_speed);
    servo_interface->setJointAngleAndSpeed(leg, 1, servo_femur, femur_speed);
    servo_interface->setJointAngleAndSpeed(leg, 2, servo_tibia, tibia_speed);
    return true;
}

// Gait planner
bool LocomotionSystem::setGaitType(GaitType gait) {
    if (!walk_ctrl)
        return false;
    current_gait = gait;
    gait_phase = 0.0f;

    // Set phase offsets based on gait type
    switch (gait) {
    case WAVE_GAIT:
        // Wave: OpenSHC-compatible phase offsets
        // Based on offset_multiplier: [2,3,4,1,0,5] with base_offset=2, total_period=12
        leg_phase_offsets[0] = 2.0f / 6.0f; // AR: mult=2 -> 0.333
        leg_phase_offsets[1] = 3.0f / 6.0f; // BR: mult=3 -> 0.500
        leg_phase_offsets[2] = 4.0f / 6.0f; // CR: mult=4 -> 0.667
        leg_phase_offsets[3] = 1.0f / 6.0f; // CL: mult=1 -> 0.167
        leg_phase_offsets[4] = 0.0f / 6.0f; // BL: mult=0 -> 0.000
        leg_phase_offsets[5] = 5.0f / 6.0f; // AL: mult=5 -> 0.833
        break;
    case RIPPLE_GAIT:
        // Ripple: OpenSHC-compatible phase offsets
        // Based on offset_multiplier: [2,0,4,1,3,5] with base_offset=1, total_period=6
        leg_phase_offsets[0] = 2.0f / 6.0f; // AR: mult=2 -> 0.333
        leg_phase_offsets[1] = 0.0f / 6.0f; // BR: mult=0 -> 0.000
        leg_phase_offsets[2] = 4.0f / 6.0f; // CR: mult=4 -> 0.667
        leg_phase_offsets[3] = 1.0f / 6.0f; // CL: mult=1 -> 0.167
        leg_phase_offsets[4] = 3.0f / 6.0f; // BL: mult=3 -> 0.500
        leg_phase_offsets[5] = 5.0f / 6.0f; // AL: mult=5 -> 0.833
        break;
    case METACHRONAL_GAIT:
        // Metachronal: Smooth wave-like progression clockwise
        // Creates a flowing wave motion around the body perimeter
        // Sequence: AR → BR → CR → CL → BL → AL (clockwise progression)
        leg_phase_offsets[0] = 0.0f / 6.0f; // AR: 0.000 (starts first)
        leg_phase_offsets[1] = 1.0f / 6.0f; // BR: 0.167
        leg_phase_offsets[2] = 2.0f / 6.0f; // CR: 0.333
        leg_phase_offsets[3] = 3.0f / 6.0f; // CL: 0.500
        leg_phase_offsets[4] = 4.0f / 6.0f; // BL: 0.667
        leg_phase_offsets[5] = 5.0f / 6.0f; // AL: 0.833
        break;
    case ADAPTIVE_GAIT:
        // Adaptive: Dynamic pattern that changes based on conditions
        // Starts with a stable base pattern similar to ripple but with adaptive spacing
        leg_phase_offsets[0] = 1.0f / 8.0f; // AR: 0.125 (fine-tuned timing)
        leg_phase_offsets[1] = 0.0f / 8.0f; // BR: 0.000 (anchor leg)
        leg_phase_offsets[2] = 3.0f / 8.0f; // CR: 0.375
        leg_phase_offsets[3] = 6.0f / 8.0f; // CL: 0.750
        leg_phase_offsets[4] = 4.0f / 8.0f; // BL: 0.500
        leg_phase_offsets[5] = 7.0f / 8.0f; // AL: 0.875
        break;
    case TRIPOD_GAIT:
    default:
        // Tripod (default): two groups of 3 legs, 180° out of phase
        // Group A (L1, L3, L5): offset 0.0
        // Group B (L2, L4, L6): offset 0.5
        leg_phase_offsets[0] = 0.0f; // L1
        leg_phase_offsets[1] = 0.5f; // L2
        leg_phase_offsets[2] = 0.0f; // L3
        leg_phase_offsets[3] = 0.5f; // L4
        leg_phase_offsets[4] = 0.0f; // L5
        leg_phase_offsets[5] = 0.5f; // L6
        break;
    }

    return walk_ctrl->setGaitType(gait);
}

// Advanced gait methods
void LocomotionSystem::updateMetachronalPattern() {
    // Metachronal gait creates smooth wave-like motion
    // The wave can flow in either direction for forward/backward movement

    if (current_gait != METACHRONAL_GAIT)
        return;

    // Calculate wave direction based on movement command
    bool reverse_wave = false;
    if (walk_ctrl) {
        // Get current velocity commands to determine movement direction
        const auto &velocities = walk_ctrl->getCurrentVelocities();

        // Check if we're moving backward based on velocity commands
        // Consider backward movement if X velocity is negative (robot coordinate frame)
        // Also account for purely lateral or rotational movements
        double threshold = 0.001f; // Small threshold to avoid noise

        if (abs(velocities.linear_x) > threshold) {
            // Primary movement is in X direction
            reverse_wave = (velocities.linear_x < 0.0f);
        } else if (abs(velocities.angular_z) > threshold) {
            // Primarily rotational movement - use angular direction
            // Negative angular velocity (clockwise) can be considered "reverse"
            reverse_wave = (velocities.angular_z < 0.0f);
        } else {
            // For lateral movement or stationary, maintain forward wave pattern
            reverse_wave = false;
        }
    }

    if (reverse_wave) {
        // Reverse wave pattern: AL → BL → CL → CR → BR → AR
        leg_phase_offsets[0] = 5.0f / 6.0f; // AR: 0.833
        leg_phase_offsets[1] = 4.0f / 6.0f; // BR: 0.667
        leg_phase_offsets[2] = 3.0f / 6.0f; // CR: 0.500
        leg_phase_offsets[3] = 2.0f / 6.0f; // CL: 0.333
        leg_phase_offsets[4] = 1.0f / 6.0f; // BL: 0.167
        leg_phase_offsets[5] = 0.0f / 6.0f; // AL: 0.000
    } else {
        // Forward wave pattern: AR → BR → CR → CL → BL → AL
        leg_phase_offsets[0] = 0.0f / 6.0f; // AR: 0.000
        leg_phase_offsets[1] = 1.0f / 6.0f; // BR: 0.167
        leg_phase_offsets[2] = 2.0f / 6.0f; // CR: 0.333
        leg_phase_offsets[3] = 3.0f / 6.0f; // CL: 0.500
        leg_phase_offsets[4] = 4.0f / 6.0f; // BL: 0.667
        leg_phase_offsets[5] = 5.0f / 6.0f; // AL: 0.833
    }
}

void LocomotionSystem::updateAdaptivePattern() {
    if (current_gait != ADAPTIVE_GAIT)
        return;

    // Check if we need to adapt the gait pattern
    if (shouldAdaptGaitPattern()) {
        calculateAdaptivePhaseOffsets();
    }
}

bool LocomotionSystem::shouldAdaptGaitPattern() {
    // Preconditions
    if (!system_enabled || !fsr_interface || !imu_interface)
        return false;

    // Read all FSR data once
    std::array<FSRData, NUM_LEGS> fsr_readings;
    for (int i = 0; i < NUM_LEGS; ++i) {
        fsr_readings[i] = fsr_interface->readFSR(i);
    }

    // Compute average pressure for legs in contact
    double avg_pressure = 0.0f;
    int contact_count = 0;
    for (auto &d : fsr_readings) {
        if (d.in_contact) {
            avg_pressure += d.pressure;
            ++contact_count;
        }
    }
    if (contact_count == 0)
        return false;
    avg_pressure /= contact_count;

    // Compute variance
    double pressure_variance = 0.0f;
    for (auto &d : fsr_readings) {
        if (d.in_contact) {
            double diff = d.pressure - avg_pressure;
            pressure_variance += diff * diff;
        }
    }
    pressure_variance /= contact_count;

    // Read IMU data
    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return false;

    // Calculate tilt magnitude
    double tilt_magnitude = 0.0f;
    if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
        tilt_magnitude = std::hypot(
            imu_data.absolute_data.absolute_roll,
            imu_data.absolute_data.absolute_pitch);
        // Dynamic motion triggers adaptation
        if (imu_data.absolute_data.linear_acceleration_valid) {
            double dyn = std::sqrt(
                imu_data.absolute_data.linear_accel_x * imu_data.absolute_data.linear_accel_x +
                imu_data.absolute_data.linear_accel_y * imu_data.absolute_data.linear_accel_y +
                imu_data.absolute_data.linear_accel_z * imu_data.absolute_data.linear_accel_z);
            if (dyn > 3.0f)
                return true;
        }
    } else {
        tilt_magnitude = std::hypot(imu_data.roll, imu_data.pitch);
    }

    // Adapt if any condition met
    bool high_variance = pressure_variance > (params.fsr_max_pressure * 0.1f);
    bool significant_tilt = tilt_magnitude > 5.0f;
    bool high_pressure = avg_pressure > (params.fsr_max_pressure * 0.7f);
    return high_variance || significant_tilt || high_pressure;
}

void LocomotionSystem::calculateAdaptivePhaseOffsets() {
    if (!system_enabled || !fsr_interface || !imu_interface) {
        return;
    }

    // Get sensor data
    IMUData imu_data = imu_interface->readIMU();
    double stability_index = calculateStabilityIndex();

    // Base pattern (similar to ripple but more conservative)
    double base_offsets[NUM_LEGS] = {
        1.0f / 8.0f, // AR: 0.125
        0.0f / 8.0f, // BR: 0.000 (anchor)
        3.0f / 8.0f, // CR: 0.375
        6.0f / 8.0f, // CL: 0.750
        4.0f / 8.0f, // BL: 0.500
        7.0f / 8.0f  // AL: 0.875
    };

    // Adaptation factors based on conditions
    double tilt_magnitude = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);

    if (tilt_magnitude > 10.0f) {
        // On steep slopes, use more conservative tripod-like pattern
        double tripod_factor = (tilt_magnitude - 10.0f) / 20.0f; // 0-1 over 10-30 degrees
        tripod_factor = std::clamp<double>(tripod_factor, 0.0, 1.0);

        for (int i = 0; i < NUM_LEGS; i++) {
            double tripod_offset = (i % 2) * 0.5f;
            leg_phase_offsets[i] = base_offsets[i] * (1.0f - tripod_factor) +
                                   tripod_offset * tripod_factor;
        }
    } else if (stability_index < 0.3f) {
        // Low stability - move toward wave gait pattern
        double wave_offsets[NUM_LEGS] = {
            2.0f / 6.0f, // AR: 0.333
            3.0f / 6.0f, // BR: 0.500
            4.0f / 6.0f, // CR: 0.667
            1.0f / 6.0f, // CL: 0.167
            0.0f / 6.0f, // BL: mult=0 -> 0.000
            5.0f / 6.0f  // AL: mult=5 -> 0.833
        };

        double wave_factor = (0.3f - stability_index) / 0.3f; // 0-1 as stability decreases

        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = base_offsets[i] * (1.0f - wave_factor) +
                                   wave_offsets[i] * wave_factor;
        }
    } else {
        // Good conditions - use base adaptive pattern
        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = base_offsets[i];
        }
    }
}

// Gait sequence planning
bool LocomotionSystem::planGaitSequence(double vx, double vy, double omega) {
    if (!walk_ctrl)
        return false;

    // Update velocity controller with current velocity commands
    if (velocity_controller) {
        velocity_controller->updateServoSpeeds(vx, vy, omega, current_gait);
    }

    return walk_ctrl->planGaitSequence(vx, vy, omega);
}

// Gait phase update
void LocomotionSystem::updateGaitPhase() {
    if (walk_ctrl) {
        walk_ctrl->updateGaitPhase(dt);
        // Synchronize gait phase - use cycle_frequency to determine progression rate
        gait_phase += dt * cycle_frequency;
        if (gait_phase >= 1.0f)
            gait_phase -= 1.0f;
    }
}

// Foot trajectory calculation
Point3D LocomotionSystem::calculateFootTrajectory(int leg_index, double phase) {
    if (!walk_ctrl)
        return Point3D();
    return walk_ctrl->footTrajectory(leg_index, phase, step_height, step_length,
                                     stance_duration, swing_duration, params.robot_height,
                                     leg_phase_offsets, leg_states, fsr_interface, imu_interface);
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
        if (leg_states[leg] != STANCE_PHASE)
            continue;

        // Current foot position world -> body
        Point3D tip_body = transformWorldToBody(leg_positions[leg]);

        // IK for the new body orientation
        JointAngles q_new = calculateInverseKinematics(leg, tip_body);

        // Apply angles to servos and RAM
        setLegJointAngles(leg, q_new);

        // Update world position for consistency
        leg_positions[leg] = calculateForwardKinematics(leg, q_new);
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
    return admittance_ctrl->checkStability(leg_positions, leg_states);
}

// Compute center of pressure
Eigen::Vector2d LocomotionSystem::calculateCenterOfPressure() {
    Eigen::Vector2d cop(0.0f, 0.0f);
    double total_force = 0.0f;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_states[i] == STANCE_PHASE) {
            FSRData fsr_data = fsr_interface->readFSR(i);
            if (fsr_data.in_contact && fsr_data.pressure > 0) {
                cop[0] += leg_positions[i].x * fsr_data.pressure;
                cop[1] += leg_positions[i].y * fsr_data.pressure;
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
        if (leg_states[i] == STANCE_PHASE) {
            stance_positions[stance_count] = leg_positions[i];
            support_polygon.push_back(leg_positions[i]);
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
    if (!system_enabled || !pose_ctrl)
        return false;
    if (!pose_ctrl->setBodyPose(position, orientation, leg_positions, joint_angles)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }
    body_position = position;
    body_orientation = orientation;

    // Reproject standing feet to maintain contact during pose changes
    reprojectStandingFeet();

    return true;
}

// Set standing pose
bool LocomotionSystem::setStandingPose() {
    if (!pose_ctrl)
        return false;
    return pose_ctrl->setStandingPose(leg_positions, joint_angles);
}

// Smooth trajectory configuration methods (OpenSHC-style movement)
bool LocomotionSystem::configureSmoothMovement(bool enable, double interpolation_speed, uint8_t max_steps) {
    if (!pose_ctrl)
        return false;

    // Configure the pose controller's smooth trajectory settings
    pose_ctrl->configureSmoothTrajectory(enable, interpolation_speed, max_steps);
    return true;
}

// Smooth movement methods for OpenSHC-style pose control
bool LocomotionSystem::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    if (!system_enabled || !pose_ctrl)
        return false;

    // Use the smooth trajectory method explicitly
    if (!pose_ctrl->setBodyPoseSmooth(position, orientation, leg_positions, joint_angles)) {
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
    if (!system_enabled || !pose_ctrl)
        return false;

    // Use the immediate (non-smooth) method for compatibility
    if (!pose_ctrl->setBodyPoseImmediate(position, orientation, leg_positions, joint_angles)) {
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
    if (!pose_ctrl)
        return false;
    return pose_ctrl->isTrajectoryInProgress();
}

void LocomotionSystem::resetSmoothMovement() {
    if (pose_ctrl)
        pose_ctrl->resetTrajectory();
}

// Main system update
bool LocomotionSystem::update(double dt_override) {
    if (!system_enabled)
        return false;

    if (dt_override >= 0.0) {
        dt = dt_override;
    } else {
        unsigned long current_time = millis();
        dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
        last_update_time = current_time;
        // Limit dt to avoid large jumps
        if (dt > 0.1f)
            dt = 0.1f;
    }

    // PARALLEL SENSOR READING IMPLEMENTATION
    // Update both FSR and IMU sensors simultaneously for optimal performance
    bool sensors_updated = updateSensorsParallel();
    if (!sensors_updated) {
        last_error = SENSOR_ERROR;
        return false;
    }

    // Adapt gait and step parameters depending on terrain
    adaptGaitToTerrain();
    updateStepParameters();
    adjustStepParameters();
    compensateForSlope();

    // Update leg contact states from FSR sensors
    updateLegStates();

    // Update gait phase
    updateGaitPhase();

    // Update advanced gait patterns
    updateMetachronalPattern();
    updateAdaptivePattern();

    // Calculate new leg positions
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D target_position = calculateFootTrajectory(i, gait_phase);

        // Compute inverse kinematics
        JointAngles target_angles = calculateInverseKinematics(i, target_position);

        // Use setLegJointAngles to enforce constraints at servo level
        // This implements the CSIRO syropod approach of joint position clamping
        if (setLegJointAngles(i, target_angles)) {
            leg_positions[i] = target_position;
        } else {
            last_error = KINEMATICS_ERROR;
#if defined(ENABLE_LOG) && defined(ARDUINO)
            Serial.print("Joint limit violation on leg ");
            Serial.println(i);
#endif
        }
    }

    // Automatic orientation control
    if (imu_interface && imu_interface->isConnected()) {
        correctBodyTilt();
    }

    // Check stability
    if (!checkStabilityMargin()) {
        // Implement corrective actions if necessary
        last_error = STABILITY_ERROR;
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
    if (!fsr_interface)
        return;

    // Update circular buffer index for filtered contact history
    fsr_history_index = (fsr_history_index + 1) % 3;

    // Iterate over each leg and filter FSR contact
    for (int i = 0; i < NUM_LEGS; ++i) {
        FSRData fsr = fsr_interface->readFSR(i);

        // Store contact value in instance history buffer (1.0 for contact, 0.0 for no contact)
        fsr_contact_history[i][fsr_history_index] = fsr.in_contact ? 1.0f : 0.0f;

        // Calculate filtered contact using 3-sample average
        double contact_average = (fsr_contact_history[i][0] + fsr_contact_history[i][1] + fsr_contact_history[i][2]) / 3.0f;

        // Hysteresis thresholds to prevent chattering
        const double CONTACT_THRESHOLD = 0.7f; // Need 70% confidence for contact
        const double RELEASE_THRESHOLD = 0.3f; // Need 30% confidence for release

        LegState current_state = leg_states[i];

        // State transition logic with hysteresis
        if (current_state == SWING_PHASE && contact_average > CONTACT_THRESHOLD) {
            leg_states[i] = STANCE_PHASE;
        } else if (current_state == STANCE_PHASE && contact_average < RELEASE_THRESHOLD) {
            leg_states[i] = SWING_PHASE;
        }
        // Otherwise maintain current state

        // Additional validation: Don't allow contact during planned swing phase
        // Only validate during active movement (non-zero gait phase progression)
        if (cycle_frequency > 0.0f) {
            // Check if leg should be in swing based on gait phase
            double leg_phase = fmod(gait_phase + leg_phase_offsets[i], 1.0f);
            bool should_be_swinging = (leg_phase > stance_duration);

            if (should_be_swinging && leg_states[i] == STANCE_PHASE) {
                // Only override if pressure is very low (potential sensor error)
                if (fsr.pressure < 10.0f) { // Low pressure threshold
                    leg_states[i] = SWING_PHASE;
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

// Additional implementations pending

bool LocomotionSystem::setLegPosition(int leg_index, const Point3D &position) {
    if (!system_enabled || leg_index < 0 || leg_index >= NUM_LEGS || !pose_ctrl)
        return false;
    if (!pose_ctrl->setLegPosition(leg_index, position, leg_positions, joint_angles)) {
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
