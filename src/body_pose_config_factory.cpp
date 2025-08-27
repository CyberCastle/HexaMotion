#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <cmath>

/**
 * @file pose_config_factory.cpp
 * @brief OpenSHC-equivalent pose configuration factory for HexaMotion robot
 *
 * This implementation follows OpenSHC's stance positioning system where:
 * - Standing pose joint angles are pre-configured (not calculated)
 * - Leg stance positions define neutral foot positions
 * - Configuration matches OpenSHC's default.yaml structure
 *
 * Physical robot characteristics from AGENTS.md:
 * - Robot height: 208 mm
 * - Body hexagon radius: 200 mm
 * - Coxa length: 50 mm
 * - Femur length: 101 mm
 * - Tibia length: 208 mm
 * - Robot weight: 6.5 Kg
 */

/**
 * @brief Calculate joint angles for a given height using analytic IK
 * @param target_height_mm Target height in millimeters.
 * @param params Robot parameters containing dimensions and joint limits.
 * @return Calculated individual servo angles or default values if no solution
 *         is found.
 */
CalculatedServoAngles calculateServoAnglesForHeight(double target_height_mm, const Parameters &params) {
    CalculatedServoAngles result{0.0, 0.0, 0.0, false};

    // Based on analytic_robot_model.cpp leg transform:
    // T = T_base * R_coxa * T_coxa * R_femur * T_femur * R_tibia * T_tibia
    //
    // For leg height calculation with coxa = 0° (radial stance):
    // - T_base: hexagon_radius in XY plane (Z = 0)
    // - R_coxa: rotation around Z axis (coxa = 0°)
    // - T_coxa: translation along X axis (coxa_length)
    // - R_femur: rotation around Y axis (femur angle)
    // - T_femur: translation along X axis (femur_length)
    // - R_tibia: rotation around Y axis (tibia angle)
    // - T_tibia: translation along Z axis (-tibia_length)

    // With coxa = 0°, the Z component of foot position is:
    // Z = -femur_length * sin(femur_angle) - tibia_length * cos(femur_angle + tibia_angle)
    //
    // For standing pose, we want tibia to be vertical (pointing down):
    // femur_angle + tibia_angle = 0° (so tibia points straight down)
    // Therefore: tibia_angle = -femur_angle
    //
    // Substituting:
    // Z = -femur_length * sin(femur_angle) - tibia_length * cos(0°)
    // Z = -femur_length * sin(femur_angle) - tibia_length
    //
    // Solving for femur_angle:
    // target_height = -femur_length * sin(femur_angle) - tibia_length
    // sin(femur_angle) = -(target_height + tibia_length) / femur_length

    double target_z = -target_height_mm; // Convert to negative Z (150 -> -150)
    double sin_femur = -(target_z + params.tibia_length) / params.femur_length;

    // Check if solution is physically possible
    if (sin_femur < -1.0 || sin_femur > 1.0) {
        return result; // No valid solution
    }

    // Calculate femur angle in radians
    double femur_rad = std::asin(sin_femur);

    // Calculate tibia angle in radians (to keep tibia vertical)
    double tibia_rad = -femur_rad;

    // Convert to degrees for limit checking (assuming limits are in degrees)
    double femur_deg = femur_rad * 180.0 / M_PI;
    double tibia_deg = tibia_rad * 180.0 / M_PI;

    // Check servo limits
    if (femur_deg < params.femur_angle_limits[0] ||
        femur_deg > params.femur_angle_limits[1]) {
        return result;
    }
    if (tibia_deg < params.tibia_angle_limits[0] ||
        tibia_deg > params.tibia_angle_limits[1]) {
        return result;
    }

    // Return angles in radians
    result.femur = femur_rad;
    result.tibia = tibia_rad;
    result.coxa = 0.0; // Coxa remains at 0° for radial stance
    result.valid = true;

    return result;
}

/**
 * @brief Calculate hexagonal leg stance positions based on robot parameters
 * Following OpenSHC's stance positioning approach from default.yaml
 * Uses DH parameter-based forward kinematics for accurate stance positioning
 *
 * Based on AGENTS.md physical characteristics:
 * - Robot height: 208 mm (with all angles at 0°)
 * - Body hexagon radius: 200 mm
 * - Coxa length: 50 mm
 * - Femur length: 101 mm
 * - Tibia length: 208 mm
 *
 * @param params Robot parameters containing dimensions and joint limits.
 * @return Array of calculated stance positions in millimeters
 */
std::array<LegStancePosition, NUM_LEGS> getDefaultStandPositions(const Parameters &params) {
    std::array<LegStancePosition, NUM_LEGS> positions;

    // Create a temporary RobotModel to use DH-based calculations
    RobotModel temp_model(params);

    // Calculate servo angles analytically to guarantee the desired height
    CalculatedServoAngles calc =
        calculateServoAnglesForHeight(params.standing_height, params);

    JointAngles neutral_angles;
    if (calc.valid) {
        neutral_angles = JointAngles{
            0.0, // Coxa angle is 0° for radial alignment
            calc.femur,
            calc.tibia};
    } else {
        neutral_angles = JointAngles{
            0.0, // Coxa angle is 0° for radial alignment
            0.0, // Femur angle is 0° for horizontal alignment
            0.0  // Tibia angle is 0° for vertical alignment
        };
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        // Use DH-based forward kinematics to calculate the foot position
        // This gives us the exact position where the foot would be with 0° angles
        Point3D foot_position = temp_model.forwardKinematicsGlobalCoordinates(i, neutral_angles);

        // Store the stance position (x, y coordinates only, z is the height)
        positions[i].x = foot_position.x;
        positions[i].y = foot_position.y;
        positions[i].z = foot_position.z; // Use Z from FK for accurate stance height
    }

    return positions;
}

/**
 * @brief Get default standing pose joint angles (OpenSHC equivalent)
 * For a hexagonal robot in static equilibrium:
 * - Coxa ≈ 0° (each leg aligned radially with mounting at 60°)
 * - Femur and Tibia calculated using inverse kinematics for target height
 *
 * Based on AGENTS.md: "Physically, if the robot has all servo angles at 0°,
 * the femur remains horizontal, in line with the coxa. The tibia, on the other hand,
 * remains vertical, perpendicular to the ground. This allows the robot to stand stably by default."
 * However, with 0° angles, the robot height would be 208mm (full tibia length).
 * To achieve the target height of 208mm, we calculate the optimal joint angles.
 *
 * For the given robot dimensions (coxa=50mm, femur=101mm, tibia=208mm, height=208mm),
 * the proper standing configuration should be symmetric across all legs.
 * @return Array of standing pose joint configurations
 */
std::array<StandingPoseJoints, NUM_LEGS> getDefaultStandingPoseJoints(const Parameters &params) {
    std::array<StandingPoseJoints, NUM_LEGS> joints{};

    // Calculate servo angles analytically to guarantee the desired height
    CalculatedServoAngles calc =
        calculateServoAnglesForHeight(params.standing_height, params);

    for (int i = 0; i < NUM_LEGS; i++) {
        if (calc.valid) {
            joints[i].coxa = 0.0; // Coxa angle is 0° for radial alignment
            joints[i].femur = calc.femur;
            joints[i].tibia = calc.tibia;
        } else {
            // Fallback to a neutral configuration
            // Fine-tuned angles that achieve exactly -208mm height:
            // femur=0°, tibia=0° gives FK_Z = -208.0mm (error = 0.0mm)
            // This is the precise configuration for the target robot height
            // calculated with finetune_angles_test.cpp
            joints[i].coxa = 0.0; // Coxa angle is 0° for radial alignment
            joints[i].femur = 0.0;
            joints[i].tibia = 0.0;
        }
    }

    return joints;
}

/**
 * @brief Create pose configuration with OpenSHC-equivalent parameters
 * @param params Robot parameters from HexaModel
 * @param config_type Type of configuration ("default", "conservative", "high_speed")
 * @return Complete pose configuration following OpenSHC structure
 *
 * Note: This creates the gait-independent body pose configuration.
 * Gait-specific parameters (like step length/height) are handled separately in GaitConfiguration.
 * This follows OpenSHC's separation between:
 * - Walker parameters (default.yaml): gait-independent settings like body_clearance, swing_height
 * - Gait parameters (gait.yaml): gait-specific stance/swing phases and offsets
 * - Auto-pose parameters (auto_pose.yaml): gait-specific compensation amplitudes
 */
BodyPoseConfiguration createPoseConfiguration(const Parameters &params, const std::string &config_type) {
    BodyPoseConfiguration config(params);
    config.leg_stance_positions = getDefaultStandPositions(params);
    config.standing_pose_joints = getDefaultStandingPoseJoints(params);

    // Derive standing horizontal reach from standing pose joints (use leg 0 as representative; all symmetric)
    const StandingPoseJoints &sj0 = config.standing_pose_joints[0];

    // Horizontal component: coxa_length + femur_length * cos(femur_angle)
    config.standing_horizontal_reach = params.coxa_length + params.femur_length * std::cos(sj0.femur);

    // OpenSHC equivalent pose controller parameters
    config.auto_pose_type = "auto";
    config.start_up_sequence = false; // Match OpenSHC default.yaml
    config.time_to_start = 6.0f;      // Match OpenSHC default.yaml

    // OpenSHC equivalent body clearance and swing parameters
    config.body_clearance = params.standing_height; // Body clearance in millimeters - use standing_height for consistency
    // Use swing height factor from OpenSHC equivalent constants for body pose (gait-independent)
    config.swing_height = static_cast<float>(params.standing_height * BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR);

    // OpenSHC equivalent pose limits (from default.yaml)
    config.max_translation = {25.0f, 25.0f, 25.0f}; // 25mm translation limits
    config.max_rotation = {0.25f, 0.25f, 0.25f};    // 0.25 radian rotation limits
    config.max_translation_velocity = 50.0f;        // 50mm/s velocity limit
    config.max_rotation_velocity = 0.200f;          // 0.2 rad/s rotation limit

    // OpenSHC equivalent pose control flags
    config.gravity_aligned_tips = false;          // Match OpenSHC default.yaml
    config.force_symmetric_pose = true;           // Maintain hexagonal symmetry
    config.leg_manipulation_mode = "tip_control"; // Match OpenSHC default.yaml

    // Modify based on configuration type
    if (config_type == "conservative") {
        config.start_up_sequence = true;
        config.time_to_start = 8.0f;      // Longer startup time
        config.max_translation.x *= 0.8f; // Reduced limits for safety
        config.max_translation.y *= 0.8f;
        config.max_translation.z *= 0.8f;
        config.max_rotation.roll *= 0.8f;
        config.max_rotation.pitch *= 0.8f;
        config.max_rotation.yaw *= 0.8f;
        config.max_translation_velocity *= 0.7f;
        config.max_rotation_velocity *= 0.7f;
        config.swing_height *= 0.75f; // Lower swing height
    } else if (config_type == "high_speed") {
        config.start_up_sequence = false;
        config.time_to_start = 3.0f;      // Faster startup
        config.max_translation.x *= 1.5f; // Increased limits for performance
        config.max_translation.y *= 1.5f;
        config.max_translation.z *= 1.2f;
        config.max_rotation.roll *= 1.2f;
        config.max_rotation.pitch *= 1.2f;
        config.max_rotation.yaw *= 1.5f;
        config.max_translation_velocity *= 2.0f;
        config.max_rotation_velocity *= 2.0f;
        config.swing_height *= 1.5f;         // Higher swing height
        config.gravity_aligned_tips = false; // Disable for speed
    }

    return config;
}

/**
 * @brief Get default pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Default pose configuration calculated from robot specifications
 */
BodyPoseConfiguration getDefaultPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "default");
}

/**
 * @brief Get conservative pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Conservative pose configuration with reduced limits for safety
 */
BodyPoseConfiguration getConservativePoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "conservative");
}

/**
 * @brief Get high-speed pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return High-speed pose configuration optimized for faster locomotion
 */
BodyPoseConfiguration getHighSpeedPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "high_speed");
}

// Implementation for the linker
BodyPoseConfiguration getDefaultBodyPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "default");
}

/**
 * @brief Create auto-pose configuration for tripod gait (OpenSHC equivalent)
 * Based on OpenSHC's auto_pose.yaml configuration
 * @param params Robot parameters
 * @return Auto-pose configuration structure
 */
AutoPoseConfiguration createAutoPoseConfiguration(const Parameters &params) {
    AutoPoseConfiguration config;

    // OpenSHC auto_pose.yaml equivalent settings
    config.enabled = false;       // Disable auto-pose for consistent testing
    config.pose_frequency = -1.0; // Synchronize with gait cycle

    // Tripod gait phase configuration (OpenSHC equivalent)
    config.pose_phase_starts = {1, 3}; // Phase starts for compensation
    config.pose_phase_ends = {3, 1};   // Phase ends for compensation

    // Auto-pose amplitudes (from OpenSHC auto_pose.yaml)
    config.roll_amplitudes = {-0.015, 0.015}; // Roll compensation (radians)
    config.pitch_amplitudes = {0.000, 0.000}; // Pitch compensation (radians)
    config.yaw_amplitudes = {0.000, 0.000};   // Yaw compensation (radians)
    config.x_amplitudes = {0.000, 0.000};     // X translation (millimeters)
    config.y_amplitudes = {0.000, 0.000};     // Y translation (millimeters)
    config.z_amplitudes = {20.0, 20.0};       // Z translation (millimeters)

    // Tripod group configuration
    config.tripod_group_a_legs = {0, 2, 4}; // AR, CR, BL
    config.tripod_group_b_legs = {1, 3, 5}; // BR, CL, AL

    return config;
}

/**
 * @brief Create conservative auto-pose configuration
 * @param params Robot parameters
 * @return Conservative auto-pose configuration
 */
AutoPoseConfiguration createConservativeAutoPoseConfiguration(const Parameters &params) {
    AutoPoseConfiguration config = createAutoPoseConfiguration(params);

    // Reduced amplitudes for conservative operation
    config.roll_amplitudes = {-0.010, 0.010}; // Reduced roll compensation
    config.z_amplitudes = {15.0, 15.0};       // Reduced Z compensation (millimeters)

    return config;
}

/**
 * @brief Create high-speed auto-pose configuration
 * @param params Robot parameters
 * @return High-speed auto-pose configuration
 */
AutoPoseConfiguration createHighSpeedAutoPoseConfiguration(const Parameters &params) {
    AutoPoseConfiguration config = createAutoPoseConfiguration(params);

    // Increased amplitudes for high-speed operation
    config.roll_amplitudes = {-0.020, 0.020}; // Increased roll compensation
    config.z_amplitudes = {25.0, 25.0};       // Increased Z compensation (millimeters)

    return config;
}

BodyPoseConfiguration getConservativeBodyPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "conservative");
}

BodyPoseConfiguration getHighSpeedBodyPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "high_speed");
}
