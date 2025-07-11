#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <cmath>
#include <limits>

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
 * @brief Calculate hexagonal leg stance positions based on robot parameters
 * Following OpenSHC's stance positioning approach from default.yaml
 * @param params Robot parameters containing dimensions and joint limits.
 * @return Array of calculated stance positions in millimeters
 */
std::array<LegStancePosition, NUM_LEGS> calculateHexagonalStancePositions(const Parameters &params) {
    std::array<LegStancePosition, NUM_LEGS> positions;
    double total_radius_mm = params.hexagon_radius + params.coxa_length;
    // Create a temporary RobotModel to get leg base positions
    RobotModel temp_model(params);
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D base_pos = temp_model.getAnalyticLegBasePosition(i);
        positions[i].x = base_pos.x + params.coxa_length * cos(atan2(base_pos.y, base_pos.x));
        positions[i].y = base_pos.y + params.coxa_length * sin(atan2(base_pos.y, base_pos.x));
    }
    return positions;
}

/**
 * @brief Calculate joint angles for a given height using analytic IK
 *
 * This helper mimics the logic from angle_calculus.cpp. It sweeps the
 * tibia angle (\f$\beta\f$) across its allowed range and solves for the
 * femur angle (\f$\alpha\f$) so the tibia remains as vertical as possible
 * while respecting the servo limits.
 *
 * @param target_height_mm Target height in millimeters.
 * @param params Robot parameters containing dimensions and joint limits.
 * @return Calculated individual servo angles or default values if no solution
 *         is found.
 */
CalculatedServoAngles calculateServoAnglesForHeight(double target_height_mm, const Parameters &params) {
    CalculatedServoAngles best{0.0, 0.0, 0.0, false};
    double bestErr = std::numeric_limits<double>::infinity();

    // Ranges based on servo limits (already in degrees, convert to radians for calculations)
    const double alphaMin = math_utils::degreesToRadians(params.femur_angle_limits[0]);
    const double alphaMax = math_utils::degreesToRadians(params.femur_angle_limits[1]);
    const double betaMin = math_utils::degreesToRadians(params.tibia_angle_limits[0]);
    const double betaMax = math_utils::degreesToRadians(params.tibia_angle_limits[1]);
    const double dBeta = math_utils::degreesToRadians(0.1);

    const double A = params.coxa_length;
    const double B = params.femur_length;
    const double C = params.tibia_length;

    // Sweep the tibia angle to search for a valid configuration
    for (double beta = betaMin; beta <= betaMax; beta += dBeta) {
        double sum = A + B * std::cos(beta);
        double discriminant = sum * sum - (target_height_mm * target_height_mm - C * C);
        if (discriminant < 0.0)
            continue;

        double sqrtD = std::sqrt(discriminant);
        // Evaluate both quadratic roots for alpha
        for (int sign : {-1, 1}) {
            double t = (-sum + sign * sqrtD) / (target_height_mm + C);
            double alpha = 2.0 * std::atan(t);
            if (alpha < alphaMin || alpha > alphaMax)
                continue;

            // Measure deviation from a perfectly vertical tibia
            double err = std::fabs(alpha + beta);
            if (err >= bestErr)
                continue;

            double theta1 = math_utils::radiansToDegrees(alpha - beta);
            double theta2 = math_utils::radiansToDegrees(-beta);

            // Skip solutions that exceed servo limits
            if (theta1 < params.femur_angle_limits[0] ||
                theta1 > params.femur_angle_limits[1])
                continue;
            if (theta2 < params.tibia_angle_limits[0] ||
                theta2 > params.tibia_angle_limits[1])
                continue;

            // Keep the best solution found so far
            bestErr = err;
            best = {0.0, theta1, theta2, true};
        }
    }

    return best;
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
        calculateServoAnglesForHeight(params.robot_height, params);

    for (int i = 0; i < NUM_LEGS; i++) {
        if (calc.valid) {
            joints[i].coxa = 0.0; // Coxa angle is 0° for radial alignment
            joints[i].femur = math_utils::degreesToRadians(calc.femur);
            joints[i].tibia = math_utils::degreesToRadians(calc.tibia);
        } else {
            // Fallback to a neutral configuration
            // Fine-tuned angles that achieve exactly -208mm height:
            // femur=0°, tibia=0° gives FK_Z = -208.0mm (error = 0.0mm)
            // This is the precise configuration for the target robot height
            // calculated with finetune_angles_test.cpp
            joints[i].coxa = 0.0; // Coxa angle is 0° for radial alignment
            joints[i].femur = math_utils::degreesToRadians(0.0);
            joints[i].tibia = math_utils::degreesToRadians(0.0);
        }
    }

    return joints;
}

/**
 * @brief Create pose configuration with OpenSHC-equivalent parameters
 * @param params Robot parameters from HexaModel
 * @param config_type Type of configuration ("default", "conservative", "high_speed")
 * @return Complete pose configuration following OpenSHC structure
 */
BodyPoseConfiguration createPoseConfiguration(const Parameters &params, const std::string &config_type) {
    BodyPoseConfiguration config(params);
    config.leg_stance_positions = calculateHexagonalStancePositions(params);
    config.standing_pose_joints = getDefaultStandingPoseJoints(params);

    // OpenSHC equivalent pose controller parameters
    config.auto_pose_type = "auto";
    config.start_up_sequence = false; // Match OpenSHC default.yaml
    config.time_to_start = 6.0f;      // Match OpenSHC default.yaml

    // OpenSHC equivalent body clearance and swing parameters
    config.body_clearance = params.robot_height; // Body clearance in millimeters
    config.swing_height = 20.0f;                 // Default 20mm swing height (OpenSHC typical)

    // OpenSHC equivalent pose limits (from default.yaml)
    config.max_translation = {25.0f, 25.0f, 25.0f}; // 25mm translation limits
    config.max_rotation = {0.250f, 0.250f, 0.250f}; // 0.25 radian rotation limits
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

// Implementación para el linker
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
    config.enabled = true;
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
