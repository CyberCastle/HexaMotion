#include "pose_config_factory.h"
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
 * - Robot height: 120 mm
 * - Body hexagon radius: 200 mm
 * - Coxa length: 50 mm
 * - Femur length: 101 mm
 * - Tibia length: 208 mm
 * - Robot weight: 6.5 Kg
 */

/**
 * @brief Calculate hexagonal leg stance positions based on robot parameters
 * Following OpenSHC's stance positioning approach from default.yaml
 * @param hexagon_radius Body hexagon radius in mm
 * @param coxa_length Coxa length in mm
 * @return Array of calculated stance positions in meters
 */
std::array<LegStancePosition, NUM_LEGS> calculateHexagonalStancePositions(
    float hexagon_radius, float coxa_length) {

    std::array<LegStancePosition, NUM_LEGS> positions;

    // Calculate total radius from center to leg tip (OpenSHC equivalent)
    float total_radius_mm = hexagon_radius + coxa_length;
    float total_radius_m = total_radius_mm / 1000.0f; // Convert to meters

    // Calculate positions for each leg using hexagonal spacing (OpenSHC style)
    // OpenSHC leg naming: AR, BR, CR, CL, BL, AL (right-front to left-front)
    // HexaMotion mapping: 0=AR, 1=BR, 2=CR, 3=CL, 4=BL, 5=AL
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle_deg = i * LEG_ANGLE_SPACING;
        float angle_rad = math_utils::degreesToRadians(angle_deg);

        positions[i].x = total_radius_m * cos(angle_rad);
        positions[i].y = total_radius_m * sin(angle_rad);
    }

    return positions;
}

/**
 * @brief Get default standing pose joint angles (OpenSHC equivalent)
 * For a hexagonal robot in static equilibrium:
 * - Coxa ≈ 0° (each leg aligned radially with mounting at 60°)
 * - Femur and Tibia equal for all legs (symmetric posture)
 *
 * Based on AGENTS.md: "Physically, with coxa-femur = 0º and femur-tibia = 0º,
 * the femur remains horizontal, while the tibia remains vertical, standing stably."
 *
 * For the given robot dimensions (coxa=50mm, femur=101mm, tibia=208mm, height=120mm),
 * the proper standing configuration should be symmetric across all legs.
 * @return Array of standing pose joint configurations
 */
std::array<StandingPoseJoints, NUM_LEGS> getDefaultStandingPoseJoints() {
    std::array<StandingPoseJoints, NUM_LEGS> joints;

    // For a hexagonal robot with height=120mm, using the robot dimensions:
    // - Coxa length: 50mm
    // - Femur length: 101mm
    // - Tibia length: 208mm
    // - Target height from ground to body: 120mm

    // Standing pose for stable equilibrium (all legs identical for symmetry)
    // This ensures coxa ≈ 0° and femur/tibia equal for all legs
    for (int i = 0; i < NUM_LEGS; i++) {
        joints[i].coxa = 0.0f;    // Radially aligned (0° relative to mounting)
        joints[i].femur = 30.0f;  // Slight forward angle for stability
        joints[i].tibia = -60.0f; // Knee bent to achieve target height
    }

    return joints;
}

/**
 * @brief Create pose configuration with OpenSHC-equivalent parameters
 * @param params Robot parameters from HexaModel
 * @param config_type Type of configuration ("default", "conservative", "high_speed")
 * @return Complete pose configuration following OpenSHC structure
 */
PoseConfiguration createPoseConfiguration(const Parameters &params, const std::string &config_type) {
    PoseConfiguration config;

    // Calculate stance positions based on robot dimensions (OpenSHC equivalent)
    config.leg_stance_positions = calculateHexagonalStancePositions(
        params.hexagon_radius, params.coxa_length);

    // Set standing pose joints (OpenSHC equivalent - configured, not calculated)
    config.standing_pose_joints = getDefaultStandingPoseJoints();

    // OpenSHC equivalent pose controller parameters
    config.auto_pose_type = "auto";
    config.start_up_sequence = false; // Match OpenSHC default.yaml
    config.time_to_start = 6.0f;      // Match OpenSHC default.yaml

    // OpenSHC equivalent body clearance and swing parameters
    config.body_clearance = params.robot_height / 1000.0f; // Convert to meters
    config.swing_height = 0.020f;                          // Default 20mm swing height (OpenSHC typical)

    // OpenSHC equivalent pose limits (from default.yaml)
    config.max_translation = {0.025f, 0.025f, 0.025f}; // 25mm translation limits
    config.max_rotation = {0.250f, 0.250f, 0.250f};    // 0.25 radian rotation limits
    config.max_translation_velocity = 0.050f;          // 50mm/s velocity limit
    config.max_rotation_velocity = 0.200f;             // 0.2 rad/s rotation limit

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
PoseConfiguration getDefaultPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "default");
}

/**
 * @brief Get conservative pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Conservative pose configuration with reduced limits for safety
 */
PoseConfiguration getConservativePoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "conservative");
}

/**
 * @brief Get high-speed pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return High-speed pose configuration optimized for faster locomotion
 */
PoseConfiguration getHighSpeedPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params, "high_speed");
}
