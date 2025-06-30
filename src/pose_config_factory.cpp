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
    double hexagon_radius, double coxa_length) {

    std::array<LegStancePosition, NUM_LEGS> positions;

    // Calculate total radius from center to leg tip (OpenSHC equivalent)
    double total_radius_mm = hexagon_radius + coxa_length;
    double total_radius_m = total_radius_mm / 1000.0f; // Convert to meters

    // Calculate positions for each leg using hexagonal spacing (OpenSHC style)
    // OpenSHC leg naming: AR, BR, CR, CL, BL, AL (right-front to left-front)
    // HexaMotion mapping: 0=AR, 1=BR, 2=CR, 3=CL, 4=BL, 5=AL
    for (int i = 0; i < NUM_LEGS; i++) {
        double angle_deg = i * LEG_ANGLE_SPACING;
        double angle_rad = math_utils::degreesToRadians(angle_deg);

        positions[i].x = total_radius_m * cos(angle_rad);
        positions[i].y = total_radius_m * sin(angle_rad);
    }

    return positions;
}

/**
 * @brief Calculate joint angles for a given height using inverse kinematics
 * Based on the angle_calculus.cpp implementation
 * @param target_height_mm Target height in millimeters
 * @param params Robot parameters containing dimensions and joint limits
 * @return Calculated individual servo angles or default values if no solution found
 */
struct CalculatedServoAngles {
    double coxa;  // coxa servo angle (degrees)
    double femur; // femur servo angle (degrees)
    double tibia; // tibia servo angle (degrees)
    bool valid;  // solution validity flag
};

CalculatedServoAngles calculateServoAnglesForHeight(double target_height_mm, const Parameters &params) {
    // Robot dimensions from parameters
    const double coxa_length = params.coxa_length;
    const double femur_length = params.femur_length;
    const double tibia_length = params.tibia_length;

    // Joint limits from parameters (converted to radians)
    const double alphaMin = params.femur_angle_limits[0] * DEGREES_TO_RADIANS_FACTOR;
    const double alphaMax = params.femur_angle_limits[1] * DEGREES_TO_RADIANS_FACTOR;
    const double betaMin = params.tibia_angle_limits[0] * DEGREES_TO_RADIANS_FACTOR;
    const double betaMax = params.tibia_angle_limits[1] * DEGREES_TO_RADIANS_FACTOR;

    const double Ytarget = target_height_mm - tibia_length;

    CalculatedServoAngles best;
    best.coxa = 0;
    best.femur = 0;
    best.tibia = 0;
    best.valid = false;
    double bestScore = 1e9f;

    // Iterative search for optimal angles (following angle_calculus.cpp algorithm)
    for (double beta = betaMin; beta <= betaMax; beta += 0.5f * DEGREES_TO_RADIANS_FACTOR) {
        double yRem = Ytarget - femur_length * sin(beta);
        double s = yRem / coxa_length; // argument of asin

        if (s < -1.0f || s > 1.0f)
            continue;

        double alpha = asin(s);

        if (alpha < alphaMin || alpha > alphaMax)
            continue;

        // Calculate relative angles
        double theta1 = (beta - alpha) * RADIANS_TO_DEGREES_FACTOR; // coxa-femur relative angle
        if (theta1 < params.femur_angle_limits[0] || theta1 > params.femur_angle_limits[1])
            continue;

        double theta2 = -beta * RADIANS_TO_DEGREES_FACTOR; // femur-tibia relative angle

        // Convert to individual servo angles based on geometric relationships:
        // From angle_calculus.cpp geometric relationships:
        //   θ₂ = −β     ⇒  β = −θ₂
        //   θ₁ = β − α  ⇒  α = β − θ₁
        // Therefore:
        //   femur_angle = α = β − θ₁
        //   tibia_angle = β = −θ₂
        double coxa_angle = 0.0f;                               // Coxa remains aligned radially
        double femur_angle = alpha * RADIANS_TO_DEGREES_FACTOR; // Convert alpha to degrees
        double tibia_angle = beta * RADIANS_TO_DEGREES_FACTOR;  // Convert beta to degrees

        double score = fabs(alpha) + fabs(beta);
        if (score < bestScore) {
            best.coxa = coxa_angle;
            best.femur = femur_angle;
            best.tibia = tibia_angle;
            best.valid = true;
            bestScore = score;
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
 * To achieve the target height of 120mm, we calculate the optimal joint angles.
 *
 * For the given robot dimensions (coxa=50mm, femur=101mm, tibia=208mm, height=120mm),
 * the proper standing configuration should be symmetric across all legs.
 * @return Array of standing pose joint configurations
 */
std::array<StandingPoseJoints, NUM_LEGS> getDefaultStandingPoseJoints(const Parameters &params) {
    std::array<StandingPoseJoints, NUM_LEGS> joints;

    // Target height for standing pose (use robot_height from parameters)
    const double target_height_mm = params.robot_height;

    // Calculate optimal servo angles for the target height using inverse kinematics
    CalculatedServoAngles calculated = calculateServoAnglesForHeight(target_height_mm, params);

    // Standing pose for stable equilibrium (all legs identical for symmetry)
    for (int i = 0; i < NUM_LEGS; i++) {
        if (calculated.valid) {
            // Use calculated individual servo angles from inverse kinematics
            joints[i].coxa = calculated.coxa;   // Calculated coxa angle (typically 0°)
            joints[i].femur = calculated.femur; // Calculated femur servo angle
            joints[i].tibia = calculated.tibia; // Calculated tibia servo angle
        } else {
            // Fallback to safe default values if calculation fails
            joints[i].coxa = 0.0f;   // Radially aligned (0° relative to mounting)
            joints[i].femur = 30.0f; // Conservative forward angle for stability
            joints[i].tibia = 20.0f; // Conservative knee bend for target height
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
PoseConfiguration createPoseConfiguration(const Parameters &params, const std::string &config_type) {
    PoseConfiguration config;

    // Calculate stance positions based on robot dimensions (OpenSHC equivalent)
    config.leg_stance_positions = calculateHexagonalStancePositions(
        params.hexagon_radius, params.coxa_length);

    // Set standing pose joints (OpenSHC equivalent - calculated using parameters)
    config.standing_pose_joints = getDefaultStandingPoseJoints(params);

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
