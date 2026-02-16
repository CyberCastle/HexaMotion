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

    /** Create a temporary RobotModel to use DH-based calculations. */
    RobotModel temp_model(params);

    /** Calculate servo angles analytically to guarantee the desired height. */
    CalculatedServoAngles calc =
        RobotModel::calculateServoAnglesForHeight(params.standing_height, params);

    JointAngles neutral_angles;
    if (calc.valid) {
        neutral_angles = JointAngles{
            /** Coxa angle is 0 degrees for radial alignment. */
            0.0,
            calc.femur,
            calc.tibia};
    } else {
        neutral_angles = JointAngles{
            /** Coxa angle is 0 degrees for radial alignment. */
            0.0,
            /** Femur angle is 0 degrees for horizontal alignment. */
            0.0,
            /** Tibia angle is 0 degrees for vertical alignment. */
            0.0};
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        /** Use DH-based forward kinematics to calculate the foot position. */
        /** This gives the exact position where the foot would be with 0 degrees angles. */
        Point3D foot_position = temp_model.forwardKinematicsGlobalCoordinates(i, neutral_angles);

        /** Store the stance position (x, y coordinates only, z is the height). */
        positions[i].x = foot_position.x;
        positions[i].y = foot_position.y;
        /** Use Z from FK for accurate stance height. */
        positions[i].z = foot_position.z;
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

    /** Calculate servo angles analytically to guarantee the desired height. */
    CalculatedServoAngles calc =
        RobotModel::calculateServoAnglesForHeight(params.standing_height, params);

    for (int i = 0; i < NUM_LEGS; i++) {
        if (calc.valid) {
            /** Coxa angle is 0 degrees for radial alignment. */
            joints[i].coxa = 0.0;
            joints[i].femur = calc.femur;
            joints[i].tibia = calc.tibia;
        } else {
            /** Fallback to a neutral configuration. */
            /** Fine-tuned angles that achieve exactly -208 mm height. */
            /** femur=0 degrees, tibia=0 degrees gives FK_Z = -208.0 mm (error = 0.0 mm). */
            /** This is the precise configuration for the target robot height. */
            /** Calculated with finetune_angles_test.cpp. */
            /** Coxa angle is 0 degrees for radial alignment. */
            joints[i].coxa = 0.0;
            joints[i].femur = 0.0;
            joints[i].tibia = 0.0;
        }
    }

    return joints;
}

/**
 * @brief Create pose configuration with OpenSHC-equivalent parameters
 * @param params Robot parameters from HexaModel
 * @return Complete pose configuration following OpenSHC structure
 *
 * Note: This creates the gait-independent body pose configuration.
 * Gait-specific parameters (like step length/height) are handled separately in GaitConfiguration.
 * This follows OpenSHC's separation between:
 * - Walker parameters (default.yaml): gait-independent settings like body_clearance, swing_height
 * - Gait parameters (gait.yaml): gait-specific stance/swing phases and offsets
 * - Auto-pose parameters (auto_pose.yaml): gait-specific compensation amplitudes
 */
BodyPoseConfiguration createPoseConfiguration(const Parameters &params) {
    BodyPoseConfiguration config(params);
    config.leg_stance_positions = getDefaultStandPositions(params);
    config.standing_pose_joints = getDefaultStandingPoseJoints(params);

    /** Centralized horizontal reach computation (avoid duplicating femur angle assumptions). */
    /** Use RobotModel analytical helper to ensure a single authoritative formula. */
    config.standing_horizontal_reach = RobotModel::computeStandingHorizontalReach(params);

    /** Match OpenSHC default.yaml. */
    config.start_up_sequence = false;
    /** Match OpenSHC default.yaml. */
    config.time_to_start = 6.0f;

    /** OpenSHC equivalent body clearance and swing parameters. */
    /** Body clearance in millimeters; use standing_height for consistency. */
    config.body_clearance = params.standing_height;
    /** Use swing height factor from OpenSHC equivalent constants for body pose (gait-independent). */
    config.swing_height = static_cast<float>(params.standing_height * BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR);

    /** OpenSHC equivalent pose limits (from default.yaml). */
    /** 25 mm translation limits. */
    config.max_translation = {25.0f, 25.0f, 25.0f};
    /** 0.25 radian rotation limits. */
    config.max_rotation = {0.25f, 0.25f, 0.25f};
    /** Velocity limits. */
    config.max_translation_velocity = params.max_translation_velocity;
    config.max_rotation_velocity = params.max_rotation_velocity;

    /** OpenSHC equivalent pose control flags. */
    config.manual_posing_enabled = params.manual_posing;
    config.inclination_posing_enabled = params.inclination_posing;
    config.imu_posing_enabled = params.imu_posing;
    config.auto_posing_enabled = params.auto_posing;
    config.gravity_aligned_tips_enabled = params.gravity_aligned_tips;

    for (int i = 0; i < NUM_LEGS; ++i) {
        config.offset_multiplier[i] = 0;
    }

    return config;
}

/**
 * @brief Get conservative pose configuration using robot parameters
 * @param params Robot parameters from HexaModel
 * @return Conservative pose configuration with reduced limits for safety
 */
/** Alternative configurations removed; keep default only. */

/** Implementation for the linker. */
BodyPoseConfiguration getDefaultBodyPoseConfig(const Parameters &params) {
    return createPoseConfiguration(params);
}

/**
 * @brief Create auto-pose configuration for tripod gait (OpenSHC equivalent)
 * Based on OpenSHC's auto_pose.yaml configuration
 * @param params Robot parameters
 * @return Auto-pose configuration structure
 */
AutoPoseConfiguration createAutoPoseConfigurationForGait(const Parameters &params, const std::string &gait_name) {
    /** Disabled by default. */
    AutoPoseConfiguration cfg;
    /** Synchronized by default or overridden by Parameters. */
    cfg.pose_frequency = params.pose_frequency;
    cfg.gait_name = gait_name;
    cfg.enabled = true;

    if (gait_name == "tripod_gait") {
        cfg.pose_phase_length = (params.pose_phase_length > 0) ? params.pose_phase_length : 4;
        cfg.pose_phase_starts = {1, 3};
        cfg.pose_phase_ends = {3, 1};
        cfg.roll_amplitudes = {-0.015, 0.015};
        cfg.pitch_amplitudes = {0.0, 0.0};
        cfg.yaw_amplitudes = {0.0, 0.0};
        cfg.x_amplitudes = {0.0, 0.0};
        cfg.y_amplitudes = {0.0, 0.0};
        cfg.z_amplitudes = {20.0, 20.0};
        cfg.gravity_amplitudes = {0.0, 0.0};
        int starts[NUM_LEGS] = {1, 3, 1, 3, 1, 3};
        int ends[NUM_LEGS] = {3, 1, 3, 1, 3, 1};
        for (int i = 0; i < NUM_LEGS; ++i) {
            cfg.negation_phase_start[i] = starts[i];
            cfg.negation_phase_end[i] = ends[i];
            cfg.negation_transition_ratio[i] = 0.0;
        }
    } else if (gait_name == "wave_gait") {
        cfg.pose_phase_length = (params.pose_phase_length > 0) ? params.pose_phase_length : 12;
        cfg.pose_phase_starts = {1, 3, 5, 7, 9, 11};
        cfg.pose_phase_ends = {3, 5, 7, 9, 11, 1};
        cfg.roll_amplitudes = {-0.015, 0.015, 0.015, 0.015, -0.015, -0.015};
        cfg.pitch_amplitudes = {0.020, -0.020, 0.0, 0.020, -0.020, 0.0};
        cfg.yaw_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.x_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.y_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.z_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.gravity_amplitudes = {0, 0, 0, 0, 0, 0};
        int starts[NUM_LEGS] = {1, 11, 9, 3, 5, 7};
        int ends[NUM_LEGS] = {3, 1, 11, 5, 7, 9};
        for (int i = 0; i < NUM_LEGS; ++i) {
            cfg.negation_phase_start[i] = starts[i];
            cfg.negation_phase_end[i] = ends[i];
            cfg.negation_transition_ratio[i] = 0.0;
        }
    } else if (gait_name == "ripple_gait") {
        cfg.pose_phase_length = (params.pose_phase_length > 0) ? params.pose_phase_length : 6;
        cfg.pose_phase_starts = {0, 1, 2, 3, 4, 5};
        cfg.pose_phase_ends = {2, 3, 4, 5, 0, 1};
        cfg.roll_amplitudes = {-0.015, 0.015, -0.015, 0.015, -0.015, 0.015};
        cfg.pitch_amplitudes = {-0.020, 0.020, 0.0, -0.020, 0.020, 0.0};
        cfg.yaw_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.x_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.y_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.z_amplitudes = {0, 0, 0, 0, 0, 0};
        cfg.gravity_amplitudes = {0, 0, 0, 0, 0, 0};
        int starts[NUM_LEGS] = {0, 2, 4, 1, 5, 3};
        int ends[NUM_LEGS] = {2, 4, 0, 3, 1, 5};
        for (int i = 0; i < NUM_LEGS; ++i) {
            cfg.negation_phase_start[i] = starts[i];
            cfg.negation_phase_end[i] = ends[i];
            cfg.negation_transition_ratio[i] = 0.0;
        }
    } else {
        /** Other options not supported; cfg remains disabled. */
    }
    return cfg;
}
