#ifndef POSE_CONFIG_H
#define POSE_CONFIG_H

#include "HexaModel.h"
#include <array>
#include <map>
#include <string>

/**
 * @file pose_config.h
 * @brief OpenSHC-equivalent pose configuration data structures
 *
 * This implementation follows OpenSHC's stance positioning system where:
 * - Standing pose joint angles are pre-configured, not calculated
 * - Leg stance positions define the neutral foot positions
 * - Configuration parameters match OpenSHC's parameters_and_states.h structure
 */

/**
 * @brief 2D position configuration for leg tip positions (OpenSHC equivalent)
 */
struct LegStancePosition {
    float x; ///< X position relative to body center (meters)
    float y; ///< Y position relative to body center (meters)
};

/**
 * @brief Standing pose joint configuration (OpenSHC equivalent)
 * In OpenSHC, standing pose is configured, not calculated from kinematics
 */
struct StandingPoseJoints {
    float coxa;  ///< Coxa joint angle in degrees
    float femur; ///< Femur joint angle in degrees
    float tibia; ///< Tibia joint angle in degrees
};

/**
 * @brief Complete pose configuration for the hexapod robot
 * Equivalent to OpenSHC's stance positioning and pose control system
 */
struct PoseConfiguration {
    // OpenSHC equivalent stance positions
    std::array<LegStancePosition, NUM_LEGS> leg_stance_positions;

    // OpenSHC equivalent standing pose (configured, not calculated)
    std::array<StandingPoseJoints, NUM_LEGS> standing_pose_joints;

    // OpenSHC equivalent pose controller parameters
    std::string auto_pose_type; ///< String denoting the default auto posing cycle type
    bool start_up_sequence;     ///< Flag allowing execution of start up and shutdown sequences
    float time_to_start;        ///< The time to complete a direct start up

    // OpenSHC equivalent body clearance and swing parameters
    float body_clearance; ///< The requested height of the robot body above ground (m)
    float swing_height;   ///< Vertical displacement of swing trajectory above default (m)

    // OpenSHC equivalent pose limits
    struct {
        float x; ///< Maximum X translation (meters)
        float y; ///< Maximum Y translation (meters)
        float z; ///< Maximum Z translation (meters)
    } max_translation;

    struct {
        float roll;  ///< Maximum roll rotation (radians)
        float pitch; ///< Maximum pitch rotation (radians)
        float yaw;   ///< Maximum yaw rotation (radians)
    } max_rotation;

    // OpenSHC equivalent velocity limits
    float max_translation_velocity; ///< Maximum translation velocity (m/s)
    float max_rotation_velocity;    ///< Maximum rotation velocity (rad/s)

    // OpenSHC equivalent pose control flags
    bool gravity_aligned_tips;         ///< Flag denoting if tip should align with gravity direction
    bool force_symmetric_pose;         ///< Force hexagonal symmetry if true
    std::string leg_manipulation_mode; ///< String denoting the type of leg manipulation
};

#endif // POSE_CONFIG_H