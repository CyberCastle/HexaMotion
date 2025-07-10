#ifndef BODY_POSE_CONFIG_H
#define BODY_POSE_CONFIG_H

#include "robot_model.h"
#include <array>
#include <map>
#include <string>
#include <vector>

/**
 * @file body_pose_config.h
 * @brief OpenSHC-equivalent body pose configuration data structures
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
    double x; //< X position relative to body center (millimeters)
    double y; //< Y position relative to body center (millimeters)
};

/**
 * @brief Standing pose joint configuration (OpenSHC equivalent)
 * In OpenSHC, standing pose is configured, not calculated from kinematics
 */
struct StandingPoseJoints {
    double coxa;  //< Coxa joint angle in degrees
    double femur; //< Femur joint angle in degrees
    double tibia; //< Tibia joint angle in degrees
};

/**
 * @brief Complete body pose configuration for the hexapod robot
 * Equivalent to OpenSHC's stance positioning and body pose control system
 */
struct BodyPoseConfiguration {
    Parameters params;
    BodyPoseConfiguration(const Parameters& p) : params(p) {}
    // OpenSHC equivalent stance positions
    std::array<LegStancePosition, NUM_LEGS> leg_stance_positions;

    // OpenSHC equivalent standing pose (configured, not calculated)
    std::array<StandingPoseJoints, NUM_LEGS> standing_pose_joints;

    // OpenSHC equivalent pose controller parameters
    std::string auto_pose_type; //< String denoting the default auto posing cycle type
    bool start_up_sequence;     //< Flag allowing execution of start up and shutdown sequences
    double time_to_start;        //< The time to complete a direct start up

    // OpenSHC equivalent body clearance and swing parameters
    double body_clearance; //< The requested height of the robot body above ground (mm)
    double swing_height;   //< Vertical displacement of swing trajectory above default (mm)

    // OpenSHC equivalent pose limits
    struct {
        double x; //< Maximum X translation (millimeters)
        double y; //< Maximum Y translation (millimeters)
        double z; //< Maximum Z translation (millimeters)
    } max_translation;

    struct {
        double roll;  //< Maximum roll rotation (radians)
        double pitch; //< Maximum pitch rotation (radians)
        double yaw;   //< Maximum yaw rotation (radians)
    } max_rotation;

    // OpenSHC equivalent velocity limits
    double max_translation_velocity; //< Maximum translation velocity (mm/s)
    double max_rotation_velocity;    //< Maximum rotation velocity (rad/s)

    // OpenSHC equivalent pose control flags
    bool gravity_aligned_tips;         //< Flag denoting if tip should align with gravity direction
    bool force_symmetric_pose;         //< Force hexagonal symmetry if true
    std::string leg_manipulation_mode; //< String denoting the type of leg manipulation
};

/**
 * @brief Auto-pose configuration for tripod gait (OpenSHC equivalent)
 * Based on OpenSHC's auto_pose.yaml configuration structure
 */
struct AutoPoseConfiguration {
    bool enabled;                    //< Enable/disable auto-pose during gait
    bool tripod_mode_enabled;        //< Enable/disable tripod-specific compensation
    double pose_frequency;           //< Pose frequency (-1.0 = sync with gait cycle)

    // Phase configuration for tripod gait
    std::vector<int> pose_phase_starts;  //< Phase starts for compensation
    std::vector<int> pose_phase_ends;    //< Phase ends for compensation

    // Auto-pose amplitudes (OpenSHC auto_pose.yaml equivalent)
    std::vector<double> roll_amplitudes;   //< Roll compensation amplitudes (radians)
    std::vector<double> pitch_amplitudes;  //< Pitch compensation amplitudes (radians)
    std::vector<double> yaw_amplitudes;    //< Yaw compensation amplitudes (radians)
    std::vector<double> x_amplitudes;      //< X translation amplitudes (millimeters)
    std::vector<double> y_amplitudes;      //< Y translation amplitudes (millimeters)
    std::vector<double> z_amplitudes;      //< Z translation amplitudes (millimeters)

    // Tripod group configuration
    std::vector<int> tripod_group_a_legs;  //< Group A legs (AR, CR, BL)
    std::vector<int> tripod_group_b_legs;  //< Group B legs (BR, CL, AL)
};

#endif // BODY_POSE_CONFIG_H