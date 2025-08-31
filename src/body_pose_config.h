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
    double z; //< Z position relative to body center (millimeters)
};

/**
 * @brief Standing pose joint configuration (OpenSHC equivalent)
 * In OpenSHC, standing pose is configured, not calculated from kinematics
 */
struct StandingPoseJoints {
    double coxa;  //< Coxa joint angle in radians
    double femur; //< Femur joint angle in radians
    double tibia; //< Tibia joint angle in radians
};

/**
 * @brief Complete body pose configuration for the hexapod robot
 * Equivalent to OpenSHC's stance positioning and body pose control system
 */
struct BodyPoseConfiguration {
    Parameters params;
    BodyPoseConfiguration(const Parameters &p) : params(p) {

        // Sensible defaults to silence static analysis uninit warnings
        auto_pose_type = "none";
        start_up_sequence = false;
        time_to_start = 0.0;
        body_clearance = params.standing_height; // default clearance
        swing_height = params.standing_height * BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR;
        max_translation = {0.0, 0.0, 0.0};
        max_rotation = {0.0, 0.0, 0.0};
        max_translation_velocity = 0.0;
        max_rotation_velocity = 0.0;
        gravity_aligned_tips = false;
        force_symmetric_pose = false;
        leg_manipulation_mode = "none";
        // Zero initialize stance & pose arrays
        for (auto &ls : leg_stance_positions) {
            ls = {0.0, 0.0, 0.0};
        }
        for (auto &sj : standing_pose_joints) {
            sj = {0.0, 0.0, 0.0};
        }
        standing_horizontal_reach = 0.0;
    }
    // OpenSHC equivalent stance positions
    std::array<LegStancePosition, NUM_LEGS> leg_stance_positions;

    // OpenSHC equivalent standing pose (configured, not calculated)
    std::array<StandingPoseJoints, NUM_LEGS> standing_pose_joints;

    // Horizontal reach (from body center to foot) contribution beyond hexagon radius
    // computed from the configured standing pose joints (coxa pivot projection):
    // standing_horizontal_reach = coxa_length + femur_length * cos(femur_angle_standing)
    // (tibia vertical => no horizontal component). Used for stance & walkspace sizing.
    double standing_horizontal_reach;

    // OpenSHC equivalent pose controller parameters
    std::string auto_pose_type; //< String denoting the default auto posing cycle type
    bool start_up_sequence;     //< Flag allowing execution of start up and shutdown sequences
    double time_to_start;       //< The time to complete a direct start up

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
    bool enabled = false;         //< Enable/disable auto-pose during gait
    double pose_frequency = -1.0; //< Pose frequency (-1.0 = sync with gait cycle)
    int pose_phase_length = 0;    //< Base phase length (from YAML) used when pose_frequency != -1

    // Phase segmentation (ordered as in auto_pose.yaml for the active gait)
    std::vector<int> pose_phase_starts; //< Start indices (inclusive)
    std::vector<int> pose_phase_ends;   //< End indices (exclusive cyclic) matching starts

    // Per-leg negation windows (indices into unified posing cycle). Size NUM_LEGS
    int negation_phase_start[NUM_LEGS] = {0};
    int negation_phase_end[NUM_LEGS] = {0};
    double negation_transition_ratio[NUM_LEGS] = {0.0}; //< 0 => cambio inmediato, >0 suaviza transición

    // Auto-pose amplitudes por fase (vector length = number of phases)
    std::vector<double> roll_amplitudes;    //< rad
    std::vector<double> pitch_amplitudes;   //< rad
    std::vector<double> yaw_amplitudes;     //< rad
    std::vector<double> x_amplitudes;       //< mm
    std::vector<double> y_amplitudes;       //< mm
    std::vector<double> z_amplitudes;       //< mm
    std::vector<double> gravity_amplitudes; //< unitless / factor

    // Metadata
    std::string gait_name; //< Nombre del gait al que pertenece esta configuración

    // Application threshold: minimum absolute displacement magnitude (mm) to apply
    // the computed auto-pose offset to avoid micro jitter due to noise / phase edge blending.
    double apply_threshold_mm = 0.5; //< Default matches legacy heuristic
};

#endif // BODY_POSE_CONFIG_H