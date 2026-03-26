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
    double x; /**< X position relative to body center (millimeters). */
    double y; /**< Y position relative to body center (millimeters). */
    double z; /**< Z position relative to body center (millimeters). */
};

/**
 * @brief Standing pose joint configuration (OpenSHC equivalent)
 * In OpenSHC, standing pose is configured, not calculated from kinematics
 */
struct StandingPoseJoints {
    double coxa;  /**< Coxa joint angle in radians. */
    double femur; /**< Femur joint angle in radians. */
    double tibia; /**< Tibia joint angle in radians. */
};

/**
 * @brief Complete body pose configuration for the hexapod robot
 * Equivalent to OpenSHC's stance positioning and body pose control system
 */
struct BodyPoseConfiguration {
    BodyPoseConfiguration(const Parameters &p) {
        /** Set defaults to silence static analysis uninitialized warnings. */
        start_up_sequence = false;
        time_to_start = 0.0;
        /** Default body clearance. */
        body_clearance = p.standing_height;
        swing_height = p.standing_height * BODY_POSE_DEFAULT_SWING_HEIGHT_FACTOR;
        max_translation = {0.0, 0.0, 0.0};
        max_rotation = {0.0, 0.0, 0.0};
        max_translation_velocity = 0.0;
        max_rotation_velocity = 0.0;
        manual_posing_enabled = true;
        inclination_posing_enabled = false;
        imu_posing_enabled = false;
        auto_posing_enabled = false;
        /** Zero-initialize stance and pose arrays. */
        for (auto &ls : leg_stance_positions) {
            ls = {0.0, 0.0, 0.0};
        }
        for (auto &sj : standing_pose_joints) {
            sj = {0.0, 0.0, 0.0};
        }
        standing_horizontal_reach = 0.0;
    }
    /** OpenSHC-equivalent stance positions. */
    std::array<LegStancePosition, NUM_LEGS> leg_stance_positions;

    /** OpenSHC-equivalent standing pose (configured, not calculated). */
    std::array<StandingPoseJoints, NUM_LEGS> standing_pose_joints;

    /**
     * @brief Horizontal reach beyond hexagon radius (body center to foot).
     *
     * Computed from the configured standing pose joints (coxa pivot projection):
     * standing_horizontal_reach = coxa_length + femur_length * cos(femur_angle_standing)
     * (tibia vertical => no horizontal component). Used for stance and walkspace sizing.
     */
    double standing_horizontal_reach;

    bool start_up_sequence; /**< Allow startup and shutdown sequences. */
    double time_to_start;   /**< Time to complete a direct startup. */

    /** OpenSHC-equivalent body clearance and swing parameters. */
    double body_clearance; /**< Requested body height above ground (mm). */
    double swing_height;   /**< Swing trajectory vertical displacement above default (mm). */

    /** OpenSHC-equivalent pose limits. */
    struct {
        double x; /**< Maximum X translation (millimeters). */
        double y; /**< Maximum Y translation (millimeters). */
        double z; /**< Maximum Z translation (millimeters). */
    } max_translation;

    struct {
        double roll;  /**< Maximum roll rotation (radians). */
        double pitch; /**< Maximum pitch rotation (radians). */
        double yaw;   /**< Maximum yaw rotation (radians). */
    } max_rotation;

    /** OpenSHC-equivalent velocity limits. */
    double max_translation_velocity; /**< Maximum translation velocity (mm/s). */
    double max_rotation_velocity;    /**< Maximum rotation velocity (rad/s). */

    /** OpenSHC-equivalent pose control flags. */
    bool manual_posing_enabled;      /**< Enable manual pose contribution. */
    bool inclination_posing_enabled; /**< Enable inclination CoG correction. */
    bool imu_posing_enabled;         /**< Enable IMU PID pose correction. */
    bool auto_posing_enabled;        /**< Enable auto-pose contribution. */

    /** OpenSHC-equivalent per-leg phase map for auto-pose offsetting. */
    std::map<int, int> offset_multiplier;
};

/**
 * @brief Auto-pose configuration for tripod gait (OpenSHC equivalent)
 * Based on OpenSHC's auto_pose.yaml configuration structure
 */
struct AutoPoseConfiguration {
    bool enabled = false;         /**< Enable auto-pose during gait. */
    double pose_frequency = -1.0; /**< Pose frequency (-1.0 = sync with gait cycle). */
    int pose_phase_length = 0;    /**< Base phase length from YAML when pose_frequency != -1. */

    /** Phase segmentation ordered as in auto_pose.yaml for the active gait. */
    std::vector<int> pose_phase_starts; /**< Start indices (inclusive). */
    std::vector<int> pose_phase_ends;   /**< End indices (exclusive, cyclic), matching starts. */

    /** Per-leg negation windows (indices into unified posing cycle). Size NUM_LEGS. */
    int negation_phase_start[NUM_LEGS] = {0};
    int negation_phase_end[NUM_LEGS] = {0};
    double negation_transition_ratio[NUM_LEGS] = {0.0}; /**< 0 = immediate change, >0 smooths transition. */

    /** Auto-pose amplitudes per phase (vector length = number of phases). */
    std::vector<double> roll_amplitudes;    /**< Radians. */
    std::vector<double> pitch_amplitudes;   /**< Radians. */
    std::vector<double> yaw_amplitudes;     /**< Radians. */
    std::vector<double> x_amplitudes;       /**< Millimeters. */
    std::vector<double> y_amplitudes;       /**< Millimeters. */
    std::vector<double> z_amplitudes;       /**< Millimeters. */
    std::vector<double> gravity_amplitudes; /**< Unitless factor. */

    /** Metadata. */
    std::string gait_name; /**< Gait name associated with this configuration. */

    /**
     * @brief Minimum displacement magnitude (mm) to apply auto-pose offsets.
     *
     * Prevents micro jitter from noise or phase edge blending.
     */
    double apply_threshold_mm = 0.5; /**< Default threshold for jitter suppression. */
};

#endif /**< BODY_POSE_CONFIG_H */