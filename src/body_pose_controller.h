#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

#include "auto_poser.h"
#include "body_pose_config.h"
#include "leg.h"
#include "leg_poser.h"
#include "robot_model.h"
#include <ArduinoEigen.h>
#include <algorithm>
#include <memory>
#include <vector>

/**
 * @brief BodyPoseController — 1:1 port of OpenSHC PoseController.
 *
 * Handles body pose composition, leg coordination, and startup/shutdown
 * sequences for the hexapod robot.  Only functional equivalents of
 * OpenSHC's PoseController live here; convenience/utility helpers
 * (setStandingPose, setBodyPose, etc.) live in LocomotionSystem.
 */
class BodyPoseController {
  public:
    // PosingState, SequenceSelection, and PoseResetMode enums are defined in locomotion_types.h
    // (included via robot_model.h) as shared free enums, matching OpenSHC's convention.

    /**
     * @brief Constructor
     * @param m Reference to the robot model
     * @param config Body pose configuration
     */
    BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config);

    /** @brief Destructor */
    ~BodyPoseController();

    // ── Initialisation (OpenSHC init()) ─────────────────────────────────────

    /**
     * @brief Initialize leg posers for all legs.
     * @param legs Array of Leg objects
     */
    void initializeLegPosers(Leg legs[NUM_LEGS]);

    /**
     * @brief Get leg poser for a specific leg.
     * @param leg_index Index of the leg
     * @return Pointer to LegPoser, or nullptr if invalid
     */
    LegPoser *getLegPoser(int leg_index) const;

    // ── Sequence execution ──────────────────────────────────────────────────

    /**
     * @brief Execute startup/shutdown sequence (OpenSHC executeSequence).
     * @param sequence START_UP or SHUT_DOWN
     * @param legs Array of Leg objects
     * @return Progress (0–100), PROGRESS_COMPLETE when finished, -1 while learning
     */
    int executeSequence(const SequenceSelection &sequence, Leg legs[NUM_LEGS]);

    /**
     * @brief Direct startup via joint-space interpolation (OpenSHC directStartup).
     * @param legs Array of legs
     * @return Progress percentage (0–100)
     */
    int directStartup(Leg legs[NUM_LEGS]);

    /**
     * @brief Step legs to new stance positions (OpenSHC stepToNewStance).
     * @return Progress percentage (0–100)
     */
    int stepToNewStance();

    /**
     * @brief Pose for manual leg manipulation (OpenSHC poseForLegManipulation).
     * @return Progress percentage (0–100)
     */
    int poseForLegManipulation();

    /**
     * @brief Pack all legs to configured packed pose.
     * @param time_to_pack Transition duration in seconds
     * @param legs Array of legs
     * @return Progress percentage (0–100)
     */
    int packLegs(double time_to_pack, Leg legs[NUM_LEGS]);

    /**
     * @brief Unpack all legs from packed pose.
     * @param time_to_unpack Transition duration in seconds
     * @param legs Array of legs
     * @return Progress percentage (0–100)
     */
    int unpackLegs(double time_to_unpack, Leg legs[NUM_LEGS]);

    /**
     * @brief Transition all legs to desired joint configuration (OpenSHC).
     * @param transition_time Duration in seconds
     * @param legs Array of legs
     * @return Minimum progress across legs
     */
    int transitionConfiguration(double transition_time, Leg legs[NUM_LEGS]);

    // ── Pose update pipeline (OpenSHC updateCurrentPose) ────────────────────

    /**
     * @brief Compose body pose from all contributors (OpenSHC updateCurrentPose).
     * @param robot_state Current robot state integer
     * @param legs Array of legs
     */
    void updateCurrentPose(int robot_state, Leg legs[NUM_LEGS]);

    /** @brief Update stance tip references per leg state (OpenSHC updateStance). */
    void updateStance(Leg legs[NUM_LEGS]);

    /** @brief Update manual pose integration from velocity inputs. */
    void updateManualPose();

    /** @brief Update inclination pose (CoG shift from IMU incline). */
    void updateInclinationPose();

    /** @brief Update walk plane pose from stance legs (OpenSHC updateWalkPlanePose). */
    void updateWalkPlanePose(Leg legs[NUM_LEGS]);

    /** @brief Update IMU PID-based body rotation correction (OpenSHC updateIMUPose). */
    void updateIMUPosePID();

    /**
     * @brief Calculate default zero-moment balance pose (OpenSHC calculateDefaultPose).
     * @param legs Array of legs
     */
    void calculateDefaultPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Update IK error compensation pose (OpenSHC updateIKErrorPose).
     * @param legs Array of legs
     */
    void updateIKErrorPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Update auto-pose (OpenSHC updateAutoPose).
     * @param legs Array of legs
     * @return true on success
     */
    bool updateAutoPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Overload that sets explicit gait phase before updating.
     * @param gait_phase Explicit phase index
     * @param legs Array of legs
     */
    bool updateAutoPose(int gait_phase, Leg legs[NUM_LEGS]);

    // ── Input setters ───────────────────────────────────────────────────────

    /**
     * @brief Set manual pose velocity input (simple setter, OpenSHC parity).
     * @param translation_velocity Normalised translation velocity (-1..1)
     * @param rotation_velocity Normalised rotation velocity (-1..1)
     */
    void setManualPoseInput(const Eigen::Vector3d &translation_velocity,
                            const Eigen::Vector3d &rotation_velocity);

    /** @brief Set manual pose reset mode. */
    void setPoseResetMode(PoseResetMode mode) { pose_reset_mode_ = mode; }

    /** @brief Get manual pose reset mode. */
    PoseResetMode getPoseResetMode() const { return pose_reset_mode_; }

    /** @brief Provide latest IMU data for pose corrections. */
    void setIMUData(const IMUData &imu_data);

    // ── Enable/disable individual pose contributors ─────────────────────────

    void setManualPoseEnabled(bool enabled) { manual_pose_enabled_ = enabled; }
    void setIMUPoseEnabled(bool enabled) { imu_pose_enabled_ = enabled; }
    void setInclinationPoseEnabled(bool enabled) { inclination_pose_enabled_ = enabled; }
    void setAutoPoseEnabled(bool enabled) { auto_pose_enabled = enabled; }
    bool isAutoPoseEnabled() const { return auto_pose_enabled; }
    void setIKErrorPoseEnabled(bool enabled) { ik_error_pose_enabled_ = enabled; }
    void setDefaultPoseEnabled(bool enabled) { default_pose_enabled_ = enabled; }

    // ── Gait type ───────────────────────────────────────────────────────────

    void setCurrentGaitType(GaitType gait_type) {
        current_gait_type_ = gait_type;
        resetSequenceStates();
    }
    GaitType getCurrentGaitType() const { return current_gait_type_; }

    /**
     * @brief Set gait phase parameters needed for auto-pose gait-synced calculation.
     *
     * Called when gait changes or on initialisation. OpenSHC accesses these via
     * params_.stance_phase / params_.swing_phase / params_.phase_offset directly.
     *
     * @param stance_phase Stance phase ratio (iterations)
     * @param swing_phase Swing phase ratio (iterations)
     * @param phase_offset Phase offset between legs (iterations)
     */
    void setGaitPhaseParams(int stance_phase, int swing_phase, int phase_offset) {
        gait_stance_phase_ = stance_phase;
        gait_swing_phase_ = swing_phase;
        gait_phase_offset_ = phase_offset;
    }

    /**
     * @brief Refresh auto-pose parameters after gait-related settings change.
     */
    void refreshAutoPoseParameters() { setAutoPoseParams(); }

    /**
     * @brief Set current walk state for auto-pose lifecycle coordination.
     *
     * OpenSHC reads this from auto_pose_reference_leg_->getLegStepper()->getWalkState().
     * In HexaMotion this is provided externally since BPC has no direct access to WalkController.
     *
     * @param walk_state Current WalkState from WalkController
     */
    void setCurrentWalkState(WalkState walk_state) { current_walk_state_ = walk_state; }

    /**
     * @brief Set whether body velocity is zero (for auto-pose STOP_POSING detection).
     *
     * OpenSHC checks stride_vector.norm() == 0.
     *
     * @param zero True if body velocity is zero
     */
    void setBodyVelocityZero(bool zero) { body_velocity_zero_ = zero; }

    // ── Sequence state management ───────────────────────────────────────────

    /** @brief Reset all startup/shutdown sequence states. */
    void resetSequenceStates() {
        first_sequence_execution_ = true;
        executing_transition_ = false;
        transition_step_ = 0;
        transition_step_count_ = 0;
        horizontal_transition_complete_ = false;
        vertical_transition_complete_ = false;
        set_target_ = true;
        proximity_alert_ = false;
        legs_completed_step_ = 0;
        current_group_ = 0;
        reset_transition_sequence_ = true;
    }

    /** @brief Force re-learning on next startup sequence. */
    void resetStartupSequence() { first_sequence_execution_ = true; }

    /** @brief Current startup transition step (0=horizontal,1=vertical,...). */
    int getStartupPhase() const { return transition_step_; }

    // ── Accessors ───────────────────────────────────────────────────────────

    const BodyPoseConfiguration &getBodyPoseConfig() const { return body_pose_config; }
    void setBodyPoseConfig(const BodyPoseConfiguration &config) { body_pose_config = config; }

    const Pose &getCurrentBodyPose() const { return body_pose_current_; }
    Pose getDefaultBodyPose() const { return body_pose_current_; }

    const AutoPoseConfiguration &getAutoPoseConfig() const { return auto_pose_config; }
    void setAutoPoseConfig(const AutoPoseConfiguration &config) { auto_pose_config = config; }

    PosingState getAutoPoseState() const { return auto_posing_state_; }
    Pose getAutoPose() const { return auto_pose_; }

    int getPhaseLength() const { return pose_phase_length_; }
    int getNormaliser() const { return normaliser_; }
    double getPoseFrequency() const { return pose_frequency_; }
    void setPhaseLength(const int &phase_length) { pose_phase_length_ = std::max(1, phase_length); }
    void setNormaliser(const int &normaliser) { normaliser_ = std::max(1, normaliser); }

    Pose getWalkPlanePose() const;
    void setWalkPlanePose(const Pose &pose);

    Eigen::Vector3d getRotationAbsementError() const { return rotation_absement_error_; }
    Eigen::Vector3d getRotationPositionError() const { return rotation_position_error_; }
    Eigen::Vector3d getRotationVelocityError() const { return rotation_velocity_error_; }

    /**
     * @brief Check if legs are bearing load based on average tip height.
     * @param legs Array of legs
     * @return True if estimated body height indicates load-bearing stance
     */
    bool legsBearingLoad(const Leg legs[NUM_LEGS]) const;

    /** @brief Reset all pose contributors to identity (OpenSHC resetAllPosing). */
    void resetAllPosing() {
        manual_pose_ = Pose::Identity();
        imu_pose_ = Pose::Identity();
        inclination_pose_ = Pose::Identity();
        default_pose_ = Pose::Identity();
        ik_error_pose_ = Pose::Identity();
        walk_plane_pose_ = Pose::Identity();
        origin_walk_plane_pose_ = walk_plane_pose_;
        rotation_absement_error_ = Eigen::Vector3d::Zero();
        rotation_position_error_ = Eigen::Vector3d::Zero();
        rotation_velocity_error_ = Eigen::Vector3d::Zero();
    }

  private:
    RobotModel &model;
    BodyPoseConfiguration body_pose_config;
    AutoPoseConfiguration auto_pose_config;

    // Auto-pose state
    Pose auto_pose_ = Pose::Identity();
    std::vector<std::shared_ptr<AutoPoser>> auto_poser_container_;
    int auto_pose_reference_leg_ = 0;
    PosingState auto_posing_state_ = POSING_COMPLETE;
    int pose_phase_ = 0;
    double pose_frequency_ = -1.0;
    int pose_phase_length_ = 0;
    int normaliser_ = 1;
    bool auto_pose_enabled;

    // Leg posers
    class LegPoserImpl;
    LegPoserImpl *leg_posers_[NUM_LEGS];
    Leg *legs_ref_ = nullptr;

    // Gait type
    GaitType current_gait_type_;

    // Walk state for auto-pose lifecycle coordination (set externally)
    WalkState current_walk_state_ = WALK_STOPPED;
    bool body_velocity_zero_ = true;

    // Gait phase parameters for auto-pose gait-synced calculation (set externally)
    int gait_stance_phase_ = 0;
    int gait_swing_phase_ = 0;
    int gait_phase_offset_ = 0;

    // OpenSHC sequence execution state
    bool executing_transition_;
    int transition_step_;
    int transition_step_count_;
    bool set_target_;
    bool proximity_alert_;
    bool horizontal_transition_complete_;
    bool vertical_transition_complete_;
    bool first_sequence_execution_;
    bool reset_transition_sequence_;
    int legs_completed_step_;
    int current_group_;
    int pack_step_ = 0;

    // Walk plane pose (OpenSHC parity)
    Pose walk_plane_pose_;
    Pose origin_walk_plane_pose_;

    // Composed body pose
    Pose body_pose_current_ = Pose::Identity();

    // Individual pose contributors (OpenSHC parity)
    Pose manual_pose_ = Pose::Identity();
    Pose imu_pose_ = Pose::Identity();
    Pose inclination_pose_ = Pose::Identity();
    Pose default_pose_ = Pose::Identity();
    Pose ik_error_pose_ = Pose::Identity();

    // IMU data
    IMUData imu_data_{};
    bool imu_data_valid_ = false;

    // Enable flags
    bool manual_pose_enabled_ = false;
    bool imu_pose_enabled_ = false;
    bool inclination_pose_enabled_ = false;
    bool ik_error_pose_enabled_ = false;
    bool default_pose_enabled_ = false;
    bool recalculate_default_pose_ = true;

    // Manual pose input state
    PoseResetMode pose_reset_mode_ = NO_RESET;
    Eigen::Vector3d translation_velocity_input_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_velocity_input_ = Eigen::Vector3d::Zero();

    // PID state for IMU posing (OpenSHC parity)
    Eigen::Vector3d rotation_absement_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_position_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_velocity_error_ = Eigen::Vector3d::Zero();

    // ── Private methods ─────────────────────────────────────────────────────

    /** @brief Internal sequence execution (OpenSHC). */
    int executeSequenceInternal(const SequenceSelection &sequence, Leg legs[NUM_LEGS]);

    /** @brief Configure auto-poser objects from current configuration. */
    void setAutoPoseParams();

    /** @brief Calculate walk plane normal from stance legs. */
    Point3D calculateWalkPlaneNormal(Leg legs[NUM_LEGS]) const;

    /** @brief Calculate walk plane height from stance legs. */
    double calculateWalkPlaneHeight(Leg legs[NUM_LEGS]) const;

    // Tripod gait leg groupings (OpenSHC compatible)
    static constexpr int tripod_leg_groups[2][3] = {{0, 2, 4}, {1, 3, 5}};
};

#endif // BODY_POSE_CONTROLLER_H
