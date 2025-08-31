#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

#include "body_pose_config.h"
#include "imu_auto_pose.h"
#include "leg.h"
#include "leg_poser.h"
#include "robot_model.h"
#include "s_curve_profile.h" // SCurveProfile definition for initial standing transition
#include <ArduinoEigen.h>
#include <memory>
#include <vector>

// Forward declarations
class IServoInterface;

/**
 * @brief BodyPoseController class for HexaMotion
 *
 * This class handles body pose control and leg coordination for the hexapod robot.
 * It provides functionality for setting body poses, managing leg positions,
 * and coordinating stance transitions.
 *
 * Adapted from OpenSHC's PoseController but simplified for HexaMotion architecture.
 * No progress tracking - that's handled by LocomotionSystem.
 */
class BodyPoseController {
  public:
    /**
     * @brief Constructor
     * @param m Reference to the robot model
     * @param config Body pose configuration
     */
    BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config);

    /**
     * @brief Destructor
     */
    ~BodyPoseController();

    /**
     * @brief Initialize leg posers for all legs
     * @param legs Array of Leg objects
     */
    void initializeLegPosers(Leg legs[NUM_LEGS]);

    /**
     * @brief Get leg poser for a specific leg
     * @param leg_index Index of the leg
     * @return Pointer to LegPoser, or nullptr if invalid
     */
    LegPoser *getLegPoser(int leg_index) const;

    /**
     * @brief Set body pose with position and orientation
     * @param position Body position in world coordinates
     * @param orientation Body orientation in euler angles (degrees)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    // orientation now expected in radians (roll,pitch,yaw)
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                     Leg legs[NUM_LEGS]);

    /**
     * @brief Set body pose with quaternion orientation
     * @param position Body position in world coordinates
     * @param quaternion Body orientation as quaternion
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                               Leg legs[NUM_LEGS]);

    /**
     * @brief Set individual leg position
     * @param leg_index Index of the leg to move
     * @param position Target position in world coordinates
     * @param legs Array of Leg objects
     * @return true if successful, false otherwise
     */
    bool setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]);

    /**
     * @brief Calculate body pose from configuration
     * @param height_offset Height offset from default pose
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]);

    /**
     * @brief Initialize default pose for all legs
     * @param legs Array of Leg objects to initialize
     */
    void initializeDefaultPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Set standing pose for all legs
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool setStandingPose(Leg legs[NUM_LEGS]);

    /**
     * @brief Access standing pose joint configuration (radians) for a leg.
     * @param leg_index Leg index (0..NUM_LEGS-1)
     * @return StandingPoseJoints structure (coxa,femur,tibia) in radians.
     */
    StandingPoseJoints getStandingPoseJoints(int leg_index) const;

    /**
     * @brief Interpolate between two poses
     * @param start_pos Starting position
     * @param start_quat Starting quaternion
     * @param end_pos Ending position
     * @param end_quat Ending quaternion
     * @param t Interpolation parameter (0.0 to 1.0)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
                         const Eigen::Vector3d &end_pos, const Eigen::Vector4d &end_quat,
                         double t, Leg legs[NUM_LEGS]);

    /**
     * @brief Calculate body position based on current leg positions
     * @param legs Array of Leg objects
     * @return Current body position as Vector3d
     */
    Eigen::Vector3d calculateBodyPosition(Leg legs[NUM_LEGS]) const;

    /**
     * @brief Check if body pose is within configured limits
     * @param position Body position to check
     * @param orientation Body orientation to check
     * @return true if within limits, false otherwise
     */
    bool checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /**
     * @brief Execute startup sequence (READY -> RUNNING transition)
     * @param legs Array of Leg objects to update
     * @return true if sequence is complete, false if still in progress
     */
    bool executeStartupSequence(Leg legs[NUM_LEGS]);

    /**
     * @brief Execute shutdown sequence (RUNNING -> READY transition)
     * @param legs Array of Leg objects to update
     * @return true if sequence is complete, false if still in progress
     */
    bool executeShutdownSequence(Leg legs[NUM_LEGS]);

    /**
     * @brief Execute pack sequence (OpenSHC equivalent)
     * Moves legs to packed configuration for storage/transport
     * @param legs Array of Leg objects to update
     * @param time_to_pack Time to complete packing sequence
     * @return Progress percentage (0-100), 100 indicates completion
     */
    int packLegs(Leg legs[NUM_LEGS], double time_to_pack);

    /**
     * @brief Execute unpack sequence (OpenSHC equivalent)
     * Moves legs from packed to ready configuration
     * @param legs Array of Leg objects to update
     * @param time_to_unpack Time to complete unpacking sequence
     * @return Progress percentage (0-100), 100 indicates completion
     */
    int unpackLegs(Leg legs[NUM_LEGS], double time_to_unpack);

    /**
     * @brief Pose for leg manipulation (OpenSHC equivalent)
     * Generates poses for manual leg manipulation while maintaining stability
     * @param legs Array of Leg objects to update
     * @return Progress percentage (0-100), 100 indicates completion
     */
    int poseForLegManipulation(Leg legs[NUM_LEGS]);

    /**
     * @brief Execute sequence with OpenSHC-style alternating transitions
     * This is the main sequence execution method that handles complex startup/shutdown
     * @param sequence_type Type of sequence (startup or shutdown)
     * @param legs Array of Leg objects to update
     * @return Progress percentage (0-100), 100 indicates completion
     */
    int executeSequence(const std::string &sequence_type, Leg legs[NUM_LEGS]);

    /**
     * @brief Update auto-pose during gait execution (OpenSHC equivalent)
     * @param gait_phase Current gait phase (0.0 to 1.0)
     * @param legs Array of Leg objects to update
     * @return true if successful, false otherwise
     */
    bool updateAutoPose(double gait_phase, Leg legs[NUM_LEGS]);

    /**
     * @brief Tripod leg coordination for stance transition (tripod gait only)
     * @param legs Array of Leg objects to update
     * @param step_height Height for leg lifting during transition
     * @param step_time Time for the step transition
     * @return true if transition is complete, false if still in progress
     */
    bool stepToNewStance(Leg legs[NUM_LEGS], double step_height, double step_time);

    /**
     * @brief Set current gait type for startup sequence selection
     * @param gait_type The GaitType enum value
     */
    void setCurrentGaitType(GaitType gait_type) {
        current_gait_type_ = gait_type;
        // Reset startup sequences for new gait
        resetSequenceStates();
    }

    /**
     * @brief Get current gait type
     * @return Current gait type as GaitType enum
     */
    GaitType getCurrentGaitType() const { return current_gait_type_; }

    /**
     * @brief Reset all startup/shutdown sequence states
     */
    void resetSequenceStates() {
        step_to_new_stance_current_group = 0;
        step_to_new_stance_sequence_generated = false;
        shutdown_sequence_initialized = false;

        // Extended: also reset OpenSHC-style startup transition state
        first_sequence_execution_ = true;
        executing_transition_ = false;
        transition_step_ = 0;
        transition_step_count_ = 0;
        horizontal_transition_complete_ = false;
        vertical_transition_complete_ = false;
        set_target_ = false;
        proximity_alert_ = false;
        legs_completed_step_ = 0;
        current_group_ = 0;
        pack_step_ = 0;
    }

    // Explicit public reset for startup learning cycle
    void resetStartupSequence() { first_sequence_execution_ = true; }

    // Accessor for current startup phase (0=horizontal,1=vertical,2=complete)
    int getStartupPhase() const { return transition_step_; }

    // Return startup sequence progress (0-100). Implemented in cpp to avoid incomplete type access.
    int getStartupProgressPercent() const;

    // Compatibility methods for existing tests
    const BodyPoseConfiguration &getBodyPoseConfig() const { return body_pose_config; }
    void setBodyPoseConfig(const BodyPoseConfiguration &config) { body_pose_config = config; }
    void configureSmoothTrajectory(bool use_current_positions, double interpolation_speed = 0.1, uint8_t max_steps = 20);

    // Auto-pose configuration accessors
    const AutoPoseConfiguration &getAutoPoseConfig() const { return auto_pose_config; }
    void setAutoPoseConfig(const AutoPoseConfiguration &config) { auto_pose_config = config; }

    // Auto-pose state control
    bool isAutoPoseEnabled() const { return auto_pose_enabled; }
    void setAutoPoseEnabled(bool enabled) { auto_pose_enabled = enabled; }

    // Smooth trajectory methods
    // orientation in radians
    bool setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                           Leg legs[NUM_LEGS], IServoInterface *servos = nullptr);
    bool setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                     Leg legs[NUM_LEGS]);
    bool setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                              Leg legs[NUM_LEGS]);
    bool getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]);
    bool initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                         const Eigen::Vector3d &target_orientation,
                                         Leg legs[NUM_LEGS], IServoInterface *servos);
    bool updateTrajectoryStep(Leg legs[NUM_LEGS]);
    void resetTrajectory();
    bool isTrajectoryInProgress() const { return trajectory_in_progress; }

    /**
     * @brief Begin jerk-limited initial standing pose transition (non-blocking).
     * @details Creates 7-segment S-curve motion profiles (in radians) from current leg joint angles to configured
     *          standing pose angles and arms the internal state machine. Does not publish servo commands.
     * @return true if a transition was started or immediately finished (already at target). Use isInitialStandingPoseActive() to know if ongoing.
     */
    bool beginInitialStandingPoseTransition(Leg legs[NUM_LEGS]);

    /**
     * @brief Advance the active initial standing pose transition by dt.
     * @param legs Leg array to update with new joint sample.
     * @param dt   Timestep (seconds).
     * @param out_positions Optional: joint positions (radians) sampled this step [leg][joint]; can be nullptr.
     * @param out_vel Optional: joint velocities (rad/s) sampled this step [leg][joint]; can be nullptr.
     * @param out_acc Optional: joint accelerations (rad/s^2) sampled this step [leg][joint]; can be nullptr.
     * @return true when transition completes at this call (final sample applied), false if still in progress or no active transition.
     */
    bool stepInitialStandingPoseTransition(Leg legs[NUM_LEGS], double dt,
                                           double out_positions[NUM_LEGS][DOF_PER_LEG] = nullptr,
                                           double out_vel[NUM_LEGS][DOF_PER_LEG] = nullptr,
                                           double out_acc[NUM_LEGS][DOF_PER_LEG] = nullptr);

    /** Active flag accessor for the initial standing pose transition */
    bool isInitialStandingPoseActive() const { return initial_standing_active_; }
    /** True if in alignment (coxa) phase */
    bool isInitialStandingAlignmentPhase() const { return initial_standing_active_ && initial_standing_phase_ == InitialStandingPhase::ALIGN; }
    /** Set tolerance (radians) used to validate coxa alignment */
    void setInitialStandingAlignmentTolerance(double radians) { initial_standing_align_tolerance_ = radians; }
    /** Returns true if all coxa joints are within alignment tolerance of target (valid during ALIGN phase) */
    bool isInitialStandingAligned(const Leg legs[NUM_LEGS]) const;
    /** Progress [0,1] of the initial standing pose transition (0 if inactive) */
    double getInitialStandingPoseProgress() const {
        return initial_standing_total_time_ > 0.0 ? math_utils::clamp(initial_standing_time_ / initial_standing_total_time_, 0.0, 1.0) : 0.0;
    }

    // Walk plane pose system (OpenSHC equivalent)
    void updateWalkPlanePose(Leg legs[NUM_LEGS]);
    Pose getWalkPlanePose() const;
    void setWalkPlanePose(const Pose &pose);
    void setWalkPlanePoseEnabled(bool enabled);
    bool isWalkPlanePoseEnabled() const;

    /**
     * @brief Update current body pose state (partial OpenSHC PoseController::updateCurrentPose equivalent).
     * @details Minimal adaptation that only forwards to auto-pose and walk plane pose update mechanisms.
     *          It intentionally omits IMU fusion, manual pose input handling, reset logic and stiffness
     *          modulation present in the full OpenSHC implementation. Gait phase is propagated so that
     *          phase-synchronised auto pose patterns can be evaluated consistently.
     * @param gait_phase Normalised gait phase in [0,1).
     * @param legs Array of Leg objects (needed for walk plane estimation and per-leg auto pose updates).
     */
    void updateCurrentPose(double gait_phase, Leg legs[NUM_LEGS]);

  private:
    RobotModel &model;                      //< Reference to robot model
    BodyPoseConfiguration body_pose_config; //< Body pose configuration
    AutoPoseConfiguration auto_pose_config; //< Auto-pose configuration

    // Leg posers for each leg
    class LegPoserImpl;
    LegPoserImpl *leg_posers_[NUM_LEGS];

    // Auto-pose state
    bool auto_pose_enabled;

    // Current gait type for startup sequence selection (OpenSHC compatibility)
    GaitType current_gait_type_;

    // Smooth trajectory support
    bool trajectory_in_progress;
    double trajectory_progress;
    int trajectory_step_count;
    Point3D trajectory_start_positions[NUM_LEGS];
    JointAngles trajectory_start_angles[NUM_LEGS];
    Point3D trajectory_target_positions[NUM_LEGS];
    JointAngles trajectory_target_angles[NUM_LEGS];

    // stepToNewStance state variables (moved from static to class level)
    int step_to_new_stance_current_group;
    bool step_to_new_stance_sequence_generated;

    // executeShutdownSequence state variables (moved from static to class level)
    bool shutdown_sequence_initialized;

    // OpenSHC sequence execution state variables
    bool executing_transition_;           //< Flag denoting if pose controller is executing a transition
    int transition_step_;                 //< Current transition step in sequence being executed
    int transition_step_count_;           //< Total number of transition steps in sequence
    bool set_target_;                     //< Flag if new tip target is to be calculated and set
    bool proximity_alert_;                //< Flag if joint has moved beyond limit proximity buffer
    bool horizontal_transition_complete_; //< Flag if horizontal transition completed without error
    bool vertical_transition_complete_;   //< Flag if vertical transition completed without error
    bool first_sequence_execution_;       //< Flag if controller has executed its first sequence
    bool reset_transition_sequence_;      //< Flag if saved transition sequence needs regeneration
    int legs_completed_step_;             //< Number of legs having completed required step in sequence
    int current_group_;                   //< Current leg group executing stepping maneuver
    int pack_step_;                       //< Current step in pack/unpack sequence

    // Startup sequence (OpenSHC-style) targets
    Point3D startup_horizontal_targets_[NUM_LEGS]; //< Intermediate horizontal (XY only) targets preserving initial Z
    Point3D startup_final_targets_[NUM_LEGS];      //< Final standing pose targets (full XYZ)

    // OpenSHC walk plane pose system with Bézier curves
    Pose walk_plane_pose_;              //< Current walk plane pose for body clearance maintenance
    bool walk_plane_pose_enabled;       //< Enable/disable walk plane pose system
    double walk_plane_update_threshold; //< Minimum change threshold for updates

    // Bézier curve control system for smooth transitions
    bool walk_plane_bezier_in_progress;              //< Whether a Bézier transition is in progress
    double walk_plane_bezier_time;                   //< Current time in Bézier transition
    double walk_plane_bezier_duration;               //< Duration of Bézier transition
    Point3D walk_plane_position_nodes[5];            //< Position control nodes for quartic Bézier
    Eigen::Quaterniond walk_plane_rotation_nodes[5]; //< Rotation control nodes for quartic Bézier

    // Initial standing pose S-curve transition state
    bool initial_standing_active_ = false;
    double initial_standing_time_ = 0.0;
    double initial_standing_total_time_ = 0.0;
    enum class InitialStandingPhase { ALIGN,
                                      LIFT };
    InitialStandingPhase initial_standing_phase_ = InitialStandingPhase::ALIGN;
    // For phase 2 lazy profile creation
    bool initial_standing_lift_profiles_created_ = false;
    double initial_standing_align_tolerance_ = 1.0 * DEGREES_TO_RADIANS_FACTOR; // default 1 degree
    // S-curve profiles per joint for initial standing pose
    SCurveProfile *initial_standing_profiles_[NUM_LEGS][DOF_PER_LEG] = {nullptr};

    // Walk plane pose helper methods
    Point3D calculateWalkPlaneNormal(Leg legs[NUM_LEGS]) const;
    double calculateWalkPlaneHeight(Leg legs[NUM_LEGS]) const;

    // Tripod gait leg groupings (OpenSHC compatible)
    // Group A: AR (0), CR (2), BL (4) - Anterior Right, Center Right, Back Left
    // Group B: BR (1), CL (3), AL (5) - Back Right, Center Left, Anterior Left
    static constexpr int tripod_leg_groups[2][3] = {{0, 2, 4}, {1, 3, 5}};
};

#endif // BODY_POSE_CONTROLLER_H
