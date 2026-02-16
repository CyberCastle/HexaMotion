#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#include "admittance_controller.h"
#include "body_pose_controller.h"
#include "robot_model.h"
#include "state_controller_context.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <memory>

/** Maximum number of legs that can be manually controlled simultaneously. */
#define MAX_MANUAL_LEGS 2
/** Joint transition time during pack/unpack sequences (seconds). */
#define PACK_TIME 2.0f
/** Progress value indicating completion. */
#define PROGRESS_COMPLETE 100
/** Joint position tolerance for state detection (radians, OpenSHC parity). */
#define JOINT_TOLERANCE 0.01
/** Throttle period for repeated messages. */
#define THROTTLE_PERIOD 1.0f

/** Designation for potential manual body posing input modes. */
enum PosingMode {
    POSING_NONE,       /**< No manual body posing. */
    POSING_X_Y,        /**< Manual body posing via x/y translation. */
    POSING_PITCH_ROLL, /**< Manual body posing via pitch/roll rotation. */
    POSING_Z_YAW,      /**< Manual body posing via z translation and yaw rotation. */
    POSING_EXTERNAL,   /**< Manual body posing from an external source. */
    POSING_MODE_COUNT, /**< Number of posing modes. */
};

// PosingState, PoseResetMode, and SequenceSelection are defined in gait_types.h
// (shared between BodyPoseController, StateController, and WalkController).

/** Designations for potential legs within the robot model (up to 6 legs for hexapod). */
enum LegDesignation {
    LEG_0,                 /**< 1st leg - front right most leg of the robot. */
    LEG_1,                 /**< 2nd leg - clockwise following leg 0 around the body. */
    LEG_2,                 /**< 3rd leg - middle right leg. */
    LEG_3,                 /**< 4th leg - rear right leg. */
    LEG_4,                 /**< 5th leg - rear left leg. */
    LEG_5,                 /**< 6th leg - middle left leg. */
    LEG_DESIGNATION_COUNT, /**< Number of leg designations. */
    LEG_UNDESIGNATED = -1, /**< Undesignated leg. */
};

/** State machine configuration parameters. */
struct StateMachineConfig {
    bool enable_startup_sequence = true; /**< Enable multi-step startup sequence (OpenSHC start_up_sequence). */
    double transition_timeout = 10.0f;   /**< Maximum time for state transitions (seconds). */
    double pack_unpack_time = 2.0f;      /**< Time for pack/unpack sequences (seconds). */
    bool enable_auto_posing = false;     /**< Enable automatic body posing. */
    bool enable_manual_posing = true;    /**< Enable manual body posing. */
    int max_manual_legs = 2;             /**< Maximum number of manually controlled legs. */
};

/**
 * @brief Equivalent hierarchical state machine to OpenSHC.
 *
 * Manages system, robot, walk and leg states as well as various
 * operational modes. The controller coordinates transitions and
 * interacts with the walk, pose and admittance controllers.
 */
class StateController {
  public:
    /**
     * @brief Constructor for the StateController.
     * @param context Reference to the locomotion orchestration context
     * @param config Configuration for the state machine
     */
    StateController(StateControllerContext &context, const StateMachineConfig &config = StateMachineConfig());

    /**
     * @brief Destructor for the StateController.
     */
    ~StateController();

    /**
     * @brief Initialize the state controller.
     * @param pose_config Pose configuration for the robot.
     * @return True if initialization successful
     */
    bool initialize(const BodyPoseConfiguration &body_pose_config);

    /**
     * @brief Main update loop for the state machine.
     * This should be called regularly from the main control loop.
     * @param time_delta Delta time since last update (seconds)
     */
    void update(double time_delta);

    /** State accessors. */

    /**
     * @brief Get the current system state.
     * @return Current system state
     */
    inline SystemState getSystemState() const { return current_system_state_; }

    /**
     * @brief Get the current robot state.
     * @return Current robot state
     */
    inline RobotState getRobotState() const { return current_robot_state_; }

    /**
     * @brief Get the current walk state.
     * @return Current walk state
     */
    inline WalkState getWalkState() const { return current_walk_state_; }

    /**
     * @brief Get the current posing mode.
     * @return Current posing mode
     */
    inline PosingMode getPosingMode() const { return current_posing_mode_; }

    /**
     * @brief Get the current pose reset mode.
     * @return Current pose reset mode
     */
    inline PoseResetMode getPoseResetMode() const { return current_pose_reset_mode_; }

    /**
     * @brief Check if the state machine is initialized.
     * @return True if initialized
     */
    inline bool isInitialized() const { return is_initialized_; }

    /**
     * @brief Check if the robot is ready for operation.
     * @return True if robot is in ROBOT_RUNNING state
     */
    inline bool isReadyForOperation() const { return current_robot_state_ == ROBOT_RUNNING; }

    /** State setters. */

    /**
     * @brief Request a system state transition.
     * @param new_state Desired system state
     * @return True if transition request accepted
     */
    bool requestSystemState(SystemState new_state);

    /**
     * @brief Request a robot state transition.
     * @param new_state Desired robot state
     * @return True if transition request accepted
     */
    bool requestRobotState(RobotState new_state);

    /**
     * @brief Notify the state controller that the robot has been placed in READY state externally.
     *
     * Called when LocomotionSystem::setStandingPose() succeeds so that the state machine
     * resolves ROBOT_UNKNOWN correctly without relying on heuristic body-position detection.
     * Also captures current joint angles as the ready-state reference.
     */
    void notifyRobotReady();

    /**
     * @brief Set the posing mode.
     * @param mode Desired posing mode
     * @return True if mode change successful
     */
    bool setPosingMode(PosingMode mode);

    /**
     * @brief Set the pose reset mode.
     * @param mode Desired pose reset mode
     * @return True if mode change successful
     */
    bool setPoseResetMode(PoseResetMode mode);

    /** Leg control. */

    /**
     * @brief Get the state of a specific leg.
     * @param leg_index Index of the leg (0-5)
     * @return Current leg state
     */
    LegState getLegState(int leg_index) const;

    /**
     * @brief Get the number of legs currently in manual mode.
     * @return Number of manually controlled legs
     */
    int getManualLegCount() const;

    /**
     * @brief Set the state of a specific leg.
     * @param leg_index Index of the leg (0-5)
     * @param state Desired leg state
     * @return True if leg state change successful
     */
    bool setLegState(int leg_index, LegState state);

    /**
     * @brief Request a leg state toggle between WALKING and MANUAL (OpenSHC equivalent).
     *
     * This initiates a gradual transition: the walk controller is stopped first,
     * then poseForLegManipulation drives the legs to appropriate poses before
     * the final state change. Called once per user request; the transition is
     * completed over subsequent update() ticks.
     *
     * @param leg_index Index of the leg to toggle (0-5)
     * @return True if toggle request was accepted
     */
    bool requestLegToggle(int leg_index);

    /** Velocity and pose control. */

    /**
     * @brief Set desired body velocity.
     * @param linear_velocity Linear velocity [x, y] in mm/s
     * @param angular_velocity Angular velocity in degrees/s
     */
    void setDesiredVelocity(const Eigen::Vector2d &linear_velocity, double angular_velocity);

    /**
     * @brief Set desired body pose.
     * @param position Desired position [x, y, z] in mm
     * @param orientation Desired orientation [roll, pitch, yaw] in degrees
     */
    void setDesiredPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /**
     * @brief Set desired tip velocity for manual leg control.
     * @param leg_index Index of the leg (0-5)
     * @param velocity Desired tip velocity [x, y, z] in mm/s
     */
    void setLegTipVelocity(int leg_index, const Eigen::Vector3d &velocity);

    /**
     * @brief Set desired tip pose for manual leg control.
     * @param leg_index Index of the leg (0-5)
     * @param position Desired tip position in mm
     */
    void setLegTipPose(int leg_index, const Point3D &position);

    /**
     * @brief Get desired tip pose for manual leg control.
     * @param leg_index Index of the leg (0-5)
     * @param out_position Output position if available
     * @return True if a valid pose is set for this leg
     */
    bool getLegTipPose(int leg_index, Point3D &out_position) const;

    /**
     * @brief Get desired tip velocity for a leg.
     * @param leg_index Index of the leg (0-5)
     * @return Desired tip velocity
     */
    Eigen::Vector3d getLegTipVelocity(int leg_index) const;

    /**
     * @brief Get up to two manual leg indices for manual control inputs.
     * @param primary_leg Output primary leg index (-1 if none)
     * @param secondary_leg Output secondary leg index (-1 if none)
     */
    void getManualLegIndices(int &primary_leg, int &secondary_leg) const;

    /**
     * @brief Update velocity control based on current mode.
     * This method is exposed for testing purposes.
     */
    void updateVelocityControl();

    /**
     * @brief Set desired body position for pose control.
     * @param position Desired body position (x, y, z)
     * @return True if position is valid and set
     */
    bool setDesiredBodyPosition(const Eigen::Vector3d &position);

    /**
     * @brief Set desired body orientation for pose control.
     * @param orientation Desired body orientation (roll, pitch, yaw in degrees)
     * @return True if orientation is valid and set
     */
    bool setDesiredBodyOrientation(const Eigen::Vector3d &orientation);

    /**
     * @brief Get current body position.
     * @return Current body position
     */
    inline Eigen::Vector3d getDesiredBodyPosition() const { return desired_body_position_; }

    /**
     * @brief Get current body orientation.
     * @return Current body orientation
     */
    inline Eigen::Vector3d getDesiredBodyOrientation() const { return desired_body_orientation_; }

    /** Gait control. */

    /**
     * @brief Change the current gait type.
     * @param gait Desired gait type
     * @return True if gait change successful
     */
    bool changeGait(GaitType gait);

    /** Status and diagnostics. */

    /**
     * @brief Check if the state machine has any errors.
     * @return True if there are errors
     */
    bool hasErrors() const;

    /**
     * @brief Check if the system is currently transitioning between states.
     * @return True if transitioning
     */
    bool isTransitioning() const;

    /**
     * @brief Get the last error message.
     * @return Error message string
     */
    String getLastErrorMessage() const;

    /**
     * @brief Get diagnostic information as a formatted string.
     * @return Diagnostic string
     */
    String getDiagnosticInfo() const;

    /**
     * @brief Emergency stop - immediately halt all motion and transition to safe state.
     */
    void emergencyStop();

    /**
     * @brief Reset the state machine to initial state.
     */
    void reset();

    /**
     * @brief Clear error state.
     */
    void clearError();

  private:
    /** Private members. */

    StateControllerContext &context_;
    StateMachineConfig config_;

    /** Current states. */
    SystemState current_system_state_;
    RobotState current_robot_state_;
    WalkState current_walk_state_;
    PosingMode current_posing_mode_;
    PoseResetMode current_pose_reset_mode_;

    /** Desired states for transitions. */
    SystemState desired_system_state_;
    RobotState desired_robot_state_;

    /** Leg states. */
    LegState leg_states_[NUM_LEGS];
    int manual_leg_count_;

    /** Leg state toggle tracking (OpenSHC equivalent). */
    int toggle_leg_index_;          /**< Index of leg currently toggling (-1 if none). */
    bool toggle_leg_state_pending_; /**< Flag indicating a leg state toggle is in progress. */

    /** Transition management. */
    bool is_transitioning_;

    /** Control inputs. */
    Eigen::Vector2d desired_linear_velocity_;
    double desired_angular_velocity_;
    Eigen::Vector3d desired_body_position_;
    Eigen::Vector3d desired_body_orientation_;
    Eigen::Vector3d leg_tip_velocities_[NUM_LEGS];
    Point3D leg_tip_poses_[NUM_LEGS];
    bool leg_tip_pose_valid_[NUM_LEGS];

    /** Timing. */
    unsigned long last_update_time_;
    double time_delta_; /**< Measured time delta for diagnostics (nominal params.time_delta otherwise). */

    /** Error handling. */
    bool has_error_;
    String last_error_message_;
    /** Sequence execution state to avoid static locals. */
    int startup_step_;
    bool startup_transition_initialized_;
    int startup_transition_step_count_;
    int shutdown_step_;
    bool shutdown_transition_initialized_;
    int shutdown_transition_step_count_;
    bool executing_pack_transition_;

    /** Initialization flag. */
    bool is_initialized_;

    /** Pose control. */
    std::unique_ptr<BodyPoseController> body_pose_controller_;
    /** Precomputed joint angle targets for packed and ready states. */
    JointAngles packed_target_angles_[NUM_LEGS];
    JointAngles ready_target_angles_[NUM_LEGS];

    /** Private methods. */

    /**
     * @brief Update the main state machine logic.
     */
    void updateStateMachine();

    /**
     * @brief Handle system state transitions.
     */
    void handleSystemStateTransition();

    /**
     * @brief Handle robot state transitions.
     */
    void handleRobotStateTransition();

    /**
     * @brief Handle walk state updates.
     */
    void updateWalkState();

    /**
     * @brief Handle leg state transitions.
     */
    void handleLegStateTransitions();

    /**
     * @brief Update pose control based on current mode.
     */
    void updatePoseControl();

    /**
     * @brief Execute startup sequence.
     * @return Progress percentage (0-100)
     */
    int executeStartupSequence();

    /**
     * @brief Execute shutdown sequence.
     * @return Progress percentage (0-100)
     */
    int executeShutdownSequence();

    /**
     * @brief Execute pack sequence.
     * @return Progress percentage (0-100)
     */
    int executePackSequence();

    /**
     * @brief Execute unpack sequence.
     * @return Progress percentage (0-100)
     */
    int executeUnpackSequence();

    /**
     * @brief Check if robot is in packed position.
     * @return True if all joints are in packed positions
     */
    bool isRobotPacked() const;

    /**
     * @brief Check if robot is in ready position.
     * @return True if all joints are in ready positions
     */
    bool isRobotReady() const;

    /**
     * @brief Validate state transition request.
     * @param current_state Current state
     * @param desired_state Desired state
     * @return True if transition is valid
     */
    bool isValidStateTransition(RobotState current_state, RobotState desired_state) const;

    /**
     * @brief Set error state with message.
     * @param message Error message
     */
    void setError(const String &message);

    /**
     * @brief Log debug information.
     * @param message Debug message
     */
    void logDebug(const String &message);

    /**
     * @brief Log error information.
     * @param message Error message
     */
    void logError(const String &message);

    /**
     * @brief Apply pose reset based on current reset mode.
     */
    void applyPoseReset();
};

#endif /**< STATE_CONTROLLER_H */
