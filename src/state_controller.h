#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

class LocomotionSystem;

#include "admittance_controller.h"
#include "body_pose_controller.h"
#include "locomotion_system.h"
#include "robot_model.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <memory>

// Maximum number of legs that can be manually controlled simultaneously
#define MAX_MANUAL_LEGS 2
#define PACK_TIME 2.0f        // Joint transition time during pack/unpack sequences (seconds)
#define PROGRESS_COMPLETE 100 // Progress value indicating completion
#define JOINT_TOLERANCE 0.1f  // Joint position tolerance for state detection
#define THROTTLE_PERIOD 1.0f  // Throttle period for repeated messages

//
// Designation for potential states of the robot.
//
enum RobotState {
    ROBOT_PACKED,       //< The robot is in a 'packed' state with all joints at defined 'packed' positions
    ROBOT_READY,        //< The robot is in a 'ready' state with all joints at defined 'unpacked' positions
    ROBOT_RUNNING,      //< The robot is in a 'running' state. This state is where all posing and walking occurs
    ROBOT_STATE_COUNT,  //< Misc enum defining number of Robot States
    ROBOT_UNKNOWN = -1, //< The robot is in an initial 'unknown' state, controller will estimate an actual state from it
    ROBOT_OFF = -2,     //< The robot is in 'off' state. Only used as alternative to 'running' state for direct start up
};

//
// Designation for potential manual body posing input modes.
//
enum PosingMode {
    POSING_NONE,       //< Mode allowing no manual body posing
    POSING_X_Y,        //< Mode allowing for manual posing of the robot body via x/y axis translation
    POSING_PITCH_ROLL, //< Mode allowing for manual posing of the robot body via pitch/roll rotation
    POSING_Z_YAW,      //< Mode allowing for manual posing of the robot body via z axis translation and yaw rotation
    POSING_EXTERNAL,   //< Mode allowing for posing input from external source
    POSING_MODE_COUNT, //< Misc enum defining number of Posing Modes
};

//
// Designation for potential cruise control modes.
//
enum CruiseControlMode {
    CRUISE_CONTROL_OFF,           //< Cruise control mode is off
    CRUISE_CONTROL_ON,            //< Cruise control mode is on
    CRUISE_CONTROL_MODE_COUNT,    //< Misc enum defining number of Cruise Control Modes
    CRUISE_CONTROL_EXTERNAL = -1, //< Cruise control mode is external
};

//
// Designation for potential posing states used in auto-posing.
//
enum PosingState {
    POSE_POSING,             //< State signifiying that auto-poser objects should start their posing cycle
    POSE_STOP_POSING,        //< State signifiying that auto-poser objects should end their posing cycle
    POSE_POSING_COMPLETE,    //< State signifiying that ALL auto-poser objects have completed their individual posing cycles
    POSE_POSING_STATE_COUNT, //< Misc enum defining number of Posing States
};

//
// Designation for potential manual pose reset input modes.
//
enum PoseResetMode {
    POSE_RESET_NONE,           //< State requesting no reseting of manual body posing is requested
    POSE_RESET_Z_AND_YAW,      //< State requesting manual body z-axis translation or yaw rotation posing to reset to zero
    POSE_RESET_X_AND_Y,        //< State requesting manual body x-axis or y-axis translation posing to reset to zero
    POSE_RESET_PITCH_AND_ROLL, //< State requesting manual body roll or pitch rotation posing to reset to zero
    POSE_RESET_ALL,            //< State requesting all manual body posing (in any axis) to reset to zero
    POSE_RESET_IMMEDIATE_ALL,  //< State forcing all manual body posing (in any axis) to reset to zero instantaneously
    POSE_RESET_MODE_COUNT,     //< Misc enum defining number of Pose-Reset States
};

//
// Designations for potential legs within the robot model - up to 6 legs for hexapod.
//
enum LegDesignation {
    LEG_0,                 //< 1st leg - The front right most leg of the robot
    LEG_1,                 //< 2nd leg - The leg following the 1st leg in a clockwise direction around the robot body
    LEG_2,                 //< 3rd leg - Middle right leg
    LEG_3,                 //< 4th leg - Rear right leg
    LEG_4,                 //< 5th leg - Rear left leg
    LEG_5,                 //< 6th leg - Middle left leg
    LEG_DESIGNATION_COUNT, //< Misc enum defining number of Leg Designations
    LEG_UNDESIGNATED = -1, //< Undesignated leg
};

//
// Sequence execution types for pose controller transitions.
//
enum SequenceType {
    SEQUENCE_START_UP,  //< Start-up sequence from ready to running
    SEQUENCE_SHUT_DOWN, //< Shut-down sequence from running to ready
    SEQUENCE_PACK,      //< Pack sequence to packed state
    SEQUENCE_UNPACK,    //< Unpack sequence from packed state
    SEQUENCE_COUNT,     //< Number of sequence types
};

//
// State machine configuration parameters.
//
struct StateMachineConfig {
    bool enable_startup_sequence = true;     //< Enable multi-step startup sequence
    bool enable_direct_startup = false;      //< Allow direct startup without sequences
    double transition_timeout = 10.0f;       //< Maximum time for state transitions (seconds)
    double pack_unpack_time = 2.0f;          //< Time for pack/unpack sequences (seconds)
    bool enable_auto_posing = false;         //< Enable automatic body posing
    bool enable_manual_posing = true;        //< Enable manual body posing
    bool enable_cruise_control = true;       //< Enable cruise control mode
    double cruise_control_time_limit = 0.0f; //< Time limit for cruise control (0 = unlimited)
    int max_manual_legs = 2;                 //< Maximum number of manually controlled legs
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
     * @param locomotion Reference to the main locomotion system
     * @param config Configuration for the state machine
     */
    StateController(LocomotionSystem &locomotion, const StateMachineConfig &config = StateMachineConfig());

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

    // ==============================
    // STATE ACCESSORS
    // ==============================

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
     * @brief Get the current cruise control mode.
     * @return Current cruise control mode
     */
    inline CruiseControlMode getCruiseControlMode() const { return current_cruise_control_mode_; }

    /**
     * @brief Get the current cruise control velocity.
     * @return Current cruise velocity vector (x, y, angular_z)
     */
    inline Eigen::Vector3d getCruiseVelocity() const { return cruise_velocity_; }

    /**
     * @brief Get the cruise control start time.
     * @return Start time in milliseconds
     */
    inline unsigned long getCruiseStartTime() const { return cruise_start_time_; }

    /**
     * @brief Get the cruise control remaining time.
     * @return Remaining time in seconds (0 if unlimited)
     */
    double getCruiseRemainingTime() const {
        if (cruise_end_time_ == 0 || current_cruise_control_mode_ != CruiseControlMode::CRUISE_CONTROL_ON) {
            return 0.0f; // Unlimited or not active
        }
        unsigned long current_time = millis();
        if (current_time >= cruise_end_time_) {
            return 0.0f; // Expired
        }
        return (cruise_end_time_ - current_time) / 1000.0f;
    }

    /**
     * @brief Check if cruise control is active and within time limit.
     * @return True if cruise control is currently active
     */
    bool isCruiseControlActive() const {
        if (current_cruise_control_mode_ != CruiseControlMode::CRUISE_CONTROL_ON) {
            return false;
        }
        if (cruise_end_time_ == 0) {
            return true; // No time limit
        }
        return millis() < cruise_end_time_;
    }

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

    // ==============================
    // STATE SETTERS
    // ==============================

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
     * @brief Set the posing mode.
     * @param mode Desired posing mode
     * @return True if mode change successful
     */
    bool setPosingMode(PosingMode mode);

    /**
     * @brief Set the cruise control mode.
     * @param mode Desired cruise control mode
     * @param velocity Initial velocity for cruise control
     * @return True if mode change successful
     */
    bool setCruiseControlMode(CruiseControlMode mode, const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero());

    /**
     * @brief Set the pose reset mode.
     * @param mode Desired pose reset mode
     * @return True if mode change successful
     */
    bool setPoseResetMode(PoseResetMode mode);

    // ==============================
    // LEG CONTROL
    // ==============================

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

    // ==============================
    // VELOCITY AND POSE CONTROL
    // ==============================

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

    // ==============================
    // GAIT CONTROL
    // ==============================

    /**
     * @brief Change the current gait type.
     * @param gait Desired gait type
     * @return True if gait change successful
     */
    bool changeGait(GaitType gait);

    // ==============================
    // STATUS AND DIAGNOSTICS
    // ==============================

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
    // ==============================
    // PRIVATE MEMBERS
    // ==============================

    LocomotionSystem &locomotion_system_;
    StateMachineConfig config_;

    // Current states
    SystemState current_system_state_;
    RobotState current_robot_state_;
    WalkState current_walk_state_;
    PosingMode current_posing_mode_;
    CruiseControlMode current_cruise_control_mode_;
    PoseResetMode current_pose_reset_mode_;

    // Desired states (for transitions)
    SystemState desired_system_state_;
    RobotState desired_robot_state_;

    // Leg states
    LegState leg_states_[NUM_LEGS];
    int manual_leg_count_;

    // Transition management
    bool is_transitioning_;
    unsigned long transition_start_time_;

    // Control inputs
    Eigen::Vector2d desired_linear_velocity_;
    double desired_angular_velocity_;
    Eigen::Vector3d desired_body_position_;
    Eigen::Vector3d desired_body_orientation_;
    Eigen::Vector3d leg_tip_velocities_[NUM_LEGS];

    // Cruise control
    Eigen::Vector3d cruise_velocity_;
    unsigned long cruise_start_time_;
    unsigned long cruise_end_time_; //< End time for cruise control (when time limit is set)

    // Timing
    unsigned long last_update_time_;
    double time_delta_; // Measured time delta for diagnostics (nominal params.time_delta otherwise)

    // Error handling
    bool has_error_;
    String last_error_message_;
    // Sequence execution state to avoid static locals
    int startup_step_;
    bool startup_transition_initialized_;
    int startup_transition_step_count_;
    int shutdown_step_;
    bool shutdown_transition_initialized_;
    int shutdown_transition_step_count_;
    int pack_step_;
    int unpack_step_;

    // Initialization flag
    bool is_initialized_;

    // Pose control
    std::unique_ptr<BodyPoseController> body_pose_controller_;
    // Precomputed joint angle targets for packed and ready states
    JointAngles packed_target_angles_[NUM_LEGS];
    JointAngles ready_target_angles_[NUM_LEGS];

    // ==============================
    // PRIVATE METHODS
    // ==============================

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
     * @brief Calculate timeout for state transitions.
     * @return Timeout in milliseconds
     */
    unsigned long calculateTransitionTimeout() const;

    /**
     * @brief Apply body position control for specific axes.
     * @param enable_x Enable X-axis control
     * @param enable_y Enable Y-axis control
     * @param enable_z Enable Z-axis control
     */
    void applyBodyPositionControl(bool enable_x, bool enable_y, bool enable_z);

    /**
     * @brief Apply body orientation control for specific axes.
     * @param enable_roll Enable roll control
     * @param enable_pitch Enable pitch control
     * @param enable_yaw Enable yaw control
     */
    void applyBodyOrientationControl(bool enable_roll, bool enable_pitch, bool enable_yaw);

    /**
     * @brief Apply pose reset based on current reset mode.
     */
    void applyPoseReset();
};

#endif // STATE_CONTROLLER_H
