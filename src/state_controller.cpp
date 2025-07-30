#include "state_controller.h"
#include "body_pose_config_factory.h"
#include "hexamotion_constants.h"
#include "locomotion_system.h"

/**
 * @file state_controller.cpp
 * @brief Implements the high-level state machine.
 */
#include <sstream>

// Helper functions to convert enums to strings for logging
namespace {
std::string toString(SystemState state) {
    switch (state) {
    case SystemState::SYSTEM_PACKED:
        return "SYSTEM_PACKED";
    case SystemState::SYSTEM_READY:
        return "SYSTEM_READY";
    case SystemState::SYSTEM_RUNNING:
        return "SYSTEM_RUNNING";
    default:
        return "UNKNOWN";
    }
}

std::string toString(RobotState state) {
    switch (state) {
    case RobotState::ROBOT_UNKNOWN:
        return "ROBOT_UNKNOWN";
    case RobotState::ROBOT_PACKED:
        return "ROBOT_PACKED";
    case RobotState::ROBOT_READY:
        return "ROBOT_READY";
    case RobotState::ROBOT_RUNNING:
        return "ROBOT_RUNNING";
    default:
        return "UNKNOWN";
    }
}

std::string toString(WalkState state) {
    switch (state) {
    case WalkState::WALK_STOPPED:
        return "WALK_STOPPED";
    case WalkState::WALK_STARTING:
        return "WALK_STARTING";
    case WalkState::WALK_MOVING:
        return "WALK_MOVING";
    case WalkState::WALK_STOPPING:
        return "WALK_STOPPING";
    default:
        return "UNKNOWN";
    }
}

std::string toString(PosingMode mode) {
    switch (mode) {
    case PosingMode::POSING_NONE:
        return "POSING_NONE";
    case PosingMode::POSING_X_Y:
        return "POSING_X_Y";
    case PosingMode::POSING_PITCH_ROLL:
        return "POSING_PITCH_ROLL";
    case PosingMode::POSING_Z_YAW:
        return "POSING_Z_YAW";
    case PosingMode::POSING_EXTERNAL:
        return "POSING_EXTERNAL";
    default:
        return "UNKNOWN";
    }
}

std::string toString(PoseResetMode mode) {
    switch (mode) {
    case PoseResetMode::POSE_RESET_NONE:
        return "POSE_RESET_NONE";
    case PoseResetMode::POSE_RESET_Z_AND_YAW:
        return "POSE_RESET_Z_AND_YAW";
    case PoseResetMode::POSE_RESET_X_AND_Y:
        return "POSE_RESET_X_AND_Y";
    case PoseResetMode::POSE_RESET_PITCH_AND_ROLL:
        return "POSE_RESET_PITCH_AND_ROLL";
    case PoseResetMode::POSE_RESET_ALL:
        return "POSE_RESET_ALL";
    case PoseResetMode::POSE_RESET_IMMEDIATE_ALL:
        return "POSE_RESET_IMMEDIATE_ALL";
    default:
        return "UNKNOWN";
    }
}

std::string toString(CruiseControlMode mode) {
    switch (mode) {
    case CruiseControlMode::CRUISE_CONTROL_OFF:
        return "CRUISE_CONTROL_OFF";
    case CruiseControlMode::CRUISE_CONTROL_ON:
        return "CRUISE_CONTROL_ON";
    default:
        return "UNKNOWN";
    }
}

std::string toString(LegState state) {
    switch (state) {
    case LegState::LEG_WALKING:
        return "LEG_WALKING";
    case LegState::LEG_MANUAL:
        return "LEG_MANUAL";
    case LegState::LEG_WALKING_TO_MANUAL:
        return "LEG_WALKING_TO_MANUAL";
    case LegState::LEG_MANUAL_TO_WALKING:
        return "LEG_MANUAL_TO_WALKING";
    default:
        return "UNKNOWN";
    }
}

std::string toString(double value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

std::string toString(int value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// Convert std::string to String (for Arduino compatibility)
String toArduinoString(const std::string &str) {
    return String(str.c_str());
}
} // namespace

StateController::StateController(LocomotionSystem &locomotion, const StateMachineConfig &config)
    : locomotion_system_(locomotion), config_(config), current_system_state_(SystemState::SYSTEM_PACKED), current_robot_state_(RobotState::ROBOT_UNKNOWN), current_walk_state_(WalkState::WALK_STOPPED), current_posing_mode_(PosingMode::POSING_NONE), current_cruise_control_mode_(CruiseControlMode::CRUISE_CONTROL_OFF), current_pose_reset_mode_(PoseResetMode::POSE_RESET_NONE), desired_system_state_(SystemState::SYSTEM_PACKED), desired_robot_state_(RobotState::ROBOT_UNKNOWN), manual_leg_count_(0), is_transitioning_(false), desired_linear_velocity_(Eigen::Vector2d::Zero()), desired_angular_velocity_(0.0f), desired_body_position_(Eigen::Vector3d::Zero()), desired_body_orientation_(Eigen::Vector3d::Zero()), cruise_velocity_(Eigen::Vector3d::Zero()), cruise_start_time_(0), cruise_end_time_(0), last_update_time_(0), dt_(0.02f), has_error_(false), startup_step_(0), startup_transition_initialized_(false), startup_transition_step_count_(4), shutdown_step_(0), shutdown_transition_initialized_(false), shutdown_transition_step_count_(3), pack_step_(0), unpack_step_(0), is_initialized_(false) {

    // Initialize leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LegState::LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
    }

    // Initialize transition progress
    // Remove initialization of transition_progress_
    // transition_progress_.current_step = 0;
    // transition_progress_.total_steps = 0;
    // transition_progress_.completion_percentage = 0.0f;
    // transition_progress_.is_complete = true;
    // transition_progress_.has_error = false;
    // transition_progress_.error_message = "";

    // Initialize pose controller
    body_pose_controller_ = nullptr;
}

StateController::~StateController() {
    // Clean shutdown
    if (is_initialized_) {
        emergencyStop();
    }

    // Clean up pose controller
    if (body_pose_controller_) {
        body_pose_controller_.reset();
    }
}

bool StateController::initialize(const BodyPoseConfiguration &pose_config) {
    logDebug("Initializing StateController...");

    // Check if locomotion system is available
    if (!locomotion_system_.isSystemEnabled()) {
        setError("Locomotion system not enabled");
        return false;
    }

    // Initialize timing
    last_update_time_ = millis();

    // Set initial state based on robot position
    current_robot_state_ = RobotState::ROBOT_UNKNOWN;
    desired_robot_state_ = RobotState::ROBOT_UNKNOWN;

    // Detect initial robot state
    if (isRobotPacked()) {
        current_robot_state_ = RobotState::ROBOT_PACKED;
        logDebug("Robot detected in PACKED state");
    } else if (isRobotReady()) {
        current_robot_state_ = RobotState::ROBOT_READY;
        logDebug("Robot detected in READY state");
    } else {
        current_robot_state_ = RobotState::ROBOT_UNKNOWN;
        logDebug("Robot state UNKNOWN - will estimate from joint positions");
    }

    desired_robot_state_ = current_robot_state_;

    // Initialize system state
    current_system_state_ = SystemState::SYSTEM_RUNNING;
    desired_system_state_ = SystemState::SYSTEM_RUNNING;

    // Initialize pose controller (equivalent to OpenSHC poser_)
    try {
        body_pose_controller_ = std::make_unique<BodyPoseController>(locomotion_system_.getRobotModel(),
                                                                     pose_config);
        logDebug("PoseController initialized successfully");
    } catch (const std::exception &e) {
        body_pose_controller_.reset();
        logError("Failed to initialize BodyPoseController: " + String(e.what()));
        setError("BodyPoseController initialization failed");
        return false;
    }

    is_initialized_ = true;
    clearError();

    logDebug("StateController initialized successfully");
    return true;
}

void StateController::update(double dt) {
    if (!is_initialized_) {
        return;
    }

    // Update timing
    unsigned long current_time = millis();
    dt_ = (current_time - last_update_time_) / 1000.0f;
    last_update_time_ = current_time;

    // Main state machine update
    updateStateMachine();

    // Update walk state
    updateWalkState();

    // Handle state transitions
    if (is_transitioning_) {
        handleRobotStateTransition();
    }

    // Handle leg state transitions
    handleLegStateTransitions();

    // Update control systems based on current state
    if (current_robot_state_ == RobotState::ROBOT_RUNNING) {
        updateVelocityControl();
        updatePoseControl();
    }
}

bool StateController::requestSystemState(SystemState new_state) {
    if (new_state == current_system_state_) {
        return true;
    }

    // Validate transition
    if (is_transitioning_) {
        setError("Cannot change system state during active transition");
        return false;
    }

    desired_system_state_ = new_state;
    logDebug("System state change requested: " + toArduinoString(toString(new_state)));
    return true;
}

bool StateController::requestRobotState(RobotState new_state) {
    if (new_state == current_robot_state_) {
        return true;
    }

    // Validate transition
    if (!isValidStateTransition(current_robot_state_, new_state)) {
        setError("Invalid robot state transition requested");
        return false;
    }

    if (is_transitioning_) {
        setError("Cannot change robot state during active transition");
        return false;
    }

    desired_robot_state_ = new_state;
    is_transitioning_ = true;
    transition_start_time_ = millis();

    logDebug("Robot state transition requested: " + toArduinoString(toString(current_robot_state_)) + " -> " + toArduinoString(toString(new_state)));
    return true;
}

bool StateController::setPosingMode(PosingMode mode) {
    if (!config_.enable_manual_posing && mode != PosingMode::POSING_NONE) {
        setError("Manual posing is disabled in configuration");
        return false;
    }

    current_posing_mode_ = mode;
    logDebug("Posing mode changed to: " + toArduinoString(toString(mode)));
    return true;
}

bool StateController::setCruiseControlMode(CruiseControlMode mode, const Eigen::Vector3d &velocity) {
    if (!config_.enable_cruise_control && mode != CruiseControlMode::CRUISE_CONTROL_OFF) {
        setError("Cruise control is disabled in configuration");
        return false;
    }

    current_cruise_control_mode_ = mode;

    if (mode == CruiseControlMode::CRUISE_CONTROL_ON) {
        // If velocity is provided (non-zero), use it as cruise velocity
        if (velocity.norm() > 0.001f) {
            cruise_velocity_ = velocity;
            logDebug("Cruise control enabled with specified velocity: [" +
                     toArduinoString(toString(velocity.x())) + ", " +
                     toArduinoString(toString(velocity.y())) + ", " +
                     toArduinoString(toString(velocity.z())) + "]");
        } else {
            // Save current velocity input as cruise velocity (equivalent to OpenSHC behavior)
            cruise_velocity_.x() = desired_linear_velocity_.x();
            cruise_velocity_.y() = desired_linear_velocity_.y();
            cruise_velocity_.z() = desired_angular_velocity_;
            logDebug("Cruise control enabled with current velocity: [" +
                     toArduinoString(toString(cruise_velocity_.x())) + ", " +
                     toArduinoString(toString(cruise_velocity_.y())) + ", " +
                     toArduinoString(toString(cruise_velocity_.z())) + "]");
        }
        cruise_start_time_ = millis();

        // Set end time if time limit is configured (equivalent to OpenSHC cruise_control_time_limit)
        if (config_.cruise_control_time_limit > 0.0f) {
            cruise_end_time_ = cruise_start_time_ + (unsigned long)(config_.cruise_control_time_limit * 1000.0f);
            logDebug("Cruise control time limit set to " + toArduinoString(toString(config_.cruise_control_time_limit)) + " seconds");
        } else {
            cruise_end_time_ = 0; // No time limit
        }
    } else {
        cruise_velocity_ = Eigen::Vector3d::Zero();
        cruise_start_time_ = 0;
        cruise_end_time_ = 0;
        logDebug("Cruise control disabled");
    }

    return true;
}

bool StateController::setPoseResetMode(PoseResetMode mode) {
    current_pose_reset_mode_ = mode;
    logDebug("Pose reset mode changed to: " + toArduinoString(toString(mode)));
    return true;
}

bool StateController::setLegState(int leg_index, LegState state) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        setError("Invalid leg index: " + toArduinoString(toString(leg_index)));
        return false;
    }

    if (current_robot_state_ != RobotState::ROBOT_RUNNING) {
        setError("Cannot change leg state when robot is not in RUNNING state");
        return false;
    }

    // Check manual leg limit
    if (state == LegState::LEG_MANUAL && leg_states_[leg_index] != LegState::LEG_MANUAL) {
        if (manual_leg_count_ >= config_.max_manual_legs) {
            setError("Maximum number of manual legs (" + toArduinoString(toString(config_.max_manual_legs)) + ") already reached");
            return false;
        }
    }

    // Update manual leg count
    if (leg_states_[leg_index] == LegState::LEG_MANUAL && state != LegState::LEG_MANUAL) {
        manual_leg_count_--;
    } else if (leg_states_[leg_index] != LegState::LEG_MANUAL && state == LegState::LEG_MANUAL) {
        manual_leg_count_++;
    }

    leg_states_[leg_index] = state;
    logDebug("Leg " + toArduinoString(toString(leg_index)) + " state changed to: " + toArduinoString(toString(state)));
    return true;
}

LegState StateController::getLegState(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return LegState::LEG_WALKING; // Default safe state
    }
    return leg_states_[leg_index];
}

int StateController::getManualLegCount() const {
    return manual_leg_count_;
}

void StateController::setDesiredVelocity(const Eigen::Vector2d &linear_velocity, double angular_velocity) {
    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

void StateController::setDesiredPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    desired_body_position_ = position;
    desired_body_orientation_ = orientation;
}

void StateController::setLegTipVelocity(int leg_index, const Eigen::Vector3d &velocity) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_tip_velocities_[leg_index] = velocity;
    }
}

bool StateController::setDesiredBodyPosition(const Eigen::Vector3d &position) {
    // Validate position limits (basic safety check)
    const double MAX_POSITION_OFFSET = 200.0f; // mm
    if (abs(position.x()) > MAX_POSITION_OFFSET ||
        abs(position.y()) > MAX_POSITION_OFFSET ||
        abs(position.z()) > MAX_POSITION_OFFSET) {
        setError("Desired body position exceeds safety limits");
        return false;
    }

    desired_body_position_ = position;
    logDebug("Desired body position set to: [" +
             toArduinoString(toString(position.x())) + ", " +
             toArduinoString(toString(position.y())) + ", " +
             toArduinoString(toString(position.z())) + "]");
    return true;
}

bool StateController::setDesiredBodyOrientation(const Eigen::Vector3d &orientation) {
    // Validate orientation limits (basic safety check)
    const double MAX_ANGLE = TIBIA_ANGLE_MAX; // degrees
    if (abs(orientation.x()) > MAX_ANGLE ||
        abs(orientation.y()) > MAX_ANGLE ||
        abs(orientation.z()) > MAX_ANGLE) {
        setError("Desired body orientation exceeds safety limits");
        return false;
    }

    desired_body_orientation_ = orientation;
    logDebug("Desired body orientation set to: [" +
             toArduinoString(toString(orientation.x())) + ", " +
             toArduinoString(toString(orientation.y())) + ", " +
             toArduinoString(toString(orientation.z())) + "]");
    return true;
}

bool StateController::changeGait(GaitType gait) {
    if (current_robot_state_ != RobotState::ROBOT_RUNNING) {
        setError("Cannot change gait when robot is not in RUNNING state");
        return false;
    }

    // Force robot to stop before changing gait
    if (current_walk_state_ != WalkState::WALK_STOPPED) {
        desired_linear_velocity_ = Eigen::Vector2d::Zero();
        desired_angular_velocity_ = 0.0f;
        logDebug("Stopping robot to change gait...");
        return false; // Will retry when stopped
    }

    return locomotion_system_.setGaitType(gait);
}

// Remove getTransitionProgress() method
// TransitionProgress StateController::getTransitionProgress() const {
//     return transition_progress_;
// }

bool StateController::hasErrors() const {
    return has_error_;
}

String StateController::getLastErrorMessage() const {
    return last_error_message_;
}

String StateController::getDiagnosticInfo() const {
    String info = "StateController Diagnostics:\n";
    info += "  System State: " + toArduinoString(toString(current_system_state_)) + "\n";
    info += "  Robot State: " + toArduinoString(toString(current_robot_state_)) + "\n";
    info += "  Walk State: " + toArduinoString(toString(current_walk_state_)) + "\n";
    info += "  Posing Mode: " + toArduinoString(toString(current_posing_mode_)) + "\n";
    info += "  Cruise Control: " + toArduinoString(toString(current_cruise_control_mode_)) + "\n";
    info += "  Manual Legs: " + toArduinoString(toString(manual_leg_count_)) + "/" + toArduinoString(toString(config_.max_manual_legs)) + "\n";
    info += "  Transitioning: " + String(is_transitioning_ ? "Yes" : "No") + "\n";

    // Remove transition_progress_ updates in transition and reset methods
    // if (is_transitioning_) {
    //     info += "  Transition Progress: " + toArduinoString(toString(transition_progress_.completion_percentage)) + "%\n";
    // }

    if (has_error_) {
        info += "  Error: " + last_error_message_ + "\n";
    }

    return info;
}

bool StateController::isTransitioning() const {
    return is_transitioning_;
}

void StateController::emergencyStop() {
    logDebug("EMERGENCY STOP activated");

    // Stop all motion immediately
    desired_linear_velocity_ = Eigen::Vector2d::Zero();
    desired_angular_velocity_ = 0.0f;

    // Disable cruise control
    current_cruise_control_mode_ = CruiseControlMode::CRUISE_CONTROL_OFF;

    // Reset all leg tip velocities
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
    }

    // Cancel any ongoing transitions
    is_transitioning_ = false;

    // Set robot to safe state if possible
    if (current_robot_state_ == RobotState::ROBOT_RUNNING) {
        current_walk_state_ = WalkState::WALK_STOPPED;
    }

    clearError();
}

void StateController::reset() {
    logDebug("Resetting StateController");

    // Reset to initial states
    current_system_state_ = SystemState::SYSTEM_PACKED;
    current_robot_state_ = RobotState::ROBOT_UNKNOWN;
    current_walk_state_ = WalkState::WALK_STOPPED;
    current_posing_mode_ = PosingMode::POSING_NONE;
    current_cruise_control_mode_ = CruiseControlMode::CRUISE_CONTROL_OFF;
    current_pose_reset_mode_ = PoseResetMode::POSE_RESET_NONE;

    desired_system_state_ = SystemState::SYSTEM_PACKED;
    desired_robot_state_ = RobotState::ROBOT_UNKNOWN;

    // Reset leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LegState::LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
    }
    manual_leg_count_ = 0;

    // Reset control inputs
    desired_linear_velocity_ = Eigen::Vector2d::Zero();
    desired_angular_velocity_ = 0.0f;
    desired_body_position_ = Eigen::Vector3d::Zero();
    desired_body_orientation_ = Eigen::Vector3d::Zero();

    // Reset transition state
    is_transitioning_ = false;
    // Remove transition_progress_ updates in transition and reset methods
    // transition_progress_.current_step = 0;
    // transition_progress_.total_steps = 0;
    // transition_progress_.completion_percentage = 0.0f;
    // transition_progress_.is_complete = true;
    // transition_progress_.has_error = false;
    // transition_progress_.error_message = "";

    // Initialize pose controller
    body_pose_controller_ = nullptr;
}

void StateController::updateStateMachine() {
    // Handle system state transitions
    handleSystemStateTransition();

    // Update transition timeouts
    if (is_transitioning_) {
        unsigned long elapsed_time = millis() - transition_start_time_;
        unsigned long timeout = calculateTransitionTimeout();

        if (elapsed_time > timeout) {
            setError("State transition timeout");
            is_transitioning_ = false;
            // Remove transition_progress_ updates in transition and reset methods
            // transition_progress_.has_error = true;
            // transition_progress_.error_message = "Transition timeout";
        }
    }
}

void StateController::handleSystemStateTransition() {
    if (current_system_state_ == desired_system_state_) {
        return;
    }

    switch (desired_system_state_) {
    case SystemState::SYSTEM_PACKED:
        // Stop all motion and pack operations
        emergencyStop();
        current_system_state_ = SystemState::SYSTEM_PACKED;
        logDebug("System packed");
        break;

    case SystemState::SYSTEM_READY:
        // Resume normal operations
        current_system_state_ = SystemState::SYSTEM_READY;
        clearError();
        logDebug("System ready");
        break;

    case SystemState::SYSTEM_RUNNING:
        // System is running - full operation
        current_system_state_ = SystemState::SYSTEM_RUNNING;
        break;

    default:
        setError("Unknown system state requested");
        break;
    }
}

void StateController::handleRobotStateTransition() {
    if (!is_transitioning_ || current_robot_state_ == desired_robot_state_) {
        is_transitioning_ = false;
        return;
    }

    int progress = 0;

    // Execute appropriate transition sequence
    switch (current_robot_state_) {
    case RobotState::ROBOT_UNKNOWN:
        // Determine actual state and transition
        if (isRobotPacked()) {
            current_robot_state_ = RobotState::ROBOT_PACKED;
            logDebug("Robot state determined: PACKED");
        } else if (isRobotReady()) {
            current_robot_state_ = RobotState::ROBOT_READY;
            logDebug("Robot state determined: READY");
        } else {
            // Fallback to READY state if robot doesn't meet packed criteria
            current_robot_state_ = RobotState::ROBOT_READY;
            logDebug("Robot state defaulted to: READY");
        }

        // Continue with transition if we're now at desired state
        if (current_robot_state_ == desired_robot_state_) {
            is_transitioning_ = false;
            logDebug("Robot state transition completed: " + toArduinoString(toString(current_robot_state_)));
        }
        break;

    case RobotState::ROBOT_PACKED:
        if (desired_robot_state_ == RobotState::ROBOT_READY) {
            progress = executeUnpackSequence();
        } else if (desired_robot_state_ == RobotState::ROBOT_RUNNING && !config_.enable_startup_sequence) {
            // Direct startup
            progress = executeUnpackSequence();
            if (progress == PROGRESS_COMPLETE) {
                current_robot_state_ = RobotState::ROBOT_READY;
            }
        }
        break;

    case RobotState::ROBOT_READY:
        if (desired_robot_state_ == RobotState::ROBOT_PACKED) {
            progress = executePackSequence();
        } else if (desired_robot_state_ == RobotState::ROBOT_RUNNING) {
            if (config_.enable_startup_sequence) {
                progress = executeStartupSequence();
            } else {
                // Direct startup
                current_robot_state_ = RobotState::ROBOT_RUNNING;
                current_walk_state_ = WalkState::WALK_STOPPED;
                progress = PROGRESS_COMPLETE;
                logDebug("Direct startup to RUNNING state");
            }
        }
        break;

    case RobotState::ROBOT_RUNNING:
        if (desired_robot_state_ == RobotState::ROBOT_READY) {
            // Must stop walking first
            if (current_walk_state_ != WalkState::WALK_STOPPED) {
                desired_linear_velocity_ = Eigen::Vector2d::Zero();
                desired_angular_velocity_ = 0.0f;
                return; // Wait for walking to stop
            }

            // Execute shutdown sequence
            progress = executeShutdownSequence();
        }
        break;

    default:
        setError("Invalid robot state transition");
        is_transitioning_ = false;
        return;
    }

    // Remove transition_progress_ updates in transition and reset methods
    // transition_progress_.completion_percentage = progress;
    // transition_progress_.is_complete = (progress == PROGRESS_COMPLETE);

    if (progress == PROGRESS_COMPLETE) {
        current_robot_state_ = desired_robot_state_;
        is_transitioning_ = false;
        logDebug("Robot state transition completed: " + toArduinoString(toString(current_robot_state_)));
    }
}

void StateController::updateWalkState() {
    if (current_robot_state_ != RobotState::ROBOT_RUNNING) {
        current_walk_state_ = WalkState::WALK_STOPPED;
        return;
    }

    // Determine walk state based on velocity inputs
    double linear_magnitude = desired_linear_velocity_.norm();
    double angular_magnitude = abs(desired_angular_velocity_);

    bool has_velocity_input = (linear_magnitude > 0.01f) || (angular_magnitude > 0.01f);

    switch (current_walk_state_) {
    case WalkState::WALK_STOPPED:
        if (has_velocity_input) {
            current_walk_state_ = WalkState::WALK_STARTING;
            logDebug("Walk state: STARTING");
        }
        break;

    case WalkState::WALK_STARTING:
        // Check if all legs are at correct phase
        current_walk_state_ = WalkState::WALK_MOVING;
        logDebug("Walk state: MOVING");
        break;

    case WalkState::WALK_MOVING:
        if (!has_velocity_input) {
            current_walk_state_ = WalkState::WALK_STOPPING;
            logDebug("Walk state: STOPPING");
        }
        break;

    case WalkState::WALK_STOPPING:
        // Check if all legs have reached default positions
        current_walk_state_ = WalkState::WALK_STOPPED;
        logDebug("Walk state: STOPPED");
        break;

    default:
        current_walk_state_ = WalkState::WALK_STOPPED;
        break;
    }
}

void StateController::handleLegStateTransitions() {
    for (int i = 0; i < NUM_LEGS; i++) {
        switch (leg_states_[i]) {
        case LegState::LEG_WALKING_TO_MANUAL:
            // Transition logic for walking to manual
            if (current_walk_state_ == WalkState::WALK_STOPPED) {
                leg_states_[i] = LegState::LEG_MANUAL;
                logDebug("Leg " + toArduinoString(toString(i)) + " transitioned to MANUAL");
            }
            break;

        case LegState::LEG_MANUAL_TO_WALKING:
            // Transition logic for manual to walking
            leg_states_[i] = LegState::LEG_WALKING;
            manual_leg_count_--;
            logDebug("Leg " + toArduinoString(toString(i)) + " transitioned to WALKING");
            break;

        default:
            // No transition needed
            break;
        }
    }
}

void StateController::updateVelocityControl() {
    double linear_x, linear_y, angular_z;

    // Check if cruise control should be used (equivalent to OpenSHC cruise control logic)
    bool use_cruise_control = (current_cruise_control_mode_ == CruiseControlMode::CRUISE_CONTROL_ON);

    // Check cruise control time limit (equivalent to OpenSHC cruise_control_time_limit check)
    if (use_cruise_control && cruise_end_time_ > 0) {
        unsigned long current_time = millis();
        if (current_time >= cruise_end_time_) {
            // Time limit exceeded, disable cruise control
            logDebug("Cruise control time limit exceeded, disabling");
            current_cruise_control_mode_ = CruiseControlMode::CRUISE_CONTROL_OFF;
            cruise_velocity_ = Eigen::Vector3d::Zero();
            use_cruise_control = false;
        }
    }

    if (use_cruise_control) {
        // Use cruise control velocity - equivalent to OpenSHC cruise control behavior
        linear_x = cruise_velocity_.x();
        linear_y = cruise_velocity_.y();
        angular_z = cruise_velocity_.z();

        logDebug("Using cruise velocity: [" +
                 toArduinoString(toString(linear_x)) + ", " +
                 toArduinoString(toString(linear_y)) + ", " +
                 toArduinoString(toString(angular_z)) + "]");
    } else {
        // Use direct velocity input
        linear_x = desired_linear_velocity_.x();
        linear_y = desired_linear_velocity_.y();
        angular_z = desired_angular_velocity_;
    }

    // Apply velocity control using combined gait planning (equivalent to OpenSHC walker_->updateWalk())
    if (abs(linear_x) > 0.01f || abs(linear_y) > 0.01f || abs(angular_z) > 0.01f) {
        // Use planGaitSequence for combined movement (x, y, angular)
        if (!locomotion_system_.planGaitSequence(linear_x, linear_y, angular_z)) {
            logError("Failed to plan gait sequence for velocity control");
            has_error_ = true;
            last_error_message_ = "Velocity control gait planning failed";
        }
    } else {
        // Stop movement when all velocities are near zero
        locomotion_system_.stopMovement();
    }
}

void StateController::updatePoseControl() {
    // Equivalent to OpenSHC poser_->updateCurrentPose() and pose control logic
    if (current_posing_mode_ == PosingMode::POSING_NONE) {
        return; // No pose control active
    }

    // Update pose control based on current mode (equivalent to OpenSHC pose modes)
    switch (current_posing_mode_) {
    case PosingMode::POSING_X_Y: {
        // Allow manual posing via X/Y axis translation (equivalent to OpenSHC X_Y mode)
        logDebug("Applying X/Y pose control");
        applyBodyPositionControl(true, true, false); // X, Y, not Z
        break;
    }

    case PosingMode::POSING_PITCH_ROLL: {
        // Allow manual posing via pitch/roll rotation (equivalent to OpenSHC PITCH_ROLL mode)
        logDebug("Applying pitch/roll pose control");
        applyBodyOrientationControl(true, true, false); // Roll, Pitch, not Yaw
        break;
    }

    case PosingMode::POSING_Z_YAW: {
        // Allow manual posing via Z axis translation and yaw rotation (equivalent to OpenSHC Z_YAW mode)
        logDebug("Applying Z/yaw pose control");
        applyBodyPositionControl(false, false, true);    // Not X, not Y, Z
        applyBodyOrientationControl(false, false, true); // Not Roll, not Pitch, Yaw
        break;
    }

    case PosingMode::POSING_EXTERNAL: {
        // Allow posing input from external source (equivalent to OpenSHC EXTERNAL mode)
        logDebug("Applying external pose control");
        applyBodyPositionControl(true, true, true);    // All position axes
        applyBodyOrientationControl(true, true, true); // All orientation axes
        break;
    }

    default:
        // Unknown mode
        logError("Unknown posing mode: " + toArduinoString(toString(static_cast<int>(current_posing_mode_))));
        break;
    }

    // Apply pose reset if needed (equivalent to OpenSHC pose reset logic)
    if (current_pose_reset_mode_ != PoseResetMode::POSE_RESET_NONE) {
        applyPoseReset();
    }
}

void StateController::applyBodyPositionControl(bool enable_x, bool enable_y, bool enable_z) {
    if (!body_pose_controller_) {
        logError("BodyPoseController not initialized - cannot apply body position control");
        return;
    }

    // Get current desired position
    Eigen::Vector3d current_position = desired_body_position_;

    // Apply control for enabled axes (equivalent to OpenSHC axis-specific control)
    Eigen::Vector3d controlled_position = current_position;

    if (!enable_x) {
        controlled_position.x() = 0.0f; // Reset to default if not enabled
    }
    if (!enable_y) {
        controlled_position.y() = 0.0f; // Reset to default if not enabled
    }
    if (!enable_z) {
        controlled_position.z() = 0.0f; // Reset to default if not enabled
    }

    // Apply the position control via locomotion system
    bool success = locomotion_system_.setBodyPose(controlled_position, desired_body_orientation_);

    if (!success) {
        logError("Failed to apply body position control");
    } else {
        logDebug("Applied body position control: X=" + toArduinoString(enable_x ? "ON" : "OFF") +
                 " Y=" + toArduinoString(enable_y ? "ON" : "OFF") +
                 " Z=" + toArduinoString(enable_z ? "ON" : "OFF"));
    }
}

void StateController::applyBodyOrientationControl(bool enable_roll, bool enable_pitch, bool enable_yaw) {
    if (!body_pose_controller_) {
        logError("BodyPoseController not initialized - cannot apply body orientation control");
        return;
    }

    // Get current desired orientation
    Eigen::Vector3d current_orientation = desired_body_orientation_;

    // Apply control for enabled axes (equivalent to OpenSHC axis-specific control)
    Eigen::Vector3d controlled_orientation = current_orientation;

    if (!enable_roll) {
        controlled_orientation.x() = 0.0f; // Reset to default if not enabled
    }
    if (!enable_pitch) {
        controlled_orientation.y() = 0.0f; // Reset to default if not enabled
    }
    if (!enable_yaw) {
        controlled_orientation.z() = 0.0f; // Reset to default if not enabled
    }

    // Apply the orientation control via locomotion system
    bool success = locomotion_system_.setBodyPose(desired_body_position_, controlled_orientation);

    if (!success) {
        logError("Failed to apply body orientation control");
    } else {
        logDebug("Applied body orientation control: Roll=" + toArduinoString(enable_roll ? "ON" : "OFF") +
                 " Pitch=" + toArduinoString(enable_pitch ? "ON" : "OFF") +
                 " Yaw=" + toArduinoString(enable_yaw ? "ON" : "OFF"));
    }
}

void StateController::applyPoseReset() {
    if (!body_pose_controller_) {
        logError("BodyPoseController not initialized - cannot apply pose reset");
        return;
    }

    // Apply pose reset based on current mode (equivalent to OpenSHC pose reset logic)
    Eigen::Vector3d reset_position = desired_body_position_;
    Eigen::Vector3d reset_orientation = desired_body_orientation_;

    switch (current_pose_reset_mode_) {
    case PoseResetMode::POSE_RESET_Z_AND_YAW: {
        // Reset Z position and yaw orientation
        reset_position.z() = 0.0f;
        reset_orientation.z() = 0.0f; // yaw
        logDebug("Applying Z and Yaw pose reset");
        break;
    }

    case PoseResetMode::POSE_RESET_X_AND_Y: {
        // Reset X and Y positions
        reset_position.x() = 0.0f;
        reset_position.y() = 0.0f;
        logDebug("Applying X and Y pose reset");
        break;
    }

    case PoseResetMode::POSE_RESET_PITCH_AND_ROLL: {
        // Reset pitch and roll orientations
        reset_orientation.x() = 0.0f; // roll
        reset_orientation.y() = 0.0f; // pitch
        logDebug("Applying pitch and roll pose reset");
        break;
    }

    case PoseResetMode::POSE_RESET_ALL: {
        // Reset all pose parameters gradually
        reset_position = Eigen::Vector3d::Zero();
        reset_orientation = Eigen::Vector3d::Zero();
        logDebug("Applying full pose reset");
        break;
    }

    case PoseResetMode::POSE_RESET_IMMEDIATE_ALL: {
        // Reset all pose parameters immediately
        reset_position = Eigen::Vector3d::Zero();
        reset_orientation = Eigen::Vector3d::Zero();
        logDebug("Applying immediate full pose reset");
        break;
    }

    default:
        logError("Unknown pose reset mode: " + toArduinoString(toString(static_cast<int>(current_pose_reset_mode_))));
        return;
    }

    // Apply the reset pose
    bool success = locomotion_system_.setBodyPose(reset_position, reset_orientation);

    if (success) {
        // Update internal desired pose state
        desired_body_position_ = reset_position;
        desired_body_orientation_ = reset_orientation;

        // Clear the reset mode after successful application
        current_pose_reset_mode_ = PoseResetMode::POSE_RESET_NONE;

        logDebug("Pose reset applied successfully");
    } else {
        logError("Failed to apply pose reset");
    }
}

/**
 * @brief Execute startup sequence to transition from ready to running state.
 *
 * This is a comprehensive startup sequence that follows OpenSHC implementation patterns.
 * The sequence involves alternating horizontal and vertical transitions to move legs
 * from ready positions to walking stance positions safely.
 *
 * @return int Progress percentage (0-100), 100 indicates completion
 */
int StateController::executeStartupSequence() {
    // Enhanced startup sequence following OpenSHC patterns using instance members
    if (!startup_transition_initialized_) {
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.total_steps = startup_transition_step_count_;
        startup_transition_initialized_ = true;
        startup_step_ = 0;
    }

    // Execute sequence based on startup_step_
    switch (startup_step_) {
    case 0:
        // Step 1: Initialize ready stance
        startup_step_ = 1;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 1;
        return 25;

    case 1:
        // Step 2: Move to intermediate position (safer transition)
        // Use locomotion system to transition to higher stance
        startup_step_ = 2;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 2;
        return 50;

    case 2:
        // Step 3: Move to walking height
        startup_step_ = 3;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 3;
        return 75;

    case 3:
        // Step 4: Finalize walking stance and enable locomotion
        if (locomotion_system_.setStandingPose()) {
            // Update default configuration for walking
            startup_step_ = 0; // Reset for next time
            startup_transition_initialized_ = false;
            // Remove transition_progress_ updates in transition and reset methods
            // transition_progress_.current_step = startup_transition_step_count_;
            // transition_progress_.is_complete = true;
            return 100;
        }
        return 75;

    default:
        return 0;
    }
}

/**
 * @brief Execute shutdown sequence to transition from running to ready state.
 *
 * This is a comprehensive shutdown sequence that follows OpenSHC implementation patterns.
 * The sequence safely moves legs from walking stance positions back to ready positions
 * with proper coordination and safety checks.
 *
 * @return int Progress percentage (0-100), 100 indicates completion
 */
int StateController::executeShutdownSequence() {
    // Enhanced shutdown sequence following OpenSHC patterns using instance members
    if (!shutdown_transition_initialized_) {
        // Ensure walking has stopped before shutdown
        if (current_walk_state_ != WALK_STOPPED) {
            desired_linear_velocity_.setZero();
            desired_angular_velocity_ = 0.0f;
            return 10; // Stay in shutdown but indicate progress
        }
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.total_steps = shutdown_transition_step_count_;
        shutdown_transition_initialized_ = true;
        shutdown_step_ = 0;
    }

    switch (shutdown_step_) {
    case 0:
        // Step 1: Return any manually controlled legs to automatic control
        if (manual_leg_count_ > 0) {
            manual_leg_count_ = 0;
        }
        shutdown_step_ = 1;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 1;
        return 33;

    case 1:
        // Step 2: Transition to ready height (higher than walking height)
        shutdown_step_ = 2;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 2;
        return 66;

    case 2:
        // Step 3: Finalize ready position
        if (locomotion_system_.setStandingPose()) {
            shutdown_step_ = 0; // Reset for next time
            shutdown_transition_initialized_ = false;
            // Remove transition_progress_ updates in transition and reset methods
            // transition_progress_.current_step = shutdown_transition_step_count_;
            // transition_progress_.is_complete = true;
            return 100;
        }
        return 66;

    default:
        return 0;
    }
}

int StateController::executePackSequence() {
    // Simplified pack sequence using instance member
    if (pack_step_ == 0) {
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.total_steps = 2;
        pack_step_ = 1;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 1;
        return 50;
    } else if (pack_step_ == 1) {
        // Move to packed position (standing pose as approximation)
        if (locomotion_system_.setStandingPose()) {
            pack_step_ = 0; // Reset for next time
            // Remove transition_progress_ updates in transition and reset methods
            // transition_progress_.current_step = 2;
            // transition_progress_.is_complete = true;
            // Capture packed target joint angles
            for (int i = 0; i < NUM_LEGS; ++i) {
                packed_target_angles_[i] = locomotion_system_.getJointAngles(i);
            }
            return 100;
        }
    }
    return 50;
}

int StateController::executeUnpackSequence() {
    // Simplified unpack sequence using instance member
    if (unpack_step_ == 0) {
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.total_steps = 2;
        unpack_step_ = 1;
        // Remove transition_progress_ updates in transition and reset methods
        // transition_progress_.current_step = 1;
        return 50;
    } else if (unpack_step_ == 1) {
        // Move to ready position
        if (locomotion_system_.setStandingPose()) {
            unpack_step_ = 0; // Reset for next time
            // Remove transition_progress_ updates in transition and reset methods
            // transition_progress_.current_step = 2;
            // transition_progress_.is_complete = true;
            // Capture ready target joint angles
            for (int i = 0; i < NUM_LEGS; ++i) {
                ready_target_angles_[i] = locomotion_system_.getJointAngles(i);
            }
            return 100;
        }
    }
    return 50;
}

bool StateController::isRobotPacked() const {
    // Check if robot is in packed state based on body position and orientation
    Eigen::Vector3d current_position = locomotion_system_.getBodyPosition();

    // Check for invalid/uninitialized position data
    if (current_position.norm() < 0.01f) {
        // Position is zero or very close to zero - this could indicate:
        // 1. Sensor failure or uninitialized state
        // 2. Robot actually at origin (rare but possible)

        // Fallback: check if we have valid orientation data
        Eigen::Vector3d current_orientation = locomotion_system_.getBodyOrientation();
        if (current_orientation.norm() < 0.01f) {
            // Both position and orientation are zero - assume not packed for safety
            return false;
        }

        // If orientation is available but position isn't, use orientation-based heuristic
        // Large roll/pitch might indicate packed/fallen state
        return (abs(current_orientation.x()) > TIBIA_ANGLE_MAX || abs(current_orientation.y()) > TIBIA_ANGLE_MAX);
    }

    bool body_low = current_position.z() < 50.0f;
    // Joint-based packed check
    bool joints_ok = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles cur = locomotion_system_.getJointAngles(i);
        JointAngles tgt = packed_target_angles_[i];
        if (abs(cur.coxa - tgt.coxa) > JOINT_TOLERANCE ||
            abs(cur.femur - tgt.femur) > JOINT_TOLERANCE ||
            abs(cur.tibia - tgt.tibia) > JOINT_TOLERANCE) {
            joints_ok = false;
            break;
        }
    }
    return body_low && joints_ok;
}

bool StateController::isRobotReady() const {
    // Check if robot is in ready state based on body position and orientation
    Eigen::Vector3d current_position = locomotion_system_.getBodyPosition();
    Eigen::Vector3d current_orientation = locomotion_system_.getBodyOrientation();

    // Check for invalid/uninitialized position data
    if (current_position.norm() < 0.01f) {
        return false; // Cannot determine if ready with invalid data
    }

    // Check if at reasonable height and level orientation
    bool height_ok = (current_position.z() > 80.0f && current_position.z() < 200.0f);
    bool orientation_ok = (abs(current_orientation.x()) < 10.0f && abs(current_orientation.y()) < 10.0f);
    bool body_ready = height_ok && orientation_ok;
    // Joint-based ready check
    bool joints_ok = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles cur = locomotion_system_.getJointAngles(i);
        JointAngles tgt = ready_target_angles_[i];
        if (abs(cur.coxa - tgt.coxa) > JOINT_TOLERANCE ||
            abs(cur.femur - tgt.femur) > JOINT_TOLERANCE ||
            abs(cur.tibia - tgt.tibia) > JOINT_TOLERANCE) {
            joints_ok = false;
            break;
        }
    }
    return body_ready && joints_ok;
}

bool StateController::isValidStateTransition(RobotState current_state, RobotState desired_state) const {
    // Define valid state transitions
    switch (current_state) {
    case ROBOT_PACKED:
        return (desired_state == ROBOT_READY || desired_state == ROBOT_UNKNOWN);

    case ROBOT_READY:
        return (desired_state == ROBOT_RUNNING || desired_state == ROBOT_PACKED);

    case ROBOT_RUNNING:
        return (desired_state == ROBOT_READY);

    case ROBOT_UNKNOWN:
        return true; // Allow any transition from unknown state

    case ROBOT_OFF:
        return (desired_state == ROBOT_READY || desired_state == ROBOT_UNKNOWN);

    default:
        return false;
    }
}

void StateController::setError(const String &message) {
    has_error_ = true;
    last_error_message_ = message;
    logError("StateController Error: " + message);
}

void StateController::clearError() {
    has_error_ = false;
    last_error_message_ = "";
}

void StateController::logDebug(const String &message) {
// In a real implementation, this would use proper logging
#ifdef DEBUG_LOGGING
    Serial.print("DEBUG: ");
    Serial.println(message);
#endif
}

void StateController::logError(const String &message) {
    // In a real implementation, this would use proper logging
    Serial.print("ERROR: ");
    Serial.println(message);
}

unsigned long StateController::calculateTransitionTimeout() const {
    // Calculate timeout based on transition type
    // For now, use a default timeout
    return 10000; // 10 seconds
}
