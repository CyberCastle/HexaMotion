#include "state_controller.h"

// ==============================
// CONSTRUCTOR & DESTRUCTOR
// ==============================

StateController::StateController(LocomotionSystem& locomotion, const StateMachineConfig& config)
    : locomotion_system_(locomotion)
    , config_(config)
    , current_system_state_(SYSTEM_SUSPENDED)
    , current_robot_state_(ROBOT_UNKNOWN)
    , current_walk_state_(WALK_STOPPED)
    , current_posing_mode_(POSING_NONE)
    , current_cruise_control_mode_(CRUISE_CONTROL_OFF)
    , current_pose_reset_mode_(POSE_RESET_NONE)
    , desired_system_state_(SYSTEM_SUSPENDED)
    , desired_robot_state_(ROBOT_UNKNOWN)
    , manual_leg_count_(0)
    , is_transitioning_(false)
    , desired_linear_velocity_(Eigen::Vector2f::Zero())
    , desired_angular_velocity_(0.0f)
    , desired_body_position_(Eigen::Vector3f::Zero())
    , desired_body_orientation_(Eigen::Vector3f::Zero())
    , cruise_velocity_(Eigen::Vector3f::Zero())
    , cruise_start_time_(0)
    , last_update_time_(0)
    , dt_(0.02f)
    , has_error_(false)
    , is_initialized_(false) {

    // Initialize leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3f::Zero();
    }

    // Initialize transition progress
    transition_progress_.current_step = 0;
    transition_progress_.total_steps = 0;
    transition_progress_.completion_percentage = 0.0f;
    transition_progress_.is_complete = true;
    transition_progress_.has_error = false;
    transition_progress_.error_message = "";
}

StateController::~StateController() {
    // Clean shutdown
    if (is_initialized_) {
        emergencyStop();
    }
}

// ==============================
// INITIALIZATION
// ==============================

bool StateController::initialize() {
    logDebug("Initializing StateController...");

    // Check if locomotion system is available
    if (!locomotion_system_.isSystemEnabled()) {
        setError("Locomotion system not enabled");
        return false;
    }

    // Initialize timing
    last_update_time_ = millis();

    // Set initial state based on robot position
    current_robot_state_ = ROBOT_UNKNOWN;
    desired_robot_state_ = ROBOT_UNKNOWN;

    // Detect initial robot state
    if (isRobotPacked()) {
        current_robot_state_ = ROBOT_PACKED;
        logDebug("Robot detected in PACKED state");
    } else if (isRobotReady()) {
        current_robot_state_ = ROBOT_READY;
        logDebug("Robot detected in READY state");
    } else {
        current_robot_state_ = ROBOT_UNKNOWN;
        logDebug("Robot state UNKNOWN - will estimate from joint positions");
    }

    desired_robot_state_ = current_robot_state_;

    // Initialize system state
    current_system_state_ = SYSTEM_OPERATIONAL;
    desired_system_state_ = SYSTEM_OPERATIONAL;

    is_initialized_ = true;
    clearError();

    logDebug("StateController initialized successfully");
    return true;
}

// ==============================
// MAIN UPDATE LOOP
// ==============================

void StateController::update(float dt) {
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
    if (current_robot_state_ == ROBOT_RUNNING) {
        updateVelocityControl();
        updatePoseControl();
    }
}

// ==============================
// STATE ACCESSORS & SETTERS
// ==============================

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
    logDebug("System state change requested: " + String(new_state));
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

    logDebug("Robot state transition requested: " + String(current_robot_state_) + " -> " + String(new_state));
    return true;
}

bool StateController::setPosingMode(PosingMode mode) {
    if (!config_.enable_manual_posing && mode != POSING_NONE) {
        setError("Manual posing is disabled in configuration");
        return false;
    }

    current_posing_mode_ = mode;
    logDebug("Posing mode changed to: " + String(mode));
    return true;
}

bool StateController::setCruiseControlMode(CruiseControlMode mode, const Eigen::Vector3f& velocity) {
    if (!config_.enable_cruise_control && mode != CRUISE_CONTROL_OFF) {
        setError("Cruise control is disabled in configuration");
        return false;
    }

    current_cruise_control_mode_ = mode;
    
    if (mode == CRUISE_CONTROL_ON) {
        cruise_velocity_ = velocity;
        cruise_start_time_ = millis();
        logDebug("Cruise control enabled with velocity: [" + 
                String(velocity.x()) + ", " + String(velocity.y()) + ", " + String(velocity.z()) + "]");
    } else {
        cruise_velocity_ = Eigen::Vector3f::Zero();
        logDebug("Cruise control disabled");
    }

    return true;
}

void StateController::setPoseResetMode(PoseResetMode mode) {
    current_pose_reset_mode_ = mode;
    logDebug("Pose reset mode changed to: " + String(mode));
}

// ==============================
// LEG CONTROL
// ==============================

bool StateController::setLegState(int leg_index, AdvancedLegState state) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        setError("Invalid leg index: " + String(leg_index));
        return false;
    }

    if (current_robot_state_ != ROBOT_RUNNING) {
        setError("Cannot change leg state when robot is not in RUNNING state");
        return false;
    }

    // Check manual leg limit
    if (state == LEG_MANUAL && leg_states_[leg_index] != LEG_MANUAL) {
        if (manual_leg_count_ >= config_.max_manual_legs) {
            setError("Maximum number of manual legs (" + String(config_.max_manual_legs) + ") already reached");
            return false;
        }
    }

    // Update manual leg count
    if (leg_states_[leg_index] == LEG_MANUAL && state != LEG_MANUAL) {
        manual_leg_count_--;
    } else if (leg_states_[leg_index] != LEG_MANUAL && state == LEG_MANUAL) {
        manual_leg_count_++;
    }

    leg_states_[leg_index] = state;
    logDebug("Leg " + String(leg_index) + " state changed to: " + String(state));
    return true;
}

AdvancedLegState StateController::getLegState(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return LEG_WALKING; // Default safe state
    }
    return leg_states_[leg_index];
}

int StateController::getManualLegCount() const {
    return manual_leg_count_;
}

// ==============================
// VELOCITY AND POSE CONTROL
// ==============================

void StateController::setDesiredVelocity(const Eigen::Vector2f& linear_velocity, float angular_velocity) {
    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

void StateController::setDesiredPose(const Eigen::Vector3f& position, const Eigen::Vector3f& orientation) {
    desired_body_position_ = position;
    desired_body_orientation_ = orientation;
}

void StateController::setLegTipVelocity(int leg_index, const Eigen::Vector3f& velocity) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_tip_velocities_[leg_index] = velocity;
    }
}

// ==============================
// GAIT CONTROL
// ==============================

bool StateController::changeGait(GaitType gait) {
    if (current_robot_state_ != ROBOT_RUNNING) {
        setError("Cannot change gait when robot is not in RUNNING state");
        return false;
    }

    // Force robot to stop before changing gait
    if (current_walk_state_ != WALK_STOPPED) {
        desired_linear_velocity_ = Eigen::Vector2f::Zero();
        desired_angular_velocity_ = 0.0f;
        logDebug("Stopping robot to change gait...");
        return false; // Will retry when stopped
    }

    return locomotion_system_.setGaitType(gait);
}

// ==============================
// STATUS AND DIAGNOSTICS
// ==============================

TransitionProgress StateController::getTransitionProgress() const {
    return transition_progress_;
}

bool StateController::hasErrors() const {
    return has_error_;
}

String StateController::getLastErrorMessage() const {
    return last_error_message_;
}

String StateController::getDiagnosticInfo() const {
    String info = "StateController Diagnostics:\n";
    info += "  System State: " + String(current_system_state_) + "\n";
    info += "  Robot State: " + String(current_robot_state_) + "\n";
    info += "  Walk State: " + String(current_walk_state_) + "\n";
    info += "  Posing Mode: " + String(current_posing_mode_) + "\n";
    info += "  Cruise Control: " + String(current_cruise_control_mode_) + "\n";
    info += "  Manual Legs: " + String(manual_leg_count_) + "/" + String(config_.max_manual_legs) + "\n";
    info += "  Transitioning: " + String(is_transitioning_ ? "Yes" : "No") + "\n";
    
    if (is_transitioning_) {
        info += "  Transition Progress: " + String(transition_progress_.completion_percentage) + "%\n";
    }
    
    if (has_error_) {
        info += "  Error: " + last_error_message_ + "\n";
    }
    
    return info;
}

void StateController::emergencyStop() {
    logDebug("EMERGENCY STOP activated");
    
    // Stop all motion immediately
    desired_linear_velocity_ = Eigen::Vector2f::Zero();
    desired_angular_velocity_ = 0.0f;
    
    // Disable cruise control
    current_cruise_control_mode_ = CRUISE_CONTROL_OFF;
    
    // Reset all leg tip velocities
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_tip_velocities_[i] = Eigen::Vector3f::Zero();
    }
    
    // Cancel any ongoing transitions
    is_transitioning_ = false;
    
    // Set robot to safe state if possible
    if (current_robot_state_ == ROBOT_RUNNING) {
        current_walk_state_ = WALK_STOPPED;
    }
    
    clearError();
}

void StateController::reset() {
    logDebug("Resetting StateController");
    
    // Reset to initial states
    current_system_state_ = SYSTEM_SUSPENDED;
    current_robot_state_ = ROBOT_UNKNOWN;
    current_walk_state_ = WALK_STOPPED;
    current_posing_mode_ = POSING_NONE;
    current_cruise_control_mode_ = CRUISE_CONTROL_OFF;
    current_pose_reset_mode_ = POSE_RESET_NONE;
    
    desired_system_state_ = SYSTEM_SUSPENDED;
    desired_robot_state_ = ROBOT_UNKNOWN;
    
    // Reset leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3f::Zero();
    }
    manual_leg_count_ = 0;
    
    // Reset control inputs
    desired_linear_velocity_ = Eigen::Vector2f::Zero();
    desired_angular_velocity_ = 0.0f;
    desired_body_position_ = Eigen::Vector3f::Zero();
    desired_body_orientation_ = Eigen::Vector3f::Zero();
    
    // Reset transition state
    is_transitioning_ = false;
    transition_progress_.current_step = 0;
    transition_progress_.total_steps = 0;
    transition_progress_.completion_percentage = 0.0f;
    transition_progress_.is_complete = true;
    transition_progress_.has_error = false;
    transition_progress_.error_message = "";
    
    clearError();
}

// ==============================
// PRIVATE METHODS
// ==============================

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
            transition_progress_.has_error = true;
            transition_progress_.error_message = "Transition timeout";
        }
    }
}

void StateController::handleSystemStateTransition() {
    if (current_system_state_ == desired_system_state_) {
        return;
    }
    
    switch (desired_system_state_) {
        case SYSTEM_SUSPENDED:
            // Stop all motion and suspend operations
            emergencyStop();
            current_system_state_ = SYSTEM_SUSPENDED;
            logDebug("System suspended");
            break;
            
        case SYSTEM_OPERATIONAL:
            // Resume normal operations
            current_system_state_ = SYSTEM_OPERATIONAL;
            clearError();
            logDebug("System operational");
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
        case ROBOT_UNKNOWN:
            // Determine actual state and transition
            if (isRobotPacked()) {
                current_robot_state_ = ROBOT_PACKED;
            } else if (isRobotReady()) {
                current_robot_state_ = ROBOT_READY;
            } else {
                current_robot_state_ = ROBOT_PACKED; // Default assumption
            }
            is_transitioning_ = false;
            break;
            
        case ROBOT_PACKED:
            if (desired_robot_state_ == ROBOT_READY) {
                progress = executeUnpackSequence();
            } else if (desired_robot_state_ == ROBOT_RUNNING && !config_.enable_startup_sequence) {
                // Direct startup
                progress = executeUnpackSequence();
                if (progress == PROGRESS_COMPLETE) {
                    current_robot_state_ = ROBOT_READY;
                }
            }
            break;
            
        case ROBOT_READY:
            if (desired_robot_state_ == ROBOT_PACKED) {
                progress = executePackSequence();
            } else if (desired_robot_state_ == ROBOT_RUNNING) {
                if (config_.enable_startup_sequence) {
                    progress = executeStartupSequence();
                } else {
                    // Direct startup
                    current_robot_state_ = ROBOT_RUNNING;
                    current_walk_state_ = WALK_STOPPED;
                    progress = PROGRESS_COMPLETE;
                }
            }
            break;
            
        case ROBOT_RUNNING:
            if (desired_robot_state_ == ROBOT_READY) {
                // Must stop walking first
                if (current_walk_state_ != WALK_STOPPED) {
                    desired_linear_velocity_ = Eigen::Vector2f::Zero();
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
    
    // Update transition progress
    transition_progress_.completion_percentage = progress;
    transition_progress_.is_complete = (progress == PROGRESS_COMPLETE);
    
    if (progress == PROGRESS_COMPLETE) {
        current_robot_state_ = desired_robot_state_;
        is_transitioning_ = false;
        logDebug("Robot state transition completed: " + String(current_robot_state_));
    }
}

void StateController::updateWalkState() {
    if (current_robot_state_ != ROBOT_RUNNING) {
        current_walk_state_ = WALK_STOPPED;
        return;
    }
    
    // Determine walk state based on velocity inputs
    float linear_magnitude = desired_linear_velocity_.norm();
    float angular_magnitude = abs(desired_angular_velocity_);
    
    bool has_velocity_input = (linear_magnitude > 0.01f) || (angular_magnitude > 0.01f);
    
    switch (current_walk_state_) {
        case WALK_STOPPED:
            if (has_velocity_input) {
                current_walk_state_ = WALK_STARTING;
                logDebug("Walk state: STARTING");
            }
            break;
            
        case WALK_STARTING:
            // Check if all legs are at correct phase
            current_walk_state_ = WALK_MOVING;
            logDebug("Walk state: MOVING");
            break;
            
        case WALK_MOVING:
            if (!has_velocity_input) {
                current_walk_state_ = WALK_STOPPING;
                logDebug("Walk state: STOPPING");
            }
            break;
            
        case WALK_STOPPING:
            // Check if all legs have reached default positions
            current_walk_state_ = WALK_STOPPED;
            logDebug("Walk state: STOPPED");
            break;
            
        default:
            current_walk_state_ = WALK_STOPPED;
            break;
    }
}

void StateController::handleLegStateTransitions() {
    for (int i = 0; i < NUM_LEGS; i++) {
        switch (leg_states_[i]) {
            case LEG_WALKING_TO_MANUAL:
                // Transition logic for walking to manual
                if (current_walk_state_ == WALK_STOPPED) {
                    leg_states_[i] = LEG_MANUAL;
                    logDebug("Leg " + String(i) + " transitioned to MANUAL");
                }
                break;
                
            case LEG_MANUAL_TO_WALKING:
                // Transition logic for manual to walking
                leg_states_[i] = LEG_WALKING;
                manual_leg_count_--;
                logDebug("Leg " + String(i) + " transitioned to WALKING");
                break;
                
            default:
                // No transition needed
                break;
        }
    }
}

void StateController::updateVelocityControl() {
    if (current_cruise_control_mode_ == CRUISE_CONTROL_ON) {
        // Use cruise control velocity
        locomotion_system_.walkForward(cruise_velocity_.x());
        // Note: Angular velocity and y-axis velocity would need additional methods
    } else {
        // Use direct velocity input
        if (desired_linear_velocity_.norm() > 0.01f || abs(desired_angular_velocity_) > 0.01f) {
            float forward_velocity = desired_linear_velocity_.x();
            
            if (forward_velocity > 0.01f) {
                locomotion_system_.walkForward(forward_velocity);
            } else if (forward_velocity < -0.01f) {
                locomotion_system_.walkBackward(-forward_velocity);
            }
            
            if (abs(desired_angular_velocity_) > 0.01f) {
                locomotion_system_.turnInPlace(desired_angular_velocity_);
            }
            
            if (abs(desired_linear_velocity_.y()) > 0.01f) {
                locomotion_system_.walkSideways(abs(desired_linear_velocity_.y()), desired_linear_velocity_.y() > 0);
            }
        }
    }
}

void StateController::updatePoseControl() {
    if (current_posing_mode_ != POSING_NONE) {
        // Apply pose control based on current mode
        // This would interface with the pose controller
        // Implementation depends on the specific pose controller interface
    }
}

// ==============================
// SEQUENCE EXECUTION
// ==============================

int StateController::executeStartupSequence() {
    // Simplified startup sequence - step to walking stance
    static int startup_step = 0;
    
    if (startup_step == 0) {
        // Initialize startup
        transition_progress_.total_steps = 3;
        startup_step = 1;
        transition_progress_.current_step = 1;
        return 33;
    } else if (startup_step == 1) {
        // Step 1: Position legs for walking
        startup_step = 2;
        transition_progress_.current_step = 2;
        return 66;
    } else if (startup_step == 2) {
        // Step 2: Final positioning
        startup_step = 0; // Reset for next time
        transition_progress_.current_step = 3;
        return PROGRESS_COMPLETE;
    }
    
    return 0;
}

int StateController::executeShutdownSequence() {
    // Simplified shutdown sequence - step to ready stance
    static int shutdown_step = 0;
    
    if (shutdown_step == 0) {
        // Initialize shutdown
        transition_progress_.total_steps = 3;
        shutdown_step = 1;
        transition_progress_.current_step = 1;
        return 33;
    } else if (shutdown_step == 1) {
        // Step 1: Move to neutral positions
        shutdown_step = 2;
        transition_progress_.current_step = 2;
        return 66;
    } else if (shutdown_step == 2) {
        // Step 2: Final positioning
        shutdown_step = 0; // Reset for next time
        transition_progress_.current_step = 3;
        return PROGRESS_COMPLETE;
    }
    
    return 0;
}

int StateController::executePackSequence() {
    // Simplified pack sequence
    static unsigned long pack_start_time = 0;
    
    if (pack_start_time == 0) {
        pack_start_time = millis();
        transition_progress_.total_steps = 1;
        transition_progress_.current_step = 1;
    }
    
    unsigned long elapsed = millis() - pack_start_time;
    float pack_duration = config_.pack_unpack_time * 1000; // Convert to milliseconds
    
    if (elapsed >= pack_duration) {
        pack_start_time = 0; // Reset for next time
        return PROGRESS_COMPLETE;
    }
    
    return (int)((elapsed / pack_duration) * 100);
}

int StateController::executeUnpackSequence() {
    // Simplified unpack sequence
    static unsigned long unpack_start_time = 0;
    
    if (unpack_start_time == 0) {
        unpack_start_time = millis();
        transition_progress_.total_steps = 1;
        transition_progress_.current_step = 1;
    }
    
    unsigned long elapsed = millis() - unpack_start_time;
    float unpack_duration = config_.pack_unpack_time * 1000; // Convert to milliseconds
    
    if (elapsed >= unpack_duration) {
        unpack_start_time = 0; // Reset for next time
        return PROGRESS_COMPLETE;
    }
    
    return (int)((elapsed / unpack_duration) * 100);
}

// ==============================
// STATE VALIDATION
// ==============================

bool StateController::isRobotPacked() const {
    // Check if robot joints are in packed positions
    // This would require access to current joint positions
    // For now, simplified implementation
    return false; // Default assumption
}

bool StateController::isRobotReady() const {
    // Check if robot joints are in ready positions
    // This would require access to current joint positions
    // For now, simplified implementation
    return true; // Default assumption
}

bool StateController::isValidStateTransition(RobotState current_state, RobotState desired_state) const {
    // Define valid state transitions
    switch (current_state) {
        case ROBOT_UNKNOWN:
            return true; // Can transition to any state from unknown
            
        case ROBOT_PACKED:
            return (desired_state == ROBOT_READY) || 
                   (desired_state == ROBOT_RUNNING && !config_.enable_startup_sequence);
            
        case ROBOT_READY:
            return (desired_state == ROBOT_PACKED) || (desired_state == ROBOT_RUNNING);
            
        case ROBOT_RUNNING:
            return (desired_state == ROBOT_READY) || 
                   (desired_state == ROBOT_PACKED && !config_.enable_startup_sequence);
            
        default:
            return false;
    }
}

// ==============================
// UTILITY METHODS
// ==============================

void StateController::setError(const String& message) {
    has_error_ = true;
    last_error_message_ = message;
    logDebug("ERROR: " + message);
}

void StateController::clearError() {
    has_error_ = false;
    last_error_message_ = "";
}

void StateController::logDebug(const String& message) {
    // In a real implementation, this would use a proper logging system
    Serial.println("[StateController] " + message);
}

unsigned long StateController::calculateTransitionTimeout() const {
    // Return timeout in milliseconds based on current operation
    if (is_transitioning_) {
        switch (desired_robot_state_) {
            case ROBOT_PACKED:
            case ROBOT_READY:
                return (unsigned long)(config_.pack_unpack_time * 1000 * 2); // 2x safety margin
                
            case ROBOT_RUNNING:
                return (unsigned long)(config_.transition_timeout * 1000);
                
            default:
                return (unsigned long)(config_.transition_timeout * 1000);
        }
    }
    
    return (unsigned long)(config_.transition_timeout * 1000);
}
