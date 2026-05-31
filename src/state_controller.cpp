#include "state_controller.h"
#include "body_pose_config_factory.h"
#include "gait_config_factory.h"
#include "hexamotion_constants.h"

/**
 * @file state_controller.cpp
 * @brief Implements the high-level state machine.
 */
#include <cmath>
#include <sstream>

// Helper functions to convert enums to strings for logging
namespace {
std::string toString(SystemState state) {
    switch (state) {
    case SystemState::SUSPENDED:
        return "SUSPENDED";
    case SystemState::OPERATIONAL:
        return "OPERATIONAL";
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
    case PoseResetMode::NO_RESET:
        return "NO_RESET";
    case PoseResetMode::Z_AND_YAW_RESET:
        return "Z_AND_YAW_RESET";
    case PoseResetMode::X_AND_Y_RESET:
        return "X_AND_Y_RESET";
    case PoseResetMode::PITCH_AND_ROLL_RESET:
        return "PITCH_AND_ROLL_RESET";
    case PoseResetMode::ALL_RESET:
        return "ALL_RESET";
    case PoseResetMode::IMMEDIATE_ALL_RESET:
        return "IMMEDIATE_ALL_RESET";
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

StateController::StateController(StateControllerContext &context, const StateMachineConfig &config)
    : context_(context), config_(config), current_system_state_(SystemState::SUSPENDED), current_robot_state_(RobotState::ROBOT_UNKNOWN), current_walk_state_(WalkState::WALK_STOPPED), current_posing_mode_(PosingMode::POSING_NONE), current_pose_reset_mode_(PoseResetMode::NO_RESET), desired_system_state_(SystemState::SUSPENDED), desired_robot_state_(RobotState::ROBOT_UNKNOWN), leg_states_{}, manual_leg_count_(0), primary_leg_selection_(-1), secondary_leg_selection_(-1), toggle_primary_leg_state_(false), toggle_secondary_leg_state_(false), is_transitioning_(false), desired_linear_velocity_(Eigen::Vector2d::Zero()), desired_angular_velocity_(0.0f), desired_body_position_(Eigen::Vector3d::Zero()), desired_body_orientation_(Eigen::Vector3d::Zero()), last_update_time_(0), time_delta_(0.02f), has_error_(false), startup_step_(0), startup_transition_initialized_(false), startup_transition_step_count_(4), shutdown_step_(0), shutdown_transition_initialized_(false), shutdown_transition_step_count_(3), executing_pack_transition_(false), is_initialized_(false) {

    // Initialize leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LegState::LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
        leg_tip_poses_[i] = Point3D();
        leg_tip_pose_valid_[i] = false;
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

bool StateController::initialize(const BodyPoseConfiguration &body_pose_config) {
    logDebug("Initializing StateController...");

    // OpenSHC parity: startup sequence behavior is controlled by pose config.
    // When false, direct startup mode is used (READY -> RUNNING immediate).
    config_.enable_startup_sequence = body_pose_config.start_up_sequence;

    // Initialize packed/ready target angles from explicit Parameters configuration when enabled.
    // Fallback keeps ready targets aligned with configured standing pose for deterministic detection.
    const Parameters &params = context_.getParams();
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (params.use_configured_packed_unpacked_poses) {
            // Use last pack step for packed detection (OpenSHC: packed_positions_.back())
            const int last_step = std::max(0, params.num_pack_steps - 1);
            packed_target_angles_[i] = JointAngles(
                params.packed_pose_steps[i][last_step].coxa,
                params.packed_pose_steps[i][last_step].femur,
                params.packed_pose_steps[i][last_step].tibia);
            ready_target_angles_[i] = JointAngles(
                params.unpacked_pose_joints[i].coxa,
                params.unpacked_pose_joints[i].femur,
                params.unpacked_pose_joints[i].tibia);
        } else {
            packed_target_angles_[i] = JointAngles(
                body_pose_config.standing_pose_joints[i].coxa,
                body_pose_config.standing_pose_joints[i].femur,
                body_pose_config.standing_pose_joints[i].tibia);
            ready_target_angles_[i] = JointAngles(
                body_pose_config.standing_pose_joints[i].coxa,
                body_pose_config.standing_pose_joints[i].femur,
                body_pose_config.standing_pose_joints[i].tibia);
        }
    }

    // Check if locomotion system is available
    if (!context_.isSystemEnabled()) {
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

    // Initialize top-level system state (OpenSHC equivalent)
    current_system_state_ = SystemState::OPERATIONAL;
    desired_system_state_ = SystemState::OPERATIONAL;

    // Initialize pose controller (equivalent to OpenSHC poser_)
    try {
        body_pose_controller_ = std::make_unique<BodyPoseController>(context_.getRobotModel(),
                                                                     body_pose_config);
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

void StateController::update(double time_delta) {
    if (!is_initialized_) {
        return;
    }

    // Update timing
    unsigned long current_time = millis();
    time_delta_ = (current_time - last_update_time_) / 1000.0f;
    last_update_time_ = current_time;

    // Main state machine update
    updateStateMachine();

    // Update walk state (mirrors WalkController's authoritative state)
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

        int primary_leg = -1;
        int secondary_leg = -1;
        getManualLegIndices(primary_leg, secondary_leg);

        Eigen::Vector3d primary_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d secondary_velocity = Eigen::Vector3d::Zero();
        if (primary_leg >= 0) {
            primary_velocity = getLegTipVelocity(primary_leg);
        }
        if (secondary_leg >= 0) {
            secondary_velocity = getLegTipVelocity(secondary_leg);
        }

        Point3D primary_pose;
        Point3D secondary_pose;
        bool primary_pose_valid = getLegTipPose(primary_leg, primary_pose);
        bool secondary_pose_valid = getLegTipPose(secondary_leg, secondary_pose);

        if (!context_.applyManualLegInputs(primary_leg,
                                           primary_velocity,
                                           secondary_leg,
                                           secondary_velocity,
                                           primary_pose_valid,
                                           primary_pose,
                                           secondary_pose_valid,
                                           secondary_pose)) {
            setError("Failed to apply manual leg inputs");
        }
    }

    // Execute one full locomotion pipeline step through the orchestration context.
    if (!context_.runControlPipelineStep()) {
        setError("Locomotion pipeline step failed");
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

bool StateController::requestLegToggle(int leg_index) {
    return requestLegToggleSlot(leg_index, /*secondary=*/false);
}

bool StateController::requestSecondaryLegToggle(int leg_index) {
    return requestLegToggleSlot(leg_index, /*secondary=*/true);
}

bool StateController::requestLegToggleSlot(int leg_index, bool secondary) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        setError("Invalid leg index for toggle: " + toArduinoString(toString(leg_index)));
        return false;
    }
    if (current_robot_state_ != RobotState::ROBOT_RUNNING) {
        setError("Cannot toggle leg state when robot is not in RUNNING state");
        return false;
    }

    int &selection = secondary ? secondary_leg_selection_ : primary_leg_selection_;
    bool &toggle_pending = secondary ? toggle_secondary_leg_state_ : toggle_primary_leg_state_;
    const int other_selection = secondary ? primary_leg_selection_ : secondary_leg_selection_;

    if (toggle_pending) {
        setError("A leg toggle is already in progress on this selection slot");
        return false;
    }
    if (leg_index == other_selection) {
        setError("Leg " + toArduinoString(toString(leg_index)) + " is already selected on the other slot");
        return false;
    }

    LegState current = leg_states_[leg_index];
    if (current == LegState::LEG_WALKING) {
        if (manual_leg_count_ >= config_.max_manual_legs) {
            setError("Maximum number of manual legs (" +
                     toArduinoString(toString(config_.max_manual_legs)) + ") already reached");
            return false;
        }
        // Begin WALKING -> WALKING_TO_MANUAL transition
        leg_states_[leg_index] = LegState::LEG_WALKING_TO_MANUAL;
        selection = leg_index;
        toggle_pending = true;
        logDebug("Leg " + toArduinoString(toString(leg_index)) + " toggle requested: WALKING -> MANUAL");
    } else if (current == LegState::LEG_MANUAL) {
        // Begin MANUAL -> MANUAL_TO_WALKING transition
        leg_states_[leg_index] = LegState::LEG_MANUAL_TO_WALKING;
        selection = leg_index;
        toggle_pending = true;
        logDebug("Leg " + toArduinoString(toString(leg_index)) + " toggle requested: MANUAL -> WALKING");
    } else {
        setError("Leg " + toArduinoString(toString(leg_index)) + " is already transitioning");
        return false;
    }
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

void StateController::setLegTipPose(int leg_index, const Point3D &position) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        leg_tip_poses_[leg_index] = position;
        leg_tip_pose_valid_[leg_index] = true;
    }
}

bool StateController::getLegTipPose(int leg_index, Point3D &out_position) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return false;
    }
    if (!leg_tip_pose_valid_[leg_index]) {
        return false;
    }
    out_position = leg_tip_poses_[leg_index];
    return true;
}

Eigen::Vector3d StateController::getLegTipVelocity(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return Eigen::Vector3d::Zero();
    }
    return leg_tip_velocities_[leg_index];
}

void StateController::getManualLegIndices(int &primary_leg, int &secondary_leg) const {
    primary_leg = -1;
    secondary_leg = -1;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (leg_states_[i] != LegState::LEG_MANUAL) {
            continue;
        }
        if (primary_leg < 0) {
            primary_leg = i;
        } else if (secondary_leg < 0) {
            secondary_leg = i;
            break;
        }
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

    // Create gait configuration and set it
    GaitConfiguration gait_config = createGaitConfig(gait, context_.getParams());
    bool changed = context_.setGaitConfiguration(gait_config);
    if (!changed) {
        return false;
    }

    // OpenSHC parity: refresh auto-pose parameters associated with the new gait.
    if (body_pose_controller_) {
        body_pose_controller_->setCurrentGaitType(gait_config.gait_type);
        body_pose_controller_->setGaitPhaseParams(gait_config.phase_config.stance_phase,
                                                  gait_config.phase_config.swing_phase,
                                                  gait_config.phase_config.phase_offset);
        body_pose_controller_->refreshAutoPoseParameters();
    }

    return true;
}

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
    info += "  Manual Legs: " + toArduinoString(toString(manual_leg_count_)) + "/" + toArduinoString(toString(config_.max_manual_legs)) + "\n";
    info += "  Transitioning: " + String(is_transitioning_ ? "Yes" : "No") + "\n";

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

    // Reset all leg tip velocities
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
        leg_tip_poses_[i] = Point3D();
        leg_tip_pose_valid_[i] = false;
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
    current_system_state_ = SystemState::SUSPENDED;
    current_robot_state_ = RobotState::ROBOT_UNKNOWN;
    current_walk_state_ = WalkState::WALK_STOPPED;
    current_posing_mode_ = PosingMode::POSING_NONE;
    current_pose_reset_mode_ = PoseResetMode::NO_RESET;

    desired_system_state_ = SystemState::SUSPENDED;
    desired_robot_state_ = RobotState::ROBOT_UNKNOWN;

    // Reset leg states
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states_[i] = LegState::LEG_WALKING;
        leg_tip_velocities_[i] = Eigen::Vector3d::Zero();
        leg_tip_poses_[i] = Point3D();
        leg_tip_pose_valid_[i] = false;
    }
    manual_leg_count_ = 0;

    // Reset control inputs
    desired_linear_velocity_ = Eigen::Vector2d::Zero();
    desired_angular_velocity_ = 0.0f;
    desired_body_position_ = Eigen::Vector3d::Zero();
    desired_body_orientation_ = Eigen::Vector3d::Zero();

    // Reset transition state
    is_transitioning_ = false;
    // Initialize pose controller
    body_pose_controller_ = nullptr;
}

void StateController::updateStateMachine() {
    // Handle system state transitions
    handleSystemStateTransition();
}

void StateController::handleSystemStateTransition() {
    if (current_system_state_ == desired_system_state_) {
        return;
    }

    switch (desired_system_state_) {
    case SystemState::SUSPENDED:
        // Stop top-level system activity while preserving robot-state machine ownership.
        emergencyStop();
        current_system_state_ = SystemState::SUSPENDED;
        logDebug("System suspended");
        break;

    case SystemState::OPERATIONAL:
        // Resume normal operations.
        current_system_state_ = SystemState::OPERATIONAL;
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
    case RobotState::ROBOT_UNKNOWN: {
        // OpenSHC parity: detect initial state, then continue toward any pending target.
        RobotState saved_desired = desired_robot_state_;
        if (isRobotPacked()) {
            current_robot_state_ = RobotState::ROBOT_PACKED;
            if (!config_.enable_startup_sequence) {
                logError("Robot is PACKED and direct startup mode is enabled; set start_up_sequence=true to unpack via startup sequence parity.");
            }
            logDebug("Robot state determined: PACKED");
        } else if (isRobotReady()) {
            if (!config_.enable_startup_sequence) {
                // OpenSHC direct mode maps READY detection to PACKED state machine entry.
                current_robot_state_ = RobotState::ROBOT_PACKED;
                logDebug("Robot detected READY but direct mode enabled: remapping to PACKED (OpenSHC parity)");
            } else {
                current_robot_state_ = RobotState::ROBOT_READY;
                logDebug("Robot state determined: READY");
            }
        } else {
            // OpenSHC parity: unknown falls back to PACKED.
            current_robot_state_ = RobotState::ROBOT_PACKED;
            logDebug("Robot state unknown: defaulting to PACKED");
        }

        // Preserve any pending transition target (e.g. ROBOT_RUNNING requested
        // before the UNKNOWN state was resolved). Only reset desired state when
        // no explicit target was requested (desired was also UNKNOWN).
        if (saved_desired != RobotState::ROBOT_UNKNOWN && saved_desired != current_robot_state_) {
            desired_robot_state_ = saved_desired;
            // Keep is_transitioning_ true so the state machine continues
            // from the newly detected state toward the original target.
            logDebug("Continuing transition toward: " + toArduinoString(toString(saved_desired)));
        } else {
            desired_robot_state_ = current_robot_state_;
            is_transitioning_ = false;
            logDebug("Robot state transition completed: " + toArduinoString(toString(current_robot_state_)));
        }
        break;
    }

    case RobotState::ROBOT_PACKED:
        if (desired_robot_state_ == RobotState::ROBOT_READY) {
            progress = executeUnpackSequence();
        } else if (desired_robot_state_ == RobotState::ROBOT_RUNNING) {
            // Unpack first, then continue to startup sequence (or direct).
            progress = executeUnpackSequence();
            if (progress == PROGRESS_COMPLETE) {
                // Intermediate: move to READY; do NOT report COMPLETE yet so
                // the outer handler keeps is_transitioning_ and the next tick
                // enters ROBOT_READY → ROBOT_RUNNING.
                current_robot_state_ = RobotState::ROBOT_READY;
                progress = PROGRESS_COMPLETE - 1;
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
                // OpenSHC direct mode: READY -> RUNNING immediate.
                context_.activateRunningState();
                current_robot_state_ = RobotState::ROBOT_RUNNING;
                current_walk_state_ = WalkState::WALK_STOPPED;
                progress = PROGRESS_COMPLETE;
                logDebug("Direct startup to RUNNING state");
            }
        }
        break;

    case RobotState::ROBOT_RUNNING:
        if (desired_robot_state_ == RobotState::ROBOT_READY) {
            if (!config_.enable_startup_sequence) {
                // OpenSHC direct mode: disallow RUNNING -> non-RUNNING transitions.
                is_transitioning_ = false;
                logDebug("RUNNING -> READY is blocked while direct startup mode is active (OpenSHC parity)");
                return;
            }

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

    // OpenSHC parity: walk state transitions are handled inside WalkController::updateWalk().
    // StateController only mirrors the walk state for external queries.
    // Do NOT independently change walk_state_ here — it is authoritative in WalkController.
    WalkController *walker = context_.getWalkController();
    if (walker) {
        current_walk_state_ = walker->getWalkState();
    }
}

void StateController::handleLegStateTransitions() {
    // OpenSHC legStateToggle equivalent: handles gradual leg state transitions via
    // poseForLegManipulation. Two independent selection slots (primary/secondary) are supported so
    // up to max_manual_legs legs can be manipulated, matching OpenSHC's dual-selection design.

    if (!toggle_primary_leg_state_ && !toggle_secondary_leg_state_) {
        return;
    }

    // Must be WALK_STOPPED before transitions can proceed (OpenSHC forces stop first)
    if (current_walk_state_ != WalkState::WALK_STOPPED) {
        // Force velocity to zero so the walker reaches STOPPED
        desired_linear_velocity_ = Eigen::Vector2d::Zero();
        desired_angular_velocity_ = 0.0;
        return;
    }

    // OpenSHC processes the primary slot first, then the secondary slot.
    if (toggle_primary_leg_state_) {
        processLegToggleSlot(primary_leg_selection_, toggle_primary_leg_state_);
    }
    if (toggle_secondary_leg_state_) {
        processLegToggleSlot(secondary_leg_selection_, toggle_secondary_leg_state_);
    }
}

void StateController::processLegToggleSlot(int leg_index, bool &toggle_pending) {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        toggle_pending = false;
        return;
    }

    LegState &state = leg_states_[leg_index];

    // Retrieve body pose controller for poseForLegManipulation
    BodyPoseController *bpc = context_.getBodyPoseController();

    switch (state) {
    case LegState::LEG_WALKING_TO_MANUAL: {
        // Drive all legs to manipulation-ready poses via poseForLegManipulation
        int progress = 100;
        if (bpc) {
            progress = bpc->poseForLegManipulation();
        }

        // Update admittance stiffness during transition (OpenSHC: scale 0->1)
        {
            double scale_reference = static_cast<double>(progress) / PROGRESS_COMPLETE;
            context_.updateAdmittanceStiffness(leg_index, scale_reference);
        }

        if (progress >= 100) {
            state = LegState::LEG_MANUAL;
            manual_leg_count_++;
            toggle_pending = false;
            logDebug("Leg " + toArduinoString(toString(leg_index)) + " set to state: MANUAL");
        }
        break;
    }
    case LegState::LEG_MANUAL_TO_WALKING: {
        // Drive all legs back to walking-ready poses via poseForLegManipulation
        int progress = 100;
        if (bpc) {
            progress = bpc->poseForLegManipulation();
        }

        // Update admittance stiffness during transition (OpenSHC: scale 1->0)
        {
            double scale_reference = std::abs(static_cast<double>(progress) / PROGRESS_COMPLETE - 1.0);
            context_.updateAdmittanceStiffness(leg_index, scale_reference);
        }

        if (progress >= 100) {
            state = LegState::LEG_WALKING;
            manual_leg_count_--;
            toggle_pending = false;
            logDebug("Leg " + toArduinoString(toString(leg_index)) + " set to state: WALKING");
        }
        break;
    }
    default:
        // No valid transition state; clear pending flag
        toggle_pending = false;
        break;
    }
}

void StateController::updateVelocityControl() {
    double linear_x = desired_linear_velocity_.x();
    double linear_y = desired_linear_velocity_.y();
    double angular_z = desired_angular_velocity_;

    // Apply velocity control using combined gait planning (equivalent to OpenSHC walker_->updateWalk())
    if (!context_.planGaitSequence(linear_x, linear_y, angular_z)) {
        logError("Failed to plan gait sequence for velocity control");
        has_error_ = true;
        last_error_message_ = "Velocity control gait planning failed";
    }
    // OpenSHC pattern: zero velocity simply causes WalkController to
    // transition through STOPPING → STOPPED naturally while remaining
    // in RUNNING state. Do NOT call stopWalkingUniform() here — that
    // forces an immediate hard stop and transitions to READY, which
    // breaks the normal velocity-based control flow.
}

void StateController::updatePoseControl() {
    // OpenSHC parity: posing mode is primarily informative. Control gating is external.
    // Always forward full manual pose input here; mode changes are kept for user diagnostics.
    if (!context_.setManualBodyPoseInput(desired_body_position_, desired_body_orientation_)) {
        logError("Failed to apply manual body pose input");
    }

    // Apply pose reset if needed.
    if (current_pose_reset_mode_ != PoseResetMode::NO_RESET) {
        applyPoseReset();
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
    case PoseResetMode::Z_AND_YAW_RESET: {
        // Reset Z position and yaw orientation
        reset_position.z() = 0.0f;
        reset_orientation.z() = 0.0f; // yaw
        logDebug("Applying Z and Yaw pose reset");
        break;
    }

    case PoseResetMode::X_AND_Y_RESET: {
        // Reset X and Y positions
        reset_position.x() = 0.0f;
        reset_position.y() = 0.0f;
        logDebug("Applying X and Y pose reset");
        break;
    }

    case PoseResetMode::PITCH_AND_ROLL_RESET: {
        // Reset pitch and roll orientations
        reset_orientation.x() = 0.0f; // roll
        reset_orientation.y() = 0.0f; // pitch
        logDebug("Applying pitch and roll pose reset");
        break;
    }

    case PoseResetMode::ALL_RESET: {
        // Reset all pose parameters gradually
        reset_position = Eigen::Vector3d::Zero();
        reset_orientation = Eigen::Vector3d::Zero();
        logDebug("Applying full pose reset");
        break;
    }

    case PoseResetMode::IMMEDIATE_ALL_RESET: {
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
    bool success = context_.setManualBodyPoseInput(reset_position, reset_orientation);

    if (success) {
        // Update internal desired pose state
        desired_body_position_ = reset_position;
        desired_body_orientation_ = reset_orientation;

        // Clear the reset mode after successful application
        current_pose_reset_mode_ = PoseResetMode::NO_RESET;

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
    // Delegate to LocomotionSystem startup sequence (OpenSHC parity)
    bool complete = context_.executeStartupSequence();
    if (complete) {
        startup_step_ = 0;
        startup_transition_initialized_ = false;
        return PROGRESS_COMPLETE;
    }

    // Provide incremental progress from the locomotion system
    int progress = context_.getStartupProgressPercent();
    if (progress < 0)
        progress = 0;
    if (progress >= PROGRESS_COMPLETE)
        progress = PROGRESS_COMPLETE - 1;
    return progress;
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
    // Ensure walking has stopped before shutdown
    if (current_walk_state_ != WALK_STOPPED) {
        desired_linear_velocity_.setZero();
        desired_angular_velocity_ = 0.0f;
        return 10; // Stay in shutdown but indicate progress
    }

    bool complete = context_.executeShutdownSequence();
    if (complete) {
        shutdown_step_ = 0;
        shutdown_transition_initialized_ = false;
        return PROGRESS_COMPLETE;
    }

    return 0;
}

int StateController::executePackSequence() {
    BodyPoseController *bpc = context_.getBodyPoseController();
    if (!bpc) {
        return 0;
    }
    const Parameters &params = context_.getParams();
    double transition_time = PACK_TIME / params.step_frequency;
    return bpc->packLegs(transition_time, context_.getLegsArray());
}

int StateController::executeUnpackSequence() {
    BodyPoseController *bpc = context_.getBodyPoseController();
    if (!bpc) {
        return 0;
    }
    const Parameters &params = context_.getParams();
    double transition_time = PACK_TIME / params.step_frequency;
    return bpc->unpackLegs(transition_time, context_.getLegsArray());
}

bool StateController::isRobotPacked() const {
    const Parameters &params = context_.getParams();

    // Joint-based packed check
    bool joints_ok = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles cur = context_.getJointAngles(i);
        JointAngles tgt = packed_target_angles_[i];
        if (abs(cur.coxa - tgt.coxa) > JOINT_TOLERANCE ||
            abs(cur.femur - tgt.femur) > JOINT_TOLERANCE ||
            abs(cur.tibia - tgt.tibia) > JOINT_TOLERANCE) {
            joints_ok = false;
            break;
        }
    }

    if (params.use_configured_packed_unpacked_poses) {
        return joints_ok;
    }

    // Heuristic fallback path when no explicit packed configuration exists.
    Eigen::Vector3d current_position = context_.getBodyPosition();

    // Check for invalid/uninitialized position data
    if (current_position.norm() < 0.01f) {
        // Position is zero or very close to zero - this could indicate:
        // 1. Sensor failure or uninitialized state
        // 2. Robot actually at origin (rare but possible)

        // Fallback: check if we have valid orientation data
        Eigen::Vector3d current_orientation = context_.getBodyOrientation();
        if (current_orientation.norm() < 0.01f) {
            // Both position and orientation are zero - assume not packed for safety
            return false;
        }

        // If orientation is available but position isn't, use orientation-based heuristic
        // Large roll/pitch might indicate packed/fallen state
        return (abs(current_orientation.x()) > TIBIA_ANGLE_MAX || abs(current_orientation.y()) > TIBIA_ANGLE_MAX);
    }

    bool body_low = current_position.z() < 50.0f;
    return body_low && joints_ok;
}

bool StateController::isRobotReady() const {
    const Parameters &params = context_.getParams();

    // Joint-based ready check
    bool joints_ok = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles cur = context_.getJointAngles(i);
        JointAngles tgt = ready_target_angles_[i];
        if (abs(cur.coxa - tgt.coxa) > JOINT_TOLERANCE ||
            abs(cur.femur - tgt.femur) > JOINT_TOLERANCE ||
            abs(cur.tibia - tgt.tibia) > JOINT_TOLERANCE) {
            joints_ok = false;
            break;
        }
    }

    if (params.use_configured_packed_unpacked_poses) {
        return joints_ok;
    }

    // Heuristic fallback path when no explicit unpacked configuration exists.
    Eigen::Vector3d current_position = context_.getBodyPosition();
    Eigen::Vector3d current_orientation = context_.getBodyOrientation();

    // Check for invalid/uninitialized position data
    if (current_position.norm() < 0.01f) {
        return false; // Cannot determine if ready with invalid data
    }

    // Check if at reasonable height and level orientation
    // Note: body z can be negative depending on coordinate convention,
    // so use abs(z) for height magnitude check.
    double z_abs = abs(current_position.z());
    bool height_ok = (z_abs > 80.0f && z_abs < 300.0f);
    bool orientation_ok = (abs(current_orientation.x()) < 10.0f && abs(current_orientation.y()) < 10.0f);
    bool body_ready = height_ok && orientation_ok;
    return body_ready && joints_ok;
}

void StateController::notifyRobotReady() {
    // Set robot state to READY directly and capture current joint angles
    // as the ready-state reference for future detection.
    current_robot_state_ = RobotState::ROBOT_READY;
    if (desired_robot_state_ == RobotState::ROBOT_UNKNOWN) {
        desired_robot_state_ = RobotState::ROBOT_READY;
    }
    for (int i = 0; i < NUM_LEGS; ++i) {
        ready_target_angles_[i] = context_.getJointAngles(i);
    }
    logDebug("Robot state forced to READY via notifyRobotReady()");
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
