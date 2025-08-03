#include "walk_controller.h"
#include "body_pose_config.h"
#include "gait_config_factory.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "workspace_analyzer.h" // Use unified analyzer instead
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

WalkController::WalkController(RobotModel &m, Leg legs[NUM_LEGS], const BodyPoseConfiguration &pose_config)
    : model(m), terrain_adaptation_(m), body_pose_controller_(nullptr),
      desired_linear_velocity_(0, 0, 0), desired_angular_velocity_(0.0),
      walk_state_(WALK_STOPPED), pose_state_(0),
      regenerate_walkspace_(false), legs_at_correct_phase_(0), legs_completed_first_step_(0),
      return_to_default_attempted_(false), velocity_limits_(m), legs_array_(legs) {

    // Initialize leg_steppers_ with references to actual legs from LocomotionSystem
    leg_steppers_.clear();

    // Initialize gait configuration system (OpenSHC equivalent)
    gait_selection_config_ = createGaitSelectionConfig(model.getParams());
    std::string default_gait_name = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    GaitType default_gait_type = stringToGaitType(default_gait_name);
    setGait(default_gait_type);

    // Initialize workspace analyzer (unified class replaces both validator and analyzer)
    workspace_analyzer_ = std::make_unique<WorkspaceAnalyzer>(model);

    // Create LegStepper objects for each leg
    for (int i = 0; i < NUM_LEGS; i++) {

        // Use leg stance position to calculate identity tip pose
        const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[i];

        // Calculate the identity tip pose from the leg stance position
        // This assumes the stance position is in the robot's body frame
        // For HexaMotion, use actual standing height instead of Z=0
        Point3D identity_tip_pose = Point3D(
            leg_stance_position.x,
            leg_stance_position.y,
            leg_stance_position.z); // Use standing height for HexaMotion compatibility

        // Update terrain adaptation parameters
        auto stepper = std::make_shared<LegStepper>(i, identity_tip_pose, legs[i], model,
                                                    workspace_analyzer_.get());
        leg_steppers_.push_back(stepper);
    }

    // Initialize terrain adaptation
    terrain_adaptation_.initialize();

    // Set initial time delta
    time_delta_ = 1.0 / model.getParams().control_frequency;

    // Generate initial walkspace
    generateWalkspace();
}

// Gait configuration management methods (OpenSHC equivalent)
bool WalkController::setGaitConfiguration(const GaitConfiguration &gait_config) {
    // Store the new gait configuration
    current_gait_config_ = gait_config;

    // Apply the configuration to all leg steppers
    applyGaitConfigToLegSteppers(gait_config);

    // Update gait selection config
    gait_selection_config_.current_gait = gait_config.gait_name;

    return true;
}

bool WalkController::setGait(GaitType gait_type) {
    // Get gait configuration from factory using the robot parameters
    const Parameters &params = model.getParams();
    GaitConfiguration gait_config;

    switch (gait_type) {
    case TRIPOD_GAIT:
        gait_config = createTripodGaitConfig(params);
        break;
    case WAVE_GAIT:
        gait_config = createWaveGaitConfig(params);
        break;
    case RIPPLE_GAIT:
        gait_config = createRippleGaitConfig(params);
        break;
    case METACHRONAL_GAIT:
        gait_config = createMetachronalGaitConfig(params);
        break;
    default:
        // Unsupported gait type, return false
        return false;
    }

    return setGaitConfiguration(gait_config);
}

void WalkController::applyGaitConfigToLegSteppers(const GaitConfiguration &gait_config) {
    // Use configured step frequency from gait configuration (OpenSHC pattern)
    // This ensures consistency with trajectory_tip_position_test and OpenSHC behavior
    double step_frequency = gait_config.getStepFrequency(); // OpenSHC configured frequency

    // Generate StepCycle with configured frequency like OpenSHC
    StepCycle step_cycle = gait_config.generateStepCycle();

    // Apply StepCycle and gait configuration to each LegStepper
    for (int i = 0; i < NUM_LEGS && i < static_cast<int>(leg_steppers_.size()); i++) {
        auto leg_stepper = leg_steppers_[i];
        if (!leg_stepper)
            continue;

        // Set StepCycle (OpenSHC style - single call instead of multiple parameters)
        leg_stepper->setStepCycle(step_cycle);

        // Set gait-specific parameters (not part of StepCycle)
        leg_stepper->setSwingWidth(gait_config.swing_width);
        leg_stepper->setControlFrequency(gait_config.control_frequency);
        leg_stepper->setStepClearanceHeight(gait_config.swing_height);

        // Calculate phase offset using OpenSHC formula
        // OpenSHC: step_offset = (base_step_offset * multiplier) % step.period_
        int base_step_period = gait_config.phase_config.stance_phase + gait_config.phase_config.swing_phase;
        int normaliser = step_cycle.period_ / base_step_period;
        int base_step_offset = gait_config.phase_config.phase_offset * normaliser;
        int multiplier = gait_config.offsets.getForLegIndex(i);
        int phase_offset_iterations = (base_step_offset * multiplier) % step_cycle.period_;
        double phase_offset = static_cast<double>(phase_offset_iterations) / static_cast<double>(step_cycle.period_);
        leg_stepper->setPhaseOffset(phase_offset);

        // OpenSHC: Configure desired velocity for stride calculation
        leg_stepper->setDesiredVelocity(desired_linear_velocity_, desired_angular_velocity_);
    }

    // Update terrain adaptation parameters
    terrain_adaptation_.setRoughTerrainMode(gait_config.supports_rough_terrain);
}

// Terrain adaptation methods
void WalkController::enableRoughTerrainMode(bool enabled, bool force_normal_touchdown,
                                            bool proactive_adaptation) {
    terrain_adaptation_.setRoughTerrainMode(enabled);
    terrain_adaptation_.setForceNormalTouchdown(force_normal_touchdown);

    if (proactive_adaptation) {
        // Enable proactive terrain adaptation features
        terrain_adaptation_.setGravityAlignedTips(true);
    }
}

void WalkController::enableForceNormalTouchdown(bool enabled) {
    terrain_adaptation_.setForceNormalTouchdown(enabled);
}

void WalkController::enableGravityAlignedTips(bool enabled) {
    terrain_adaptation_.setGravityAlignedTips(enabled);
}

void WalkController::setExternalTarget(int leg_index, const TerrainAdaptation::ExternalTarget &target) {
    terrain_adaptation_.setExternalTarget(leg_index, target);
}

void WalkController::setExternalDefault(int leg_index, const TerrainAdaptation::ExternalTarget &default_pos) {
    terrain_adaptation_.setExternalDefault(leg_index, default_pos);
}

// Terrain state accessors
const TerrainAdaptation::WalkPlane &WalkController::getTerrainWalkPlane() const {
    return terrain_adaptation_.getWalkPlane();
}

const TerrainAdaptation::ExternalTarget &WalkController::getExternalTarget(int leg_index) const {
    return terrain_adaptation_.getExternalTarget(leg_index);
}

const TerrainAdaptation::StepPlane &WalkController::getStepPlane(int leg_index) const {
    return terrain_adaptation_.getStepPlane(leg_index);
}

bool WalkController::hasTouchdownDetection(int leg_index) const {
    return terrain_adaptation_.hasTouchdownDetection(leg_index);
}

Point3D WalkController::estimateGravity() const {
    // Simple gravity estimation - in a real implementation this would use IMU data
    return Point3D(0.0, 0.0, -9.81);
}

// Velocity limiting methods
VelocityLimits::LimitValues WalkController::getVelocityLimits(double bearing_degrees) const {
    return velocity_limits_.getLimit(bearing_degrees);
}

VelocityLimits::LimitValues WalkController::applyVelocityLimits(double vx, double vy, double omega) const {
    // Calculate bearing from velocity components
    double bearing = VelocityLimits::calculateBearing(vx, vy);

    // Get limits for this bearing
    VelocityLimits::LimitValues limits = velocity_limits_.getLimit(bearing);

    // Apply limits to input velocities
    VelocityLimits::LimitValues limited_velocities;
    limited_velocities.linear_x = std::max(-limits.linear_x, std::min(limits.linear_x, vx));
    limited_velocities.linear_y = std::max(-limits.linear_y, std::min(limits.linear_y, vy));
    limited_velocities.angular_z = std::max(-limits.angular_z, std::min(limits.angular_z, omega));
    limited_velocities.acceleration = limits.acceleration;

    return limited_velocities;
}

bool WalkController::validateVelocityCommand(double vx, double vy, double omega) const {
    return velocity_limits_.validateVelocityInputs(vx, vy, omega);
}

void WalkController::updateVelocityLimits(double frequency, double stance_ratio, double time_to_max_stride) {
    // Update time_to_max_stride if provided
    current_gait_config_.time_to_max_stride = time_to_max_stride;

    // Note: step_frequency, stance_ratio, and swing_ratio are now calculated from phase_config
    // If dynamic frequency updates are needed, they should modify phase_config and regenerate StepCycle

    // For now, we don't dynamically update phase ratios as they are intrinsic to each gait type
    // Use unified interface
    velocity_limits_.updateGaitParameters(current_gait_config_);
}

void WalkController::setVelocitySafetyMargin(double margin) {
    velocity_limits_.setSafetyMargin(margin);
}

void WalkController::setAngularVelocityScaling(double scaling) {
    velocity_limits_.setAngularVelocityScaling(scaling);
}

const VelocityLimits::LimitValues &WalkController::getCurrentVelocities() const {
    return current_velocities_;
}

VelocityLimits::WorkspaceConfig WalkController::getWorkspaceConfig() const {
    return velocity_limits_.getWorkspaceConfig();
}

// --- WalkController Methods Implementation ---
void WalkController::init(const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation) {

    walk_state_ = WALK_STOPPED;
    odometry_ideal_ = Point3D(0, 0, 0);

    // Store current robot pose
    current_body_position_ = current_body_position;
    current_body_orientation_ = current_body_orientation;

    // OpenSHC approach: Preserve LegStepper configuration instead of recalculating
    // The LegSteppers were initialized with proper default_tip_pose_ from BodyPoseConfiguration
    // OpenSHC pattern: Don't override configured stance positions during init
    for (auto &leg_stepper : leg_steppers_) {
        int leg_index = leg_stepper->getLegIndex();

        // OpenSHC exact pattern: Use configured stance positions from leg stepper
        // These were properly set during constructor using BodyPoseConfiguration
        Point3D configured_stance_pose = leg_stepper->getDefaultTipPose();

        // OpenSHC validation: Only set if not properly configured (norm check for uninitialized poses)
        if (configured_stance_pose.norm() < 1.0) {
            // Fallback: Use robot model to get properly configured stance positions
            // This follows OpenSHC's approach of using leg-specific configuration
            Point3D base_pos = model.getLegBasePosition(leg_index);
            double base_angle = model.getLegBaseAngleOffset(leg_index);

            // OpenSHC style: Use conservative stance radius based on leg geometry
            double leg_reach = model.getLegReach();                          // Use RobotModel method instead of manual calculation
            double stance_radius = leg_reach * DEFAULT_STANCE_RADIUS_FACTOR; // Use defined constant

            Point3D stance_position(
                base_pos.x + stance_radius * cos(base_angle),
                base_pos.y + stance_radius * sin(base_angle),
                current_body_position.z());

            leg_stepper->setDefaultTipPose(stance_position);
        } // OpenSHC pattern: Initialize current tip pose to default stance position
        // This ensures LegStepper starts with proper stance coordinates
        leg_stepper->setCurrentTipPose(leg_stepper->getDefaultTipPose());
    }

    // Init velocity input variables
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0;

    // Initialize global phase counter
    global_phase_ = 0;
}

/**
 * @brief Optimized WalkController::updateWalk method following OpenSHC design principles
 *
 * Key optimizations implemented:
 * 1. Early exit for STOPPED state without velocity commands
 * 2. Single-pass calculation of step cycle to avoid redundant computations
 * 3. Combined loops for leg state checking and processing
 * 4. Efficient velocity limiting using magnitude-based scaling
 * 5. Switch-case state machine for better branch prediction
 * 6. Reduced function calls by caching frequently used values
 * 7. Conditional processing to avoid unnecessary calculations
 *
 * @param linear_velocity_input Desired linear velocity (m/s)
 * @param angular_velocity_input Desired angular velocity (rad/s)
 * @param current_body_position Current robot body position
 * @param current_body_orientation Current robot body orientation
 */
void WalkController::updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input,
                                const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation) {
    // OpenSHC: Cache frequently used values to reduce function call overhead
    const Parameters &params = model.getParams();
    current_body_position_ = current_body_position;
    current_body_orientation_ = current_body_orientation;

    // OpenSHC: Early exit optimization for STOPPED state
    const bool has_velocity_command = (linear_velocity_input.x != 0.0 || linear_velocity_input.y != 0.0 || angular_velocity_input != 0.0);

    if (walk_state_ == WALK_STOPPED && !has_velocity_command) {
        // Optimize: No processing needed if stopped and no command
        for (size_t i = 0; i < leg_steppers_.size() && i < NUM_LEGS; ++i) {
            leg_steppers_[i]->setStepState(STEP_FORCE_STOP);
            leg_steppers_[i]->setPhase(0.0);
            legs_array_[i].setStepPhase(STANCE_PHASE);
        }
        return;
    }

    // OpenSHC: Optimized velocity limiting calculation
    Point3D new_linear_velocity, limited_linear_velocity;
    double new_angular_velocity, limited_angular_velocity;

    if (walk_state_ != WALK_STOPPING && has_velocity_command) {
        // Calculate bearing once for velocity limits
        const double bearing_rad = atan2(linear_velocity_input.y, linear_velocity_input.x);
        const double bearing_degrees = bearing_rad * 180.0 / M_PI + (bearing_rad < 0 ? 360.0 : 0.0);
        const VelocityLimits::LimitValues limits = velocity_limits_.getLimit(bearing_degrees);

        // Apply velocity magnitude limiting efficiently
        const double input_magnitude = sqrt(linear_velocity_input.x * linear_velocity_input.x +
                                            linear_velocity_input.y * linear_velocity_input.y);
        const double scale_factor = (input_magnitude > limits.linear_x && input_magnitude > 0.0) ? limits.linear_x / input_magnitude : 1.0;

        new_linear_velocity = linear_velocity_input * scale_factor;
        new_angular_velocity = math_utils::clamp(angular_velocity_input, -limits.angular_z, limits.angular_z);

        // OpenSHC: Optimized acceleration limiting
        const Point3D linear_diff = new_linear_velocity - desired_linear_velocity_;
        const double linear_diff_mag = sqrt(linear_diff.x * linear_diff.x + linear_diff.y * linear_diff.y);
        const double max_linear_change = limits.acceleration * time_delta_;

        if (linear_diff_mag <= max_linear_change) {
            limited_linear_velocity = new_linear_velocity;
        } else {
            const double norm_factor = max_linear_change / linear_diff_mag;
            limited_linear_velocity = desired_linear_velocity_ + linear_diff * norm_factor;
        }

        const double angular_diff = new_angular_velocity - desired_angular_velocity_;
        const double max_angular_change = limits.acceleration * time_delta_;
        limited_angular_velocity = desired_angular_velocity_ + math_utils::clamp(angular_diff, -max_angular_change, max_angular_change);
    } else {
        // Zero velocities for stopping/stopped
        limited_linear_velocity = Point3D(0, 0, 0);
        limited_angular_velocity = 0.0;
    }

    desired_linear_velocity_ = limited_linear_velocity;
    desired_angular_velocity_ = limited_angular_velocity;

    // OpenSHC: Optimized state machine with combined leg state checking
    legs_at_correct_phase_ = 0;
    legs_completed_first_step_ = 0;

    // Single loop to check all leg states
    for (const auto &leg_stepper : leg_steppers_) {
        if (leg_stepper->isAtCorrectPhase())
            legs_at_correct_phase_++;
        if (leg_stepper->hasCompletedFirstStep())
            legs_completed_first_step_++;
    }

    const int leg_count = NUM_LEGS;

    // OpenSHC: Optimized state transitions
    switch (walk_state_) {
    case WALK_STOPPED:
        if (has_velocity_command) {
            walk_state_ = WALK_STARTING;
            global_phase_ = 0;
            for (auto &leg_stepper : leg_steppers_) {
                leg_stepper->setAtCorrectPhase(false);
                leg_stepper->setCompletedFirstStep(false);
                leg_stepper->setStepState(STEP_STANCE);
            }
            return;
        }
        break;

    case WALK_STARTING:
        if (legs_at_correct_phase_ == leg_count && legs_completed_first_step_ == leg_count) {
            walk_state_ = WALK_MOVING;
        }
        break;

    case WALK_MOVING:
        if (!has_velocity_command) {
            walk_state_ = WALK_STOPPING;
            for (auto &leg_stepper : leg_steppers_) {
                leg_stepper->setStepState(STEP_STANCE);
            }
        }
        break;

    case WALK_STOPPING:
        if (legs_at_correct_phase_ == leg_count && pose_state_ == 0) {
            walk_state_ = WALK_STOPPED;
        }
        break;

    case WALK_STATE_COUNT:
        // OpenSHC: Invalid state - should never occur
        // Reset to safe state
        walk_state_ = WALK_STOPPED;
        break;
    }

    // OpenSHC: Pre-calculate shared values for leg processing
    const bool is_active_walking = (walk_state_ == WALK_MOVING || walk_state_ == WALK_STARTING);
    StepCycle step_cycle;
    bool step_cycle_calculated = false;

    if (is_active_walking) {
        // Use configured step frequency from gait configuration (OpenSHC pattern)
        step_cycle = current_gait_config_.generateStepCycle();
        global_phase_ = (global_phase_ + 1) % step_cycle.period_;
        step_cycle_calculated = true;
    }

    // OpenSHC: Optimized leg processing loop
    const size_t leg_stepper_count = std::min(static_cast<size_t>(NUM_LEGS), leg_steppers_.size());
    for (size_t i = 0; i < leg_stepper_count; ++i) {
        auto &leg_stepper = leg_steppers_[i];

        // Set velocity once per leg
        leg_stepper->setDesiredVelocity(desired_linear_velocity_, desired_angular_velocity_);

        if (is_active_walking && step_cycle_calculated) {
            // OpenSHC: Optimized phase calculation
            const int phase_offset_iterations = static_cast<int>(leg_stepper->getPhaseOffset() * step_cycle.period_);
            const int leg_phase = (global_phase_ + phase_offset_iterations) % step_cycle.period_;
            const bool in_swing = (leg_phase >= step_cycle.stance_period_);

            // Update states only when necessary
            const StepState current_state = leg_stepper->getStepState();
            if (in_swing && current_state != STEP_SWING) {
                leg_stepper->setStepState(STEP_SWING);
                leg_stepper->initializeSwingPeriod(1);
            } else if (!in_swing && current_state != STEP_STANCE) {
                leg_stepper->setStepState(STEP_STANCE);
            }

            legs_array_[i].setStepPhase(in_swing ? SWING_PHASE : STANCE_PHASE);
            leg_stepper->updateTipPositionIterative(global_phase_, time_delta_, false, false);
        } else {
            // Force stance for non-active states
            legs_array_[i].setStepPhase(STANCE_PHASE);
        }
    }

    // OpenSHC: Optimized analysis and odometry updates
    odometry_ideal_ = odometry_ideal_ + calculateOdometry(time_delta_);

    // TODO: Enable this when walkspace analysis is implemented
    // OpenSHC: Conditional walkspace analysis with optimized stability control
    // if (workspace_analyzer_ && workspace_analyzer_->isAnalysisEnabled()) {
    //     // Batch update leg positions
    //     for (int i = 0; i < NUM_LEGS; ++i) {
    //         current_leg_positions_[i] = legs_array_[i].getCurrentTipPositionGlobal();
    //     }

    //     const WalkspaceAnalyzer::WalkspaceResult analysis_result =
    //         workspace_analyzer_->analyzeWalkspace(current_leg_positions_);

    //     // OpenSHC: Optimized adaptive velocity control
    //     if (walk_state_ == WALK_MOVING) {
    //         if (!analysis_result.is_stable) {
    //             const double stability_factor = math_utils::clamp(analysis_result.stability_margin / 50.0, 0.1, 1.0);
    //             desired_linear_velocity_ = desired_linear_velocity_ * stability_factor;
    //             desired_angular_velocity_ *= stability_factor;
    //         } else {
    //             const double stability_score = workspace_analyzer_->getAnalysisInfo().overall_stability_score;
    //             if (stability_score > 0.8) {
    //                 const double boost_factor = std::min(1.2, 1.0 + (stability_score - 0.8) * 0.5);
    //                 desired_linear_velocity_ = desired_linear_velocity_ * boost_factor;
    //                 desired_angular_velocity_ *= boost_factor;
    //             }
    //         }
    //     }
    // }

    // OpenSHC: Conditional walkspace regeneration
    if (regenerate_walkspace_) {
        generateWalkspace();
    }
}

Point3D WalkController::calculateOdometry(double time_period) {
    Point3D desired_linear_velocity(desired_linear_velocity_.x, desired_linear_velocity_.y, 0);
    Point3D position_delta = desired_linear_velocity * time_period;

    // Implement proper rotation delta calculation
    double angular_velocity = desired_angular_velocity_;
    double rotation_delta = angular_velocity * time_period;

    // Apply rotation to position delta
    double cos_rot = cos(rotation_delta);
    double sin_rot = sin(rotation_delta);

    Point3D rotated_delta;
    rotated_delta.x = position_delta.x * cos_rot - position_delta.y * sin_rot;
    rotated_delta.y = position_delta.x * sin_rot + position_delta.y * cos_rot;
    rotated_delta.z = position_delta.z;

    return rotated_delta;
}

// TODO: Use defines
void WalkController::generateWalkspace() {
    // Implement full walkspace calculation like OpenSHC
    walkspace_.clear();

    // Get robot parameters for workspace calculation
    double leg_reach = model.getLegReach();                       // Use RobotModel method
    double safe_reach = leg_reach * DEFAULT_STANCE_RADIUS_FACTOR; // Use defined constant

    // Calculate workspace for each bearing angle
    for (int bearing = 0; bearing <= 360; bearing += 10) {
        double bearing_rad = bearing * M_PI / 180.0;

        // Calculate workspace radius based on leg geometry and joint limits
        double max_radius = 0.0;

        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Get leg base position
            Point3D base_pos = model.getLegBasePosition(leg);
            double base_x = base_pos.x;
            double base_y = base_pos.y;

            // Calculate reach in bearing direction
            double target_x = base_x + safe_reach * cos(bearing_rad);
            double target_y = base_y + safe_reach * sin(bearing_rad);

            // Check if target is reachable by this leg, using current body position
            // Use inverse kinematics to find angles for this target position
            Point3D target(target_x, target_y, current_body_position_.z());
            // For workspace generation, use zero angles as starting point
            JointAngles zero_angles(0, 0, 0);
            JointAngles angles = model.inverseKinematicsCurrentGlobalCoordinates(leg, zero_angles, target);

            if (model.checkJointLimits(leg, angles)) {
                double distance = sqrt((target_x - base_x) * (target_x - base_x) +
                                       (target_y - base_y) * (target_y - base_y));
                max_radius = std::max(max_radius, distance);
            }
        }

        walkspace_[bearing] = max_radius;
    }

    regenerate_walkspace_ = false;

    // Use unified configuration interface - no more conversions needed!
    velocity_limits_.generateLimits(current_gait_config_);
}

// Accessor methods
std::shared_ptr<LegStepper> WalkController::getLegStepper(int leg_index) const {
    if (leg_index >= 0 && leg_index < (int)leg_steppers_.size()) {
        return leg_steppers_[leg_index];
    }
    return nullptr;
}

// ===== WALKSPACE ANALYZER CONTROL METHODS (OpenSHC equivalent) =====

void WalkController::enableWalkspaceAnalysis(bool enabled) {
    if (workspace_analyzer_) {
        workspace_analyzer_->enableAnalysis(enabled);
    }
}

bool WalkController::isWalkspaceAnalysisEnabled() const {
    return workspace_analyzer_ ? workspace_analyzer_->isAnalysisEnabled() : false;
}

const WorkspaceAnalyzer::AnalysisInfo &WalkController::getWalkspaceAnalysisInfo() const {
    static WorkspaceAnalyzer::AnalysisInfo empty_info;
    return workspace_analyzer_ ? workspace_analyzer_->getAnalysisInfo() : empty_info;
}

std::string WalkController::getWalkspaceAnalysisInfoString() const {
    return workspace_analyzer_ ? workspace_analyzer_->getAnalysisInfoString() : "WorkspaceAnalyzer not available";
}

void WalkController::resetWalkspaceAnalysisStats() {
    if (workspace_analyzer_) {
        workspace_analyzer_->resetAnalysisStats();
    }
}

WorkspaceAnalyzer::WalkspaceResult WalkController::analyzeCurrentWalkspace() {
    if (!workspace_analyzer_) {
        return WorkspaceAnalyzer::WalkspaceResult();
    }

    // Use current leg positions for analysis
    Point3D current_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        if (i < (int)leg_steppers_.size() && leg_steppers_[i]) {
            current_positions[i] = leg_steppers_[i]->getCurrentTipPose();
        } else {
            current_positions[i] = current_leg_positions_[i];
        }
    }

    return workspace_analyzer_->analyzeWalkspace(current_positions);
}

bool WalkController::generateWalkspaceMap() {
    if (!workspace_analyzer_) {
        return false;
    }

    try {
        workspace_analyzer_->generateWorkspace();
        return true;
    } catch (...) {
        return false;
    }
}

double WalkController::getWalkspaceRadius(double bearing_degrees) const {
    return workspace_analyzer_ ? workspace_analyzer_->getWalkspaceRadius(bearing_degrees) : 0.0;
}

const std::map<int, double> &WalkController::getCurrentWalkspaceMap() const {
    static std::map<int, double> empty_map;
    return workspace_analyzer_ ? workspace_analyzer_->getWalkspaceMap() : empty_map;
}

bool WalkController::isWalkspaceMapGenerated() const {
    return workspace_analyzer_ ? workspace_analyzer_->isWalkspaceMapGenerated() : false;
}

double WalkController::getStabilityMargin() const {
    if (!workspace_analyzer_) {
        return 0.0;
    }
    const auto &analysis_info = workspace_analyzer_->getAnalysisInfo();
    return analysis_info.current_result.stability_margin;
}

double WalkController::getOverallStabilityScore() const {
    if (!workspace_analyzer_) {
        return 0.0;
    }
    const auto &analysis_info = workspace_analyzer_->getAnalysisInfo();
    return analysis_info.overall_stability_score;
}

std::map<int, double> WalkController::getLegReachabilityScores() const {
    if (!workspace_analyzer_) {
        return std::map<int, double>();
    }
    const auto &analysis_info = workspace_analyzer_->getAnalysisInfo();
    return analysis_info.leg_reachability;
}

bool WalkController::isCurrentlyStable() const {
    if (!workspace_analyzer_) {
        return false;
    }
    const auto &analysis_info = workspace_analyzer_->getAnalysisInfo();
    return analysis_info.current_result.is_stable;
}

double WalkController::calculateStabilityIndex() const {
    // TODO: Simplified stability calculation
    // In a real implementation, this would use IMU and FSR data

    if (!workspace_analyzer_) {
        return 0.5f; // Default moderate stability
    }

    const auto &analysis_info = workspace_analyzer_->getAnalysisInfo();
    return analysis_info.overall_stability_score;
}

bool WalkController::checkTerrainConditions() const {
    // TODO Simplified terrain condition check
    // In a real implementation, this would use IMU and FSR data

    // For now, return false (no challenging terrain)
    return false;
}

WalkController::LegTrajectoryInfo WalkController::getLegTrajectoryInfo(int leg_index) const {
    LegTrajectoryInfo info;

    if (leg_index < 0 || leg_index >= NUM_LEGS || leg_index >= (int)leg_steppers_.size()) {
        // Return empty info for invalid index
        info.target_position = Point3D(0, 0, 0);
        info.step_phase = STANCE_PHASE;
        info.phase_progress = 0.0;
        info.is_stance = true;
        info.velocity = Point3D(0, 0, 0);
        return info;
    }

    auto leg_stepper = leg_steppers_[leg_index];
    if (!leg_stepper) {
        // Return empty info for null stepper
        info.target_position = Point3D(0, 0, 0);
        info.step_phase = STANCE_PHASE;
        info.phase_progress = 0.0;
        info.is_stance = true;
        info.velocity = Point3D(0, 0, 0);
        return info;
    }

    // Calculate target position based on current phase and gait parameters
    double local_phase = leg_stepper->getStepProgress();
    StepState step_state = leg_stepper->getStepState();

    // Get current tip position from leg stepper (this is the calculated position)
    info.target_position = leg_stepper->getCurrentTipPose();

    // Convert StepState to StepPhase for compatibility
    info.step_phase = (step_state == STEP_STANCE) ? STANCE_PHASE : SWING_PHASE;
    info.phase_progress = local_phase;
    info.is_stance = (step_state == STEP_STANCE);

    // Calculate velocity based on desired velocity and leg stepper state
    info.velocity = leg_stepper->getCurrentTipVelocity();

    return info;
}

// Helper method implementations
GaitType WalkController::stringToGaitType(const std::string &gait_name) const {
    if (gait_name == "tripod_gait") {
        return TRIPOD_GAIT;
    } else if (gait_name == "wave_gait") {
        return WAVE_GAIT;
    } else if (gait_name == "ripple_gait") {
        return RIPPLE_GAIT;
    } else if (gait_name == "metachronal_gait") {
        return METACHRONAL_GAIT;
    } else {
        return NO_GAIT; // Default for unknown gait types
    }
}