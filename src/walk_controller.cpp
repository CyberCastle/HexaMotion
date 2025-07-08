#include "walk_controller.h"
#include "gait_config_factory.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "workspace_validator.h" // Use unified validator instead
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

WalkController::WalkController(RobotModel &m, Leg legs[NUM_LEGS])
    : model(m), terrain_adaptation_(m), velocity_limits_(m),
      step_clearance_(30.0), step_depth_(10.0), body_clearance_(100.0),
      desired_linear_velocity_(0, 0, 0), desired_angular_velocity_(0.0),
      walk_state_(WALK_STOPPED), pose_state_(0),
      regenerate_walkspace_(false), legs_at_correct_phase_(0), legs_completed_first_step_(0),
      return_to_default_attempted_(false) {

    // Initialize leg_steppers_ with references to actual legs from LocomotionSystem
    leg_steppers_.clear();

    // Initialize gait configuration system (OpenSHC equivalent)
    gait_selection_config_ = createGaitSelectionConfig(model.getParams());
    std::string default_gait = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    setGaitByName(default_gait);

    // Create LegStepper objects for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Calculate default stance position for each leg
        Point3D default_stance = calculateDefaultStancePosition(i);

        // Create LegStepper with leg reference
        auto stepper = std::make_shared<LegStepper>(i, default_stance, legs[i], model);
        leg_steppers_.push_back(stepper);
    }

    // Initialize workspace validator
    workspace_validator_ = std::make_unique<WorkspaceValidator>(model);

    // Initialize walkspace analyzer
    walkspace_analyzer_ = std::make_unique<WalkspaceAnalyzer>(model);

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

    // Update step parameters
    body_clearance_ = gait_config.body_clearance;

    // Update gait selection config
    gait_selection_config_.current_gait = gait_config.gait_name;

    return true;
}

bool WalkController::setGaitByName(const std::string &gait_name) {
    // Get gait configuration from factory usando los parámetros del robot
    const Parameters &params = model.getParams();
    GaitConfiguration gait_config;
    if (gait_name == "tripod_gait") {
        gait_config = createTripodGaitConfig(params);
    } else if (gait_name == "wave_gait") {
        gait_config = createWaveGaitConfig(params);
    } else if (gait_name == "ripple_gait") {
        gait_config = createRippleGaitConfig(params);
    } else if (gait_name == "metachronal_gait") {
        gait_config = createMetachronalGaitConfig(params);
    } else {
        // Gait not found, return false
        return false;
    }
    // Apply the gait configuration
    return setGaitConfiguration(gait_config);
}

std::vector<std::string> WalkController::getAvailableGaitNames() const {
    // Return available gait names from factory
    std::vector<std::string> names = {"tripod_gait", "wave_gait", "ripple_gait", "metachronal_gait", "adaptive_gait"};
    return names;
}

void WalkController::applyGaitConfigToLegSteppers(const GaitConfiguration &gait_config) {
    // Apply gait configuration to each leg stepper
    for (int i = 0; i < NUM_LEGS && i < leg_steppers_.size(); i++) {
        auto leg_stepper = leg_steppers_[i];
        if (!leg_stepper)
            continue;

        // Set phase offset based on gait configuration
        double phase_offset = static_cast<double>(gait_config.offsets.getForLegIndex(i) *
                              gait_config.phase_config.phase_offset) /
                              static_cast<double>(gait_config.step_cycle.period_);
        leg_stepper->setPhaseOffset(phase_offset);

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
    // Update the current gait configuration directly
    current_gait_config_.step_frequency = frequency;
    current_gait_config_.stance_ratio = stance_ratio;
    current_gait_config_.swing_ratio = 1.0 - stance_ratio; // Complement
    current_gait_config_.time_to_max_stride = time_to_max_stride;

    // Update phase configuration to maintain consistency
    double total_ratio = current_gait_config_.stance_ratio + current_gait_config_.swing_ratio;
    if (total_ratio > 0) {
        int total_phase = current_gait_config_.step_cycle.period_;
        current_gait_config_.phase_config.stance_phase = (int)(stance_ratio * total_phase / total_ratio);
        current_gait_config_.phase_config.swing_phase = total_phase - current_gait_config_.phase_config.stance_phase;
    }

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
void WalkController::init() {
    time_delta_ = 0.01; // 10ms default
    walk_state_ = WALK_STOPPED;
    walk_plane_ = Point3D(0, 0, 0);
    walk_plane_normal_ = Point3D(0, 0, 1);
    odometry_ideal_ = Point3D(0, 0, 0);

    // Set default stance tip positions from parameters
    for (auto &leg_stepper : leg_steppers_) {
        // Get stance positions from robot model parameters
        const Parameters &params = model.getParams();
        int leg_index = leg_stepper->getLegIndex();

        // Calculate stance position based on leg geometry
        Point3D base_pos = model.getAnalyticLegBasePosition(leg_index);
        double base_x = base_pos.x;
        double base_y = base_pos.y;
        double base_angle = model.getLegBaseAngleOffset(leg_index);

        // Use 65% of leg reach for safe stance position
        double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
        double safe_reach = leg_reach * 0.65f;

        Point3D stance_tip_pose(
            base_x + safe_reach * cos(base_angle),
            base_y + safe_reach * sin(base_angle),
            -params.robot_height);

        leg_stepper->setDefaultTipPose(stance_tip_pose);
    }

    // Init velocity input variables
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0;
}

void WalkController::updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input) {
    // Update global gait phase based on control frequency interval
    time_delta_ = 1.0 / model.getParams().control_frequency;

    Point3D new_linear_velocity;
    double new_angular_velocity;

    // Calculate bearing from velocity input for limit lookup
    double bearing_rad = atan2(linear_velocity_input.y, linear_velocity_input.x);
    double bearing_degrees = bearing_rad * 180.0 / M_PI;
    if (bearing_degrees < 0)
        bearing_degrees += 360.0;

    // Get limits from VelocityLimits for current bearing
    VelocityLimits::LimitValues limits = velocity_limits_.getLimit(bearing_degrees);

    double max_linear_speed = limits.linear_x;
    double max_angular_speed = limits.angular_z;
    double max_linear_acceleration = limits.acceleration;
    double max_angular_acceleration = limits.acceleration; // Use same for angular

    // Calculate desired velocities according to input mode and max limits
    if (walk_state_ != WALK_STOPPING) {
        // For now, use "real" velocity mode
        new_linear_velocity = linear_velocity_input;
        new_angular_velocity = angular_velocity_input;

        // Clamp to limits
        double linear_magnitude = sqrt(new_linear_velocity.x * new_linear_velocity.x +
                                       new_linear_velocity.y * new_linear_velocity.y);
        if (linear_magnitude > max_linear_speed) {
            new_linear_velocity = new_linear_velocity * (max_linear_speed / linear_magnitude);
        }

        if (abs(new_angular_velocity) > max_angular_speed) {
            new_angular_velocity = (new_angular_velocity > 0) ? max_angular_speed : -max_angular_speed;
        }
    } else {
        new_linear_velocity = Point3D(0, 0, 0);
        new_angular_velocity = 0.0;
    }

    bool has_velocity_command = (linear_velocity_input.x != 0 || linear_velocity_input.y != 0 || angular_velocity_input != 0);

    // Check that all legs are in WALKING state
    bool all_legs_walking = true;
    for (auto &leg_stepper : leg_steppers_) {
        if (leg_stepper->getStepState() == STEP_FORCE_STOP) {
            all_legs_walking = false;
            break;
        }
    }

    // Update velocities according to acceleration limits
    Point3D linear_acceleration = new_linear_velocity - desired_linear_velocity_;
    double linear_acc_magnitude = sqrt(linear_acceleration.x * linear_acceleration.x +
                                       linear_acceleration.y * linear_acceleration.y);

    if (linear_acc_magnitude < max_linear_acceleration * time_delta_) {
        desired_linear_velocity_ = desired_linear_velocity_ + linear_acceleration;
    } else {
        Point3D acc_direction = linear_acceleration * (1.0 / linear_acc_magnitude);
        desired_linear_velocity_ = desired_linear_velocity_ + acc_direction * max_linear_acceleration * time_delta_;
    }

    double angular_acceleration = new_angular_velocity - desired_angular_velocity_;
    if (abs(angular_acceleration) < max_angular_acceleration * time_delta_) {
        desired_angular_velocity_ += angular_acceleration;
    } else {
        desired_angular_velocity_ += (angular_acceleration > 0 ? 1 : -1) * max_angular_acceleration * time_delta_;
    }

    // State transitions for Walk State Machine
    int leg_count = NUM_LEGS;

    // State transition: STOPPED->STARTING
    if (walk_state_ == WALK_STOPPED && has_velocity_command) {
        walk_state_ = WALK_STARTING;
        // Initialize LegSteppers with proper StepCycle
        StepCycle step = current_gait_config_.step_cycle; // Usar current_gait_config_.step_cycle
        for (auto &leg_stepper : leg_steppers_) {
            leg_stepper->setAtCorrectPhase(false);
            leg_stepper->setCompletedFirstStep(false);
            leg_stepper->setStepState(STEP_STANCE);
            // Set phase offset based on gait configuration (OpenSHC style)
            double phase_offset = static_cast<double>(current_gait_config_.offsets.getForLegIndex(
                                        leg_stepper->getLegIndex()) *
                                    current_gait_config_.phase_config.phase_offset) /
                                    static_cast<double>(current_gait_config_.step_cycle.period_);
            leg_stepper->setPhaseOffset(phase_offset);
            leg_stepper->updateStepState(step);
        }
        return; // Skip iteration of phase so auto posing can catch up
    }
    // State transition: STARTING->MOVING
    else if (walk_state_ == WALK_STARTING && legs_at_correct_phase_ == leg_count && legs_completed_first_step_ == leg_count) {
        legs_at_correct_phase_ = 0;
        legs_completed_first_step_ = 0;
        walk_state_ = WALK_MOVING;
    }
    // State transition: MOVING->STOPPING
    else if (walk_state_ == WALK_MOVING && !has_velocity_command) {
        walk_state_ = WALK_STOPPING;
    }
    // State transition: STOPPING->STOPPED
    else if (walk_state_ == WALK_STOPPING && legs_at_correct_phase_ == leg_count && pose_state_ == 0) {
        legs_at_correct_phase_ = 0;
        walk_state_ = WALK_STOPPED;
    }

    // Update walk/step state and tip position along trajectory for each leg
    for (auto &leg_stepper : leg_steppers_) {
        // Set walk state in LegStepper
        leg_stepper->setWalkState(walk_state_);

        // Advance phase for this leg
        leg_stepper->iteratePhase(current_gait_config_.step_cycle);

        // Calculate local phase using step cycle and leg offset
        int current_phase = leg_stepper->getPhase();
        double local_phase = static_cast<double>(current_phase) /
                              static_cast<double>(current_gait_config_.step_cycle.period_);
        leg_stepper->updateWithPhase(local_phase, 50.0, time_delta_); // Use default step length
    }

    updateWalkPlane();
    odometry_ideal_ = odometry_ideal_ + calculateOdometry(time_delta_);

    // Integrate WalkspaceAnalyzer for real-time analysis (OpenSHC equivalent)
    if (walkspace_analyzer_ && walkspace_analyzer_->isAnalysisEnabled()) {
        // Update current leg positions for analysis
        for (int i = 0; i < NUM_LEGS; i++) {
            if (i < (int)leg_steppers_.size() && leg_steppers_[i]) {
                current_leg_positions_[i] = leg_steppers_[i]->getCurrentTipPose();
            }
        }

        // Perform real-time walkspace analysis
        WalkspaceAnalyzer::WalkspaceResult analysis_result = walkspace_analyzer_->analyzeWalkspace(current_leg_positions_);

        // Use analysis results for adaptive control if needed
        if (!analysis_result.is_stable && walk_state_ == WALK_MOVING) {
            // Reduce velocity if stability is compromised
            double stability_factor = std::clamp<double>(analysis_result.stability_margin / 50.0, 0.1, 1.0);
            desired_linear_velocity_ = desired_linear_velocity_ * stability_factor;
            desired_angular_velocity_ *= stability_factor;
        }

        // Adaptive velocity control based on stability score
        if (analysis_result.is_stable && walk_state_ == WALK_MOVING) {
            double stability_score = walkspace_analyzer_->getAnalysisInfo().overall_stability_score;

            // Boost velocity when stability is high
            if (stability_score > 0.8) {
                double boost_factor = std::min(1.2, 1.0 + (stability_score - 0.8) * 0.5);
                desired_linear_velocity_ = desired_linear_velocity_ * boost_factor;
                desired_angular_velocity_ *= boost_factor;
            }
        }
    }

    if (regenerate_walkspace_) {
        generateWalkspace();
    }
}

void WalkController::updateWalkPlane() {
    std::vector<double> raw_A;
    std::vector<double> raw_B;

    if (NUM_LEGS >= 3) { // Minimum for plane estimation
        for (auto &leg_stepper : leg_steppers_) {
            Point3D default_tip_pose = leg_stepper->getDefaultTipPose();
            raw_A.push_back(default_tip_pose.x);
            raw_A.push_back(default_tip_pose.y);
            raw_A.push_back(1.0);
            raw_B.push_back(default_tip_pose.z);
        }

        // Implement proper least squares plane fitting
        if (raw_A.size() >= 9) { // At least 3 legs * 3 coordinates
            // Use least squares to fit plane: ax + by + c = z
            int num_points = raw_A.size() / 3;

            double a, b, c;
            if (math_utils::solveLeastSquaresPlane(raw_A.data(), raw_B.data(), num_points, a, b, c)) {

                // Normalize plane normal vector
                double normal_magnitude = sqrt(a * a + b * b + 1.0);
                walk_plane_normal_ = Point3D(-a / normal_magnitude, -b / normal_magnitude, 1.0 / normal_magnitude);

                // Calculate walk plane center point
                walk_plane_ = Point3D(0, 0, c);
            } else {
                // Fallback to horizontal plane
                walk_plane_ = Point3D(0, 0, 0);
                walk_plane_normal_ = Point3D(0, 0, 1);
            }
        }
    } else {
        walk_plane_ = Point3D(0, 0, 0);
        walk_plane_normal_ = Point3D(0, 0, 1);
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

void WalkController::generateWalkspace() {
    // Implement full walkspace calculation like OpenSHC
    walkspace_.clear();

    // Get robot parameters for workspace calculation
    const Parameters &params = model.getParams();
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double safe_reach = leg_reach * 0.65f; // 65% safety margin

    // Calculate workspace for each bearing angle
    for (int bearing = 0; bearing <= 360; bearing += 10) {
        double bearing_rad = bearing * M_PI / 180.0;

        // Calculate workspace radius based on leg geometry and joint limits
        double max_radius = 0.0;

        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Get leg base position
            Point3D base_pos = model.getAnalyticLegBasePosition(leg);
            double base_x = base_pos.x;
            double base_y = base_pos.y;
            double base_angle = model.getLegBaseAngleOffset(leg);

            // Calculate reach in bearing direction
            double target_x = base_x + safe_reach * cos(bearing_rad);
            double target_y = base_y + safe_reach * sin(bearing_rad);

            // Check if target is reachable by this leg
            Point3D target(target_x, target_y, -params.robot_height);
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
    if (walkspace_analyzer_) {
        walkspace_analyzer_->enableAnalysis(enabled);
    }
}

bool WalkController::isWalkspaceAnalysisEnabled() const {
    return walkspace_analyzer_ ? walkspace_analyzer_->isAnalysisEnabled() : false;
}

const WalkspaceAnalyzer::AnalysisInfo &WalkController::getWalkspaceAnalysisInfo() const {
    static WalkspaceAnalyzer::AnalysisInfo empty_info;
    return walkspace_analyzer_ ? walkspace_analyzer_->getAnalysisInfo() : empty_info;
}

std::string WalkController::getWalkspaceAnalysisInfoString() const {
    return walkspace_analyzer_ ? walkspace_analyzer_->getAnalysisInfoString() : "WalkspaceAnalyzer not available";
}

void WalkController::resetWalkspaceAnalysisStats() {
    if (walkspace_analyzer_) {
        walkspace_analyzer_->resetAnalysisStats();
    }
}

WalkspaceAnalyzer::WalkspaceResult WalkController::analyzeCurrentWalkspace() {
    if (!walkspace_analyzer_) {
        return WalkspaceAnalyzer::WalkspaceResult();
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

    return walkspace_analyzer_->analyzeWalkspace(current_positions);
}

bool WalkController::generateWalkspaceMap() {
    if (!walkspace_analyzer_) {
        return false;
    }

    try {
        walkspace_analyzer_->generateWalkspace();
        return true;
    } catch (...) {
        return false;
    }
}

double WalkController::getWalkspaceRadius(double bearing_degrees) const {
    return walkspace_analyzer_ ? walkspace_analyzer_->getWalkspaceRadius(bearing_degrees) : 0.0;
}

const std::map<int, double> &WalkController::getCurrentWalkspaceMap() const {
    static std::map<int, double> empty_map;
    return walkspace_analyzer_ ? walkspace_analyzer_->getCurrentWalkspaceMap() : empty_map;
}

bool WalkController::isWalkspaceMapGenerated() const {
    return walkspace_analyzer_ ? walkspace_analyzer_->isWalkspaceMapGenerated() : false;
}

double WalkController::getStabilityMargin() const {
    if (!walkspace_analyzer_) {
        return 0.0;
    }
    const auto &analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.current_result.stability_margin;
}

double WalkController::getOverallStabilityScore() const {
    if (!walkspace_analyzer_) {
        return 0.0;
    }
    const auto &analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.overall_stability_score;
}

std::map<int, double> WalkController::getLegReachabilityScores() const {
    if (!walkspace_analyzer_) {
        return std::map<int, double>();
    }
    const auto &analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.leg_reachability;
}

bool WalkController::isCurrentlyStable() const {
    if (!walkspace_analyzer_) {
        return false;
    }
    const auto &analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.current_result.is_stable;
}

double WalkController::calculateStabilityIndex() const {
    // TODO: Simplified stability calculation
    // In a real implementation, this would use IMU and FSR data

    if (!walkspace_analyzer_) {
        return 0.5f; // Default moderate stability
    }

    const auto &analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.overall_stability_score;
}

bool WalkController::checkTerrainConditions() const {
    // TODO Simplified terrain condition check
    // In a real implementation, this would use IMU and FSR data

    // For now, return false (no challenging terrain)
    return false;
}

Point3D WalkController::calculateDefaultStancePosition(int leg_index) {
    const auto &params = model.getParams();

    // Use the same base positions as the main model
    Point3D base_pos = model.getAnalyticLegBasePosition(leg_index);
    double base_x = base_pos.x;
    double base_y = base_pos.y;
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double safe_reach = leg_reach * 0.65f; // 65% safety margin
    double base_angle = model.getLegBaseAngleOffset(leg_index);
    double default_foot_x = base_x + safe_reach * cos(base_angle);
    double default_foot_y = base_y + safe_reach * sin(base_angle);

    return Point3D(default_foot_x, default_foot_y, 0);
}

double WalkController::calculateLegReach() const {
    const auto &params = model.getParams();
    return params.coxa_length + params.femur_length + params.tibia_length;
}