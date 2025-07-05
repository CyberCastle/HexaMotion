#include "walk_controller.h"
#include "hexamotion_constants.h"
#include "workspace_validator.h" // Use unified validator instead
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>


WalkController::WalkController(RobotModel &m, Leg legs[NUM_LEGS])
    : model(m), current_gait(TRIPOD_GAIT), gait_phase(0.0f),
      terrain_adaptation_(m), velocity_limits_(m),
      step_clearance_(30.0), step_depth_(10.0), body_clearance_(100.0),
      desired_linear_velocity_(0, 0, 0), desired_angular_velocity_(0.0),
      walk_state_(WALK_STOPPED), pose_state_(0),
      regenerate_walkspace_(false), legs_at_correct_phase_(0), legs_completed_first_step_(0),
      return_to_default_attempted_(false),
      stance_duration_(0.5), swing_duration_(0.5), cycle_frequency_(1.2),
      step_height_(30.0), step_length_(50.0) {

    // Initialize leg_steppers_ with references to actual legs from LocomotionSystem
    leg_steppers_.clear();

    // Define phase offsets for tripod gait (reusable for other patterns)
    static const double tripod_phase_offsets[NUM_LEGS] = {
        0.0f, // Leg 0 (Anterior Right) - Group A (even)
        0.5f, // Leg 1 (Middle Right) - Group B (odd)
        0.0f, // Leg 2 (Posterior Right) - Group A (even)
        0.5f, // Leg 3 (Posterior Left) - Group B (odd)
        0.0f, // Leg 4 (Middle Left) - Group A (even)
        0.5f  // Leg 5 (Anterior Left) - Group B (odd)
    };

    // ✅ NEW: Initialize gait configurations (OpenSHC-style)
    initializeGaitConfigs();

    // ✅ NEW: Set initial gait pattern (migrated from LocomotionSystem)
    initGaitParameters(TRIPOD_GAIT);

    // Create LegStepper objects for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Calculate default stance position for each leg
        Point3D default_stance = calculateDefaultStancePosition(i);

        // Create LegStepper with leg reference
        auto stepper = std::make_shared<LegStepper>(i, default_stance, legs[i], model);
        leg_steppers_.push_back(stepper);

        // Set initial phase offset
        stepper->setPhaseOffset(tripod_phase_offsets[i]);
    }

    // Initialize workspace validator
    workspace_validator_ = std::make_unique<WorkspaceValidator>(model);

    // Initialize walkspace analyzer
    walkspace_analyzer_ = std::make_unique<WalkspaceAnalyzer>(model);

    // Initialize terrain adaptation
    terrain_adaptation_.initialize();

    // Set initial time delta
    time_delta_ = 1.0 / model.getParams().control_frequency;

    // Generate initial step cycle
    generateStepCycle();

    // Generate initial walkspace
    generateWalkspace();
}

bool WalkController::planGaitSequence(double vx, double vy, double omega) {
    // Store current velocity commands for gait pattern decisions
    current_velocities_.linear_x = vx;
    current_velocities_.linear_y = vy;
    current_velocities_.angular_z = omega;

    // Use the new OpenSHC-style updateWalk method
    Point3D linear_velocity(vx, vy, 0);
    updateWalk(linear_velocity, omega);

    return true;
}

void WalkController::updateGaitPhase(double dt) {
    // Calculate step frequency based on velocity and step parameters
    // This follows OpenSHC's approach for proper gait timing
    double step_frequency = 1.0; // Default frequency

    // If we have velocity commands, calculate appropriate step frequency
    if (std::abs(current_velocities_.linear_x) > 0.01 ||
        std::abs(current_velocities_.linear_y) > 0.01 ||
        std::abs(current_velocities_.angular_z) > 0.01) {

        // Calculate step frequency based on velocity magnitude
        double velocity_magnitude = sqrt(current_velocities_.linear_x * current_velocities_.linear_x +
                                         current_velocities_.linear_y * current_velocities_.linear_y);

        // Base frequency on velocity - typical range 0.5 to 2.0 Hz
        // Use a more reasonable divisor to prevent excessive stride lengths
        step_frequency = std::max(0.5, std::min(2.0, velocity_magnitude / 200.0));
    }

    // Update gait phase with proper frequency
    gait_phase += dt * step_frequency;
    if (gait_phase >= 1.0f)
        gait_phase -= 1.0f;
}

Point3D WalkController::footTrajectory(int leg_index, double phase, double step_height, double step_length,
                                       double stance_duration, double swing_duration, double robot_height,
                                       const double leg_phase_offsets[NUM_LEGS], LegState (&leg_states)[NUM_LEGS],
                                       IFSRInterface *fsr, IIMUInterface *imu) {
    // ✅ UPDATED: Use internal step parameters instead of passed parameters
    // This maintains backward compatibility while using internal state
    double internal_step_height = step_height_;
    double internal_step_length = step_length_;
    double internal_stance_duration = stance_duration_;
    double internal_swing_duration = swing_duration_;

    // Update terrain adaptation system
    terrain_adaptation_.update(fsr, imu);

    // Use the new LegStepper architecture
    auto leg_stepper = leg_steppers_[leg_index];
    if (!leg_stepper) {
        // Fallback to old method if leg stepper not available
        return Point3D(0, 0, 0);
    }

    // Update the leg stepper's phase based on the input phase
    double leg_phase = phase + leg_phase_offsets[leg_index];
    if (leg_phase >= 1.0f)
        leg_phase -= 1.0f;

    // Convert phase to step cycle phase
    StepCycle step = getStepCycle();
    int step_phase = (int)(leg_phase * step.period_);
    leg_stepper->setPhase(step_phase);

    // ✅ CORRECTED: Update the leg stepper's trajectory with proper parameters
    leg_stepper->updateTipPosition(internal_step_length, time_delta_, false, false);

    // Get the calculated trajectory from the leg stepper
    Point3D trajectory = leg_stepper->getCurrentTipPose();

    // Update leg states based on step state
    StepState step_state = leg_stepper->getStepState();
    // For walking legs, use LEG_WALKING state
    leg_states[leg_index] = LEG_WALKING;

    // Apply terrain adaptation for swing phase
    if (step_state == STEP_SWING) {
        double swing_progress = leg_stepper->getSwingProgress();
        // Convert StepState to StepPhase for terrain adaptation
        StepPhase terrain_leg_state = (step_state == STEP_SWING) ? SWING_PHASE : STANCE_PHASE;
        trajectory = terrain_adaptation_.adaptTrajectoryForTerrain(leg_index, trajectory,
                                                                   terrain_leg_state, swing_progress);
    }

    // Use WorkspaceValidator for all validation
    auto validation_result = workspace_validator_->validateTarget(
        leg_index, trajectory, current_leg_positions_, true);

    // Use the validated and constrained position
    trajectory = validation_result.constrained_position;

    // Update current leg position for future collision checks
    current_leg_positions_[leg_index] = trajectory;

    return trajectory;
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
    VelocityLimits::GaitConfig gait_config;
    gait_config.frequency = frequency;
    gait_config.stance_ratio = stance_ratio;
    gait_config.swing_ratio = DEFAULT_ANGULAR_SCALING - stance_ratio;
    gait_config.time_to_max_stride = time_to_max_stride;

    velocity_limits_.updateGaitParameters(gait_config);
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
        double base_angle_deg = params.dh_parameters[leg_index][0][3];
        double base_angle = base_angle_deg * M_PI / 180.0; // Convert to radians
        double base_x = params.hexagon_radius * cos(base_angle);
        double base_y = params.hexagon_radius * sin(base_angle);

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

    // Generate step timing
    generateStepCycle();
}

StepCycle WalkController::generateStepCycle(bool set_step_cycle) {
    StepCycle step;

    // Get parameters from robot model configuration
    const Parameters &params = model.getParams();

    // Use control frequency to calculate timing parameters
    double control_frequency = params.control_frequency;
    double time_delta = 1.0 / control_frequency;

    // Calculate stance and swing phases based on frequency
    int stance_phase = static_cast<int>(60.0 * control_frequency / 50.0); // 60 iterations at 50Hz
    int swing_phase = static_cast<int>(40.0 * control_frequency / 50.0);  // 40 iterations at 50Hz
    double step_frequency = 1.0;                                          // Hz - can be made configurable

    step.stance_end_ = stance_phase / 2;
    step.swing_start_ = step.stance_end_;
    step.swing_end_ = step.swing_start_ + swing_phase;
    step.stance_start_ = step.swing_end_;

    // Normalize step period to match total iterations over full step
    int base_step_period = stance_phase + swing_phase;
    double swing_ratio = double(swing_phase) / double(base_step_period);

    // Ensure step period is even and divisible by base step period
    double raw_step_period = ((1.0 / step_frequency) / time_delta_) / swing_ratio;
    step.period_ = ((int)(raw_step_period / base_step_period) + 1) * base_step_period;

    step.frequency_ = 1.0 / (step.period_ * time_delta_);
    int normaliser = step.period_ / base_step_period;
    step.stance_end_ *= normaliser;
    step.swing_start_ *= normaliser;
    step.swing_end_ *= normaliser;
    step.stance_start_ *= normaliser;

    step.stance_period_ = (step.stance_end_ - step.stance_start_ + step.period_) % step.period_;
    step.swing_period_ = step.swing_end_ - step.swing_start_;

    // Ensure stance and swing periods are divisible by two
    if (step.stance_period_ % 2 != 0)
        step.stance_period_++;
    if (step.swing_period_ % 2 != 0)
        step.swing_period_++;

    // Set step cycle in walk controller and update phase in leg steppers
    if (set_step_cycle) {
        step_ = step;
        if (walk_state_ == WALK_MOVING) {
            for (auto &leg_stepper : leg_steppers_) {
                // ✅ CORRECTED: Pass step parameter to updatePhase
                leg_stepper->updatePhase(step);
            }
        }
    }

    return step;
}

void WalkController::updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input) {
    // Update global gait phase based on control frequency interval
    time_delta_ = 1.0 / model.getParams().control_frequency;
    updateGaitPhase(time_delta_);

    Point3D new_linear_velocity;
    double new_angular_velocity;

    // Get limits for current velocity input
    double max_linear_speed = getLimit(linear_velocity_input, angular_velocity_input, max_linear_speed_);
    double max_angular_speed = getLimit(linear_velocity_input, angular_velocity_input, max_angular_speed_);
    double max_linear_acceleration = getLimit(linear_velocity_input, angular_velocity_input, max_linear_acceleration_);
    double max_angular_acceleration = getLimit(linear_velocity_input, angular_velocity_input, max_angular_acceleration_);

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
        // ✅ CORRECTED: Initialize LegSteppers with proper StepCycle
        StepCycle step = generateStepCycle(true);
        for (auto &leg_stepper : leg_steppers_) {
            leg_stepper->setAtCorrectPhase(false);
            leg_stepper->setCompletedFirstStep(false);
            leg_stepper->setStepState(STEP_STANCE);
            leg_stepper->setPhase(leg_stepper->getPhaseOffset());
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

    // ✅ CORRECTED: Update walk/step state and tip position along trajectory for each leg
    for (auto &leg_stepper : leg_steppers_) {
        // ✅ CORRECTED: Set walk state in LegStepper
        leg_stepper->setWalkState(walk_state_);

        // Calcular la fase local continua usando la fase global y el offset de la pierna
        double local_phase = std::fmod(gait_phase + leg_stepper->getPhaseOffset(), 1.0);
        if (local_phase < 0)
            local_phase += 1.0;
        // ✅ CORRECTED: Pass time_delta parameter to updateWithPhase
        leg_stepper->updateWithPhase(local_phase, step_depth_, time_delta_);
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
            double base_angle_deg = params.dh_parameters[leg][0][3];
            double base_angle = base_angle_deg * M_PI / 180.0;
            double base_x = params.hexagon_radius * cos(base_angle);
            double base_y = params.hexagon_radius * sin(base_angle);

            // Calculate reach in bearing direction
            double target_x = base_x + safe_reach * cos(bearing_rad);
            double target_y = base_y + safe_reach * sin(bearing_rad);

            // Check if target is reachable by this leg
            Point3D target(target_x, target_y, -params.robot_height);
            JointAngles angles = model.inverseKinematics(leg, target);

            if (model.checkJointLimits(leg, angles)) {
                double distance = sqrt((target_x - base_x) * (target_x - base_x) +
                                       (target_y - base_y) * (target_y - base_y));
                max_radius = std::max(max_radius, distance);
            }
        }

        walkspace_[bearing] = max_radius;
    }

    regenerate_walkspace_ = false;
    generateLimits(step_);
}

void WalkController::generateLimits(StepCycle step) {
    // Implement full limit calculation like OpenSHC

    max_linear_speed_.clear();
    max_angular_speed_.clear();
    max_linear_acceleration_.clear();
    max_angular_acceleration_.clear();

    for (auto &walkspace_entry : walkspace_) {
        double walkspace_radius = walkspace_entry.second;
        double on_ground_ratio = double(step.stance_period_) / step.period_;

        // Enhanced limit calculations based on OpenSHC approach
        double step_frequency = step.frequency_;
        double stance_duration = on_ground_ratio / step_frequency;

        // Maximum linear speed based on step length and frequency
        double max_step_length = walkspace_radius * 2.0;                    // Maximum step length
        double max_linear_speed = (max_step_length * step_frequency) / 2.0; // Average speed

        // Maximum angular speed based on stance radius
        double stance_radius = walkspace_radius * 0.8; // Effective stance radius
        double max_angular_speed = max_linear_speed / stance_radius;

        // Acceleration limits based on step timing
        double time_to_max_stride = 2.0; // 2 seconds to reach maximum stride
        double max_linear_acceleration = max_linear_speed / time_to_max_stride;
        double max_angular_acceleration = max_angular_speed / time_to_max_stride;

        max_linear_speed_[walkspace_entry.first] = max_linear_speed;
        max_angular_speed_[walkspace_entry.first] = max_angular_speed;
        max_linear_acceleration_[walkspace_entry.first] = max_linear_acceleration;
        max_angular_acceleration_[walkspace_entry.first] = max_angular_acceleration;
    }
}

double WalkController::getLimit(const Point3D &linear_velocity_input, double angular_velocity_input,
                                const std::map<int, double> &limit) {
    double min_limit = 1e6; // Large value

    for (auto &leg_stepper : leg_steppers_) {
        Point3D tip_position = leg_stepper->getCurrentTipPose();
        Point3D rotation_normal(-tip_position.y, tip_position.x, 0);
        Point3D stride_vector = linear_velocity_input + rotation_normal * angular_velocity_input;

        int bearing = (int)(atan2(stride_vector.y, stride_vector.x) * 180.0 / M_PI + 360) % 360;
        bearing = (bearing / 10) * 10; // Round to nearest 10 degrees

        auto it = limit.find(bearing);
        if (it != limit.end()) {
            min_limit = std::min(min_limit, it->second);
        }
    }

    return min_limit;
}

// Accessor methods
Point3D WalkController::getModelCurrentPose() const {
    // Get current pose from robot model - return a default pose for now
    return Point3D(0, 0, -model.getParams().robot_height);
}

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

// ===== ENHANCED WALKSPACE ANALYSIS METHODS =====

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

// ✅ NEW: Gait pattern management methods (migrated from LocomotionSystem)

void WalkController::initializeGaitConfigs() {
    // Initialize gait configurations (OpenSHC-style)

    // TRIPOD_GAIT configuration
    GaitConfig tripod_config;
    tripod_config.phase_offsets[0] = 0.0f; // AR - Group A
    tripod_config.phase_offsets[1] = 0.5f; // BR - Group B
    tripod_config.phase_offsets[2] = 0.0f; // CR - Group A
    tripod_config.phase_offsets[3] = 0.5f; // CL - Group B
    tripod_config.phase_offsets[4] = 0.0f; // BL - Group A
    tripod_config.phase_offsets[5] = 0.5f; // AL - Group B
    tripod_config.stance_duration = 0.5f;  // 50% stance - critical for tripod pattern
    tripod_config.swing_duration = 0.5f;   // 50% swing - critical for tripod pattern
    tripod_config.cycle_frequency = 1.2f;  // Faster cycle for speed
    tripod_config.description = "Tripod Gait - Fast and efficient alternating groups";
    gait_configs_[TRIPOD_GAIT] = tripod_config;

    // WAVE_GAIT configuration
    GaitConfig wave_config;
    wave_config.phase_offsets[0] = 2.0f / 6.0f; // AR: 0.333
    wave_config.phase_offsets[1] = 3.0f / 6.0f; // BR: 0.500
    wave_config.phase_offsets[2] = 4.0f / 6.0f; // CR: 0.667
    wave_config.phase_offsets[3] = 1.0f / 6.0f; // CL: 0.167
    wave_config.phase_offsets[4] = 0.0f / 6.0f; // BL: 0.000
    wave_config.phase_offsets[5] = 5.0f / 6.0f; // AL: 0.833
    wave_config.stance_duration = 0.833f;        // 83.3% stance for stability
    wave_config.swing_duration = 0.167f;         // 16.7% swing
    wave_config.cycle_frequency = 0.8f;          // Slower cycle for stability
    wave_config.description = "Wave Gait - Maximum stability with sequential stepping";
    gait_configs_[WAVE_GAIT] = wave_config;

    // RIPPLE_GAIT configuration
    GaitConfig ripple_config;
    ripple_config.phase_offsets[0] = 2.0f / 6.0f; // AR: 0.333
    ripple_config.phase_offsets[1] = 0.0f / 6.0f; // BR: 0.000
    ripple_config.phase_offsets[2] = 4.0f / 6.0f; // CR: 0.667
    ripple_config.phase_offsets[3] = 1.0f / 6.0f; // CL: 0.167
    ripple_config.phase_offsets[4] = 3.0f / 6.0f; // BL: 0.500
    ripple_config.phase_offsets[5] = 5.0f / 6.0f; // AL: 0.833
    ripple_config.stance_duration = 0.667f;       // 66.7% stance for balance
    ripple_config.swing_duration = 0.333f;        // 33.3% swing
    ripple_config.cycle_frequency = 1.0f;         // Balanced cycle
    ripple_config.description = "Ripple Gait - Balanced stability and speed";
    gait_configs_[RIPPLE_GAIT] = ripple_config;

    // METACHRONAL_GAIT configuration
    GaitConfig metachronal_config;
    metachronal_config.phase_offsets[0] = 0.0f / 6.0f; // AR: 0.000
    metachronal_config.phase_offsets[1] = 1.0f / 6.0f; // BR: 0.167
    metachronal_config.phase_offsets[2] = 2.0f / 6.0f; // CR: 0.333
    metachronal_config.phase_offsets[3] = 3.0f / 6.0f; // CL: 0.500
    metachronal_config.phase_offsets[4] = 4.0f / 6.0f; // BL: 0.667
    metachronal_config.phase_offsets[5] = 5.0f / 6.0f; // AL: 0.833
    metachronal_config.stance_duration = 0.75f;         // 75% stance for smooth motion
    metachronal_config.swing_duration = 0.25f;          // 25% swing
    metachronal_config.cycle_frequency = 1.1f;          // Smooth cycle
    metachronal_config.description = "Metachronal Gait - Smooth wave-like motion";
    gait_configs_[METACHRONAL_GAIT] = metachronal_config;

    // ADAPTIVE_GAIT configuration
    GaitConfig adaptive_config;
    adaptive_config.phase_offsets[0] = 1.0f / 8.0f; // AR: 0.125 (fine-tuned timing)
    adaptive_config.phase_offsets[1] = 0.0f / 8.0f; // BR: 0.000 (anchor leg)
    adaptive_config.phase_offsets[2] = 3.0f / 8.0f; // CR: 0.375
    adaptive_config.phase_offsets[3] = 6.0f / 8.0f; // CL: 0.750
    adaptive_config.phase_offsets[4] = 4.0f / 8.0f; // BL: 0.500
    adaptive_config.phase_offsets[5] = 7.0f / 8.0f; // AL: 0.875
    adaptive_config.stance_duration = 0.6f;          // 60% stance - adaptive
    adaptive_config.swing_duration = 0.4f;           // 40% swing - adaptive
    adaptive_config.cycle_frequency = 1.0f;          // Will adapt based on conditions
    adaptive_config.description = "Adaptive Gait - Dynamic pattern that changes based on conditions";
    gait_configs_[ADAPTIVE_GAIT] = adaptive_config;
}

void WalkController::initGaitParameters(GaitType gait) {
    current_gait = gait;
    gait_phase = 0.0f;

    // Get gait configuration
    auto it = gait_configs_.find(gait);
    if (it != gait_configs_.end()) {
        applyGaitConfig(it->second);
    } else {
        // Default to tripod gait if configuration not found
        applyGaitConfig(gait_configs_[TRIPOD_GAIT]);
    }

    // ✅ NEW: Update step parameters based on gait type
    updateStepParameters();

    // Generate new step cycle with updated parameters
    generateStepCycle();
    generateLimits(step_);
}

bool WalkController::setGaitType(GaitType gait) {
    if (gait == current_gait) {
        return true; // Already using this gait
    }

    // Store current walk state
    WalkState previous_walk_state = walk_state_;

    // Stop walking if currently moving
    if (walk_state_ == WALK_MOVING || walk_state_ == WALK_STARTING) {
        walk_state_ = WALK_STOPPING;
    }

    // Initialize new gait parameters
    initGaitParameters(gait);

    // Restore walk state if it was moving
    if (previous_walk_state == WALK_MOVING || previous_walk_state == WALK_STARTING) {
        walk_state_ = previous_walk_state;
    }

    return true;
}

void WalkController::configureGaitPhaseOffsets(GaitType gait) {
    auto it = gait_configs_.find(gait);
    if (it != gait_configs_.end()) {
        const GaitConfig& config = it->second;

        // Apply phase offsets to all leg steppers
        for (int i = 0; i < NUM_LEGS && i < (int)leg_steppers_.size(); i++) {
            if (leg_steppers_[i]) {
                leg_steppers_[i]->setPhaseOffset(config.phase_offsets[i]);
            }
        }
    }
}

void WalkController::updateMetachronalPattern() {
    if (current_gait != METACHRONAL_GAIT) {
        return;
    }

    // Calculate wave direction based on movement command
    bool reverse_wave = false;
    double threshold = 0.001f; // Small threshold to avoid noise

    if (abs(desired_linear_velocity_.x) > threshold) {
        // Primary movement is in X direction
        reverse_wave = (desired_linear_velocity_.x < 0.0f);
    } else if (abs(desired_angular_velocity_) > threshold) {
        // Primarily rotational movement - use angular direction
        reverse_wave = (desired_angular_velocity_ < 0.0f);
    } else {
        // For lateral movement or stationary, maintain forward wave pattern
        reverse_wave = false;
    }

    if (reverse_wave) {
        // Reverse wave pattern: AL → BL → CL → CR → BR → AR
        if (leg_steppers_.size() >= 6) {
            leg_steppers_[0]->setPhaseOffset(5.0f / 6.0f); // AR: 0.833
            leg_steppers_[1]->setPhaseOffset(4.0f / 6.0f); // BR: 0.667
            leg_steppers_[2]->setPhaseOffset(3.0f / 6.0f); // CR: 0.500
            leg_steppers_[3]->setPhaseOffset(2.0f / 6.0f); // CL: 0.333
            leg_steppers_[4]->setPhaseOffset(1.0f / 6.0f); // BL: 0.167
            leg_steppers_[5]->setPhaseOffset(0.0f / 6.0f); // AL: 0.000
        }
    } else {
        // Forward wave pattern: AR → BR → CR → CL → BL → AL
        if (leg_steppers_.size() >= 6) {
            leg_steppers_[0]->setPhaseOffset(0.0f / 6.0f); // AR: 0.000
            leg_steppers_[1]->setPhaseOffset(1.0f / 6.0f); // BR: 0.167
            leg_steppers_[2]->setPhaseOffset(2.0f / 6.0f); // CR: 0.333
            leg_steppers_[3]->setPhaseOffset(3.0f / 6.0f); // CL: 0.500
            leg_steppers_[4]->setPhaseOffset(4.0f / 6.0f); // BL: 0.667
            leg_steppers_[5]->setPhaseOffset(5.0f / 6.0f); // AL: 0.833
        }
    }
}

void WalkController::updateAdaptivePattern() {
    if (current_gait != ADAPTIVE_GAIT) {
        return;
    }

    // Check if we need to adapt the gait pattern
    if (shouldAdaptGaitPattern()) {
        calculateAdaptivePhaseOffsets();
    }
}

bool WalkController::shouldAdaptGaitPattern() {
    // Check terrain conditions and stability
    double stability_index = calculateStabilityIndex();
    bool terrain_conditions = checkTerrainConditions();

    // Adapt if stability is low or terrain is challenging
    return (stability_index < 0.5f) || terrain_conditions;
}

void WalkController::calculateAdaptivePhaseOffsets() {
    // Get base adaptive configuration
    auto it = gait_configs_.find(ADAPTIVE_GAIT);
    if (it == gait_configs_.end()) {
        return;
    }

    const GaitConfig& base_config = it->second;
    double base_offsets[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        base_offsets[i] = base_config.phase_offsets[i];
    }

    double stability_index = calculateStabilityIndex();
    double tilt_magnitude = 0.0f; // Would be calculated from IMU data

    if (tilt_magnitude > 10.0f) {
        // On steep slopes, use more conservative tripod-like pattern
        double tripod_factor = (tilt_magnitude - 10.0f) / 20.0f; // 0-1 over 10-30 degrees
        tripod_factor = std::clamp<double>(tripod_factor, 0.0, 1.0);

        for (int i = 0; i < NUM_LEGS && i < (int)leg_steppers_.size(); i++) {
            double tripod_offset = (i % 2) * 0.5f;
            double new_offset = base_offsets[i] * (1.0f - tripod_factor) +
                               tripod_offset * tripod_factor;
            if (leg_steppers_[i]) {
                leg_steppers_[i]->setPhaseOffset(new_offset);
            }
        }
    } else if (stability_index < 0.3f) {
        // Low stability - move toward wave gait pattern
        double wave_offsets[NUM_LEGS] = {
            2.0f / 6.0f, // AR: 0.333
            3.0f / 6.0f, // BR: 0.500
            4.0f / 6.0f, // CR: 0.667
            1.0f / 6.0f, // CL: 0.167
            0.0f / 6.0f, // BL: 0.000
            5.0f / 6.0f  // AL: 0.833
        };

        double wave_factor = (0.3f - stability_index) / 0.3f; // 0-1 as stability decreases

        for (int i = 0; i < NUM_LEGS && i < (int)leg_steppers_.size(); i++) {
            double new_offset = base_offsets[i] * (1.0f - wave_factor) +
                               wave_offsets[i] * wave_factor;
            if (leg_steppers_[i]) {
                leg_steppers_[i]->setPhaseOffset(new_offset);
            }
        }
    } else {
        // Good conditions - use base adaptive pattern
        for (int i = 0; i < NUM_LEGS && i < (int)leg_steppers_.size(); i++) {
            if (leg_steppers_[i]) {
                leg_steppers_[i]->setPhaseOffset(base_offsets[i]);
            }
        }
    }
}

void WalkController::getGaitTimingParameters(GaitType gait, double& stance_duration, double& swing_duration, double& cycle_frequency) const {
    auto it = gait_configs_.find(gait);
    if (it != gait_configs_.end()) {
        const GaitConfig& config = it->second;
        stance_duration = config.stance_duration;
        swing_duration = config.swing_duration;
        cycle_frequency = config.cycle_frequency;
    } else {
        // Default values
        stance_duration = 0.5f;
        swing_duration = 0.5f;
        cycle_frequency = 1.0f;
    }
}

void WalkController::applyGaitConfig(const GaitConfig& config) {
    // Apply timing parameters
    stance_duration_ = config.stance_duration;
    swing_duration_ = config.swing_duration;
    cycle_frequency_ = config.cycle_frequency;

    // Apply phase offsets to leg steppers
    for (int i = 0; i < NUM_LEGS && i < (int)leg_steppers_.size(); i++) {
        if (leg_steppers_[i]) {
            leg_steppers_[i]->setPhaseOffset(config.phase_offsets[i]);
        }
    }
}

double WalkController::calculateStabilityIndex() const {
    // Simplified stability calculation
    // In a real implementation, this would use IMU and FSR data

    if (!walkspace_analyzer_) {
        return 0.5f; // Default moderate stability
    }

    const auto& analysis_info = walkspace_analyzer_->getAnalysisInfo();
    return analysis_info.overall_stability_score;
}

bool WalkController::checkTerrainConditions() const {
    // Simplified terrain condition check
    // In a real implementation, this would use IMU and FSR data

    // For now, return false (no challenging terrain)
    return false;
}

Point3D WalkController::calculateDefaultStancePosition(int leg_index) {
    const auto& params = model.getParams();

    // Use the same base positions as the main model (DH parameters)
    double base_angle_deg = params.dh_parameters[leg_index][0][3];
    double base_angle = base_angle_deg * M_PI / 180.0; // Convert to radians
    double base_x = params.hexagon_radius * cos(base_angle);
    double base_y = params.hexagon_radius * sin(base_angle);
    double leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double safe_reach = leg_reach * 0.65f; // 65% safety margin
    double default_foot_x = base_x + safe_reach * cos(base_angle);
    double default_foot_y = base_y + safe_reach * sin(base_angle);

    return Point3D(default_foot_x, default_foot_y, 0);
}

// ✅ NEW: Step parameter control methods (migrated from LocomotionSystem)

bool WalkController::setStepParameters(double height, double length) {
    if (height < 15.0f || height > 50.0f || length < 20.0f || length > 80.0f) {
        return false;
    }

    step_height_ = height;
    step_length_ = length;
    return true;
}

double WalkController::getStepHeight() const {
    return step_height_;
}

double WalkController::getStepLength() const {
    // Base step length according to the current gait
    double base_step_length = step_length_;

    // Adjustment factors based on robot parameters
    double leg_reach = calculateLegReach(); // Use standardized function
    double max_safe_step = leg_reach * model.getParams().gait_factors.max_length_factor;

    // Stability adjustment - reduce step if stability is low
    double stability_factor = 1.0f;
    double stability_index = calculateStabilityIndex();
    if (stability_index < 0.5f) {
        stability_factor = 0.7f + 0.3f * stability_index; // Reduce up to 70%
    }

    // Enhanced slope adjustment using absolute positioning data
    double terrain_factor = 1.0f;
    // Note: IMU data access would need to be passed from LocomotionSystem
    // For now, use basic terrain assessment

    // Calculate final step length
    double calculated_step_length = base_step_length * stability_factor * terrain_factor;

    // Limit within safe ranges based on parameters
    double min_safe_step = leg_reach * model.getParams().gait_factors.min_length_factor;
    calculated_step_length =
        std::clamp<double>(calculated_step_length, min_safe_step, max_safe_step);

    return calculated_step_length;
}

double WalkController::calculateLegReach() const {
    const auto& params = model.getParams();
    return params.coxa_length + params.femur_length + params.tibia_length;
}

void WalkController::updateStepParameters() {
    // Calculate leg reach and robot dimensions
    double leg_reach = calculateLegReach(); // Use standardized function
    double robot_height = model.getParams().robot_height;

    // Adjust step parameters depending on gait type using parametrizable factors
    switch (current_gait) {
    case NO_GAIT:
        // No gait - robot is stationary
        step_length_ = 0.0;
        step_height_ = 0.0;
        break;
    case TRIPOD_GAIT:
        // Longer steps for speed
        step_length_ = leg_reach * model.getParams().gait_factors.tripod_length_factor;
        step_height_ = robot_height * model.getParams().gait_factors.tripod_height_factor;
        break;

    case WAVE_GAIT:
        // Shorter steps for stability
        step_length_ = leg_reach * model.getParams().gait_factors.wave_length_factor;
        step_height_ = robot_height * model.getParams().gait_factors.wave_height_factor;
        break;

    case RIPPLE_GAIT:
        // Medium steps for balance
        step_length_ = leg_reach * model.getParams().gait_factors.ripple_length_factor;
        step_height_ = robot_height * model.getParams().gait_factors.ripple_height_factor;
        break;

    case METACHRONAL_GAIT:
        // Adaptive steps
        step_length_ = leg_reach * model.getParams().gait_factors.metachronal_length_factor;
        step_height_ = robot_height * model.getParams().gait_factors.metachronal_height_factor;
        break;

    case ADAPTIVE_GAIT:
        // They will be adjusted dynamically
        step_length_ = leg_reach * model.getParams().gait_factors.adaptive_length_factor;
        step_height_ = robot_height * model.getParams().gait_factors.adaptive_height_factor;
        break;
    }

    // Calculate dynamic limits based on robot dimensions
    double min_step_length = leg_reach * model.getParams().gait_factors.min_length_factor;
    double max_step_length = leg_reach * model.getParams().gait_factors.max_length_factor;
    double min_step_height = robot_height * model.getParams().gait_factors.min_height_factor;
    double max_step_height = robot_height * model.getParams().gait_factors.max_height_factor;

    // Verify that parameters are within dynamic limits
    step_length_ = std::clamp<double>(step_length_, min_step_length, max_step_length);
    step_height_ = std::clamp<double>(step_height_, min_step_height, max_step_height);
}

void WalkController::adjustStepParameters() {
    // Enhanced step parameter adjustment with absolute positioning support
    // Note: IMU data access would need to be passed from LocomotionSystem
    // For now, use basic adjustment logic

    double total_tilt = 0.0f;
    double stability_factor = 1.0f;

    // Basic terrain assessment (would be enhanced with IMU data)
    if (total_tilt > 15.0f) {
        step_height_ *= 0.8f;
        step_length_ *= 0.7f;
    }

    // Limit parameters
    step_height_ = std::clamp<double>(step_height_, 15.0f, 50.0f);
    step_length_ = std::clamp<double>(step_length_, 20.0f, 80.0f);
}
