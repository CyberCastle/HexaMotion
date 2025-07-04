#include "walk_controller.h"
#include "hexamotion_constants.h"
#include "workspace_validator.h" // Use unified validator instead
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

// LegStepper implementation moved to header file

WalkController::WalkController(RobotModel &m)
    : model(m), current_gait(TRIPOD_GAIT), gait_phase(0.0f),
      terrain_adaptation_(m), velocity_limits_(m),
      step_clearance_(30.0), step_depth_(10.0), body_clearance_(100.0),
      desired_linear_velocity_(0,0,0), desired_angular_velocity_(0.0),
      walk_state_(WALK_STOPPED), pose_state_(0),
      regenerate_walkspace_(false), legs_at_correct_phase_(0), legs_completed_first_step_(0),
      return_to_default_attempted_(false)
{
    // Inicializar leg_steppers_ con posiciones reales del modelo
    leg_steppers_.clear();
    // Definir offsets de fase para trípode (y reutilizable para otros patrones)
    static const double tripod_phase_offsets[NUM_LEGS] = {
        0.0f, // Leg 0 (Anterior Right) - Group A (even)
        0.5f, // Leg 1 (Middle Right) - Group B (odd)
        0.0f, // Leg 2 (Posterior Right) - Group A (even)
        0.5f, // Leg 3 (Posterior Left) - Group B (odd)
        0.0f, // Leg 4 (Middle Left) - Group A (even)
        0.5f  // Leg 5 (Anterior Left) - Group B (odd)
    };
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Calcular posición por defecto de cada pierna basada en el modelo
        const Parameters &p = model.getParams();
        double base_angle = p.dh_parameters[i][0][3]; // theta DH de la base de la pierna
        double base_x = p.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
        double base_y = p.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));
        double leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
        double safe_reach = leg_reach * 0.65f; // 65% de seguridad
        double default_foot_x = base_x + safe_reach * cos(math_utils::degreesToRadians(base_angle));
        double default_foot_y = base_y + safe_reach * sin(math_utils::degreesToRadians(base_angle));
        Point3D identity_tip(default_foot_x, default_foot_y, 0);
        auto stepper = std::make_shared<LegStepper>(this, i, identity_tip);
        stepper->setPhaseOffset(tripod_phase_offsets[i]);
        // Si phase_offset_ es double, usar: stepper->phase_offset_ = tripod_phase_offsets[i] * periodo_de_pasos;
        leg_steppers_.push_back(stepper);
    }

    // Initialize terrain adaptation system
    terrain_adaptation_.initialize();

    // Initialize velocity limits with default gait parameters
    VelocityLimits::GaitConfig default_gait;
    default_gait.frequency = DEFAULT_ANGULAR_SCALING;
    default_gait.stance_ratio = 0.6f;
    default_gait.swing_ratio = 0.4f;
    default_gait.time_to_max_stride = 2.0f;
    velocity_limits_.generateLimits(default_gait);

    // Initialize current velocities to zero
    current_velocities_ = VelocityLimits::LimitValues();

    // Initialize current leg positions array for collision tracking
    for (int i = 0; i < NUM_LEGS; i++) {
        current_leg_positions_[i] = Point3D(0, 0, 0);
    }

    // Initialize workspace validator with optimized settings
    ValidationConfig config;
    config.safety_margin_factor = 0.65f;        // Same as original 65% safety margin
    config.collision_safety_margin = 30.0f;     // 30mm safety between legs
    config.enable_collision_checking = true;    // Enable collision avoidance
    config.enable_joint_limit_checking = false; // Disable for performance (IK already checks)
    workspace_validator_ = std::make_unique<WorkspaceValidator>(m, config);

    // Initialize the walk controller
    init();
}

bool WalkController::setGaitType(GaitType gait) {
    current_gait = gait;
    gait_phase = 0.0f;
    return true;
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

    // Update the leg stepper's trajectory
    leg_stepper->updateTipPosition();

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
    for (auto& leg_stepper : leg_steppers_) {
        // TODO: Get stance positions from parameters
        Point3D identity_tip_pose(0, 0, 0); // Default position
        leg_stepper->setDefaultTipPose(identity_tip_pose);
    }

    // Init velocity input variables
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0;

    // Generate step timing
    generateStepCycle();
}

StepCycle WalkController::generateStepCycle(bool set_step_cycle) {
    StepCycle step;

    // Default parameters (TODO: get from configuration)
    int stance_phase = 60;  // iterations
    int swing_phase = 40;   // iterations
    double step_frequency = 1.0; // Hz

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
    if (step.stance_period_ % 2 != 0) step.stance_period_++;
    if (step.swing_period_ % 2 != 0) step.swing_period_++;

    // Set step cycle in walk controller and update phase in leg steppers
    if (set_step_cycle) {
        step_ = step;
        if (walk_state_ == WALK_MOVING) {
            for (auto& leg_stepper : leg_steppers_) {
                leg_stepper->updatePhase();
            }
        }
    }

    return step;
}

void WalkController::updateWalk(const Point3D& linear_velocity_input, double angular_velocity_input) {
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

    // Check that all legs are in WALKING state (simplified for now)
    // TODO: Implement proper leg state checking

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
        for (auto& leg_stepper : leg_steppers_) {
            leg_stepper->setAtCorrectPhase(false);
            leg_stepper->setCompletedFirstStep(false);
            leg_stepper->setStepState(STEP_STANCE);
            leg_stepper->setPhase(leg_stepper->getPhaseOffset());
            leg_stepper->updateStepState();
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
    for (auto& leg_stepper : leg_steppers_) {
        // Walk State Machine
        if (walk_state_ == WALK_STARTING) {
            // Check if all legs have completed one step
            if (legs_at_correct_phase_ == leg_count) {
                if (leg_stepper->getPhase() == step_.swing_end_ && !leg_stepper->hasCompletedFirstStep()) {
                    leg_stepper->setCompletedFirstStep(true);
                    legs_completed_first_step_++;
                }
            }
            // Force any leg state into STANCE if it starts offset in a mid-swing state
            if (!leg_stepper->isAtCorrectPhase()) {
                if (leg_stepper->getPhaseOffset() > step_.swing_start_ &&
                    leg_stepper->getPhaseOffset() < step_.swing_end_ &&
                    leg_stepper->getPhase() != step_.swing_end_) {
                    leg_stepper->setStepState(STEP_FORCE_STANCE);
                } else {
                    legs_at_correct_phase_++;
                    leg_stepper->setAtCorrectPhase(true);
                }
            }
        } else if (walk_state_ == WALK_MOVING) {
            leg_stepper->setAtCorrectPhase(false);
        } else if (walk_state_ == WALK_STOPPING) {
            // All legs must attempt at least one step to achieve default tip position
            bool zero_body_velocity = leg_stepper->getStrideVector().x == 0 &&
                                    leg_stepper->getStrideVector().y == 0 &&
                                    leg_stepper->getStrideVector().z == 0;
            Point3D walk_plane_normal = leg_stepper->getWalkPlaneNormal();
            Point3D error = leg_stepper->getCurrentTipPose() - leg_stepper->getTargetTipPose();
            Point3D error_projection = math_utils::projectVector(error, walk_plane_normal);
            bool at_target_tip_position = (math_utils::distance(error_projection, Point3D(0,0,0)) < 1.0); // 1mm tolerance

            if (zero_body_velocity && !leg_stepper->isAtCorrectPhase() && leg_stepper->getPhase() == step_.swing_end_) {
                if (at_target_tip_position || return_to_default_attempted_) {
                    return_to_default_attempted_ = false;
                    leg_stepper->updateDefaultTipPosition();
                    leg_stepper->setStepState(STEP_FORCE_STOP);
                    leg_stepper->setAtCorrectPhase(true);
                    legs_at_correct_phase_++;
                } else {
                    return_to_default_attempted_ = true;
                }
            }
        } else if (walk_state_ == WALK_STOPPED) {
            leg_stepper->setStepState(STEP_FORCE_STOP);
            leg_stepper->setPhase(0);
        }

        // Update tip positions
        // TODO: Check leg state properly
        leg_stepper->updateTipPosition();
        leg_stepper->updateTipRotation();
        leg_stepper->iteratePhase();
    }

    updateWalkPlane();
    odometry_ideal_ = odometry_ideal_ + calculateOdometry(time_delta_);
    if (regenerate_walkspace_) {
        generateWalkspace();
    }
}

void WalkController::updateWalkPlane() {
    std::vector<double> raw_A;
    std::vector<double> raw_B;

    if (NUM_LEGS >= 3) { // Minimum for plane estimation
        for (auto& leg_stepper : leg_steppers_) {
            Point3D default_tip_pose = leg_stepper->getDefaultTipPose();
            raw_A.push_back(default_tip_pose.x);
            raw_A.push_back(default_tip_pose.y);
            raw_A.push_back(1.0);
            raw_B.push_back(default_tip_pose.z);
        }

        // Estimate walk plane using least squares
        // Simplified implementation - in practice you'd use Eigen or similar
        if (raw_A.size() >= 9) { // At least 3 legs * 3 coordinates
            // For now, use a simple plane estimation
            // TODO: Implement proper least squares plane fitting
            walk_plane_ = Point3D(0, 0, 0);
            walk_plane_normal_ = Point3D(0, 0, 1);
        }
    } else {
        walk_plane_ = Point3D(0, 0, 0);
        walk_plane_normal_ = Point3D(0, 0, 1);
    }
}

Point3D WalkController::calculateOdometry(double time_period) {
    Point3D desired_linear_velocity(desired_linear_velocity_.x, desired_linear_velocity_.y, 0);
    Point3D position_delta = desired_linear_velocity * time_period;

    // TODO: Implement proper rotation delta calculation
    // For now, return only position delta
    return position_delta;
}

void WalkController::generateWalkspace() {
    // Simplified walkspace generation
    // TODO: Implement full walkspace calculation like OpenSHC
    walkspace_.clear();

    // Populate with basic circular workspace
    for (int bearing = 0; bearing <= 360; bearing += 10) {
        double radius = 200.0; // 200mm default radius
        walkspace_[bearing] = radius;
    }

    regenerate_walkspace_ = false;
    generateLimits(step_);
}

void WalkController::generateLimits(StepCycle step) {
    // Simplified limit generation
    // TODO: Implement full limit calculation like OpenSHC

    max_linear_speed_.clear();
    max_angular_speed_.clear();
    max_linear_acceleration_.clear();
    max_angular_acceleration_.clear();

    for (auto& walkspace_entry : walkspace_) {
        double walkspace_radius = walkspace_entry.second;
        double on_ground_ratio = double(step.stance_period_) / step.period_;

        // Basic limit calculations
        double max_linear_speed = (walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
        double max_linear_acceleration = max_linear_speed / 2.0; // 2 second time to max
        double max_angular_speed = max_linear_speed / 100.0; // 100mm stance radius
        double max_angular_acceleration = max_angular_speed / 2.0;

        max_linear_speed_[walkspace_entry.first] = max_linear_speed;
        max_angular_speed_[walkspace_entry.first] = max_angular_speed;
        max_linear_acceleration_[walkspace_entry.first] = max_linear_acceleration;
        max_angular_acceleration_[walkspace_entry.first] = max_angular_acceleration;
    }
}

double WalkController::getLimit(const Point3D& linear_velocity_input, double angular_velocity_input,
                               const std::map<int, double>& limit) {
    double min_limit = 1e6; // Large value

    for (auto& leg_stepper : leg_steppers_) {
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
    // TODO: Get from robot model
    return Point3D(0, 0, 0);
}

std::shared_ptr<LegStepper> WalkController::getLegStepper(int leg_index) const {
    if (leg_index >= 0 && leg_index < (int)leg_steppers_.size()) {
        return leg_steppers_[leg_index];
    }
    return nullptr;
}
