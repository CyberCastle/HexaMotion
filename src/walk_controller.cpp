#include "walk_controller.h"
#include "body_pose_config.h"
#include "gait_config_factory.h"
#include "gait_types.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "workspace_analyzer.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

WalkController::WalkController(RobotModel &m, Leg legs[NUM_LEGS], const BodyPoseConfiguration &pose_config)
    : model(m), time_delta_(0.0), step_clearance_(0.0), step_depth_(0.0), desired_linear_velocity_(0, 0, 0), desired_angular_velocity_(0.0),
      walk_state_(WALK_STOPPED), walkspace_(), odometry_ideal_(Point3D(0, 0, 0), Eigen::Quaterniond::Identity()), pose_state_(0),
      current_body_position_(Eigen::Vector3d::Zero()), current_body_orientation_(Eigen::Vector3d::Zero()),
      regenerate_walkspace_(false), legs_at_correct_phase_(0), legs_completed_first_step_(0), return_to_default_attempted_(false),
      leg_steppers_(), current_gait_config_(), terrain_adaptation_(m), body_pose_controller_(nullptr),
      velocity_limits_(m), current_leg_positions_{Point3D(), Point3D(), Point3D(), Point3D(), Point3D(), Point3D()}, legs_array_(legs), global_phase_(0) {

    standing_horizontal_reach_ = pose_config.standing_horizontal_reach; // cache from configuration

    // Initialize leg_steppers_ with references to actual legs from LocomotionSystem
    leg_steppers_.clear();

    std::string default_gait_name = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    GaitType default_gait_type = RobotModel::stringToGaitType(default_gait_name);
    GaitConfiguration default_gait_config = createGaitConfig(default_gait_type, model.getParams());
    setGait(default_gait_config);

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
        auto stepper = std::make_shared<LegStepper>(i, identity_tip_pose, legs[i], model);
        leg_steppers_.push_back(stepper);
    }

    // Initialize terrain adaptation
    terrain_adaptation_.initialize();

    // Set initial time delta from unified global parameter
    time_delta_ = model.getParams().time_delta;

    // Generate initial walkspace
    generateWalkspace();
}

// ================== Accessor Implementations (moved from header) ==================
StepCycle WalkController::getStepCycle() const {
    // OpenSHC adaptation: use configured gait step_frequency (normalization inside generateStepCycle)
    // Removing local frequency override avoids inconsistencies with applyGaitConfigToLegSteppers()
    // which already instantiates LegStepper StepCycles using the configured frequency.
    return current_gait_config_.generateStepCycle();
}

double WalkController::getTimeDelta() const { return time_delta_; }
double WalkController::getStepClearance() const { return step_clearance_; }
double WalkController::getStepDepth() const { return step_depth_; }

Point3D WalkController::getWalkPlane() const {
    return body_pose_controller_ ? body_pose_controller_->getWalkPlanePose().position : Point3D(0, 0, 0);
}

Point3D WalkController::getWalkPlaneNormal() const {
    if (body_pose_controller_) {
        Pose pose = body_pose_controller_->getWalkPlanePose();
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d normal = pose.rotation * z_axis;
        return Point3D(normal.x(), normal.y(), normal.z());
    }
    return Point3D(0, 0, 1);
}

double WalkController::getStanceDuration() const {
    double total_period = current_gait_config_.phase_config.stance_phase + current_gait_config_.phase_config.swing_phase;
    return total_period > 0 ? static_cast<double>(current_gait_config_.phase_config.stance_phase) / total_period : 0.0;
}

double WalkController::getSwingDuration() const {
    double denom = (current_gait_config_.phase_config.stance_phase + current_gait_config_.phase_config.swing_phase);
    return denom > 0 ? static_cast<double>(current_gait_config_.phase_config.swing_phase) / denom : 0.0;
}

double WalkController::getCycleFrequency() const { return current_gait_config_.getStepFrequency(); }

// Gait configuration management methods (OpenSHC equivalent)
bool WalkController::setGaitConfiguration(const GaitConfiguration &gait_config) {
    // Store the new gait configuration
    current_gait_config_ = gait_config;

    // Apply the configuration to all leg steppers
    applyGaitConfigToLegSteppers(gait_config);

    generateLimits();

    return true;
}

bool WalkController::setGait(const GaitConfiguration &gait_config) {
    // Delegate to setGaitConfiguration for consistency
    return setGaitConfiguration(gait_config);
}

bool WalkController::setGait(GaitType gait_type) {
    // Create gait configuration using factory and robot parameters
    GaitConfiguration gait_config = createGaitConfig(gait_type, model.getParams());
    return setGaitConfiguration(gait_config);
}

void WalkController::applyGaitConfigToLegSteppers(const GaitConfiguration &gait_config) {

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
        leg_stepper->setStepClearanceHeight(gait_config.swing_height);
        leg_stepper->setStanceSpanModifier(gait_config.stance_span_modifier); // OpenSHC stance_span_modifier propagation

        // Calculate phase offset using OpenSHC formula
        // OpenSHC: step_offset = (base_step_offset * multiplier) % step.period_
        int base_step_period = gait_config.phase_config.stance_phase + gait_config.phase_config.swing_phase;
        int normaliser = step_cycle.period_ / base_step_period;
        int base_step_offset = gait_config.phase_config.phase_offset * normaliser;
        int multiplier = gait_config.offsets.getForLegIndex(i);
        int phase_offset_iterations = (base_step_offset * multiplier) % step_cycle.period_;
        // OpenSHC: Store phase offset directly as iterations (no float conversion)
        leg_stepper->setPhaseOffset(phase_offset_iterations);

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

const TerrainAdaptation::ExternalTarget &WalkController::getExternalDefault(int leg_index) const {
    return terrain_adaptation_.getExternalDefault(leg_index);
}

const TerrainAdaptation::StepPlane &WalkController::getStepPlane(int leg_index) const {
    return terrain_adaptation_.getStepPlane(leg_index);
}

bool WalkController::hasTouchdownDetection(int leg_index) const {
    return terrain_adaptation_.hasTouchdownDetection(leg_index);
}

void WalkController::updateTerrainAdaptation(IFSRInterface *fsr_interface, IIMUInterface *imu_interface) {
    terrain_adaptation_.update(fsr_interface, imu_interface);
}

Point3D WalkController::estimateGravity() const {
    // Simple gravity estimation - in a real implementation this would use IMU data
    return Point3D(0.0, 0.0, -9.81);
}

// --- WalkController Methods Implementation ---
void WalkController::init(const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation) {

    walk_state_ = WALK_STOPPED;
    odometry_ideal_ = Pose(Point3D(0, 0, 0), Eigen::Quaterniond::Identity());

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
            // Use standing-pose horizontal reach (height-aware) for stance radius
            double leg_reach = standing_horizontal_reach_;
            double stance_radius = leg_reach; // Height-aware conservative reach

            Point3D stance_position(
                base_pos.x + stance_radius * cos(base_angle),
                base_pos.y + stance_radius * sin(base_angle),
                model.getDefaultHeightOffset() + model.getParams().standing_height); // Physical reference: z = getDefaultHeightOffset() + standing_height

            leg_stepper->setDefaultTipPose(stance_position);
        } // OpenSHC pattern: Initialize current tip pose to default stance position
        // This ensures LegStepper starts with proper stance coordinates
        leg_stepper->setCurrentTipPose(leg_stepper->getDefaultTipPose());

        // Initialize walk plane and normal so swing clearance is oriented correctly from the start
        leg_stepper->setWalkPlaneNormal(getWalkPlaneNormal());
    }

    // Init velocity input variables
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0;

    // Initialize global phase counter
    global_phase_ = 0;
}

void WalkController::updateWalk(const Point3D &linear_velocity_input, double angular_velocity_input,
                                const Eigen::Vector3d &current_body_position, const Eigen::Vector3d &current_body_orientation) {

    // OpenSHC: Cache frequently used values to reduce function call overhead
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

    // OpenSHC parity: block walking while any leg is not in WALKING state.
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (legs_array_[i].getLegState() != LegState::LEG_WALKING) {
            return;
        }
    }

    Eigen::Vector2d linear_input_xy(linear_velocity_input.x, linear_velocity_input.y);
    Eigen::Vector2d new_linear_velocity = Eigen::Vector2d::Zero();
    double new_angular_velocity = 0.0;

    // Collect current tip positions for limit interpolation
    Point3D tip_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        tip_positions[i] = leg_steppers_[i]->getCurrentTipPose();
    }

    double max_linear_speed = velocity_limits_.getLimit(linear_input_xy, angular_velocity_input, max_linear_speed_, tip_positions);
    double max_angular_speed = velocity_limits_.getLimit(linear_input_xy, angular_velocity_input, max_angular_speed_, tip_positions);
    double max_linear_acceleration = velocity_limits_.getLimit(linear_input_xy, angular_velocity_input, max_linear_acceleration_, tip_positions);
    double max_angular_acceleration = velocity_limits_.getLimit(linear_input_xy, angular_velocity_input, max_angular_acceleration_, tip_positions);

    if (walk_state_ != WALK_STOPPING) {
        new_linear_velocity = math_utils::clampedVector2d(linear_input_xy, max_linear_speed);
        new_angular_velocity = math_utils::clamped(angular_velocity_input, -max_angular_speed, max_angular_speed);
        new_linear_velocity *= (max_angular_speed != 0.0 ? (1.0 - std::abs(new_angular_velocity / max_angular_speed)) : 0.0);
    }

    Eigen::Vector2d current_linear_velocity(desired_linear_velocity_.x, desired_linear_velocity_.y);
    Eigen::Vector2d linear_acceleration = new_linear_velocity - current_linear_velocity;
    if (linear_acceleration.norm() < max_linear_acceleration * time_delta_) {
        current_linear_velocity += linear_acceleration;
    } else if (linear_acceleration.norm() > 0.0) {
        current_linear_velocity += linear_acceleration.normalized() * max_linear_acceleration * time_delta_;
    }

    double angular_acceleration = new_angular_velocity - desired_angular_velocity_;
    if (std::abs(angular_acceleration) < max_angular_acceleration * time_delta_) {
        desired_angular_velocity_ += angular_acceleration;
    } else {
        desired_angular_velocity_ += math_utils::sign(angular_acceleration) * max_angular_acceleration * time_delta_;
    }

    desired_linear_velocity_.x = current_linear_velocity.x();
    desired_linear_velocity_.y = current_linear_velocity.y();
    desired_linear_velocity_.z = 0.0;

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

            // OpenSHC exact: calculate phase offsets using integer arithmetic to avoid floating-point rounding errors
            StepCycle step_cycle = current_gait_config_.generateStepCycle();
            int base_step_period = current_gait_config_.phase_config.stance_phase + current_gait_config_.phase_config.swing_phase;
            int normaliser = step_cycle.period_ / base_step_period;
            int base_step_offset = current_gait_config_.phase_config.phase_offset * normaliser;

            for (size_t i = 0; i < leg_steppers_.size() && i < NUM_LEGS; ++i) {
                auto &leg_stepper = leg_steppers_[i];
                leg_stepper->setAtCorrectPhase(false);
                leg_stepper->setCompletedFirstStep(false);
                leg_stepper->setStepState(STEP_STANCE);

                // OpenSHC exact: integer-only phase offset calculation (matches state_controller.cpp line 542)
                int multiplier = current_gait_config_.offsets.getForLegIndex(static_cast<int>(i));
                int phase_offset_iterations = (base_step_offset * multiplier) % step_cycle.period_;
                leg_stepper->setPhase(phase_offset_iterations);

                // OpenSHC: initialize step state from initial phase
                leg_stepper->updateStepStateFromPhase();
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
            // OpenSHC: reset at_correct_phase flags for STOPPING tracking
            // Do NOT force STEP_STANCE — legs continue cycling until individually stopped
            for (auto &leg_stepper : leg_steppers_) {
                leg_stepper->setAtCorrectPhase(false);
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
    const bool is_active_walking = (walk_state_ == WALK_MOVING || walk_state_ == WALK_STARTING || walk_state_ == WALK_STOPPING);
    StepCycle step_cycle;
    bool step_cycle_calculated = false;

    const bool rough_terrain_mode = terrain_adaptation_.isRoughTerrainModeEnabled();
    const bool force_normal_touchdown = terrain_adaptation_.isForceNormalTouchdownEnabled();

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

        // Propagate walk plane and its normal from the controller (BodyPoseController) to each LegStepper.
        // This aligns swing clearance and touchdown direction with the estimated walking surface.
        leg_stepper->setWalkPlaneNormal(getWalkPlaneNormal());

        // Push terrain adaptation data into the leg stepper (OpenSHC-style external targets)
        TerrainAdaptation::ExternalTarget ext_target = terrain_adaptation_.getExternalTarget(static_cast<int>(i));
        LegStepperExternalTarget ls_target;
        ls_target.position = ext_target.position;
        ls_target.swing_clearance = ext_target.swing_clearance;
        ls_target.frame_id = ext_target.frame_id;
        ls_target.timestamp = ext_target.timestamp;
        ls_target.defined = ext_target.defined;
        leg_stepper->setExternalTarget(ls_target);

        TerrainAdaptation::ExternalTarget ext_default = terrain_adaptation_.getExternalDefault(static_cast<int>(i));
        LegStepperExternalTarget ls_default;
        ls_default.position = ext_default.position;
        ls_default.swing_clearance = ext_default.swing_clearance;
        ls_default.frame_id = ext_default.frame_id;
        ls_default.timestamp = ext_default.timestamp;
        ls_default.defined = ext_default.defined;
        leg_stepper->setExternalDefault(ls_default);

        const auto &step_plane = terrain_adaptation_.getStepPlane(static_cast<int>(i));
        leg_stepper->setStepPlane(step_plane.position, step_plane.normal, step_plane.valid);
        leg_stepper->setTouchdownDetection(terrain_adaptation_.hasTouchdownDetection(static_cast<int>(i)));

        // Update LegStepper's internal StepCycle before processing (fixes phase wrapping bug)
        if (step_cycle_calculated) {
            leg_stepper->setStepCycle(step_cycle);
        }

        if (is_active_walking && step_cycle_calculated) {
            // NOTE: Do NOT call updateStepStateFromPhase() here - it will be called by iteratePhase() after position update

            // OpenSHC STARTING state: per-leg phase synchronization
            if (walk_state_ == WALK_STARTING) {
                int swing_end_wrapped = step_cycle.swing_end_ % step_cycle.period_;

                // Once ALL legs are at correct phase, track completed first step
                if (legs_at_correct_phase_ == leg_count) {
                    if (leg_stepper->getPhase() == swing_end_wrapped && !leg_stepper->hasCompletedFirstStep()) {
                        leg_stepper->setCompletedFirstStep(true);
                        legs_completed_first_step_++;
                    }
                }

                // Check if this leg is at correct phase
                if (!leg_stepper->isAtCorrectPhase()) {
                    // OpenSHC: phase offset is already in iterations (no conversion needed)
                    int offset_iters = leg_stepper->getPhaseOffset();
                    bool offset_in_swing = (offset_iters > step_cycle.swing_start_ && offset_iters < step_cycle.swing_end_);

                    if (offset_in_swing && leg_stepper->getPhase() != swing_end_wrapped) {
                        // Leg's offset is in swing range and hasn't reached swing end yet
                        leg_stepper->setStepState(STEP_FORCE_STANCE);
                    } else {
                        // Leg is at correct phase — mark it
                        legs_at_correct_phase_++;
                        leg_stepper->setAtCorrectPhase(true);
                    }
                }
            }
            // OpenSHC STOPPING state: per-leg graceful shutdown
            else if (walk_state_ == WALK_STOPPING) {
                int swing_end_wrapped = step_cycle.swing_end_ % step_cycle.period_;
                bool zero_body_velocity = (leg_stepper->getStrideVector().norm() < 1e-9);

                Point3D error = leg_stepper->getCurrentTipPose() - leg_stepper->getTargetTipPose();
                Point3D walk_normal = getWalkPlaneNormal();
                error = math_utils::rejectVector(error, walk_normal);
                bool at_target_tip_position = (error.norm() < IK_TOLERANCE);

                if (zero_body_velocity && !leg_stepper->isAtCorrectPhase() && leg_stepper->getPhase() == swing_end_wrapped) {
                    if (at_target_tip_position || return_to_default_attempted_) {
                        return_to_default_attempted_ = false;
                        leg_stepper->setStepState(STEP_FORCE_STOP);
                        leg_stepper->setAtCorrectPhase(true);
                        legs_at_correct_phase_++;
                    } else {
                        return_to_default_attempted_ = true;
                    }
                }
            }

            // OpenSHC pattern: update tip position, then iterate phase (which updates state)
            leg_stepper->updateTipPosition(time_delta_, rough_terrain_mode, force_normal_touchdown);
            leg_stepper->iteratePhase(); // OpenSHC exact: increment phase + update state atomically

            // Update Leg's StepPhase based on final state after iteration
            bool in_swing = (leg_stepper->getStepState() == STEP_SWING);
            legs_array_[i].setStepPhase(in_swing ? SWING_PHASE : STANCE_PHASE);
        } else {
            // Force stance for non-active states
            legs_array_[i].setStepPhase(STANCE_PHASE);
        }
    }

    // OpenSHC: Optimized analysis and odometry updates
    // OpenSHC: proper SE(3) pose composition (accumulates heading)
    odometry_ideal_ = odometry_ideal_.addPose(calculateOdometry(time_delta_));

    // OpenSHC: Conditional walkspace regeneration
    if (regenerate_walkspace_) {
        generateWalkspace();
    }
}

Pose WalkController::calculateOdometry(double time_period) {
    // OpenSHC exact: return Pose with position delta + rotation delta
    Point3D position_delta(desired_linear_velocity_.x * time_period,
                           desired_linear_velocity_.y * time_period,
                           0.0);
    Eigen::Quaterniond rotation_delta(
        Eigen::AngleAxisd(desired_angular_velocity_ * time_period, Eigen::Vector3d::UnitZ()));
    return Pose(position_delta, rotation_delta);
}

void WalkController::generateLimits(StepCycle step) {
    velocity_limits_.setWalkspace(walkspace_);
    velocity_limits_.generateLimits(step, current_gait_config_,
                                    &max_linear_speed_, &max_angular_speed_,
                                    &max_linear_acceleration_, &max_angular_acceleration_);
}

void WalkController::generateLimits() {
    generateLimits(current_gait_config_.generateStepCycle());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC equivalent: WalkController::updateManual (velocity-based)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WalkController::updateManual(int primary_leg_index, const Eigen::Vector3d &primary_tip_velocity,
                                  int secondary_leg_index, const Eigen::Vector3d &secondary_tip_velocity) {
    const Parameters &params = model.getParams();
    for (size_t i = 0; i < leg_steppers_.size() && i < NUM_LEGS; ++i) {
        // Only process legs in MANUAL state (checked via external leg_states_ in StateController)
        // Caller is responsible for ensuring only MANUAL legs are targeted
        auto &leg_stepper = leg_steppers_[i];
        Leg &leg = legs_array_[i];

        bool is_selected = (static_cast<int>(i) == primary_leg_index ||
                            static_cast<int>(i) == secondary_leg_index);
        if (!is_selected)
            continue;

        Eigen::Vector3d tip_velocity_input = Eigen::Vector3d::Zero();
        if (static_cast<int>(i) == primary_leg_index) {
            tip_velocity_input = primary_tip_velocity;
        } else if (static_cast<int>(i) == secondary_leg_index) {
            tip_velocity_input = secondary_tip_velocity;
        }

        if (tip_velocity_input.norm() < 1e-9)
            continue;

        if (params.manual_leg.joint_control) {
            // Joint control: velocity inputs x/y/z mapped to coxa/tibia joint positions (OpenSHC 3DOF path)
            double coxa_joint_velocity = tip_velocity_input[1] * params.manual_leg.max_rotation_velocity * time_delta_;
            double tibia_joint_velocity = tip_velocity_input[0] * params.manual_leg.max_rotation_velocity * time_delta_;

            // Get current joint angles (radians), modify, and set back
            JointAngles angles = leg.getJointAngles();
            angles.coxa += coxa_joint_velocity;
            angles.tibia += tibia_joint_velocity;

            // Clamp to joint limits (radians, OpenSHC parity)
            angles.coxa = std::max(model.getCoxaAngleLimitRad(0), std::min(model.getCoxaAngleLimitRad(1), angles.coxa));
            angles.tibia = std::max(model.getTibiaAngleLimitRad(0), std::min(model.getTibiaAngleLimitRad(1), angles.tibia));

            leg.setJointAngles(angles);
            leg.updateTipPosition(); // FK update
            leg_stepper->setCurrentTipPose(leg.getCurrentTipPositionGlobal());
        } else {
            // Tip control: move tip in cartesian space (OpenSHC tip_control path)
            Point3D desired_tip = leg.getDesiredTipPosition();
            Point3D current_tip = leg.getCurrentTipPositionGlobal();
            double ik_error = Point3D(desired_tip.x - current_tip.x,
                                      desired_tip.y - current_tip.y,
                                      desired_tip.z - current_tip.z)
                                  .norm();

            Eigen::Vector3d tip_position_change = tip_velocity_input *
                                                  params.manual_leg.max_translation_velocity * time_delta_;

            // If IK error exceeds tolerance, reverse direction (OpenSHC pattern)
            if (ik_error >= IK_TOLERANCE) {
                Eigen::Vector3d error_dir(desired_tip.x - current_tip.x,
                                          desired_tip.y - current_tip.y,
                                          desired_tip.z - current_tip.z);
                error_dir.normalize();
                tip_position_change = -tip_position_change.norm() * error_dir;
            }

            Point3D current_pose = leg_stepper->getCurrentTipPose();
            Point3D new_tip_position(current_pose.x + tip_position_change[0],
                                     current_pose.y + tip_position_change[1],
                                     current_pose.z + tip_position_change[2]);
            leg_stepper->setCurrentTipPose(new_tip_position);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC equivalent: WalkController::updateManual (cartesian position-based)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WalkController::updateManual(int primary_leg_index, const Point3D &primary_tip_position,
                                  int secondary_leg_index, const Point3D &secondary_tip_position) {
    for (size_t i = 0; i < leg_steppers_.size() && i < NUM_LEGS; ++i) {
        auto &leg_stepper = leg_steppers_[i];
        bool is_selected = (static_cast<int>(i) == primary_leg_index ||
                            static_cast<int>(i) == secondary_leg_index);
        if (!is_selected)
            continue;

        Point3D tip_position_input;
        if (static_cast<int>(i) == primary_leg_index) {
            tip_position_input = primary_tip_position;
        } else if (static_cast<int>(i) == secondary_leg_index) {
            tip_position_input = secondary_tip_position;
        }

        if (tip_position_input.norm() > 1e-9) {
            leg_stepper->setCurrentTipPose(tip_position_input);
        }
    }
}

void WalkController::generateWalkspace() {
    walkspace_.clear();

    try {
        WorkspaceAnalyzer &analyzer = model.getWorkspaceAnalyzer();

        // OpenSHC parity: pass identity and default tip positions from leg steppers
        // so that Phase 1 uses correct default positions and Phase 2 computes proper default shift.
        Point3D identity_tips[NUM_LEGS];
        Point3D default_tips[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++) {
            if (i < static_cast<int>(leg_steppers_.size()) && leg_steppers_[i]) {
                identity_tips[i] = leg_steppers_[i]->getIdentityTipPose();
                default_tips[i] = leg_steppers_[i]->getDefaultTipPose();
            } else {
                JointAngles zero(0, 0, 0);
                identity_tips[i] = model.forwardKinematicsGlobalCoordinates(i, zero);
                default_tips[i] = identity_tips[i];
            }
        }
        analyzer.setTipPositions(identity_tips, default_tips);
        velocity_limits_.setReferenceTipPosition(default_tips[0]);

        analyzer.generateWorkspace();
        const auto &analyzer_map = analyzer.getWalkspaceMap();

        if (!analyzer_map.empty()) {
            walkspace_ = analyzer_map; // copy exact bearings from analyzer (OpenSHC parity)
        }
    } catch (const std::exception &ex) {
        std::cerr << "WalkController::generateWalkspace failed: " << ex.what() << std::endl;
        walkspace_.clear();
    }

    regenerate_walkspace_ = false;

    generateLimits();
}

// Accessor methods
std::shared_ptr<LegStepper> WalkController::getLegStepper(int leg_index) const {
    if (leg_index >= 0 && leg_index < (int)leg_steppers_.size()) {
        return leg_steppers_[leg_index];
    }
    return nullptr;
}

double WalkController::calculateStabilityIndex() const {
    // TODO: Simplified stability calculation
    // In a real implementation, this would use IMU and FSR data
    return 0.5f; // Default moderate stability
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