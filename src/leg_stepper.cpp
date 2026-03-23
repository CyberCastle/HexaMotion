#include "leg_stepper.h"
#include "body_pose_controller.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "workspace_analyzer.h"
#include <algorithm>
#include <cmath>

LegStepper::LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model)
    : leg_index_(leg_index),
      leg_(leg),
      robot_model_(robot_model),
      params_(robot_model.getParams()),
      identity_tip_pose_pose_(Pose(identity_tip_pose, Eigen::Quaterniond::Identity())),
      default_tip_pose_pose_(identity_tip_pose_pose_),
      target_tip_pose_pose_(identity_tip_pose_pose_),
      current_tip_pose_pose_(identity_tip_pose_pose_),
      identity_tip_pose_(identity_tip_pose_pose_.position),
      default_tip_pose_(default_tip_pose_pose_.position),
      target_tip_pose_(target_tip_pose_pose_.position),
      current_tip_pose_(current_tip_pose_pose_.position),
      origin_tip_pose_(identity_tip_pose),
      frozen_target_tip_pose_(identity_tip_pose) {

    // Validate physical reference height (z = getDefaultHeightOffset() when all angles are 0°)
    const Parameters &params = params_; // alias for readability in constructor
    double physical_reference_height = robot_model_.getDefaultHeightOffset();
    double expected_z_range_min = physical_reference_height - params.standing_height;
    double expected_z_range_max = physical_reference_height + params.standing_height;

    // Validate that identity_tip_pose is within expected physical range
    if (identity_tip_pose_.z < expected_z_range_min || identity_tip_pose_.z > expected_z_range_max) {
        // Log warning about unexpected tip pose height but continue execution
        // In a real implementation, this would use proper logging
        // For now, we accept the pose but note the physical reference
    }

    // Pre-compute the leg-aligned basis once so that per-iteration projections can reuse it without
    // incurring additional trigonometric cost. This mirrors the fixed DH frame assigned to each leg.
    base_angle_rad_ = BASE_THETA_OFFSETS[leg_index_ % NUM_LEGS];
    double cos_base = std::cos(base_angle_rad_);
    double sin_base = std::sin(base_angle_rad_);
    lateral_unit_ = Point3D(-sin_base, cos_base, 0.0);

    // Initialize state management
    at_correct_phase_ = false;
    completed_first_step_ = false;
    phase_ = 0;
    step_progress_ = 0.0;
    step_state_ = STEP_STANCE;
    previous_step_state_ = STEP_STANCE;

    // Initialize OpenSHC timing parameters
    swing_delta_t_ = 0.0;
    stance_delta_t_ = 0.0;
    swing_iterations_ = 0;
    stance_iterations_ = 0;
    current_iteration_ = 0;

    // Initialize StepCycle with default values (will be overridden by gait configuration)
    step_cycle_.frequency_ = 1.0;
    step_cycle_.period_ = 4;
    step_cycle_.stance_period_ = 3;
    step_cycle_.swing_period_ = 1;
    step_cycle_.stance_end_ = 1;
    step_cycle_.swing_start_ = 1;
    step_cycle_.swing_end_ = 2;
    step_cycle_.stance_start_ = 2;

    // Initialize gait configuration parameters (not part of StepCycle)
    swing_width_ = 5.0;            // Default swing width (will be overridden by gait configuration)
    step_clearance_height_ = 20.0; // Default step clearance height in mm (will be overridden by gait configuration)
    step_depth_ = STEP_DEPTH_DEFAULT;
    body_pose_controller_ = nullptr;

    // Initialize swing state management
    swing_initialized_ = false;
    nodes_generated_ = false;
    last_swing_iteration_ = -1;
    last_swing_start_iteration_ = -1;

    // Initialize velocity and movement vectors
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0.0;
    walk_plane_normal_ = Point3D(0, 0, 1);
    stride_vector_ = Point3D(0, 0, 0);
    current_tip_velocity_ = Point3D(0, 0, 0);
    swing_origin_tip_position_ = identity_tip_pose_;
    swing_origin_tip_velocity_ = Point3D(0, 0, 0);
    stance_origin_tip_position_ = identity_tip_pose_;

    // Initialize all Bezier control nodes with identity pose
    for (int i = 0; i < 5; ++i) {
        swing_1_nodes_[i] = identity_tip_pose_;
        swing_2_nodes_[i] = identity_tip_pose_;
        stance_nodes_[i] = identity_tip_pose_;
    }

    // Initialize velocity tracking
    previous_tip_pose_ = identity_tip_pose_;
    has_previous_position_ = false;
    time_step_ = params.time_delta; // Unified time step

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.identity_tip_pose = identity_tip_pose_;
    debug_state_.default_tip_pose = default_tip_pose_;
    debug_state_.current_tip_pose = current_tip_pose_;
    debug_state_.raw_target_pose = identity_tip_pose_;
    debug_state_.composed_pose = identity_tip_pose_;
    debug_state_.stride_linear = Point3D(0, 0, 0);
    debug_state_.stride_angular = Point3D(0, 0, 0);
    debug_state_.stride_total = Point3D(0, 0, 0);
    debug_state_.frozen_stride_total = Point3D(0, 0, 0);
    debug_state_.active_stride = Point3D(0, 0, 0);
    debug_state_.last_delta = Point3D(0, 0, 0);
    debug_state_.iteration = 0;
    debug_state_.step_state = step_state_;
    debug_state_.step_progress = 0.0;
#endif
}

void LegStepper::setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity) {

    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

// OpenSHC-based implementation with philosophical alignment adjustments:
// Here we compute stride each call but freeze (cache) its value at phase start; downstream code uses the
// frozen copy, preventing migration of the swing target. This mirrors OpenSHC intent where stride parameters
// remain effectively constant during a phase.
void LegStepper::updateStride() {

    // In OpenSHC this comes from walker_->getWalkPlane()/getWalkPlaneNormal().
    // We decouple by allowing WalkController (or another higher layer) to call
    // setWalkPlane()/setWalkPlaneNormal() each cycle. If not provided, fall back
    // to a flat horizontal walk plane with +Z normal through identity pose.
    auto normal_mag = std::sqrt(walk_plane_normal_.x * walk_plane_normal_.x +
                                walk_plane_normal_.y * walk_plane_normal_.y +
                                walk_plane_normal_.z * walk_plane_normal_.z);

    // Normalize plane normal (protect against extreme values)
    if (normal_mag > 1e-6) {
        walk_plane_normal_ = walk_plane_normal_ / normal_mag;
    }

    // Linear stride vector (OpenSHC formula)
    Point3D stride_vector_linear(desired_linear_velocity_.x, desired_linear_velocity_.y, 0.0);

    // Angular stride vector (OpenSHC formula)
    Point3D z_unit(0, 0, 1);
    Point3D radius = math_utils::rejectVector(current_tip_pose_, z_unit); // current rejection (more responsive, less stable)

    Point3D angular_velocity_vector(0, 0, desired_angular_velocity_);

    Point3D stride_vector_angular;
    stride_vector_angular.x = angular_velocity_vector.y * radius.z - angular_velocity_vector.z * radius.y;
    stride_vector_angular.y = angular_velocity_vector.z * radius.x - angular_velocity_vector.x * radius.z;
    stride_vector_angular.z = angular_velocity_vector.x * radius.y - angular_velocity_vector.y * radius.x;

    // Combination and scaling (OpenSHC formula)
    stride_vector_ = stride_vector_linear + stride_vector_angular;

    // Use StepCycle configuration values (OpenSHC equivalent calculation)
    double on_ground_ratio = double(step_cycle_.stance_period_) / double(step_cycle_.period_);
    double stride_scale = (on_ground_ratio / step_cycle_.frequency_);
    stride_vector_ = stride_vector_ * stride_scale;

    // Track individual contributions after scaling for debug/analysis purposes
    Point3D linear_scaled = stride_vector_linear * stride_scale;
    Point3D angular_scaled = stride_vector_ - linear_scaled;

    // Apply stride validation and safety constraints
    // stride_vector_ = calculateSafeStride(stride_vector_);

    // Freeze stride if not yet frozen for current phase
    if (!stride_frozen_) {
        frozen_stride_vector_linear_ = linear_scaled;
        frozen_stride_vector_angular_ = angular_scaled; // already scaled
        frozen_stride_vector_total_ = stride_vector_;
        stride_frozen_ = true;
    }

    // Swing clearance (OpenSHC formula) using validated & normalized plane normal
    swing_clearance_ = walk_plane_normal_ * step_clearance_height_;

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.stride_linear = linear_scaled;
    debug_state_.stride_angular = angular_scaled;
    debug_state_.stride_total = stride_vector_;
    debug_state_.frozen_stride_total = frozen_stride_vector_total_;
#endif
}

void LegStepper::beginSwingPhase() {
    stride_frozen_ = false;
    target_frozen_ = false;
}

void LegStepper::beginStancePhase() {
    stride_frozen_ = false;
    target_frozen_ = false;
}

void LegStepper::calculateSwingTiming(double time_delta) {
    // Override provided time_delta with unified global value to ensure consistency
    time_delta = params_.time_delta;
    // OpenSHC EXACT timing calculation using StepCycle values
    // Follow OpenSHC formula exactly: (double(step.swing_period_) / step.period_) / (step.frequency_ * time_delta)

    // OpenSHC exact calculation
    swing_iterations_ = int((double(step_cycle_.swing_period_) / step_cycle_.period_) / (step_cycle_.frequency_ * time_delta));

    // OpenSHC exact: Must be even (roundToEvenInt equivalent)
    if (swing_iterations_ % 2 != 0)
        swing_iterations_++;

    // Ensure minimum iterations for Bezier curve development
    if (swing_iterations_ < 10)
        swing_iterations_ = 10;

    // OpenSHC exact: swing_delta_t_ = 1.0 / (swing_iterations / 2.0)
    swing_delta_t_ = 1.0 / (swing_iterations_ / 2.0);

    // Calculate stance timing using same OpenSHC formula,
    // but apply symmetric rounding & parity rules like swing
    double raw_stance_iters =
        (double(step_cycle_.stance_period_) /
         step_cycle_.period_) /
        (step_cycle_.frequency_ * time_delta);

    stance_iterations_ = static_cast<int>(std::round(raw_stance_iters));
    if (stance_iterations_ % 2 != 0) {
        // Enforce even count for midpoint consistency (mirrors swing handling)
        stance_iterations_++;
    }

    // Maintain minimum for numerical stability of integration
    if (stance_iterations_ < 10) {
        stance_iterations_ = 10;
        if (stance_iterations_ % 2 != 0)
            stance_iterations_++; // keep even
    }

    stance_delta_t_ = (stance_iterations_ > 0) ? (1.0 / static_cast<double>(stance_iterations_)) : 0.0;
}

void LegStepper::initializeSwingPeriod(int iteration) {
    // Initialize swing origin position and velocity on first call or swing reset
    // In OpenSHC, this typically happens when entering swing state

    // Reset swing initialization if we're starting a new swing cycle
    if (iteration == 1 || iteration < last_swing_iteration_) {
        swing_initialized_ = false;
    }

    if (!swing_initialized_) {
        swing_origin_tip_position_ = current_tip_pose_;
        swing_origin_tip_velocity_ = current_tip_velocity_;
        swing_initialized_ = true;
    }

    current_iteration_ = iteration;
    last_swing_iteration_ = iteration;
}

void LegStepper::generatePrimarySwingControlNodes() {
    // OpenSHC exact implementation
    Point3D mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_) / 2.0;
    mid_tip_position.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    mid_tip_position = mid_tip_position + swing_clearance_;

    double mid_lateral_shift = swing_width_; // Use configured swing width from GaitConfiguration
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    // OpenSHC exact formula: walker_->getTimeDelta() / swing_delta_t_
    double time_delta = params_.time_delta; // Unified global time delta
    Point3D stance_node_seperation = swing_origin_tip_velocity_ * 0.25 * (time_delta / swing_delta_t_);

    // Control nodes for primary swing quartic bezier curves
    // Set for position continuity at transition between stance and primary swing curves (C0 Smoothness)
    swing_1_nodes_[0] = swing_origin_tip_position_;
    // Set for velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
    swing_1_nodes_[1] = swing_origin_tip_position_ + stance_node_seperation;
    // Set for acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
    swing_1_nodes_[2] = swing_origin_tip_position_ + stance_node_seperation * 2.0;
    // Set for acceleration continuity at transition between swing curves (C2 Smoothness for symetric curves)
    swing_1_nodes_[3] = (mid_tip_position + swing_1_nodes_[2]) / 2.0;
    swing_1_nodes_[3].z = mid_tip_position.z;
    // Set to default tip position so max swing height and transition to 2nd swing curve occurs at default tip position
    swing_1_nodes_[4] = mid_tip_position;

    validateAndFixControlNodes(swing_1_nodes_);
}

void LegStepper::generateSecondarySwingControlNodes(bool ground_contact) {
    // Follow OpenSHC exact implementation for maximum precision

    // Calculate final tip velocity for stance transition (OpenSHC formula)
    double time_delta = params_.time_delta; // Unified global time delta
    Point3D final_tip_velocity = stride_vector_ * (-1.0) * (stance_delta_t_ / time_delta);
    Point3D stance_node_seperation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t_);

    // Control nodes for secondary swing quartic bezier curves
    // Set for position continuity at transition between primary and secondary swing curves (C0 Smoothness)
    swing_2_nodes_[0] = swing_1_nodes_[4];
    // Set for velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
    swing_2_nodes_[1] = swing_1_nodes_[4] - (swing_1_nodes_[3] - swing_1_nodes_[4]);
    // Set for acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
    swing_2_nodes_[2] = target_tip_pose_ - stance_node_seperation * 2.0;
    // Set for velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
    swing_2_nodes_[3] = target_tip_pose_ - stance_node_seperation;
    // Set for position continuity at transition between secondary swing and stance curves (C0 Smoothness)
    swing_2_nodes_[4] = target_tip_pose_;

    // Ensure touchdown occurs exactly on the walking plane (standing height)
    // This eliminates tiny numerical drift in Z accumulated during swing integration
    swing_2_nodes_[4].z = default_tip_pose_.z;

    // Stops further movement of tip position in direction normal to walk plane
    if (ground_contact) {
        swing_2_nodes_[0] = current_tip_pose_ + stance_node_seperation * 0.0;
        swing_2_nodes_[1] = current_tip_pose_ + stance_node_seperation * 1.0;
        swing_2_nodes_[2] = current_tip_pose_ + stance_node_seperation * 2.0;
        swing_2_nodes_[3] = current_tip_pose_ + stance_node_seperation * 3.0;
        swing_2_nodes_[4] = current_tip_pose_ + stance_node_seperation * 4.0;
    }

    if (params_.enable_workspace_constrain) {
        validateAndFixControlNodes(swing_2_nodes_);
    }
}

void LegStepper::forceNormalTouchdown() {
    if (stance_iterations_ <= 0) {
        return;
    }

    // Node separation is based on stance iterations only,
    // matching the OpenSHC reference derivation.
    Point3D final_tip_velocity = stride_vector_ * (-1.0 / static_cast<double>(stance_iterations_));
    Point3D stance_node_separation = final_tip_velocity * 0.25;

    Point3D bezier_target = target_tip_pose_;
    Point3D bezier_origin = target_tip_pose_ - stance_node_separation * 4.0;
    bezier_origin.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    bezier_origin = bezier_origin + swing_clearance_;

    swing_1_nodes_[4] = bezier_origin;
    swing_2_nodes_[0] = bezier_origin;
    swing_2_nodes_[2] = bezier_target - stance_node_separation * 2.0;
    swing_1_nodes_[3] = swing_2_nodes_[0] - (swing_2_nodes_[2] - bezier_origin) / 2.0;
    swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[2] - bezier_origin) / 2.0;
}

void LegStepper::generateStanceControlNodes(double stride_scaler) {
    // OpenSHC-based stance control node generation - direct implementation

    // Calculate stance node separation vector (OpenSHC formula)
    Point3D stride_vector_to_use = stride_frozen_ ? frozen_stride_vector_total_ : stride_vector_;
    Point3D stance_node_separation = stride_vector_to_use * (-stride_scaler * 0.25);

    // Control nodes for stance quartic bezier curve (OpenSHC formula)
    // Set as initial tip position
    stance_nodes_[0] = stance_origin_tip_position_ + stance_node_separation * 0.0;
    // Set for constant velocity in stance period
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_separation * 1.0;
    // Set for constant velocity in stance period
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_separation * 2.0;
    // Set for constant velocity in stance period
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_separation * 3.0;
    // Set as target tip position
    stance_nodes_[4] = stance_origin_tip_position_ + stance_node_separation * 4.0;

    if (params_.enable_workspace_constrain) {
        validateAndFixControlNodes(stance_nodes_);
    }
}

double LegStepper::calculateStanceStrideScaler() {
    // OpenSHC exact implementation
    // From OpenSHC: double stride_scaler = double(modified_stance_period) / (mod(step.stance_end_ - step.stance_start_, step.period_));

    // Calculate modified stance period (for STARTING state handling)
    bool standard_stance_period = (step_state_ == STEP_SWING || completed_first_step_);
    int modified_stance_period = stance_iterations_; // Default to full stance period

    if (!standard_stance_period) {
        // During STARTING state, calculate modified stance period
        modified_stance_period = stance_iterations_ - (current_iteration_ % stance_iterations_);
    }

    // OpenSHC formula: stride_scaler = modified_stance_period / normal_stance_period
    double stride_scaler = double(modified_stance_period) / double(stance_iterations_);

    // Ensure reasonable bounds (base OpenSHC-inspired limits)
    stride_scaler = std::max(0.1, std::min(2.0, stride_scaler));

    return stride_scaler;
}

void LegStepper::updateTipPositionIterative(int iteration, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // OpenSHC-style iterative update - This is the MAIN method following OpenSHC philosophy

    // Single cached reference to parameters (avoid repeated getParams() bindings further below)
    const Parameters &params = params_;

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.iteration = iteration;
    debug_state_.step_state = step_state_;
    debug_state_.identity_tip_pose = identity_tip_pose_;
#endif

    // OpenSHC: Handle FORCE_STOP state
    if (step_state_ == STEP_FORCE_STOP) {
        // Force stance position and stop iteration
        step_progress_ = 0.0;
        current_tip_pose_ = identity_tip_pose_;
#ifdef COXA_STRIDE_TESTING_ENABLED
        debug_state_.step_progress = step_progress_;
        debug_state_.default_tip_pose = default_tip_pose_;
        debug_state_.active_stride = Point3D(0, 0, 0);
        debug_state_.raw_target_pose = identity_tip_pose_;
        debug_state_.composed_pose = identity_tip_pose_;
        debug_state_.last_delta = Point3D(0, 0, 0);
        debug_state_.current_tip_pose = current_tip_pose_;
        debug_state_.frozen_stride_total = frozen_stride_vector_total_;
#endif
        return;
    }

    // Calculate swing timing if not already done
    if (swing_delta_t_ <= 0.0) {
        calculateSwingTiming(params.time_delta);
    }

    // Update current iteration (global style)
    current_iteration_ = iteration;

    int swing_iteration = 0;
    int stance_iteration = 0;

    if (step_state_ == STEP_SWING) {
        swing_iteration = iteration - step_cycle_.swing_start_ + 1;
        if (swing_iteration < 1)
            swing_iteration = 1;
        if (swing_iteration > swing_iterations_)
            swing_iteration = swing_iterations_;

        step_progress_ = (swing_iterations_ > 0) ? static_cast<double>(swing_iteration) / static_cast<double>(swing_iterations_) : 0.0;
        step_progress_ = std::min(1.0, step_progress_);

        // OpenSHC: update default tip position at start of swing when rough terrain mode enabled
        if (rough_terrain_mode && swing_iteration == 1) {
            updateDefaultTipPosition();
        }
    } else {
        // OpenSHC: phase offset is already in iterations (no conversion needed)
        int modified_stance_start = completed_first_step_
                                        ? step_cycle_.stance_start_
                                        : getPhaseOffset();
        stance_iteration = math_utils::mod(iteration + (step_cycle_.period_ - modified_stance_start), step_cycle_.period_) + 1;
        if (stance_iteration < 1)
            stance_iteration = 1;
        if (stance_iteration > stance_iterations_)
            stance_iteration = stance_iterations_;

        step_progress_ = (stance_iterations_ > 0) ? static_cast<double>(stance_iteration) / static_cast<double>(stance_iterations_) : 0.0;
        step_progress_ = std::min(1.0, step_progress_);

        // OpenSHC: update default tip position at start of stance when rough terrain mode enabled
        if (rough_terrain_mode && stance_iteration == 1) {
            updateDefaultTipPosition();
        }
    }

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.step_progress = step_progress_;
    debug_state_.step_state = step_state_;
#endif

    // Update stride (will freeze on first iteration of the phase)
    updateStride();

    // Mid-stride target from default pose using frozen stride for stability
    Point3D active_stride = stride_frozen_ ? frozen_stride_vector_total_ : stride_vector_;
    Point3D raw_target = default_tip_pose_ + active_stride * 0.5;
#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.default_tip_pose = default_tip_pose_;
    debug_state_.identity_tip_pose = identity_tip_pose_;
    debug_state_.active_stride = active_stride;
#endif
    if (!target_frozen_) {
        if (params_.enable_workspace_constrain) {
            target_tip_pose_ = calculateSafeTarget(raw_target);
        } else {
            target_tip_pose_ = raw_target;
        }
        frozen_target_tip_pose_ = target_tip_pose_;
        target_frozen_ = true;
    } else {
        target_tip_pose_ = frozen_target_tip_pose_;
    }

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.raw_target_pose = raw_target;
    debug_state_.composed_pose = default_tip_pose_ + active_stride;
#endif

    if (step_state_ == STEP_SWING) {
        // Initialize swing period on first iteration
        initializeSwingPeriod(swing_iteration);

        // Detect ground contact via FSR (if enabled) for adaptive touchdown (OpenSHC style)
        bool ground_contact = false;
        if (params_.use_fsr_contact) {
            ground_contact = leg_.isInContact();
        }

        // Rough terrain handling: step plane and reactive probing
        if (rough_terrain_mode) {
            if (touchdown_detection_) {
                if (step_plane_valid_) {
                    Point3D target_tip_position = step_plane_position_;
                    Point3D difference = target_tip_position - target_tip_pose_;
                    target_tip_pose_ = target_tip_pose_ + math_utils::projectVector(difference, walk_plane_normal_);
                } else {
                    target_tip_pose_.z -= step_depth_;
                }
            }
        }

        // OpenSHC: regenerate ALL swing control nodes EVERY iteration
        generatePrimarySwingControlNodes();
        generateSecondarySwingControlNodes(ground_contact);

        // OpenSHC: adjust nodes so touchdown approach aligns with stance velocity
        if (force_normal_touchdown && !ground_contact) {
            forceNormalTouchdown();
        }

        // Determine which half of swing we're in based on actual swing progress
        int half_iterations = swing_iterations_ / 2;
        bool first_half = swing_iteration <= half_iterations;

        // Calculate absolute position using OpenSHC approach: quarticBezierDot + delta accumulation
        Point3D delta_pos;
        double time_input = 0.0;

        if (first_half) {
            // OpenSHC exact calculation: swing_delta_t_ * iteration (1-based)
            time_input = swing_delta_t_ * swing_iteration;

            // OpenSHC pattern: Use quarticBezierDot for velocity-based calculation
            delta_pos = math_utils::quarticBezierDot(swing_1_nodes_, time_input) * swing_delta_t_;
        } else {
            // OpenSHC exact calculation: swing_delta_t_ * (iteration - swing_iterations / 2)
            time_input = swing_delta_t_ * (swing_iteration - swing_iterations_ / 2);

            // OpenSHC pattern: Use quarticBezierDot for velocity-based calculation
            delta_pos = math_utils::quarticBezierDot(swing_2_nodes_, time_input) * swing_delta_t_;
        }
        // Update current tip pose based on calculated delta position
        // OpenSHC pattern: accumulate delta position to current tip pose
#ifdef COXA_STRIDE_TESTING_ENABLED
        debug_state_.last_delta = delta_pos;
#endif
        Point3D next_pose = current_tip_pose_ + delta_pos;
        if (params_.enable_workspace_constrain) {
            current_tip_pose_ = robot_model_.getWorkspaceAnalyzer().makeReachable(leg_index_, next_pose);
        } else {
            current_tip_pose_ = next_pose;
        }
#ifdef COXA_STRIDE_TESTING_ENABLED
        debug_state_.current_tip_pose = current_tip_pose_;
#endif

        // Calculate velocity for this iteration (OpenSHC pattern)
        current_tip_velocity_ = delta_pos / time_delta;

    } else if (step_state_ == STEP_STANCE) {

        // Initialize stance origin if needed (hybrid anti-drift extension).
        // Only reinitialize when truly entering stance from another state, not on cycle wrapping
        bool just_entered_stance = (previous_step_state_ != STEP_STANCE);
        if (stance_iteration == 1 && just_entered_stance) {

            // At stance entry, compare the touchdown pose against the frozen swing target. Any residual lateral
            // offset is projected onto the leg-aligned axis and bled off immediately. This keeps opposing legs
            // synchronized for any gait while avoiding artificial symmetry enforcement or interference with
            // commanded turning motions.
            Point3D stance_target = target_frozen_ ? frozen_target_tip_pose_ : target_tip_pose_;
            Point3D touchdown_residual = current_tip_pose_ - stance_target;
            double residual_lateral = computeLateralComponent(touchdown_residual);
            double lateral_abs = std::abs(residual_lateral);
            double hard_threshold = params.drift_hard_threshold_mm;
            double soft_threshold = params.drift_soft_threshold_mm;
            bool rectilinear_command = isRectilinearCommand();
            if (lateral_abs > soft_threshold && (rectilinear_command || lateral_abs > hard_threshold)) {
                double correction_gain = 1.0;
                if (lateral_abs <= hard_threshold) {
                    correction_gain = math_utils::clamp(params.drift_soft_blend_alpha, 0.0, 1.0);
                }
                if (correction_gain > 0.0) {
                    Point3D correction = lateral_unit_ * (residual_lateral * correction_gain);
                    current_tip_pose_ = current_tip_pose_ - correction;
#ifdef COXA_STRIDE_TESTING_ENABLED
                    debug_state_.current_tip_pose = current_tip_pose_;
#endif
                }
            }

            Point3D touchdown_offset = current_tip_pose_ - default_tip_pose_;
            double offset_norm = std::sqrt(touchdown_offset.x * touchdown_offset.x +
                                           touchdown_offset.y * touchdown_offset.y +
                                           touchdown_offset.z * touchdown_offset.z);

#ifdef TESTING_ENABLED
            last_touchdown_offset_norm_ = offset_norm;
            accumulated_drift_vector_ = accumulated_drift_vector_ + touchdown_offset;
            accumulated_drift_norm_ = std::sqrt(accumulated_drift_vector_.x * accumulated_drift_vector_.x +
                                                accumulated_drift_vector_.y * accumulated_drift_vector_.y +
                                                accumulated_drift_vector_.z * accumulated_drift_vector_.z);
            planar_drift_norm_ = std::sqrt(accumulated_drift_vector_.x * accumulated_drift_vector_.x +
                                           accumulated_drift_vector_.y * accumulated_drift_vector_.y);
            vertical_drift_ = accumulated_drift_vector_.z;
            double metrics_alpha = math_utils::clamp(params.drift_metrics_ema_alpha, 0.0, 1.0);
            if (metrics_alpha > 0.0) {
                drift_ema_norm_ = (1.0 - metrics_alpha) * drift_ema_norm_ + metrics_alpha * offset_norm;
            } else {
                drift_ema_norm_ = offset_norm;
            }
            if (accumulated_drift_norm_ > params.drift_metrics_cap_mm) {
                double scale = params.drift_metrics_cap_mm / accumulated_drift_norm_;
                accumulated_drift_vector_.x *= scale;
                accumulated_drift_vector_.y *= scale;
                accumulated_drift_vector_.z *= scale;
                accumulated_drift_norm_ = params.drift_metrics_cap_mm;
                planar_drift_norm_ = std::sqrt(accumulated_drift_vector_.x * accumulated_drift_vector_.x +
                                               accumulated_drift_vector_.y * accumulated_drift_vector_.y);
                vertical_drift_ = accumulated_drift_vector_.z;
            }
            // Drift debug logging removed (debug_drift parameter eliminated)
#endif // TESTING_ENABLED

            if (params.preserve_swing_end_pose) {
                // Continuity mode: keep touchdown pose as stance origin
                stance_origin_tip_position_ = current_tip_pose_;
            } else {
                // Anti-drift hybrid:
                // If small offset -> soft blend toward default.
                // If large offset -> hard reset.
                bool hard_reset = offset_norm > params.drift_hard_threshold_mm;
                bool soft_reset = !hard_reset && offset_norm > params.drift_soft_threshold_mm;
                if (hard_reset) {
                    current_tip_pose_ = default_tip_pose_;
                    stance_origin_tip_position_ = default_tip_pose_;
                } else if (soft_reset) {
                    double soft_alpha = math_utils::clamp(params.drift_soft_blend_alpha, 0.0, 1.0);
                    current_tip_pose_ = Point3D{current_tip_pose_.x + (default_tip_pose_.x - current_tip_pose_.x) * soft_alpha,
                                                current_tip_pose_.y + (default_tip_pose_.y - current_tip_pose_.y) * soft_alpha,
                                                current_tip_pose_.z + (default_tip_pose_.z - current_tip_pose_.z) * soft_alpha};
                    stance_origin_tip_position_ = current_tip_pose_;
                } else {
                    // Offset already within soft threshold: leave as-is for seamless continuity but count as corrected
                    stance_origin_tip_position_ = current_tip_pose_;
                }
            }
            // Regardless of drift mode, lock the stance plane Z to standing height when not adapting terrain
            if ((!rough_terrain_mode || force_normal_touchdown) && !params.use_fsr_contact) {
                current_tip_pose_.z = default_tip_pose_.z;
                stance_origin_tip_position_.z = default_tip_pose_.z;
            }
        }

        // Initialize tangential stance mode state at first stance iteration
        if (stance_iteration == 1 && just_entered_stance) {
            stance_tangent_initialized_ = false; // reset every new stance phase
        }

        // OpenSHC stance integration
        double stride_scaler = calculateStanceStrideScaler();
        generateStanceControlNodes(stride_scaler);
        double time_input = stance_iteration * stance_delta_t_;
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input) * stance_delta_t_;
#ifdef COXA_STRIDE_TESTING_ENABLED
        debug_state_.last_delta = delta_pos;
#endif
        current_tip_pose_ += delta_pos;
        // Keep stance motion constrained to the walking plane when not in rough terrain mode
        if ((!rough_terrain_mode || force_normal_touchdown) && !params.use_fsr_contact) {
            current_tip_pose_.z = default_tip_pose_.z;
        }
#ifdef COXA_STRIDE_TESTING_ENABLED
        debug_state_.current_tip_pose = current_tip_pose_;
#endif
        current_tip_velocity_ = delta_pos / time_delta;
    }

    // Optional phase-end snap to frozen target (enhancement; documented difference from vanilla OpenSHC)
    if (params.enable_phase_end_snap && target_frozen_) {
        bool at_end = (step_progress_ >= 0.999);
        if (at_end) {
            Point3D err = frozen_target_tip_pose_ - current_tip_pose_;
            double e2 = err.x * err.x + err.y * err.y + err.z * err.z;
            double tol2 = params.phase_end_snap_tolerance_mm * params.phase_end_snap_tolerance_mm;
            if (params.phase_end_snap_alpha >= 1.0 || e2 <= tol2) {
                current_tip_pose_ = frozen_target_tip_pose_;
            } else {
                current_tip_pose_ = current_tip_pose_ + err * params.phase_end_snap_alpha;
            }
            // Guarantee exact touchdown height on walking plane at swing end (only if FSR is not in use)
            if (step_state_ == STEP_SWING && (!rough_terrain_mode || force_normal_touchdown) && !params.use_fsr_contact) {
                current_tip_pose_.z = default_tip_pose_.z;
            }
        }
    }

#ifdef COXA_STRIDE_TESTING_ENABLED
    debug_state_.current_tip_pose = current_tip_pose_;
    debug_state_.frozen_stride_total = frozen_stride_vector_total_;
    debug_state_.step_state = step_state_;
    debug_state_.step_progress = step_progress_;
#endif

    // Track previous state for next iteration
    previous_step_state_ = step_state_;

    // Update velocity tracking for comprehensive safety validation
    if (has_previous_position_) {
        time_step_ = time_delta;
    }
    previous_tip_pose_ = current_tip_pose_;
    has_previous_position_ = true;
}

Point3D LegStepper::calculateStanceSpanChange() {
    // This method mirrors OpenSHC exactly, operating on the precomputed discrete workplanes.
    // Direct translation of OpenSHC LegStepper::calculateStanceSpanChange logic
    // 1. Compute target height (rounded) from default shift.
    // 2. Clamp target height within this leg's workspace range.
    // 3. Fetch interpolated workplane via getWorkplane().
    // 4. Apply bearing selection XOR logic (unchanged) to pick lateral radius (90° or 270°).
    // 5. Return lateral offset scaled by stance_span_modifier symmetrically.

    Point3D default_shift = default_tip_pose_ - identity_tip_pose_;
    double target_workplane_height = math_utils::setPrecision(default_shift.z, 3);

    const auto &analyzer = robot_model_.getWorkspaceAnalyzer();
    Workspace workspace = analyzer.getLegWorkspace(leg_index_);
    if (workspace.empty()) {
        return Point3D(0, 0, 0);
    }

    // Clamp height to available range so getWorkplane() never returns empty due to out-of-range.
    double min_h = workspace.begin()->first;
    double max_h = workspace.rbegin()->first;
    double clamped_height = math_utils::clamp(target_workplane_height, min_h, max_h);

    Workplane interpolated = analyzer.getWorkplane(leg_index_, clamped_height);
    if (interpolated.empty()) {
        return Point3D(0, 0, 0); // Safety fallback (should not happen after clamping)
    }

    // Bearing selection logic preserved from original (XOR to ensure symmetric widening)
    double stance_span_modifier = stance_span_modifier_;
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    int bearing = ((positive_y_axis ^ (stance_span_modifier > 0.0)) ? 270 : 90);
    stance_span_modifier *= (positive_y_axis ? 1.0 : -1.0); // Positive modifier widens both sides

    double radius = 0.0;
    auto it = interpolated.find(bearing);
    if (it != interpolated.end()) {
        radius = it->second;
    }

    return Point3D(0.0, radius * stance_span_modifier, 0.0);
}

void LegStepper::updateDefaultTipPosition() {
    // 1. Modify identity tip position by stance span change
    Point3D identity_tip_position = identity_tip_pose_ + calculateStanceSpanChange();

    if (body_pose_controller_) {
        Pose default_body_pose = body_pose_controller_->getDefaultBodyPose();
        Eigen::Vector3d transformed = default_body_pose.transformVector(
            Eigen::Vector3d(identity_tip_position.x, identity_tip_position.y, identity_tip_position.z));
        identity_tip_position = Point3D(transformed.x(), transformed.y(), transformed.z());
    }

    // 2. Project vector from identity to stance origin onto walk plane normal
    Point3D identity_to_stance_origin = stance_origin_tip_position_ - identity_tip_position;
    Point3D projection_to_walk_plane = math_utils::projectVector(identity_to_stance_origin, walk_plane_normal_);

    Point3D new_default_tip_position = identity_tip_position + projection_to_walk_plane;

    // 3. Update default tip pose if movement exceeds IK tolerance (millimeters)
    Point3D delta = default_tip_pose_ - new_default_tip_position;
    double delta_norm = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    default_tip_pose_ = new_default_tip_position;
    if (delta_norm > IK_TOLERANCE) {
        // In OpenSHC this would trigger walker_->setRegenerateWalkspace(); intentionally omitted.
    }
}

// Update using internal phase_ (controller advances phase after this call).
void LegStepper::updateTipPosition(double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // Derive iteration number for iterative update path from internal phase_.
    // NOTE: We reuse existing logic by calling updateTipPositionIterative with 'phase_' so we do not duplicate the long body.
    updateTipPositionIterative(phase_, time_delta, rough_terrain_mode, force_normal_touchdown);
}

void LegStepper::updateStepStateFromPhase() {
    // Determine state directly from phase_ relative to configured step_cycle_.
    // step_cycle_ may have been set from gait configuration.
    if (step_cycle_.period_ <= 0) {
        return;
    }

    int local = math_utils::mod(phase_, step_cycle_.period_);
    bool in_swing = (local >= step_cycle_.swing_start_ && local < step_cycle_.swing_end_);
    if (step_state_ == STEP_FORCE_STOP)
        return;
    // OpenSHC parity: FORCE_STANCE prevents transition to SWING in swing range,
    // but when phase enters stance range FORCE_STANCE is lifted to STANCE.
    if (in_swing && step_state_ != STEP_FORCE_STANCE) {
        if (step_state_ != STEP_SWING) {
            setStepState(STEP_SWING);
            initializeSwingPeriod(1);
            beginSwingPhase();
        }
    } else if (!in_swing && (local < step_cycle_.stance_end_ || local >= step_cycle_.stance_start_)) {
        // Unconditionally transition to STANCE (including from FORCE_STANCE)
        if (step_state_ != STEP_STANCE) {
            setStepState(STEP_STANCE);
            beginStancePhase();
        }
    }
    // When in_swing && STEP_FORCE_STANCE: neither branch fires → state preserved
}

/**
 * @brief Iterate phase and update step state (OpenSHC LegStepper::iteratePhase exact equivalent).
 *
 * Atomic operation that:
 * 1. Increments phase_ by 1 (wrapping at period)
 * 2. Updates step_state_ based on new phase
 * 3. Calculates step_progress_ from new phase
 *
 * This ensures step state (SWING/STANCE) changes happen in sync with phase transitions,
 * preventing the 1-iteration lag that occurs when state is updated before phase increment.
 */
void LegStepper::iteratePhase() {
    if (step_cycle_.period_ <= 0) {
        return;
    }

    // OpenSHC exact: increment phase first, then update state based on new phase
    phase_ = (phase_ + 1) % step_cycle_.period_;
    updateStepStateFromPhase();

    // Calculate step progress from new phase (OpenSHC parity)
    step_progress_ = static_cast<double>(phase_) / static_cast<double>(step_cycle_.period_);
}

// ========================================================================
// WORKSPACE VALIDATION AND SAFETY METHODS
// ========================================================================

bool LegStepper::validateTargetTipPose(const Point3D &target_pose) const {
    Point3D reachable = robot_model_.getWorkspaceAnalyzer().makeReachable(leg_index_, target_pose);
    return math_utils::distance(reachable, target_pose) <= IK_TOLERANCE;
}

Point3D LegStepper::constrainToWorkspace(const Point3D &target_pose) const {
    return robot_model_.getWorkspaceAnalyzer().makeReachable(leg_index_, target_pose);
}

Point3D LegStepper::calculateSafeTarget(const Point3D &desired_target) const {
    // First validate if the desired target is reachable
    if (validateTargetTipPose(desired_target)) {
        return desired_target; // Target is already safe
    }

    // Target is not reachable, constrain it to workspace
    Point3D safe_target = constrainToWorkspace(desired_target);

    // If constraining still doesn't work, use default as fallback
    if (!validateTargetTipPose(safe_target)) {
        safe_target = default_tip_pose_;
    }

    return safe_target;
}

// ========================================================================
// VELOCITY LIMITING AND VALIDATION METHODS
// ========================================================================
Point3D LegStepper::calculateSafeStride(const Point3D &desired_stride) const {

    // Adjust stride to respect workspace limits
    Point3D safe_stride = desired_stride;

    // Calculate potential target position from this stride
    Point3D potential_target = default_tip_pose_ + safe_stride * 0.5;

    // Check if this target would be reachable
    if (!validateTargetTipPose(potential_target)) {
        Point3D adjusted_target = robot_model_.getWorkspaceAnalyzer().makeReachable(
            leg_index_, potential_target);
        safe_stride = (adjusted_target - default_tip_pose_) * 2.0;
    }

    return safe_stride;
}

// ========================================================================
// CONTROL NODE VALIDATION METHODS
// ========================================================================

void LegStepper::validateAndFixControlNodes(Point3D nodes[5]) const {
    // Validate and fix each control node
    for (int i = 0; i < 5; i++) {
        if (!validateTargetTipPose(nodes[i])) {
            // Node is not reachable, constrain it to workspace
            Point3D safe_node = robot_model_.getWorkspaceAnalyzer().makeReachable(
                leg_index_, nodes[i]);

            // If constraining still doesn't work, use default position as fallback
            if (!validateTargetTipPose(safe_node)) {
                safe_node = robot_model_.getLegDefaultPosition(leg_index_);
            }

            nodes[i] = safe_node;
        }
    }
}

// Comprehensive safety validation (combines all 4 steps)
bool LegStepper::validateCurrentTrajectory() const {
    // Step 1: Validate target tip pose is within workspace
    if (!validateTargetTipPose(target_tip_pose_)) {
        return false;
    }

    // Step 2: Validate current control nodes are within workspace
    // Check all 5 control nodes for each trajectory type
    for (size_t i = 0; i < 5; ++i) {
        if (!validateTargetTipPose(swing_1_nodes_[i])) {
            return false;
        }
        if (!validateTargetTipPose(swing_2_nodes_[i])) {
            return false;
        }
        if (!validateTargetTipPose(stance_nodes_[i])) {
            return false;
        }
    }

    return true;
}

// Auxiliary method for velocity calculations
Point3D LegStepper::calculateCurrentTipVelocity() const {
    // Calculate velocity based on recent position changes
    // This is a simplified approach - in practice, you might want to use
    // derivative of the Bézier curve at current parameter t
    Point3D current_velocity = Point3D(0, 0, 0);

    // If we have valid previous position, calculate velocity
    if (has_previous_position_) {
        double dt = time_step_; // Assuming constant time step
        if (dt > 0.0) {
            current_velocity = (current_tip_pose_ - previous_tip_pose_) / dt;
        }
    }

    return current_velocity;
}

/**
 * @brief Check whether the current velocity request represents straight-line motion.
 *
 * When a yaw component is present we preserve the lateral residual so that the leg can
 * follow the commanded rotation. Purely translational commands benefit from aggressive
 * lateral cleanup to keep opposing legs synchronized without introducing steering bias.
 */
bool LegStepper::isRectilinearCommand() const {
    const double angular_epsilon = 1e-6;
    if (std::abs(desired_angular_velocity_) > angular_epsilon) {
        return false;
    }

    double planar_speed_sq = desired_linear_velocity_.x * desired_linear_velocity_.x +
                             desired_linear_velocity_.y * desired_linear_velocity_.y;
    return planar_speed_sq > 1e-6;
}

/**
 * @brief Project a world-frame vector onto the leg's forward axis.
 */
double LegStepper::computeLateralComponent(const Point3D &vec) const {
    return vec.x * lateral_unit_.x + vec.y * lateral_unit_.y;
}