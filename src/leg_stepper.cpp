#include "leg_stepper.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "velocity_limits.h"
#include "workspace_analyzer.h"
#include <algorithm>
#include <cmath>

LegStepper::LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model)
    : leg_index_(leg_index),
      leg_(leg),
      robot_model_(robot_model),
      params_(robot_model.getParams()),
      identity_tip_pose_(identity_tip_pose) {

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

    // Initialize basic properties
    default_tip_pose_ = identity_tip_pose_;
    origin_tip_pose_ = identity_tip_pose_;
    target_tip_pose_ = identity_tip_pose_;
    current_tip_pose_ = identity_tip_pose_;

    // Initialize state management
    at_correct_phase_ = false;
    completed_first_step_ = false;
    phase_ = 0;
    step_progress_ = 0.0;
    step_state_ = STEP_STANCE;

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
    step_cycle_.stance_start_ = 0;
    step_cycle_.stance_end_ = 3;
    step_cycle_.swing_start_ = 3;
    step_cycle_.swing_end_ = 4;

    // Initialize gait configuration parameters (not part of StepCycle)
    swing_width_ = 5.0;            // Default swing width (will be overridden by gait configuration)
    step_clearance_height_ = 20.0; // Default step clearance height in mm (will be overridden by gait configuration)

    // Initialize swing state management
    swing_initialized_ = false;
    nodes_generated_ = false;
    last_swing_iteration_ = -1;
    last_swing_start_iteration_ = -1;

    // Initialize velocity and movement vectors
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0.0;
    walk_plane_ = Point3D(0, 0, 0);
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

    // Initialize safety systems
    velocity_limits_ = nullptr;

    // Initialize velocity tracking
    previous_tip_pose_ = identity_tip_pose_;
    has_previous_position_ = false;
    time_step_ = params.time_delta; // Unified time step
}

void LegStepper::setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity) {
    // Validate and limit velocities before setting them
    Point3D safe_velocity = validateAndLimitVelocities(linear_velocity, angular_velocity);

    desired_linear_velocity_ = safe_velocity;
    desired_angular_velocity_ = angular_velocity; // Angular velocity validation is done in validateAndLimitVelocities
}

void LegStepper::updateStride() {
    // OpenSHC-based implementation with philosophical alignment adjustments:
    // Previous HexaMotion recalculated stride every iteration using current_tip_pose_, causing intra-phase drift.
    // Here we compute stride each call but freeze (cache) its value at phase start; downstream code uses the
    // frozen copy, preventing migration of the swing target. This mirrors OpenSHC intent where stride parameters
    // remain effectively constant during a phase.

    // In OpenSHC this comes from walker_->getWalkPlane()/getWalkPlaneNormal().
    // We decouple by allowing WalkController (or another higher layer) to call
    // setWalkPlane()/setWalkPlaneNormal() each cycle. If not provided, fall back
    // to a flat horizontal walk plane with +Z normal through identity pose.
    auto normal_mag = std::sqrt(walk_plane_normal_.x * walk_plane_normal_.x +
                                walk_plane_normal_.y * walk_plane_normal_.y +
                                walk_plane_normal_.z * walk_plane_normal_.z);
    if (normal_mag < 1e-6) {
        walk_plane_normal_ = Point3D(0, 0, 1); // fallback normal
        walk_plane_ = Point3D(identity_tip_pose_.x, identity_tip_pose_.y, identity_tip_pose_.z);
    }
    // Normalize plane normal (protect against extreme values)
    if (normal_mag > 1e-6) {
        walk_plane_normal_ = walk_plane_normal_ / normal_mag;
    }

    // Linear stride vector (OpenSHC formula)
    Point3D stride_vector_linear(desired_linear_velocity_.x, desired_linear_velocity_.y, 0.0);

    // Angular stride vector (OpenSHC formula)
    Point3D z_unit(0, 0, 1);
    double dot_product = current_tip_pose_.x * z_unit.x + current_tip_pose_.y * z_unit.y + current_tip_pose_.z * z_unit.z;
    Point3D projection_on_z = z_unit * dot_product;
    // Difference vs prior implementation: use default_tip_pose_ for stable angular radius (reduces drift)
    Point3D radius = default_tip_pose_ - projection_on_z; // reference rejection (philosophically constant like OpenSHC)

    Point3D angular_velocity_vector(0, 0, desired_angular_velocity_);

    Point3D stride_vector_angular;
    stride_vector_angular.x = angular_velocity_vector.y * radius.z - angular_velocity_vector.z * radius.y;
    stride_vector_angular.y = angular_velocity_vector.z * radius.x - angular_velocity_vector.x * radius.z;
    stride_vector_angular.z = angular_velocity_vector.x * radius.y - angular_velocity_vector.y * radius.x;

    // Combination and scaling (OpenSHC formula)
    stride_vector_ = stride_vector_linear + stride_vector_angular;

    // Use StepCycle configuration values (OpenSHC equivalent calculation)
    double on_ground_ratio = double(step_cycle_.stance_period_) / double(step_cycle_.period_);
    stride_vector_ = stride_vector_ * (on_ground_ratio / step_cycle_.frequency_);

    // Apply stride validation and safety constraints
    stride_vector_ = calculateSafeStride(stride_vector_);

    // Freeze stride if not yet frozen for current phase
    if (!stride_frozen_) {
        frozen_stride_vector_linear_ = stride_vector_linear * (on_ground_ratio / step_cycle_.frequency_);
        // recompute angular component scaled the same way
        Point3D stride_vector_angular_scaled = (stride_vector_ - stride_vector_linear * (on_ground_ratio / step_cycle_.frequency_));
        frozen_stride_vector_angular_ = stride_vector_angular_scaled; // already scaled
        frozen_stride_vector_total_ = stride_vector_;
        stride_frozen_ = true;
    }

    // Swing clearance (OpenSHC formula) using validated & normalized plane normal
    swing_clearance_ = walk_plane_normal_ * step_clearance_height_;
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

    // OpenSHC: Handle FORCE_STOP state
    if (step_state_ == STEP_FORCE_STOP) {
        // Force stance position and stop iteration
        step_progress_ = 0.0;
        current_tip_pose_ = identity_tip_pose_;
        return;
    }

    // Calculate swing timing if not already done
    if (swing_delta_t_ <= 0.0) {
        calculateSwingTiming(params.time_delta);
    }

    // Update current iteration (global style)
    current_iteration_ = iteration;

    if (step_state_ == STEP_SWING) {
        // For swing phase, calculate progress based on swing iterations
        // Need to determine which iteration within the swing period we're at
        int swing_iteration = current_iteration_ % swing_iterations_;
        step_progress_ = (swing_iterations_ > 0) ? static_cast<double>(swing_iteration) / static_cast<double>(swing_iterations_) : 0.0;
        step_progress_ = std::min(1.0, step_progress_);
    } else {
        // For stance phase, calculate progress based on stance iterations
        int stance_iteration = current_iteration_ % stance_iterations_;
        step_progress_ = (stance_iterations_ > 0) ? static_cast<double>(stance_iteration) / static_cast<double>(stance_iterations_) : 0.0;
        step_progress_ = std::min(1.0, step_progress_);
    }

    // Update stride (will freeze on first iteration of the phase)
    updateStride();

    // Mid-stride target from default pose using frozen stride for stability
    Point3D active_stride = stride_frozen_ ? frozen_stride_vector_total_ : stride_vector_;
    Point3D raw_target = default_tip_pose_ + active_stride * 0.5;
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

    if (step_state_ == STEP_SWING) {
        // Initialize swing period on first iteration
        initializeSwingPeriod(iteration);

        // Generate control nodes ONLY once at the beginning of swing

        // Detect if this is a new swing cycle (reset on iteration 1 or when we restart swing)
        if (iteration == 1 || (iteration <= swing_iterations_ && last_swing_start_iteration_ > iteration)) {
            nodes_generated_ = false;
            last_swing_start_iteration_ = iteration;
        }

        if (!nodes_generated_) {
            // Generate initial swing control nodes (primary + secondary without contact)
            generatePrimarySwingControlNodes();
            generateSecondarySwingControlNodes(false);
            nodes_generated_ = true;
        }

        // Determine which half of swing we're in based on actual swing progress
        int swing_iteration = iteration % swing_iterations_;
        if (swing_iteration == 0)
            swing_iteration = swing_iterations_; // Handle modulo edge case

        int half_iterations = swing_iterations_ / 2;
        bool first_half = swing_iteration <= half_iterations;

        // Calculate absolute position using OpenSHC approach: quarticBezierDot + delta accumulation
        Point3D delta_pos;
        double time_input = 0.0;

        // Detect ground contact via FSR (if enabled) for adaptive touchdown (OpenSHC style)
        bool ground_contact = false;
        if (params_.use_fsr_contact) {
            ground_contact = leg_.isInContact();
        }

        if (first_half) {
            // OpenSHC exact calculation: swing_delta_t_ * iteration (1-based)
            time_input = swing_delta_t_ * swing_iteration;

            // OpenSHC pattern: Use quarticBezierDot for velocity-based calculation
            delta_pos = math_utils::quarticBezierDot(swing_1_nodes_, time_input) * swing_delta_t_;
        } else {
            // OpenSHC exact calculation: swing_delta_t_ * (iteration - swing_iterations / 2)
            time_input = swing_delta_t_ * (swing_iteration - swing_iterations_ / 2);

            // Regenerate secondary swing nodes continuously in second half to allow early flattening when contact detected
            generateSecondarySwingControlNodes(ground_contact);

            // OpenSHC pattern: Use quarticBezierDot for velocity-based calculation
            delta_pos = math_utils::quarticBezierDot(swing_2_nodes_, time_input) * swing_delta_t_;
        }
        // Update current tip pose based on calculated delta position
        // OpenSHC pattern: accumulate delta position to current tip pose
        Point3D next_pose = current_tip_pose_ + delta_pos;
        if (params_.enable_workspace_constrain) {
            current_tip_pose_ = robot_model_.getWorkspaceAnalyzer().constrainToGeometricWorkspace(leg_index_, next_pose);
        } else {
            current_tip_pose_ = next_pose;
        }

        // Calculate velocity for this iteration (OpenSHC pattern)
        current_tip_velocity_ = delta_pos / time_delta;

        // Improvement 2: Early swing termination when contact detected in second half of swing
        if (!first_half && params_.use_fsr_contact && ground_contact) {
            // Transition immediately to stance on next update
            step_state_ = STEP_STANCE;
            // Reset swing-related flags so a fresh stance origin is established
            swing_initialized_ = false;
            nodes_generated_ = false; // force regeneration when next swing starts
            // Force stance iteration start by resetting current_iteration_ to swing_iterations_
            current_iteration_ = swing_iterations_; // so next call maps to stance_iteration==1
        }

    } else if (step_state_ == STEP_STANCE) {

        // Stance iteration mapping: we convert the running iteration counter into a 1-based local index
        // inside [1..stance_iterations_]. A prior bug compared against swing_iterations_ (often equal),
        // freezing stance_iteration at 1 and eliminating displacement. The direct modulo mapping below
        // fixes that and is used by both modes (tangential or original).
        int stance_iteration = (iteration % std::max(1, stance_iterations_)) + 1; // 1..stance_iterations_

        // Defensive clamp (in case stance_iterations_ changes dynamically)
        if (stance_iteration > stance_iterations_) {
            stance_iteration = stance_iterations_;
        }

        // Initialize stance origin if needed (hybrid anti-drift extension).
        if (stance_iteration == 1) {
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
        }

        // Initialize tangential stance mode state at first stance iteration
        if (stance_iteration == 1) {
            stance_tangent_initialized_ = false; // reset every new stance phase
        }

        // OpenSHC stance integration
        double stride_scaler = calculateStanceStrideScaler();
        generateStanceControlNodes(stride_scaler);
        double time_input = stance_iteration * stance_delta_t_;
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input) * stance_delta_t_;
        current_tip_pose_ += delta_pos;
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
        }
    }

    // Update velocity tracking for comprehensive safety validation
    if (has_previous_position_) {
        time_step_ = time_delta;
    }
    previous_tip_pose_ = current_tip_pose_;
    has_previous_position_ = true;
}

// Update using internal phase_ (phase_ already incremented externally or will be incremented after call)
void LegStepper::updateTipPosition(double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // Derive iteration number for legacy code path from internal phase_.
    // NOTE: We reuse existing logic by calling updateTipPositionIterative with 'phase_' so we do not duplicate the long body.
    updateTipPositionIterative(phase_, time_delta, rough_terrain_mode, force_normal_touchdown);
}

void LegStepper::updateStepStateFromPhase() {
    // Determine state directly from phase_ relative to configured step_cycle_.
    // step_cycle_ may have been set from gait configuration.
    int swing_start_iter = step_cycle_.stance_period_; // stance first, then swing

    // swing_end_iter conceptually equal to step_cycle_.period_ (end exclusive) but not needed here.
    int local = phase_ % step_cycle_.period_;
    bool in_swing = (local >= swing_start_iter);
    if (step_state_ == STEP_FORCE_STOP)
        return;
    if (in_swing) {
        if (step_state_ != STEP_SWING) {
            setStepState(STEP_SWING);
            initializeSwingPeriod(1);
            beginSwingPhase();
        }
    } else {
        if (step_state_ != STEP_STANCE && step_state_ != STEP_FORCE_STANCE) {
            setStepState(STEP_STANCE);
            beginStancePhase();
        }
    }
}

// ========================================================================
// WORKSPACE VALIDATION AND SAFETY METHODS (STEP 1 IMPLEMENTATION)
// ========================================================================

bool LegStepper::validateTargetTipPose(const Point3D &target_pose) const {
    // Use WorkspaceAnalyzer to check if position is reachable
    // Get workspace bounds for validation
    auto bounds = robot_model_.getWorkspaceAnalyzer().getWorkspaceBounds(leg_index_);

    // Calculate distance from leg base
    Point3D leg_base = robot_model_.getLegBasePosition(leg_index_);
    Point3D relative_pos = target_pose - leg_base;
    double distance = std::sqrt(relative_pos.x * relative_pos.x + relative_pos.y * relative_pos.y + relative_pos.z * relative_pos.z);

    // Check basic distance constraints
    if (distance > bounds.max_reach || distance < bounds.min_reach) {
        return false;
    }

    // Use detailed workspace validation if available
    return robot_model_.getWorkspaceAnalyzer().isPositionReachable(leg_index_, target_pose);
}

Point3D LegStepper::constrainToWorkspace(const Point3D &target_pose) const {
    // Use WorkspaceAnalyzer to constrain position to valid workspace
    Point3D current_leg_positions[NUM_LEGS];

    // Get current positions of all legs for collision avoidance
    for (int i = 0; i < NUM_LEGS; i++) {
        if (i == leg_index_) {
            current_leg_positions[i] = current_tip_pose_;
        } else {
            // For other legs, we would need access to their current positions
            // For now, use a default safe position
            current_leg_positions[i] = robot_model_.getLegDefaultPosition(i);
        }
    }

    return robot_model_.getWorkspaceAnalyzer().constrainToValidWorkspace(
        leg_index_, target_pose, current_leg_positions);
}

Point3D LegStepper::calculateSafeTarget(const Point3D &desired_target) const {
    // First validate if the desired target is reachable
    if (validateTargetTipPose(desired_target)) {
        return desired_target; // Target is already safe
    }

    // Target is not reachable, constrain it to workspace
    Point3D safe_target = constrainToWorkspace(desired_target);

    // If constraining still doesn't work, use a fallback strategy
    if (!validateTargetTipPose(safe_target)) {
        // Fall back to a safe position relative to default pose
        Point3D direction = desired_target - default_tip_pose_;
        double original_distance = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

        if (original_distance > 0.0) {
            // Scale down the movement to stay within safe bounds
            auto bounds = robot_model_.getWorkspaceAnalyzer().getWorkspaceBounds(leg_index_);
            double safe_distance = bounds.max_reach * 0.8; // Use 80% of max reach for safety
            double scale_factor = std::min(1.0, safe_distance / original_distance);

            safe_target = default_tip_pose_ + direction * scale_factor;
        } else {
            safe_target = default_tip_pose_; // Ultimate fallback
        }
    }

    return safe_target;
}

// ========================================================================
// VELOCITY LIMITING AND VALIDATION METHODS (STEP 2 IMPLEMENTATION)
// ========================================================================

Point3D LegStepper::validateAndLimitVelocities(const Point3D &linear_velocity, double angular_velocity) {
    Point3D limited_velocity = linear_velocity;

    // Method 1: Use VelocityLimits if available (preferred method)
    if (velocity_limits_ != nullptr) {
        // Calculate bearing direction
        double bearing = 0.0;
        if (std::abs(linear_velocity.x) > 1e-6 || std::abs(linear_velocity.y) > 1e-6) {
            bearing = std::atan2(linear_velocity.y, linear_velocity.x) * 180.0 / M_PI;
            if (bearing < 0.0)
                bearing += 360.0;
        }

        // Get limits from VelocityLimits system
        auto limits = velocity_limits_->getLimit(bearing);

        // Apply limits
        if (std::abs(limited_velocity.x) > limits.linear_x) {
            limited_velocity.x = (limited_velocity.x > 0) ? limits.linear_x : -limits.linear_x;
        }
        if (std::abs(limited_velocity.y) > limits.linear_y) {
            limited_velocity.y = (limited_velocity.y > 0) ? limits.linear_y : -limits.linear_y;
        }

        // Limit angular velocity
        if (std::abs(angular_velocity) > limits.angular_z) {
            desired_angular_velocity_ = (angular_velocity > 0) ? limits.angular_z : -limits.angular_z;
        } else {
            desired_angular_velocity_ = angular_velocity;
        }

        return limited_velocity;
    }

    // Method 2: Fallback to WorkspaceAnalyzer constraints
    double bearing = 0.0;
    if (std::abs(linear_velocity.x) > 1e-6 || std::abs(linear_velocity.y) > 1e-6) {
        bearing = std::atan2(linear_velocity.y, linear_velocity.x) * 180.0 / M_PI;
        if (bearing < 0.0)
            bearing += 360.0;
    }

    // Get velocity constraints from WorkspaceAnalyzer
    auto constraints = robot_model_.getWorkspaceAnalyzer().calculateVelocityConstraints(
        leg_index_, bearing, step_cycle_.frequency_,
        double(step_cycle_.stance_period_) / double(step_cycle_.period_));

    // Apply constraints
    double max_linear = constraints.max_linear_velocity;

    if (std::abs(limited_velocity.x) > max_linear) {
        limited_velocity.x = (limited_velocity.x > 0) ? max_linear : -max_linear;
    }
    if (std::abs(limited_velocity.y) > max_linear) {
        limited_velocity.y = (limited_velocity.y > 0) ? max_linear : -max_linear;
    }

    // Limit angular velocity
    double max_angular = constraints.max_angular_velocity;
    if (std::abs(angular_velocity) > max_angular) {
        desired_angular_velocity_ = (angular_velocity > 0) ? max_angular : -max_angular;
    } else {
        desired_angular_velocity_ = angular_velocity;
    }

    return limited_velocity;
}

Point3D LegStepper::calculateSafeStride(const Point3D &desired_stride) const {

    // Adjust stride to respect workspace limits
    Point3D safe_stride = desired_stride;

    // Calculate potential target position from this stride
    Point3D potential_target = default_tip_pose_ + safe_stride * 0.5;

    // Check if this target would be reachable
    if (!validateTargetTipPose(potential_target)) {

        // Calculate maximum safe stride in this direction
        Point3D stride_direction = desired_stride;
        double stride_magnitude = std::sqrt(stride_direction.x * stride_direction.x +
                                            stride_direction.y * stride_direction.y +
                                            stride_direction.z * stride_direction.z);

        if (stride_magnitude > 0.0) {
            stride_direction = stride_direction / stride_magnitude; // Normalize

            // Get workspace bounds for this leg
            auto bounds = robot_model_.getWorkspaceAnalyzer().getWorkspaceBounds(leg_index_);

            // Calculate safe stride magnitude
            // Use 80% of max reach as safety margin
            double max_safe_displacement = bounds.max_reach * 0.8;
            Point3D leg_base = robot_model_.getLegBasePosition(leg_index_);
            Point3D current_offset = default_tip_pose_ - leg_base;
            double current_distance = std::sqrt(current_offset.x * current_offset.x +
                                                current_offset.y * current_offset.y +
                                                current_offset.z * current_offset.z);

            // Maximum additional displacement we can afford
            double max_additional = max_safe_displacement - current_distance;

            // Scale stride to stay within limits (multiply by 2 because target uses stride * 0.5)
            double safe_magnitude = std::min(stride_magnitude, max_additional * 2.0);
            safe_stride = stride_direction * safe_magnitude;
        }
    }

    return safe_stride;
}

// ========================================================================
// CONTROL NODE VALIDATION METHODS (STEP 4 IMPLEMENTATION)
// ========================================================================

void LegStepper::validateAndFixControlNodes(Point3D nodes[5]) const {
    // Get current leg positions for collision avoidance context
    Point3D current_leg_positions[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        current_leg_positions[i] = robot_model_.getLegDefaultPosition(i);
    }
    current_leg_positions[leg_index_] = current_tip_pose_;

    // Validate and fix each control node
    for (int i = 0; i < 5; i++) {
        if (!validateTargetTipPose(nodes[i])) {
            // Node is not reachable, constrain it to workspace
            Point3D safe_node = robot_model_.getWorkspaceAnalyzer().constrainToValidWorkspace(
                leg_index_, nodes[i], current_leg_positions);

            // If constraining still doesn't work, use a graduated fallback
            if (!validateTargetTipPose(safe_node)) {
                // Calculate direction from leg base to problematic node
                Point3D leg_base = robot_model_.getLegBasePosition(leg_index_);
                Point3D direction = nodes[i] - leg_base;
                double distance = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

                if (distance > 0.0) {
                    // Get workspace bounds and scale to safe distance
                    auto bounds = robot_model_.getWorkspaceAnalyzer().getWorkspaceBounds(leg_index_);
                    double safe_distance = bounds.max_reach * 0.75; // Use 75% of max reach
                    double scale_factor = safe_distance / distance;

                    safe_node = leg_base + direction * scale_factor;
                } else {
                    // Ultimate fallback: use default position
                    safe_node = robot_model_.getLegDefaultPosition(leg_index_);
                }
            }

            nodes[i] = safe_node;
        }
    }
}

// Comprehensive safety validation (combines all 4 steps)
bool LegStepper::validateCurrentTrajectory() const {
    // Step 1: Validate target tip pose is within workspace
    if (!robot_model_.getWorkspaceAnalyzer().isPositionReachable(leg_index_, target_tip_pose_)) {
        return false;
    }

    // Step 2: Check velocity constraints if available
    if (velocity_limits_) {
        Point3D current_velocity = calculateCurrentTipVelocity();
        if (!velocity_limits_->validateVelocityInputs(current_velocity.x, current_velocity.y, 0.0)) {
            return false;
        }
    }

    // Step 3: Validate current control nodes are within workspace
    // Check all 5 control nodes for each trajectory type
    for (size_t i = 0; i < 5; ++i) {
        if (!robot_model_.getWorkspaceAnalyzer().isPositionReachable(leg_index_, swing_1_nodes_[i])) {
            return false;
        }
        if (!robot_model_.getWorkspaceAnalyzer().isPositionReachable(leg_index_, swing_2_nodes_[i])) {
            return false;
        }
        if (!robot_model_.getWorkspaceAnalyzer().isPositionReachable(leg_index_, stance_nodes_[i])) {
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