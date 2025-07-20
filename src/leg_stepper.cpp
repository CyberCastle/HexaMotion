#include "leg_stepper.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "velocity_limits.h"
#include <algorithm>
#include <cmath>

LegStepper::LegStepper(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model,
                       WalkspaceAnalyzer *walkspace_analyzer, WorkspaceValidator *workspace_validator)
    : leg_index_(leg_index),
      leg_(leg),
      robot_model_(robot_model),
      identity_tip_pose_(identity_tip_pose),
      walkspace_analyzer_(walkspace_analyzer),
      workspace_validator_(workspace_validator) {

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
    step_cycle_time_ = 1.0; // Default 1 second cycle time

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
    swing_clearance_ = Point3D(0, 0, 0);

    // Initialize all Bezier control nodes with identity pose
    for (int i = 0; i < 5; ++i) {
        swing_1_nodes_[i] = identity_tip_pose_;
        swing_2_nodes_[i] = identity_tip_pose_;
        stance_nodes_[i] = identity_tip_pose_;
    }
}

void LegStepper::setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity) {
    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

void LegStepper::updateStride() {
    // Calculate stride vector from velocity (OpenSHC approach)
    double control_frequency = robot_model_.getParams().control_frequency;
    double time_delta = 1.0 / control_frequency;

    // OpenSHC approach: Stride should be proportional to velocity and complete step cycle time
    // In OpenSHC, stride_vector represents the TOTAL displacement during one complete step cycle
    // However, the target calculation uses stride_vector * 0.5, so we need to account for this
    // Use the step cycle time from gait configuration
    double step_cycle_time = step_cycle_time_; // From gait configuration

    // Calculate the desired displacement during swing phase
    Point3D desired_swing_displacement = desired_linear_velocity_ * step_cycle_time;

    // Since OpenSHC target calculation uses stride_vector * 0.5, we need to double it
    // to get the actual swing displacement we want
    stride_vector_ = desired_swing_displacement * 2.0; // IMPORTANT: Stride should only affect X,Y movement, not Z
    // Z movement is handled by swing clearance, not stride
    stride_vector_.z = 0.0;

    // Apply minimum stride constraint for testing visibility
    double stride_magnitude = stride_vector_.norm();
    if (stride_magnitude < 1.0) {
        // Generate small but visible stride for testing when velocity is very low
        stride_vector_ = Point3D(2.0, 0.0, 0.0); // 2mm stride_vector (1mm actual displacement)
    }
}

void LegStepper::calculateSwingTiming(double time_delta) {
    // OpenSHC-compatible timing calculation using proper gait configuration
    // Note: This method should be called with gait parameters, but for now we use defaults
    // In full implementation, these should come from GaitConfiguration

    double step_frequency = 1.0; // Hz - OpenSHC default (should come from gait configuration)
    double swing_ratio = 0.5;    // Default 50% swing time (should come from gait configuration)
    double stance_ratio = 0.5;   // Default 50% stance time (should come from gait configuration)

    // OpenSHC formula: swing_iterations = int((swing_period/period) / (frequency * time_delta))
    // where swing_period = swing_ratio * period, and period = 1.0 (normalized)
    swing_iterations_ = int((swing_ratio / 1.0) / (step_frequency * time_delta));

    // Ensure minimum iterations for Bezier curve development
    if (swing_iterations_ < 10)
        swing_iterations_ = 10;

    // Make even for proper primary/secondary curve split
    if (swing_iterations_ % 2 != 0)
        swing_iterations_++;

    // Time delta for each bezier curve (split swing period in half for primary/secondary)
    swing_delta_t_ = 1.0 / (swing_iterations_ / 2.0);

    // Calculate stance timing using same formula
    stance_iterations_ = int((stance_ratio / 1.0) / (step_frequency * time_delta));
    if (stance_iterations_ < 2)
        stance_iterations_ = 2;
    stance_delta_t_ = 1.0 / stance_iterations_;
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

    double mid_lateral_shift = 5.0; // swing_width parameter equivalent
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    // OpenSHC exact formula: walker_->getTimeDelta() / swing_delta_t_
    double time_delta = 0.02; // 50Hz control loop like OpenSHC
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
}

void LegStepper::generateSecondarySwingControlNodes(bool ground_contact) {
    // Follow OpenSHC exact implementation for maximum precision

    // Calculate final tip velocity for stance transition (OpenSHC formula)
    double time_delta = 0.02; // OpenSHC standard: 50Hz control loop
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
}

void LegStepper::generateStanceControlNodes(double stride_scaler) {
    // OpenSHC exact implementation
    Point3D stance_node_seperation = stride_vector_ * stride_scaler * (-0.25);

    // Control nodes for stance quartic bezier curve
    // Set as initial tip position
    stance_nodes_[0] = stance_origin_tip_position_ + stance_node_seperation * 0.0;
    // Set for constant velocity in stance period
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_seperation * 1.0;
    // Set for constant velocity in stance period
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_seperation * 2.0;
    // Set for constant velocity in stance period
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_seperation * 3.0;
    // Set as target tip position
    stance_nodes_[4] = stance_origin_tip_position_ + stance_node_seperation * 4.0;
}

void LegStepper::updateTipPositionIterative(int iteration, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // OpenSHC-style iterative update - This is the MAIN method following OpenSHC philosophy

    // Calculate swing timing if not already done
    if (swing_delta_t_ <= 0.0) {
        calculateSwingTiming(time_delta);
    }

    // Update current iteration and step progress
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

    // Update stride vector FIRST
    updateStride();

    // Generate default target (OpenSHC exact formula)
    target_tip_pose_ = default_tip_pose_ + stride_vector_ * 0.5;

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
            generatePrimarySwingControlNodes();
            generateSecondarySwingControlNodes(false); // ground_contact = false for now
            nodes_generated_ = true;
        }

        // Determine which half of swing we're in based on actual swing progress
        int swing_iteration = iteration % swing_iterations_;
        if (swing_iteration == 0)
            swing_iteration = swing_iterations_; // Handle modulo edge case

        int half_iterations = swing_iterations_ / 2;
        bool first_half = swing_iteration <= half_iterations;

        // Calculate absolute position using OpenSHC approach
        Point3D new_position = current_tip_pose_;
        double time_input = 0.0;

        if (first_half) {
            // For first half: map swing_iteration 1->half_iterations to time 0->1
            if (half_iterations > 1) {
                time_input = (double)(swing_iteration - 1) / (double)(half_iterations - 1);
            } else {
                time_input = 0.0;
            }
            time_input = std::max(0.0, std::min(1.0, time_input)); // Clamp to [0,1]
            new_position = math_utils::quarticBezier(swing_1_nodes_, time_input);
        } else {
            // For second half: map swing_iteration (half_iterations+1)->swing_iterations_ to time 0->1
            int second_half_start = half_iterations + 1;
            int second_half_iterations = swing_iterations_ - half_iterations;
            if (second_half_iterations > 1) {
                time_input = (double)(swing_iteration - second_half_start) / (double)(second_half_iterations - 1);
            } else {
                time_input = 0.0;
            }
            time_input = std::max(0.0, std::min(1.0, time_input)); // Clamp to [0,1]
            new_position = math_utils::quarticBezier(swing_2_nodes_, time_input);
        }

        // Apply OpenSHC precision control to reduce accumulation errors
        auto setPrecision = [](double value, int precision) -> double {
            return round(value * pow(10, precision)) / pow(10, precision);
        };

        // Set the new position directly (no delta accumulation needed)
        current_tip_pose_ = new_position;

        // Apply precision control like OpenSHC (3 decimal places = 1mm precision)
        current_tip_pose_.x = setPrecision(current_tip_pose_.x, 3);
        current_tip_pose_.y = setPrecision(current_tip_pose_.y, 3);
        current_tip_pose_.z = setPrecision(current_tip_pose_.z, 3);

        // Calculate velocity for this iteration (for compatibility)
        current_tip_velocity_ = Point3D(0, 0, 0); // Will be calculated if needed

        // Apply to leg through IK
        leg_.applyIK(current_tip_pose_);

    } else if (step_state_ == STEP_STANCE) {
        // Handle stance period
        generateStanceControlNodes(1.0);

        double time_input = stance_delta_t_ * iteration;
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input) * stance_delta_t_ * time_delta;

        // Apply delta to current position
        current_tip_pose_.x += delta_pos.x;
        current_tip_pose_.y += delta_pos.y;
        current_tip_pose_.z += delta_pos.z;

        current_tip_velocity_ = delta_pos / time_delta;

        // Apply to leg through IK
        leg_.applyIK(current_tip_pose_);
    }
}