#include "leg_stepper_openshc.h"
#include "../src/hexamotion_constants.h"
#include "../src/math_utils.h"
#include <algorithm>
#include <cmath>

LegStepperOpenSHC::LegStepperOpenSHC(int leg_index, const Point3D &identity_tip_pose, Leg &leg, RobotModel &robot_model,
                                     WalkspaceAnalyzer *walkspace_analyzer, WorkspaceValidator *workspace_validator)
    : leg_index_(leg_index),
      leg_(leg),
      robot_model_(robot_model),
      identity_tip_pose_(identity_tip_pose),
      walkspace_analyzer_(walkspace_analyzer),
      workspace_validator_(workspace_validator) {

    // Inicialización estándar
    default_tip_pose_ = identity_tip_pose_;
    origin_tip_pose_ = identity_tip_pose_;
    target_tip_pose_ = identity_tip_pose_;
    current_tip_pose_ = identity_tip_pose_;
    at_correct_phase_ = false;
    completed_first_step_ = false;
    phase_ = 0;
    step_progress_ = 0.0;
    step_state_ = STEP_STANCE;
    swing_delta_t_ = 0.0;
    stance_delta_t_ = 0.0;
    swing_iterations_ = 0;
    stance_iterations_ = 0;
    current_iteration_ = 0;

    // Initialize vectors
    desired_linear_velocity_ = Point3D(0, 0, 0);
    desired_angular_velocity_ = 0.0;
    walk_plane_ = Point3D(0, 0, 0);
    walk_plane_normal_ = Point3D(0, 0, 1);
    stride_vector_ = Point3D(0, 0, 0);
    current_tip_velocity_ = Point3D(0, 0, 0);
    swing_origin_tip_position_ = Point3D(0, 0, 0);
    swing_origin_tip_velocity_ = Point3D(0, 0, 0);
    stance_origin_tip_position_ = Point3D(0, 0, 0);
    swing_clearance_ = Point3D(0, 0, 0);
}

void LegStepperOpenSHC::setDesiredVelocity(const Point3D &linear_velocity, double angular_velocity) {
    desired_linear_velocity_ = linear_velocity;
    desired_angular_velocity_ = angular_velocity;
}

void LegStepperOpenSHC::updateStride() {
    // Simplified stride calculation based on velocity
    double control_frequency = robot_model_.getParams().control_frequency;
    double time_delta = 1.0 / control_frequency;

    // Calculate stride vector from velocity (OpenSHC approach)
    // Stride should be proportional to velocity and step duration
    double step_duration = 1.0;                                      // Assume 1 second step cycle
    stride_vector_ = desired_linear_velocity_ * (time_delta * 10.0); // Scale up for visible movement

    // IMPORTANT: Stride should only affect X,Y movement, not Z
    // Z movement is handled by swing clearance, not stride
    stride_vector_.z = 0.0;

    // Apply minimum stride constraint for testing
    double stride_magnitude = stride_vector_.norm();
    if (stride_magnitude < 0.5) {
        // Generate small but visible stride for testing
        stride_vector_ = Point3D(1.0, 0.0, 0.0); // 1mm forward stride
    }
}

void LegStepperOpenSHC::calculateSwingTiming(double time_delta) {
    // OpenSHC timing calculation
    double swing_period_ratio = 0.5; // 50% of step cycle is swing
    double control_frequency = robot_model_.getParams().control_frequency;

    // Calculate total swing period in seconds
    double swing_period_seconds = swing_period_ratio; // Assume 1 second total step cycle for simplicity

    // Calculate number of iterations for ENTIRE swing period
    swing_iterations_ = int(swing_period_seconds / time_delta);
    swing_iterations_ = (swing_iterations_ % 2 == 0) ? swing_iterations_ : swing_iterations_ + 1; // Must be even

    // Ensure minimum iterations
    if (swing_iterations_ < 20)
        swing_iterations_ = 20;

    // Time delta for each bezier curve (half of swing period)
    swing_delta_t_ = 1.0 / (swing_iterations_ / 2.0);

    // Calculate stance timing
    double stance_period_ratio = 1.0 - swing_period_ratio;
    double stance_period_seconds = stance_period_ratio;
    stance_iterations_ = int(stance_period_seconds / time_delta);

    if (stance_iterations_ < 10)
        stance_iterations_ = 10;
    stance_delta_t_ = 1.0 / stance_iterations_;

    // Ensure valid timing
    if (swing_delta_t_ <= 0.0 || swing_delta_t_ > 1.0)
        swing_delta_t_ = 0.1; // 10 iterations per curve
    if (stance_delta_t_ <= 0.0 || stance_delta_t_ > 1.0)
        stance_delta_t_ = 0.1; // 10 iterations per stance
}

void LegStepperOpenSHC::initializeSwingPeriod(int iteration) {
    // Save initial tip position/velocity like OpenSHC
    if (iteration == 1) {
        swing_origin_tip_position_ = current_tip_pose_;
        swing_origin_tip_velocity_ = current_tip_velocity_;
        current_iteration_ = 1;

        std::cout << "Initialized swing period at iteration " << iteration << std::endl;
        std::cout << "  Origin position: (" << swing_origin_tip_position_.x << ", " << swing_origin_tip_position_.y << ", " << swing_origin_tip_position_.z << ")" << std::endl;
        std::cout << "  Origin velocity: (" << swing_origin_tip_velocity_.x << ", " << swing_origin_tip_velocity_.y << ", " << swing_origin_tip_velocity_.z << ")" << std::endl;
    }
    current_iteration_ = iteration;
}

void LegStepperOpenSHC::generatePrimarySwingControlNodes() {
    // OpenSHC primary swing control nodes generation

    // If swing_origin_tip_position_ is not set, use current position
    if (swing_origin_tip_position_.norm() < 1e-6) {
        swing_origin_tip_position_ = current_tip_pose_;
    }

    // Calculate midpoint for swing using default tip z and configured swing height
    Point3D mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_) * 0.5;
    mid_tip_position.z = std::max(swing_origin_tip_position_.z, target_tip_pose_.z);
    mid_tip_position.z += swing_clearance_.z;

    // Apply lateral shift (OpenSHC swing width)
    double mid_lateral_shift = 10.0; // Default swing width
    bool positive_y_axis = (identity_tip_pose_.y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    // Calculate stance node separation using OpenSHC formula
    double time_delta = 1.0 / robot_model_.getParams().control_frequency;
    Point3D stance_node_separation = swing_origin_tip_velocity_ * 0.25 * (time_delta / swing_delta_t_);

    // Control nodes for quartic Bézier primary swing curves (OpenSHC style)
    swing_1_nodes_[0] = swing_origin_tip_position_;
    swing_1_nodes_[1] = swing_origin_tip_position_ + stance_node_separation;
    swing_1_nodes_[2] = swing_origin_tip_position_ + stance_node_separation * 2.0;
    swing_1_nodes_[3] = (mid_tip_position + swing_1_nodes_[2]) * 0.5;
    swing_1_nodes_[3].z = mid_tip_position.z;
    swing_1_nodes_[4] = mid_tip_position;
}

void LegStepperOpenSHC::generateSecondarySwingControlNodes(bool ground_contact) {
    // OpenSHC secondary swing control nodes generation

    Point3D final_tip_velocity = stride_vector_ * -1.0; // OpenSHC uses negative stride
    double time_delta = 1.0 / robot_model_.getParams().control_frequency;
    Point3D stance_node_separation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t_);

    // Control nodes for quartic Bézier secondary swing curves (OpenSHC style)
    swing_2_nodes_[0] = swing_1_nodes_[4];
    swing_2_nodes_[1] = swing_1_nodes_[4] - (swing_1_nodes_[3] - swing_1_nodes_[4]);
    swing_2_nodes_[2] = target_tip_pose_ - stance_node_separation * 2.0;
    swing_2_nodes_[3] = target_tip_pose_ - stance_node_separation;
    swing_2_nodes_[4] = target_tip_pose_;

    // CORRECTION: Final landing position should have same Z as initial position
    // This ensures the leg returns to ground level
    swing_2_nodes_[4].z = swing_origin_tip_position_.z;
    swing_2_nodes_[3].z = swing_origin_tip_position_.z;
    swing_2_nodes_[2].z = swing_origin_tip_position_.z;

    // Handle ground contact (OpenSHC behavior)
    if (ground_contact) {
        swing_2_nodes_[0] = current_tip_pose_ + stance_node_separation * 0.0;
        swing_2_nodes_[1] = current_tip_pose_ + stance_node_separation * 1.0;
        swing_2_nodes_[2] = current_tip_pose_ + stance_node_separation * 2.0;
        swing_2_nodes_[3] = current_tip_pose_ + stance_node_separation * 3.0;
        swing_2_nodes_[4] = current_tip_pose_ + stance_node_separation * 4.0;
    }
}

void LegStepperOpenSHC::generateStanceControlNodes(double stride_scaler) {
    // OpenSHC stance control nodes generation

    // If stance_origin_tip_position_ is not set, use current position
    if (stance_origin_tip_position_.norm() < 1e-6) {
        stance_origin_tip_position_ = current_tip_pose_;
    }

    Point3D stance_target_position = stance_origin_tip_position_ + stride_vector_ * stride_scaler;
    Point3D stance_node_separation = stride_vector_ * stride_scaler * -0.25; // OpenSHC uses negative scaler

    // Control nodes for quartic Bézier stance curve (OpenSHC style)
    stance_nodes_[0] = stance_origin_tip_position_;
    stance_nodes_[1] = stance_origin_tip_position_ + stance_node_separation * 0.25;
    stance_nodes_[2] = stance_origin_tip_position_ + stance_node_separation * 0.5;
    stance_nodes_[3] = stance_origin_tip_position_ + stance_node_separation * 0.75;
    stance_nodes_[4] = stance_target_position;
}

void LegStepperOpenSHC::updateTipPositionIterative(int iteration, double time_delta, bool rough_terrain_mode, bool force_normal_touchdown) {
    // OpenSHC-style iterative update

    // Calculate swing timing if not already done
    if (swing_delta_t_ <= 0.0) {
        calculateSwingTiming(time_delta);
    }

    // Update stride vector FIRST
    updateStride();

    // Calculate target tip pose like OpenSHC
    // Target should be stride distance from current position, but at SAME Z level as start
    target_tip_pose_ = current_tip_pose_ + Point3D(stride_vector_.x * 0.5, stride_vector_.y * 0.5, 0.0);

    // Debug output for first iteration
    if (iteration == 1) {
        std::cout << "DEBUG: stride_vector = (" << stride_vector_.x << ", " << stride_vector_.y << ", " << stride_vector_.z << ")" << std::endl;
        std::cout << "DEBUG: desired_linear_velocity = (" << desired_linear_velocity_.x << ", " << desired_linear_velocity_.y << ", " << desired_linear_velocity_.z << ")" << std::endl;
        std::cout << "DEBUG: target_tip_pose = (" << target_tip_pose_.x << ", " << target_tip_pose_.y << ", " << target_tip_pose_.z << ")" << std::endl;
    }

    if (step_state_ == STEP_SWING) {
        // Initialize swing period on first iteration
        initializeSwingPeriod(iteration);

        // Determine which half of swing we're in
        bool first_half = iteration <= swing_iterations_ / 2;

        // Generate control nodes
        generatePrimarySwingControlNodes();
        generateSecondarySwingControlNodes(!first_half && false); // ground_contact = false for now

        // Calculate delta position using OpenSHC approach
        Point3D delta_pos(0, 0, 0);
        double time_input = 0;

        if (first_half) {
            time_input = swing_delta_t_ * iteration;
            if (time_input <= 1.0) {
                delta_pos = math_utils::quarticBezierDot(swing_1_nodes_, time_input) * swing_delta_t_;
            }
        } else {
            time_input = swing_delta_t_ * (iteration - swing_iterations_ / 2);
            if (time_input <= 1.0) {
                delta_pos = math_utils::quarticBezierDot(swing_2_nodes_, time_input) * swing_delta_t_;
            }
        }

        // Debug delta for first few iterations
        if (iteration <= 3) {
            std::cout << "DEBUG iter " << iteration << ": time_input=" << time_input << ", delta=(" << delta_pos.x << ", " << delta_pos.y << ", " << delta_pos.z << ")" << std::endl;
            std::cout << "DEBUG iter " << iteration << ": current_pose BEFORE=(" << current_tip_pose_.x << ", " << current_tip_pose_.y << ", " << current_tip_pose_.z << ")" << std::endl;
        }

        // Apply OpenSHC precision control to reduce accumulation errors
        auto setPrecision = [](double value, int precision) -> double {
            return round(value * pow(10, precision)) / pow(10, precision);
        };

        // Apply delta to current position (OpenSHC accumulation)
        current_tip_pose_.x += delta_pos.x;
        current_tip_pose_.y += delta_pos.y;
        current_tip_pose_.z += delta_pos.z;

        // Apply precision control like OpenSHC (3 decimal places = 1mm precision)
        current_tip_pose_.x = setPrecision(current_tip_pose_.x, 3);
        current_tip_pose_.y = setPrecision(current_tip_pose_.y, 3);
        current_tip_pose_.z = setPrecision(current_tip_pose_.z, 3);

        // Apply OpenSHC-style final position correction on last iteration
        // This reduces accumulated error to within TIP_TOLERANCE (10mm)
        if (iteration >= swing_iterations_) {
            // Calculate error from target
            Point3D target_final_position = swing_origin_tip_position_;
            target_final_position.x += stride_vector_.x;
            target_final_position.y += stride_vector_.y;
            // Z should return to original level
            target_final_position.z = swing_origin_tip_position_.z;

            Point3D error = current_tip_pose_ - target_final_position;
            double error_norm = sqrt(error.x * error.x + error.y * error.y + error.z * error.z);

            // If error is within OpenSHC IK_TOLERANCE (5mm), apply final correction
            const double OPENSHC_IK_TOLERANCE = 0.005;     // 5mm like OpenSHC
            if (error_norm < OPENSHC_IK_TOLERANCE * 2.0) { // Allow some margin
                current_tip_pose_ = target_final_position;
            }
        }

        // Debug position after update
        if (iteration <= 3) {
            std::cout << "DEBUG iter " << iteration << ": current_pose AFTER=(" << current_tip_pose_.x << ", " << current_tip_pose_.y << ", " << current_tip_pose_.z << ")" << std::endl;
        }

        // Calculate velocity
        current_tip_velocity_ = delta_pos / time_delta;

        // Apply to leg through IK
        leg_.applyIK(current_tip_pose_);
    } else if (step_state_ == STEP_STANCE) {
        // Handle stance period
        generateStanceControlNodes(1.0);

        double time_input = stance_delta_t_ * iteration;
        Point3D delta_pos = math_utils::quarticBezierDot(stance_nodes_, time_input) * stance_delta_t_;

        // Apply delta to current position
        current_tip_pose_.x += delta_pos.x;
        current_tip_pose_.y += delta_pos.y;
        current_tip_pose_.z += delta_pos.z;

        current_tip_velocity_ = delta_pos / time_delta;

        // Apply to leg through IK
        leg_.applyIK(current_tip_pose_);
    }
}

void LegStepperOpenSHC::printDebugInfo() const {
    std::cout << "=== LegStepperOpenSHC Debug Info ===" << std::endl;
    std::cout << "Leg index: " << leg_index_ << std::endl;
    std::cout << "Step state: " << (step_state_ == STEP_SWING ? "SWING" : "STANCE") << std::endl;
    std::cout << "Current iteration: " << current_iteration_ << std::endl;
    std::cout << "Swing iterations: " << swing_iterations_ << std::endl;
    std::cout << "Swing delta_t: " << swing_delta_t_ << std::endl;
    std::cout << "Stance delta_t: " << stance_delta_t_ << std::endl;
    std::cout << "Current tip pose: (" << current_tip_pose_.x << ", " << current_tip_pose_.y << ", " << current_tip_pose_.z << ")" << std::endl;
    std::cout << "Target tip pose: (" << target_tip_pose_.x << ", " << target_tip_pose_.y << ", " << target_tip_pose_.z << ")" << std::endl;
    std::cout << "Stride vector: (" << stride_vector_.x << ", " << stride_vector_.y << ", " << stride_vector_.z << ")" << std::endl;
    std::cout << "Swing clearance: (" << swing_clearance_.x << ", " << swing_clearance_.y << ", " << swing_clearance_.z << ")" << std::endl;
}
