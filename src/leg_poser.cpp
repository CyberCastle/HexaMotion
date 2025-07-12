#include "leg_poser.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

/**
 * @file leg_poser.cpp
 * @brief Implementation of LegPoser for HexaMotion
 *
 * This implementation is adapted from OpenSHC's LegPoser class to work with HexaMotion's architecture.
 * It provides smooth transitions for leg positioning using Bezier curves.
 */

LegPoser::LegPoser(int leg_index, Leg &leg, RobotModel &robot_model)
    : leg_index_(leg_index), leg_(leg), robot_model_(robot_model), first_iteration_(true), master_iteration_count_(0) {

    // Initialize poses
    auto_pose_ = Pose();
    origin_tip_pose_ = Pose();
    current_tip_pose_ = Pose();
    target_tip_pose_ = Pose();
    external_target_ = ExternalTarget();

    // Initialize flags
    leg_completed_step_ = false;

    // Initialize Bezier control nodes (5 nodes for quartic curves)
    for (int i = 0; i < 5; i++) {
        control_nodes_primary_[i] = Point3D(0, 0, 0);
        control_nodes_secondary_[i] = Point3D(0, 0, 0);
    }
}

LegPoser::LegPoser(const LegPoser *leg_poser)
    : leg_index_(leg_poser->leg_index_), leg_(leg_poser->leg_), robot_model_(leg_poser->robot_model_), auto_pose_(leg_poser->auto_pose_), first_iteration_(leg_poser->first_iteration_), master_iteration_count_(leg_poser->master_iteration_count_), origin_tip_pose_(leg_poser->origin_tip_pose_), current_tip_pose_(leg_poser->current_tip_pose_), target_tip_pose_(leg_poser->target_tip_pose_), external_target_(leg_poser->external_target_), leg_completed_step_(leg_poser->leg_completed_step_) {

    // Copy Bezier control nodes
    for (int i = 0; i < 5; i++) {
        control_nodes_primary_[i] = leg_poser->control_nodes_primary_[i];
        control_nodes_secondary_[i] = leg_poser->control_nodes_secondary_[i];
    }
}

bool LegPoser::stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
                              double lift_height, double time_to_step, bool apply_delta) {
    // Store origin pose on first iteration
    if (first_iteration_) {
        origin_tip_pose_ = getCurrentTipPose();
        target_tip_pose_ = target_tip_pose;
        first_iteration_ = false;
        master_iteration_count_ = 0;
    }

    // Calculate number of iterations based on time_to_step and time_delta
    // Get time_delta from robot model parameters (OpenSHC style)
    double time_delta = 1.0 / robot_model_.getParams().control_frequency;
    int num_iterations = std::max(1, static_cast<int>(std::round(time_to_step / time_delta)));
    double delta_t = 1.0 / num_iterations;

    master_iteration_count_++;
    double completion_ratio = static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(num_iterations);

    // Check if transition is needed
    Point3D origin_pos = origin_tip_pose_.position;
    Point3D target_pos = target_tip_pose.position;
    if (apply_delta) {
        target_pos.x += target_pose.position.x;
        target_pos.y += target_pose.position.y;
        target_pos.z += target_pose.position.z;
    }

    // Calculate position delta
    Point3D position_delta = origin_pos - target_pos;
    double distance = std::sqrt(position_delta.x * position_delta.x +
                                position_delta.y * position_delta.y +
                                position_delta.z * position_delta.z);

    // If no significant movement needed and no lift height, complete immediately
    if (distance < TIP_TOLERANCE && lift_height == 0.0) {
        first_iteration_ = true;
        current_tip_pose_ = origin_tip_pose_;
        leg_completed_step_ = true;
        return true;
    }

    // Generate Bezier control nodes on first iteration
    if (master_iteration_count_ == 1) {
        generatePrimarySwingControlNodes(origin_pos, target_pos, lift_height);
        generateSecondarySwingControlNodes(origin_pos, target_pos, lift_height);
    }

    // Calculate time input using delta_t (OpenSHC style)
    double time_input = master_iteration_count_ * delta_t;

    // Calculate swing iteration count and determine which bezier curve to use
    int half_swing_iteration = num_iterations / 2;
    int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % num_iterations + 1;

    Point3D new_tip_position;

    // Use dual quartic bezier curves for swing trajectory (OpenSHC style)
    if (swing_iteration_count <= half_swing_iteration) {
        // First half of swing - use primary bezier curve
        time_input = swing_iteration_count * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_primary_, time_input);
    } else {
        // Second half of swing - use secondary bezier curve
        time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_secondary_, time_input);
    }

    // Update current tip pose
    current_tip_pose_ = Pose(new_tip_position, Eigen::Vector3d(0, 0, 0));
    leg_.setCurrentTipPositionGlobal(robot_model_, current_tip_pose_.position);

    // Check if step is complete
    if (master_iteration_count_ >= num_iterations) {
        first_iteration_ = true;
        leg_completed_step_ = true;
        return true;
    } else {
        return false;
    }
}

void LegPoser::updateAutoPose(int phase) {
    // Enhanced OpenSHC-style auto pose implementation
    // This method updates the leg-specific auto pose based on the gait phase
    // with improved compensation for body balance and stability

    // Get default stance position for this leg
    Point3D default_stance_pos = leg_.getCurrentTipPositionGlobal();

    // Calculate phase-based compensation (OpenSHC style)
    // Phase is typically 0-100, representing the gait cycle progress
    double phase_ratio = static_cast<double>(phase) / 100.0;

    // Get robot parameters for compensation calculations
    const auto &params = robot_model_.getParams();
    double body_clearance = params.robot_height;
    double hexagon_radius = params.hexagon_radius;

    // Calculate auto pose compensation based on phase
    // This simulates the body pose adjustments during gait execution
    Point3D compensated_position = default_stance_pos;

    // Enhanced phase-based Z compensation (body height variation)
    // Use sinusoidal compensation for smooth transitions
    double z_compensation = 0.0;
    if (phase_ratio < 0.5) {
        // First half of gait cycle - gradual lift for stability
        z_compensation = body_clearance * 0.015 * sin(phase_ratio * 2.0 * M_PI);
    } else {
        // Second half of gait cycle - gradual drop for natural movement
        z_compensation = -body_clearance * 0.015 * sin((phase_ratio - 0.5) * 2.0 * M_PI);
    }

    // Enhanced phase-based XY compensation (body roll/pitch compensation)
    // Consider leg position relative to body center for more accurate compensation
    double x_compensation = 0.0;
    double y_compensation = 0.0;

    // Calculate leg position relative to body center
    Point3D leg_base_pos = robot_model_.getLegBasePosition(leg_index_);
    double leg_angle = robot_model_.getLegBaseAngleOffset(leg_index_);

    // Roll compensation based on leg position (Y-axis variation)
    // Legs on opposite sides of Y-axis get opposite compensation
    double roll_factor = sin(math_utils::degreesToRadians(leg_angle));
    y_compensation = body_clearance * 0.008 * roll_factor * sin(phase_ratio * 4.0 * M_PI);

    // Pitch compensation based on leg position (X-axis variation)
    // Legs on opposite sides of X-axis get opposite compensation
    double pitch_factor = cos(math_utils::degreesToRadians(leg_angle));
    x_compensation = body_clearance * 0.008 * pitch_factor * cos(phase_ratio * 4.0 * M_PI);

    // Additional stability compensation for tripod gait
    // This helps maintain center of mass during leg transitions
    double stability_compensation = 0.0;
    if (phase_ratio > 0.4 && phase_ratio < 0.6) {
        // During critical transition phase, add extra stability
        stability_compensation = body_clearance * 0.005 * cos((phase_ratio - 0.5) * 10.0 * M_PI);
    }

    // Apply all compensations
    compensated_position.x += x_compensation;
    compensated_position.y += y_compensation;
    compensated_position.z += z_compensation + stability_compensation;

    // Create auto pose with compensated position and identity rotation
    auto_pose_ = Pose(compensated_position, Eigen::Vector3d(0, 0, 0));

    // Apply auto pose to current tip position if compensation is significant
    double compensation_magnitude = std::sqrt(
        (compensated_position.x - default_stance_pos.x) * (compensated_position.x - default_stance_pos.x) +
        (compensated_position.y - default_stance_pos.y) * (compensated_position.y - default_stance_pos.y) +
        (compensated_position.z - default_stance_pos.z) * (compensated_position.z - default_stance_pos.z));

    // Only apply compensation if it's above a minimum threshold to avoid jitter
    if (compensation_magnitude > 0.5) { // 0.5mm threshold
        leg_.setCurrentTipPositionGlobal(robot_model_, auto_pose_.position);

        // Recalculate joint angles for the new position using current angles as starting point
        JointAngles current_angles = leg_.getJointAngles();
        JointAngles new_angles = robot_model_.inverseKinematicsCurrentGlobalCoordinates(
            leg_index_, current_angles, auto_pose_.position);

        // Apply joint limits to ensure safety
        new_angles.coxa = robot_model_.constrainAngle(new_angles.coxa,
                                                      params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        new_angles.femur = robot_model_.constrainAngle(new_angles.femur,
                                                       params.femur_angle_limits[0], params.femur_angle_limits[1]);
        new_angles.tibia = robot_model_.constrainAngle(new_angles.tibia,
                                                       params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

        leg_.setJointAngles(new_angles);
    }
}

void LegPoser::generatePrimarySwingControlNodes(const Point3D &origin_pos, const Point3D &target_pos, double lift_height) {
    // Calculate midpoint for the swing trajectory
    Point3D mid_pos = (origin_pos + target_pos) * 0.5;
    mid_pos.z = origin_pos.z + lift_height; // Apply lift height at midpoint

    // Calculate separation vector for control nodes
    Point3D origin_to_target = target_pos - origin_pos;
    Point3D node_separation = origin_to_target * 0.25; // 25% of total distance

    // Control nodes for primary swing quartic bezier curve (OpenSHC style)
    control_nodes_primary_[0] = origin_pos;
    control_nodes_primary_[1] = origin_pos + node_separation;
    control_nodes_primary_[2] = origin_pos + node_separation * 2.0;
    control_nodes_primary_[3] = mid_pos - node_separation * 0.5;
    control_nodes_primary_[4] = mid_pos;
}

void LegPoser::generateSecondarySwingControlNodes(const Point3D &origin_pos, const Point3D &target_pos, double lift_height) {
    // Calculate midpoint for the swing trajectory
    Point3D mid_pos = (origin_pos + target_pos) * 0.5;
    mid_pos.z = origin_pos.z + lift_height; // Apply lift height at midpoint

    // Calculate separation vector for control nodes
    Point3D origin_to_target = target_pos - origin_pos;
    Point3D node_separation = origin_to_target * 0.25; // 25% of total distance

    // Control nodes for secondary swing quartic bezier curve (OpenSHC style)
    control_nodes_secondary_[0] = mid_pos;
    control_nodes_secondary_[1] = mid_pos + node_separation * 0.5;
    control_nodes_secondary_[2] = target_pos - node_separation * 2.0;
    control_nodes_secondary_[3] = target_pos - node_separation;
    control_nodes_secondary_[4] = target_pos;
}