#include "leg_poser.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "workspace_analyzer.h"
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

    // Initialize physical reference height (z = getDefaultHeightOffset() when all angles are 0°)
    physical_reference_height_ = robot_model_.getDefaultHeightOffset();

    // Initialize poses
    auto_pose_ = Pose();
    origin_tip_pose_ = Pose();
    current_tip_pose_ = Pose();
    target_tip_pose_ = Pose();
    external_target_ = ExternalTarget();

    // Initialize flags
    leg_completed_step_ = false;
}

LegPoser::LegPoser(const LegPoser *leg_poser)
    : leg_index_(leg_poser->leg_index_), leg_(leg_poser->leg_), robot_model_(leg_poser->robot_model_), auto_pose_(leg_poser->auto_pose_), first_iteration_(leg_poser->first_iteration_), master_iteration_count_(leg_poser->master_iteration_count_), origin_tip_pose_(leg_poser->origin_tip_pose_), current_tip_pose_(leg_poser->current_tip_pose_), target_tip_pose_(leg_poser->target_tip_pose_), external_target_(leg_poser->external_target_), leg_completed_step_(leg_poser->leg_completed_step_), physical_reference_height_(leg_poser->physical_reference_height_) {
}

bool LegPoser::stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
                              double lift_height, double time_to_step, bool apply_delta) {
    // OpenSHC EXACT implementation
    if (first_iteration_) {
        origin_tip_pose_ = getCurrentTipPose();
        master_iteration_count_ = 0;
        first_iteration_ = false;
    }

    Pose desired_tip_pose = target_tip_pose;

    // Check if transition is needed (OpenSHC exact)
    Point3D position_delta = origin_tip_pose_.position - desired_tip_pose.position;
    bool transition_position = std::sqrt(position_delta.x * position_delta.x +
                                         position_delta.y * position_delta.y +
                                         position_delta.z * position_delta.z) > TIP_TOLERANCE;

    if (!transition_position && lift_height == 0.0) {
        first_iteration_ = true;
        current_tip_pose_ = origin_tip_pose_;
        leg_completed_step_ = true;
        return true;
    }

    // Apply delta positioning (admittance control equivalent - simplified)
    if (apply_delta) {
        desired_tip_pose.position.x += target_pose.position.x;
        desired_tip_pose.position.y += target_pose.position.y;
        desired_tip_pose.position.z += target_pose.position.z;
    }

    master_iteration_count_++;

    // OpenSHC exact timing calculation
    double time_delta = 1.0 / robot_model_.getParams().control_frequency;
    int num_iterations = std::max(1, static_cast<int>(std::round(time_to_step / time_delta)));
    double delta_t = 1.0 / num_iterations;

    double completion_ratio = static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(num_iterations);

    // Interpolate pose applied to body between identity and target (OpenSHC exact)
    // Simplified: target_pose influence over completion_ratio
    Point3D pose_influence = target_pose.position * completion_ratio;

    Point3D new_tip_position = origin_tip_pose_.position;

    // OpenSHC EXACT bezier curve implementation
    int half_swing_iteration = num_iterations / 2;

    // Control nodes for dual 3D quartic bezier curves (OpenSHC exact)
    Point3D control_nodes_primary[5];
    Point3D control_nodes_secondary[5];
    Point3D origin_to_target = origin_tip_pose_.position - desired_tip_pose.position;

    // OpenSHC exact control node calculation
    control_nodes_primary[0] = origin_tip_pose_.position;
    control_nodes_primary[1] = origin_tip_pose_.position;
    control_nodes_primary[2] = origin_tip_pose_.position;
    control_nodes_primary[3] = desired_tip_pose.position + origin_to_target * 0.75;
    control_nodes_primary[4] = desired_tip_pose.position + origin_to_target * 0.5;
    control_nodes_primary[2].z += lift_height;
    control_nodes_primary[3].z += lift_height;
    control_nodes_primary[4].z += lift_height;

    control_nodes_secondary[0] = desired_tip_pose.position + origin_to_target * 0.5;
    control_nodes_secondary[1] = desired_tip_pose.position + origin_to_target * 0.25;
    control_nodes_secondary[2] = desired_tip_pose.position;
    control_nodes_secondary[3] = desired_tip_pose.position;
    control_nodes_secondary[4] = desired_tip_pose.position;
    control_nodes_secondary[0].z += lift_height;
    control_nodes_secondary[1].z += lift_height;
    control_nodes_secondary[2].z += lift_height;

    // OpenSHC exact swing iteration calculation
    int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % num_iterations + 1;

    double time_input;
    // Calculate change in position using 1st/2nd bezier curve (OpenSHC exact)
    if (swing_iteration_count <= half_swing_iteration) {
        time_input = swing_iteration_count * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_primary, time_input);
    } else {
        time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_secondary, time_input);
    }

    // Apply workspace constraints to the calculated position
    new_tip_position = robot_model_.getWorkspaceAnalyzer().constrainToGeometricWorkspace(leg_index_, new_tip_position);

    // OpenSHC pattern: Apply pose transformation to tip position
    current_tip_pose_.position = new_tip_position - pose_influence;
    current_tip_pose_.rotation = target_tip_pose.rotation;

    // Return completion status (OpenSHC exact)
    if (master_iteration_count_ >= num_iterations) {
        first_iteration_ = true;
        leg_completed_step_ = true;
        current_tip_pose_ = target_tip_pose; // Ensure exact final position
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
    double body_clearance = params.standing_height;

    // Calculate auto pose compensation based on phase
    // This simulates the body pose adjustments during gait execution
    Point3D compensated_position = default_stance_pos;

    // Enhanced phase-based Z compensation (body height variation)
    // Use sinusoidal compensation for smooth transitions relative to physical reference
    double z_compensation = 0.0;
    if (phase_ratio < 0.5) {
        // First half of gait cycle - gradual lift for stability
        z_compensation = body_clearance * 0.015 * sin(phase_ratio * 2.0 * M_PI);
    } else {
        // Second half of gait cycle - gradual drop for natural movement
        z_compensation = -body_clearance * 0.015 * sin((phase_ratio - 0.5) * 2.0 * M_PI);
    }

    // Consider physical reference height in Z compensation calculations
    // This ensures compensation is relative to the actual physical robot configuration
    double base_z_position = physical_reference_height_ + body_clearance;
    compensated_position.z = base_z_position + z_compensation;

    // Enhanced phase-based XY compensation (body roll/pitch compensation)
    // Consider leg position relative to body center for more accurate compensation
    double x_compensation = 0.0;
    double y_compensation = 0.0;

    // Calculate leg position relative to body center
    double leg_angle = robot_model_.getLegBaseAngleOffset(leg_index_);

    // Roll compensation based on leg position (Y-axis variation)
    // Legs on opposite sides of Y-axis get opposite compensation
    double roll_factor = sin(leg_angle);
    y_compensation = body_clearance * 0.008 * roll_factor * sin(phase_ratio * 4.0 * M_PI);

    // Pitch compensation based on leg position (X-axis variation)
    // Legs on opposite sides of X-axis get opposite compensation
    double pitch_factor = cos(leg_angle);
    x_compensation = body_clearance * 0.008 * pitch_factor * cos(phase_ratio * 4.0 * M_PI);

    // Additional stability compensation for tripod gait
    // This helps maintain center of mass during leg transitions
    double stability_compensation = 0.0;
    if (phase_ratio > 0.4 && phase_ratio < 0.6) {
        // During critical transition phase, add extra stability
        stability_compensation = body_clearance * 0.005 * cos((phase_ratio - 0.5) * 10.0 * M_PI);
    }

    // Apply XY compensations
    compensated_position.x += x_compensation;
    compensated_position.y += y_compensation;
    // Z position was already set above with base_z_position + z_compensation
    compensated_position.z += stability_compensation;

    // Create auto pose with compensated position and identity rotation
    auto_pose_ = Pose(compensated_position, Eigen::Vector3d(0, 0, 0));

    // Apply auto pose to current tip position if compensation is significant
    double compensation_magnitude = std::sqrt(
        (compensated_position.x - default_stance_pos.x) * (compensated_position.x - default_stance_pos.x) +
        (compensated_position.y - default_stance_pos.y) * (compensated_position.y - default_stance_pos.y) +
        (compensated_position.z - default_stance_pos.z) * (compensated_position.z - default_stance_pos.z));

    // Only apply compensation if it's above a minimum threshold to avoid jitter
    if (compensation_magnitude > 0.5) { // 0.5mm threshold
        // OpenSHC pattern: Only update position, NO IK here
        // IK is applied externally by the controller, not by trajectory generators
        leg_.setCurrentTipPositionGlobal(auto_pose_.position);
    }
}