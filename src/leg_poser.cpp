#include "leg_poser.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <cmath>
#include <algorithm>

/**
 * @file leg_poser.cpp
 * @brief Implementation of LegPoser for HexaMotion
 *
 * This implementation is adapted from OpenSHC's LegPoser class to work with HexaMotion's architecture.
 * It provides smooth transitions for leg positioning using Bezier curves and manages auto-posing.
 */

LegPoser::LegPoser(int leg_index, Leg& leg)
    : leg_index_(leg_index)
    , leg_(leg)
    , first_iteration_(true)
    , master_iteration_count_(0) {

    // Initialize poses
    auto_pose_ = Pose();
    origin_tip_pose_ = Pose();
    current_tip_pose_ = Pose();
    target_tip_pose_ = Pose();
    external_target_ = ExternalTarget();

    // Initialize configurations
    desired_configuration_.clear();
    origin_configuration_.clear();

    // Initialize transition poses
    transition_poses_.clear();

    // Initialize flags
    leg_completed_step_ = false;
    pose_negation_phase_start_ = 0;
    pose_negation_phase_end_ = 0;
    negation_transition_ratio_ = 0.0;
}

LegPoser::LegPoser(const LegPoser* leg_poser)
    : leg_index_(leg_poser->leg_index_)
    , leg_(leg_poser->leg_)
    , auto_pose_(leg_poser->auto_pose_)
    , negate_auto_pose_(leg_poser->negate_auto_pose_)
    , pose_negation_phase_start_(leg_poser->pose_negation_phase_start_)
    , pose_negation_phase_end_(leg_poser->pose_negation_phase_end_)
    , negation_transition_ratio_(leg_poser->negation_transition_ratio_)
    , first_iteration_(leg_poser->first_iteration_)
    , master_iteration_count_(leg_poser->master_iteration_count_)
    , desired_configuration_(leg_poser->desired_configuration_)
    , origin_configuration_(leg_poser->origin_configuration_)
    , origin_tip_pose_(leg_poser->origin_tip_pose_)
    , current_tip_pose_(leg_poser->current_tip_pose_)
    , target_tip_pose_(leg_poser->target_tip_pose_)
    , external_target_(leg_poser->external_target_)
    , transition_poses_(leg_poser->transition_poses_)
    , leg_completed_step_(leg_poser->leg_completed_step_) {
}

int LegPoser::transitionConfiguration(double transition_time) {
    // Simple implementation for now - return complete immediately
    // In a full implementation, this would use bezier curves for smooth joint transitions
    return PROGRESS_COMPLETE;
}

int LegPoser::stepToPosition(const Pose& target_tip_pose, const Pose& target_pose,
                           double lift_height, double time_to_step, bool apply_delta) {
    // Store origin pose
    if (first_iteration_) {
        origin_tip_pose_ = getCurrentTipPose();
        target_tip_pose_ = target_tip_pose;
        first_iteration_ = false;
        master_iteration_count_ = 0;
    }

    // Simple implementation - move directly to target
    // In a full implementation, this would use bezier curves for smooth tip transitions
    Point3D target_position = target_tip_pose.position;
    if (apply_delta) {
        target_position.x += target_pose.position.x;
        target_position.y += target_pose.position.y;
        target_position.z += target_pose.position.z;
    }

    // Update leg position
    leg_.setCurrentTipPositionGlobal(target_position);

    // Mark step as completed
    leg_completed_step_ = true;
    first_iteration_ = true; // Reset for next step

    return PROGRESS_COMPLETE;
}

void LegPoser::updateAutoPose(int phase) {
    // Simple implementation - set auto pose based on phase
    // In a full implementation, this would calculate auto pose from default pose with negation if required

    // Calculate auto pose position based on phase
    double angle = phase * 2.0 * M_PI / 100.0; // Convert phase to radians
    double radius = 50.0; // Default radius

    Point3D auto_position;
    auto_position.x = radius * cos(angle);
    auto_position.y = radius * sin(angle);
    auto_position.z = -120.0; // Default height

    // Apply negation if required
    if (negate_auto_pose_) {
        auto_position.x = -auto_position.x;
        auto_position.y = -auto_position.y;
    }

    auto_pose_ = Pose(auto_position, Eigen::Vector3d(0, 0, 0));
}