#include "leg_poser.h"
#include "locomotion_system.h"
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

LegPoser::LegPoser(PoseController* pose_controller, LocomotionSystem* locomotion_system, int leg_index, Leg& leg)
    : pose_controller_(pose_controller)
    , locomotion_system_(locomotion_system)
    , leg_index_(leg_index)
    , leg_(leg)
    , auto_pose_(Pose::Identity())
    , target_tip_pose_(Pose::Undefined()) {
}

LegPoser::LegPoser(const LegPoser* leg_poser)
    : pose_controller_(leg_poser->pose_controller_)
    , locomotion_system_(leg_poser->locomotion_system_)
    , leg_index_(leg_poser->leg_index_)
    , leg_(leg_poser->leg_)
    , auto_pose_(leg_poser->auto_pose_)
    , target_tip_pose_(leg_poser->target_tip_pose_) {

    pose_negation_phase_start_ = leg_poser->pose_negation_phase_start_;
    pose_negation_phase_end_ = leg_poser->pose_negation_phase_end_;
    negate_auto_pose_ = leg_poser->negate_auto_pose_;
    first_iteration_ = leg_poser->first_iteration_;
    master_iteration_count_ = leg_poser->master_iteration_count_;
    desired_configuration_ = leg_poser->desired_configuration_;
    origin_configuration_ = leg_poser->origin_configuration_;
    origin_tip_pose_ = leg_poser->origin_tip_pose_;
    external_target_ = leg_poser->external_target_;
    transition_poses_ = leg_poser->transition_poses_;
    leg_completed_step_ = leg_poser->leg_completed_step_;
}

int LegPoser::transitionConfiguration(double transition_time) {
    // Return early if desired configuration is undefined
    if (desired_configuration_.size() == 0) {
        return PROGRESS_COMPLETE;
    }

    // Setup origin and target joint positions for bezier curve
    if (first_iteration_) {
        origin_configuration_.clear();

        // Get current joint angles from leg object
        JointAngles current_angles = leg_.getJointAngles();

        // Setup origin configuration
        origin_configuration_.names.push_back("coxa");
        origin_configuration_.positions.push_back(math_utils::degreesToRadians(current_angles.coxa));
        origin_configuration_.names.push_back("femur");
        origin_configuration_.positions.push_back(math_utils::degreesToRadians(current_angles.femur));
        origin_configuration_.names.push_back("tibia");
        origin_configuration_.positions.push_back(math_utils::degreesToRadians(current_angles.tibia));

        first_iteration_ = false;
        master_iteration_count_ = 0;
    }

    // Calculate number of iterations based on control frequency
    const Parameters& params = locomotion_system_->getParameters();
    double time_delta = 1.0 / params.control_frequency;
    int num_iterations = std::max(1, math_utils::roundToInt(transition_time / time_delta));
    double delta_t = 1.0 / num_iterations;

    master_iteration_count_++;

    // Calculate new joint positions using cubic Bezier curves
    JointAngles new_angles;

    // Coxa joint
    double coxa_control_nodes[4] = {
        origin_configuration_.positions[0],
        origin_configuration_.positions[0],
        desired_configuration_.positions[0],
        desired_configuration_.positions[0]
    };
    new_angles.coxa = math_utils::radiansToDegrees(
        math_utils::cubicBezier(coxa_control_nodes, master_iteration_count_ * delta_t)
    );

    // Femur joint
    double femur_control_nodes[4] = {
        origin_configuration_.positions[1],
        origin_configuration_.positions[1],
        desired_configuration_.positions[1],
        desired_configuration_.positions[1]
    };
    new_angles.femur = math_utils::radiansToDegrees(
        math_utils::cubicBezier(femur_control_nodes, master_iteration_count_ * delta_t)
    );

    // Tibia joint
    double tibia_control_nodes[4] = {
        origin_configuration_.positions[2],
        origin_configuration_.positions[2],
        desired_configuration_.positions[2],
        desired_configuration_.positions[2]
    };
    new_angles.tibia = math_utils::radiansToDegrees(
        math_utils::cubicBezier(tibia_control_nodes, master_iteration_count_ * delta_t)
    );

    // Apply new joint angles to leg object
    leg_.setJointAngles(new_angles);

    // Return percentage of progress completion (1%->100%)
    int progress = static_cast<int>((static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(num_iterations)) * PROGRESS_COMPLETE);
    progress = math_utils::clamped(progress, 1, PROGRESS_COMPLETE); // Ensures 1 percent is minimum return

    // Complete once reached total number of iterations
    if (master_iteration_count_ >= num_iterations) {
        first_iteration_ = true;
        return PROGRESS_COMPLETE;
    } else {
        return progress;
    }
}

int LegPoser::stepToPosition(const Pose& target_tip_pose, const Pose& target_pose,
                             double lift_height, double time_to_step, bool apply_delta) {
    if (first_iteration_) {
        // Get current tip pose from leg object
        Point3D current_position = leg_.getTipPosition();
        origin_tip_pose_ = Pose(current_position, Eigen::Quaterniond::Identity());
        master_iteration_count_ = 0;
        first_iteration_ = false;
    }

    Pose desired_tip_pose = target_tip_pose;
    if (desired_tip_pose == Pose::Undefined()) {
        desired_tip_pose = origin_tip_pose_;
    }

    // Check if transition is needed
    Eigen::Vector3d position_delta =
        origin_tip_pose_.position - target_pose.inverseTransformVector(desired_tip_pose.position);
    bool transition_position = position_delta.norm() > TIP_TOLERANCE;
    bool transition_rotation = false;

    if (!desired_tip_pose.rotation.isApprox(Eigen::Quaterniond::Identity())) {
        Eigen::Vector3d origin_tip_direction = origin_tip_pose_.rotation * Eigen::Vector3d::UnitX();
        Eigen::Vector3d desired_tip_direction = desired_tip_pose.rotation * Eigen::Vector3d::UnitX();
        Eigen::AngleAxisd rotation_delta(Eigen::Quaterniond::FromTwoVectors(origin_tip_direction, desired_tip_direction));
        transition_rotation = rotation_delta.angle() > JOINT_TOLERANCE;
    }

    if (!transition_position && !transition_rotation && lift_height == 0.0) {
        first_iteration_ = true;
        current_tip_pose_ = origin_tip_pose_;
        return PROGRESS_COMPLETE;
    }

    master_iteration_count_++;

    // Calculate number of iterations based on control frequency
    const Parameters& params = locomotion_system_->getParameters();
    double time_delta = 1.0 / params.control_frequency;
    int num_iterations = std::max(1, math_utils::roundToInt(time_to_step / time_delta));
    double delta_t = 1.0 / num_iterations;

    double completion_ratio = (static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(num_iterations));

    // Interpolate pose applied to body between identity and target
    Pose desired_pose = Pose::Identity().interpolate(math_utils::smoothStep(completion_ratio), target_pose);

    // Interpolate tip rotation between origin and target (if target is defined)
    Eigen::Quaterniond new_tip_rotation = Eigen::Quaterniond::Identity();
    if (!desired_tip_pose.rotation.isApprox(Eigen::Quaterniond::Identity())) {
        Eigen::Vector3d origin_tip_direction = origin_tip_pose_.rotation * Eigen::Vector3d::UnitX();
        Eigen::Vector3d desired_tip_direction = desired_tip_pose.rotation * Eigen::Vector3d::UnitX();
                Eigen::Vector3d new_tip_direction =
            math_utils::interpolate(origin_tip_direction, desired_tip_direction, math_utils::smoothStep(completion_ratio));
        new_tip_rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), new_tip_direction.normalized());
    }

    // Calculate new tip position using dual quartic Bezier curves
    Eigen::Vector3d new_tip_position = origin_tip_pose_.position;
    if (desired_tip_pose.position != Point3D(UNDEFINED_VALUE, UNDEFINED_VALUE, UNDEFINED_VALUE)) {
        int half_swing_iteration = num_iterations / 2;

        // Control nodes for dual 3d quartic bezier curves
        Eigen::Vector3d control_nodes_primary[5];
        Eigen::Vector3d control_nodes_secondary[5];
        Eigen::Vector3d origin_to_target = origin_tip_pose_.position - desired_tip_pose.position;

        // Primary swing curve control nodes
        control_nodes_primary[0] = origin_tip_pose_.position;
        control_nodes_primary[1] = origin_tip_pose_.position;
        control_nodes_primary[2] = origin_tip_pose_.position;
        control_nodes_primary[3] = desired_tip_pose.position + 0.75 * origin_to_target;
        control_nodes_primary[4] = desired_tip_pose.position + 0.5 * origin_to_target;
        control_nodes_primary[2].z() += lift_height;
        control_nodes_primary[3].z() += lift_height;
        control_nodes_primary[4].z() += lift_height;

        // Secondary swing curve control nodes
        control_nodes_secondary[0] = desired_tip_pose.position + 0.5 * origin_to_target;
        control_nodes_secondary[1] = desired_tip_pose.position + 0.25 * origin_to_target;
        control_nodes_secondary[2] = desired_tip_pose.position;
        control_nodes_secondary[3] = desired_tip_pose.position;
        control_nodes_secondary[4] = desired_tip_pose.position;
        control_nodes_secondary[0].z() += lift_height;
        control_nodes_secondary[1].z() += lift_height;
        control_nodes_secondary[2].z() += lift_height;

        int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % num_iterations + 1;

        // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
        if (swing_iteration_count <= half_swing_iteration) {
            double time_input = swing_iteration_count * delta_t * 2.0;
            new_tip_position = math_utils::quarticBezier(control_nodes_primary, time_input);
        } else {
            double time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
            new_tip_position = math_utils::quarticBezier(control_nodes_secondary, time_input);
        }
    }

    // Update current tip pose
    current_tip_pose_.position = desired_pose.inverseTransformVector(new_tip_position);
    current_tip_pose_.rotation = new_tip_rotation;

    // Apply new tip position to locomotion system
    locomotion_system_->setLegPosition(leg_index_, Point3D(new_tip_position.x(), new_tip_position.y(), new_tip_position.z()));

    // Return ratio of completion (1.0 when fully complete)
    if (master_iteration_count_ >= num_iterations) {
        first_iteration_ = true;
        return PROGRESS_COMPLETE;
    } else {
        return static_cast<int>(completion_ratio * PROGRESS_COMPLETE);
    }
}

void LegPoser::updateAutoPose(int phase) {
    // This is a simplified version of OpenSHC's updateAutoPose
    // In a full implementation, this would handle auto-posing negation cycles

    int start_phase = pose_negation_phase_start_;
    int end_phase = pose_negation_phase_end_;
    int negation_phase = phase;

    // Handle phase wrapping
    if (start_phase == 0) {
        start_phase = 100; // Default phase length
    }
    if (end_phase == 0) {
        end_phase = 100; // Default phase length
    }

    // Handle phase overlapping
    if (start_phase > end_phase) {
        end_phase += 100; // Default phase length
        if (negation_phase < start_phase) {
            negation_phase += 100; // Default phase length
        }
    }

    // Switch on/off auto pose negation
    if (negation_phase == start_phase) {
        negate_auto_pose_ = true;
    }
    if (negation_phase < start_phase || negation_phase > end_phase) {
        negate_auto_pose_ = false;
    }

    // Assign leg auto pose (simplified - in full implementation this would come from pose controller)
    auto_pose_ = Pose::Identity();

    // Negate auto pose for this leg during negation period as defined by parameters
    if (negate_auto_pose_) {
        int iteration = negation_phase - start_phase + 1;
        int num_iterations = end_phase - start_phase;
        bool first_half = iteration <= num_iterations / 2;
        double control_input = 1.0;

        if (negation_transition_ratio_ > 0.0) {
            if (first_half) {
                control_input = std::min(1.0, static_cast<double>(iteration) / (num_iterations * negation_transition_ratio_));
            } else {
                control_input = std::min(1.0, static_cast<double>(num_iterations - iteration) / (num_iterations * negation_transition_ratio_));
            }
        }
        control_input = math_utils::smoothStep(control_input);
        Pose negation = Pose::Identity().interpolate(control_input, auto_pose_);
        auto_pose_ = auto_pose_.removePose(negation);
    }
}