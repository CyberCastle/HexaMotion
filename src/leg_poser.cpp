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
    : leg_index_(leg_poser->leg_index_), leg_(leg_poser->leg_), robot_model_(leg_poser->robot_model_), auto_pose_(leg_poser->auto_pose_), first_iteration_(leg_poser->first_iteration_), master_iteration_count_(leg_poser->master_iteration_count_), origin_tip_pose_(leg_poser->origin_tip_pose_), current_tip_pose_(leg_poser->current_tip_pose_), target_tip_pose_(leg_poser->target_tip_pose_), external_target_(leg_poser->external_target_), leg_completed_step_(leg_poser->leg_completed_step_), transition_poses_(leg_poser->transition_poses_), physical_reference_height_(leg_poser->physical_reference_height_) {
}

int LegPoser::stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
                             double lift_height, double time_to_step, bool apply_delta) {
    // Initialize on first iteration
    if (first_iteration_) {
        origin_tip_pose_ = getCurrentTipPose();
        master_iteration_count_ = 0;
        first_iteration_ = false;
    }

    // Distance check to early-complete
    Point3D delta_pos = origin_tip_pose_.position - target_tip_pose.position;
    double dist = std::sqrt(delta_pos.x * delta_pos.x + delta_pos.y * delta_pos.y + delta_pos.z * delta_pos.z);
    if (dist < TIP_TOLERANCE && lift_height == 0.0) {
        first_iteration_ = true;
        current_tip_pose_ = origin_tip_pose_;
        leg_completed_step_ = true;
        return PROGRESS_COMPLETE;
    }

    // Timing discretization
    double time_delta = robot_model_.getTimeDelta();
    int num_iterations = std::max(1, static_cast<int>(std::round(time_to_step / time_delta)));
    current_num_iterations_ = num_iterations;
    master_iteration_count_++;
    double completion_ratio = static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(num_iterations);
    if (completion_ratio < 0.0)
        completion_ratio = 0.0;
    if (completion_ratio > 1.0)
        completion_ratio = 1.0;

    // Eased interpolation of full target pose (translation + rotation)
    auto smoothStep = [](double x) { x = math_utils::clamp(x, 0.0, 1.0); return x * x * (3.0 - 2.0 * x); };
    double eased = smoothStep(completion_ratio);
    Pose desired_pose = Pose().interpolate(eased, target_pose); // Equivalent to Identity().interpolate

    // Dual quartic bezier control nodes like original
    Point3D control_nodes_primary[5];
    Point3D control_nodes_secondary[5];
    Point3D origin_to_target = origin_tip_pose_.position - target_tip_pose.position;
    control_nodes_primary[0] = origin_tip_pose_.position;
    control_nodes_primary[1] = origin_tip_pose_.position;
    control_nodes_primary[2] = origin_tip_pose_.position;
    control_nodes_primary[3] = target_tip_pose.position + origin_to_target * 0.75;
    control_nodes_primary[4] = target_tip_pose.position + origin_to_target * 0.5;
    control_nodes_primary[2].z += lift_height;
    control_nodes_primary[3].z += lift_height;
    control_nodes_primary[4].z += lift_height;
    control_nodes_secondary[0] = target_tip_pose.position + origin_to_target * 0.5;
    control_nodes_secondary[1] = target_tip_pose.position + origin_to_target * 0.25;
    control_nodes_secondary[2] = target_tip_pose.position;
    control_nodes_secondary[3] = target_tip_pose.position;
    control_nodes_secondary[4] = target_tip_pose.position;
    control_nodes_secondary[0].z += lift_height;
    control_nodes_secondary[1].z += lift_height;
    control_nodes_secondary[2].z += lift_height;

    int half_swing_iteration = num_iterations / 2;
    int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % num_iterations + 1;
    double delta_t = 1.0 / num_iterations;
    double bez_time_input;
    Point3D new_tip_position;
    if (swing_iteration_count <= half_swing_iteration) {
        bez_time_input = swing_iteration_count * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_primary, bez_time_input);
    } else {
        bez_time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
        new_tip_position = math_utils::quarticBezier(control_nodes_secondary, bez_time_input);
    }

    // Workspace constraint
    new_tip_position = robot_model_.getWorkspaceAnalyzer().constrainToGeometricWorkspace(leg_index_, new_tip_position);
    // Apply pose transform inverse (OpenSHC semantics) and admittance delta if requested
    Point3D transformed = desired_pose.inverseTransformVector(new_tip_position);
    if (apply_delta) {
        // Defensive safeguard: if admittance controller not yet initialized or delta unreasonably large, ignore.
        auto safe_component = [&](double v) { return (std::isfinite(v) && std::fabs(v) <= ADMITTANCE_MAX_ABS_DELTA_MM) ? v : 0.0; };
        transformed.x += safe_component(admittance_delta_.x);
        transformed.y += safe_component(admittance_delta_.y);
        transformed.z += safe_component(admittance_delta_.z);
    }
    current_tip_pose_.position = transformed;
    current_tip_pose_.rotation = target_tip_pose.rotation; // Pass-through target tip orientation

    int progress_percent = static_cast<int>(std::round(completion_ratio * 100.0));
    if (progress_percent >= PROGRESS_COMPLETE) {
        first_iteration_ = true;
        leg_completed_step_ = true;
        current_tip_pose_ = target_tip_pose; // finalize
        progress_percent = PROGRESS_COMPLETE;
    }
    return progress_percent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC LegPoser::updateAutoPose() equivalent — 1:1 port
// Takes global auto_pose_ from BPC and applies per-leg negation using iteration-based
// first_half/smoothStep logic matching OpenSHC exactly.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LegPoser::updateAutoPose(int phase, const Pose &global_auto_pose, int normaliser, int phase_length) {
    // Scale negation phases by normaliser (OpenSHC parity)
    int start_phase = pose_negation_phase_start_ * normaliser;
    int end_phase = pose_negation_phase_end_ * normaliser;
    int negation_phase = phase;

    // Changes start/end phases from zero to phase length value (which is equivalent) (OpenSHC parity)
    if (start_phase == 0) {
        start_phase = phase_length;
    }
    if (end_phase == 0) {
        end_phase = phase_length;
    }

    // Handle phase overlapping master phase start/end (OpenSHC parity)
    if (start_phase > end_phase) {
        end_phase += phase_length;
        if (negation_phase < start_phase) {
            negation_phase += phase_length;
        }
    }

    // Switch on/off auto pose negation (OpenSHC parity)
    // Note: OpenSHC also checks step_state != FORCE_STANCE && FORCE_STOP,
    // but HexaMotion's LegPoser doesn't have LegStepper access. Omitted as
    // FORCE states only occur during transitions, not normal auto-posing.
    if (negation_phase == start_phase) {
        negate_auto_pose_ = true;
    }
    if (negation_phase < start_phase || negation_phase > end_phase) {
        negate_auto_pose_ = false;
    }

    // Assign leg auto pose according to default auto pose (OpenSHC parity)
    auto_pose_ = global_auto_pose;

    // Negate auto pose for this leg during negation period as defined by parameters (OpenSHC parity)
    if (negate_auto_pose_) {
        int iteration = negation_phase - start_phase + 1;
        int num_iterations = end_phase - start_phase;
        bool first_half = iteration <= num_iterations / 2;
        double control_input = 1.0;
        if (negation_transition_ratio_ > 0.0) {
            if (first_half) {
                control_input = std::min(1.0, static_cast<double>(iteration) /
                                                  (num_iterations * negation_transition_ratio_));
            } else {
                control_input = std::min(1.0, static_cast<double>(num_iterations - iteration) /
                                                  (num_iterations * negation_transition_ratio_));
            }
        }
        control_input = math_utils::smoothStep(control_input);
        Pose negation = Pose::Identity().interpolate(control_input, auto_pose_);
        auto_pose_ = auto_pose_.removePose(negation);
    }
}

// ================================================================================================
// OpenSHC LegPoser::transitionConfiguration equivalent
// Cubic Bezier interpolation of joint positions from current to target configuration
// ================================================================================================
int LegPoser::transitionConfiguration(double transition_time) {
    // Return early if no desired configuration set
    if (!desired_config_set_) {
        return PROGRESS_COMPLETE;
    }

    // Setup origin and target on first iteration
    if (config_first_iteration_) {
        // OpenSHC parity: joint_angles_ are stored in radians (same unit as
        // desired_config targets from Parameters). No conversion needed.
        JointAngles current_angles = leg_.getJointAngles();
        origin_config_coxa_ = current_angles.coxa;
        origin_config_femur_ = current_angles.femur;
        origin_config_tibia_ = current_angles.tibia;

        // Check if already at target (joint tolerance check)
        constexpr double JOINT_TOLERANCE_RAD = 0.01;
        bool at_target = std::abs(desired_config_coxa_ - origin_config_coxa_) < JOINT_TOLERANCE_RAD &&
                         std::abs(desired_config_femur_ - origin_config_femur_) < JOINT_TOLERANCE_RAD &&
                         std::abs(desired_config_tibia_ - origin_config_tibia_) < JOINT_TOLERANCE_RAD;
        if (at_target) {
            desired_config_set_ = false;
            return PROGRESS_COMPLETE;
        }

        config_first_iteration_ = false;
        config_iteration_count_ = 0;
    }

    double time_delta = robot_model_.getTimeDelta();
    int num_iterations = std::max(1, static_cast<int>(std::round(transition_time / time_delta)));
    double delta_t = 1.0 / num_iterations;

    config_iteration_count_++;
    double t = config_iteration_count_ * delta_t;
    if (t > 1.0)
        t = 1.0;

    // Cubic Bezier interpolation for each joint (C1 continuity at endpoints)
    // control_nodes: [origin, origin, target, target]
    auto cubicBezier = [](double p0, double p1, double p2, double p3, double t) {
        double u = 1.0 - t;
        return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3;
    };

    double new_coxa = cubicBezier(origin_config_coxa_, origin_config_coxa_,
                                  desired_config_coxa_, desired_config_coxa_, t);
    double new_femur = cubicBezier(origin_config_femur_, origin_config_femur_,
                                   desired_config_femur_, desired_config_femur_, t);
    double new_tibia = cubicBezier(origin_config_tibia_, origin_config_tibia_,
                                   desired_config_tibia_, desired_config_tibia_, t);

    // Apply joint angles (radians throughout, matching OpenSHC convention)
    JointAngles new_angles;
    new_angles.coxa = new_coxa;
    new_angles.femur = new_femur;
    new_angles.tibia = new_tibia;
    leg_.setJointAngles(new_angles);

    // Update tip position via FK
    Point3D new_tip = robot_model_.forwardKinematicsGlobalCoordinates(leg_index_, new_angles);
    leg_.setCurrentTipPositionGlobal(new_tip);
    current_tip_pose_.position = new_tip;

    // Progress calculation
    int progress = static_cast<int>(std::round((static_cast<double>(config_iteration_count_ - 1) /
                                                static_cast<double>(num_iterations)) *
                                               PROGRESS_COMPLETE));
    progress = math_utils::clamp(progress, 1, PROGRESS_COMPLETE);

    if (config_iteration_count_ >= num_iterations) {
        config_first_iteration_ = true;
        desired_config_set_ = false;
        return PROGRESS_COMPLETE;
    }

    return progress;
}