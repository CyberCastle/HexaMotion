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

void LegPoser::updateAutoPose(int phase_index, const AutoPoseConfiguration &auto_cfg, const BodyPoseConfiguration &body_cfg) {
    // Full auto-pose update (OpenSHC-style) with subtraction-based negation.
    if (!auto_cfg.enabled) {
        auto_pose_ = Pose();
        return;
    }

    // Determine base posing cycle length (robust fallback for malformed configs)
    int base_period = auto_cfg.pose_phase_length;
    if (base_period <= 0) {
        int max_idx = 0;
        for (int v : auto_cfg.pose_phase_starts)
            if (v > max_idx)
                max_idx = v;
        for (int v : auto_cfg.pose_phase_ends)
            if (v > max_idx)
                max_idx = v;
        base_period = std::max(4, max_idx + 1);
    }
    int phase = (phase_index % base_period + base_period) % base_period; // normalised phase index

    // Helper lambdas
    auto inWindow = [base_period](int start, int end, int value) {
        if (start == end)
            return false; // empty window
        if (start < end)
            return value >= start && value < end;
        return value >= start || value < end; // wrapped window
    };
    auto modDist = [base_period](int from, int to) { return (to - from + base_period) % base_period; };
    auto smoothstep = [](double x) { x = math_utils::clamp(x, 0.0, 1.0); return x * x * (3.0 - 2.0 * x); };

    // Aggregate base amplitudes (average of all active windows covering current phase) per axis
    auto computeAxis = [&](const std::vector<double> &amps) {
        if (amps.empty())
            return 0.0;
        double acc = 0.0;
        int count = 0;
        for (size_t i = 0; i < auto_cfg.pose_phase_starts.size() && i < amps.size(); ++i) {
            if (inWindow(auto_cfg.pose_phase_starts[i], auto_cfg.pose_phase_ends[i], phase)) {
                acc += amps[i];
                ++count;
            }
        }
        return count ? acc / count : 0.0;
    };

    // Amplitudes are defined in radians (config file). Use directly.
    double base_roll = computeAxis(auto_cfg.roll_amplitudes);   // radians
    double base_pitch = computeAxis(auto_cfg.pitch_amplitudes); // radians
    double base_yaw = computeAxis(auto_cfg.yaw_amplitudes);     // radians
    double base_x = computeAxis(auto_cfg.x_amplitudes);         // mm
    double base_y = computeAxis(auto_cfg.y_amplitudes);         // mm
    double base_z = computeAxis(auto_cfg.z_amplitudes);         // mm

    // Orientation-induced positional offsets (small-angle approximation) relative to stance reference
    const LegStancePosition &stance = body_cfg.leg_stance_positions[leg_index_];
    double xs = stance.x;                // mm
    double ys = stance.y;                // mm
    double orient_dx = (-base_yaw * ys); // small-angle approximation (rad * mm)
    double orient_dy = (base_yaw * xs);
    double orient_dz = base_roll * ys - base_pitch * xs; // roll/pitch induce vertical shift (rad*mm)

    // Build base pose (translation + orientation quaternion)
    Point3D base_translation(base_x + orient_dx, base_y + orient_dy, base_z + orient_dz);
    Eigen::Vector3d euler_rad(base_roll, base_pitch, base_yaw);
    Eigen::Quaterniond q = Eigen::AngleAxisd(euler_rad.z(), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler_rad.y(), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler_rad.x(), Eigen::Vector3d::UnitX());
    base_auto_pose_ = Pose(base_translation, q);

    // Negation window (per leg): cancellation by subtracting an interpolated fraction of the base pose
    int ns = auto_cfg.negation_phase_start[leg_index_] % base_period;
    int ne = auto_cfg.negation_phase_end[leg_index_] % base_period;
    bool inside = inWindow(ns, ne, phase);
    int window_len = modDist(ns, ne);
    if (window_len == 0)
        window_len = base_period; // full cycle window
    double tr_ratio = math_utils::clamp(auto_cfg.negation_transition_ratio[leg_index_], 0.0, 0.49);
    double transition_span = window_len * tr_ratio;
    double control_input = 0.0; // 0=no cancellation, 1=full cancellation
    if (inside) {
        if (transition_span <= 0.0) {
            control_input = 1.0; // full cancel inside window
        } else {
            int dist_from_start = modDist(ns, phase);
            int dist_to_end = modDist(phase, ne);
            if (dist_from_start < transition_span) { // entry ramp
                control_input = smoothstep(double(dist_from_start) / transition_span);
            } else if (dist_to_end < transition_span) { // exit ramp
                control_input = smoothstep(double(dist_to_end) / transition_span);
            } else {
                control_input = 1.0; // middle region full cancel
            }
        }
    }

    if (control_input <= 0.0) {
        auto_pose_ = base_auto_pose_; // unchanged
    } else if (control_input >= 1.0) {

        // Fully cancelled (identity pose)
        auto_pose_ = Pose();
    } else {
        // Subtract interpolated fraction of base pose
        Pose portion = Pose().interpolate(control_input, base_auto_pose_);
        auto_pose_ = base_auto_pose_.removePose(portion);
    }
}