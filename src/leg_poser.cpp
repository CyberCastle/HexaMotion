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
    double time_delta = robot_model_.getTimeDelta();
    int num_iterations = std::max(1, static_cast<int>(std::round(time_to_step / time_delta)));
    current_num_iterations_ = num_iterations; // store for progress reporting
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

void LegPoser::updateAutoPose(int phase_index, const AutoPoseConfiguration &auto_cfg, const BodyPoseConfiguration &body_cfg) {
    // OpenSHC-style per-leg negation of already computed global auto pose (emulated locally).
    // This version mirrors OpenSHC LegPoser::updateAutoPose logic:
    // 1. Determine negation window (start/end) mapped into unified phase domain.
    // 2. Handle wrap-around windows (start > end) by extending end beyond period.
    // 3. Decide if current phase_index triggers negation enable flag.
    // 4. If negated, build a smooth removal pose via smoothstep ramps (negation_transition_ratio).
    // 5. Compose final auto_pose_ relative to stance reference for this leg.

    if (!auto_cfg.enabled) {
        auto_pose_ = Pose();
        return;
    }

    // Determine base period (robust fallbacks like body controller)
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
    int phase = (phase_index % base_period + base_period) % base_period; // normalize

    // Window membership helper (same semantics as body controller)
    auto inWindow = [base_period](int start, int end, int value) {
        if (start == end)
            return false;
        if (start < end)
            return value >= start && value < end;
        return value >= start || value < end;
    };

    auto modDist = [base_period](int from, int to) { return (to - from + base_period) % base_period; };

    // Cubic smoothstep easing function f(x)=3x^2-2x^3 with clamped domain [0,1].
    // Derived by solving a cubic a x^3 + b x^2 + c x + d subject to:
    //  f(0)=0 (start at 0), f(1)=1 (end at 1), f'(0)=0 and f'(1)=0 (zero slope at boundaries).
    // These four boundary conditions yield a=-2, b=3, c=0, d=0 -> f(x)=3x^2-2x^3.
    // Provides C1 continuity (no velocity jump) minimizing jerk at start/end of negation ramps.
    // Chosen over linear (discontinuous derivative) and over quintic smootherstep (6x^5-15x^4+10x^3)
    // to balance smoothness with minimal computational cost in the control loop.
    auto smoothstep = [](double x) {
        x = math_utils::clamp(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    };

    // Compute base aggregated auto pose (same averaging strategy as BodyPoseController) for this phase.
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

    double base_roll = computeAxis(auto_cfg.roll_amplitudes);
    double base_pitch = computeAxis(auto_cfg.pitch_amplitudes);
    double base_yaw = computeAxis(auto_cfg.yaw_amplitudes);
    double base_x = computeAxis(auto_cfg.x_amplitudes);
    double base_y = computeAxis(auto_cfg.y_amplitudes);
    double base_z = computeAxis(auto_cfg.z_amplitudes);

    // Negation window for this leg
    int ns = auto_cfg.negation_phase_start[leg_index_] % base_period;
    int ne = auto_cfg.negation_phase_end[leg_index_] % base_period;
    bool inside = inWindow(ns, ne, phase);
    int window_len = modDist(ns, ne);
    if (window_len == 0)
        window_len = base_period; // window covers full cycle

    // Determine current sign / blending factor based on negation transition
    double tr_ratio = math_utils::clamp(auto_cfg.negation_transition_ratio[leg_index_], 0.0, 0.49);
    double transition_span = window_len * tr_ratio;
    double sign = 1.0;  // +1 means not negated, -1 means fully negated
    double blend = 1.0; // 1.0 = full amplitude, 0 = zero (during cross-fade removal)

    if (transition_span <= 0.0) {
        sign = inside ? -1.0 : 1.0;
    } else {
        if (inside) {
            int dist_from_start = modDist(ns, phase);
            int dist_to_end = modDist(phase, ne);
            if (dist_from_start < transition_span) {          // entry ramp
                double t = dist_from_start / transition_span; // 0->1
                double s = smoothstep(t);
                // Transition +1 -> -1 crossing zero at s=0.5
                sign = 1.0 - 2.0 * s;
            } else if (dist_to_end < transition_span) {   // exit ramp
                double t = dist_to_end / transition_span; // 0->1
                double s = smoothstep(t);
                sign = -1.0 + 2.0 * s; // -1 -> +1
            } else {
                sign = -1.0;
            }
        } else {
            sign = 1.0;
        }
    }

    // Orientation induced positional offsets (small-angle) relative to stance reference
    const LegStancePosition &stance = body_cfg.leg_stance_positions[leg_index_];
    double xs = stance.x;
    double ys = stance.y;
    double orient_dx = (-base_yaw * ys);
    double orient_dy = (base_yaw * xs);
    double orient_dz = base_roll * ys - base_pitch * xs;

    Point3D offset;
    offset.x = base_x + orient_dx;        // X translation + yaw-induced shift
    offset.y = base_y + orient_dy;        // Y translation + yaw-induced shift
    offset.z = base_z * sign + orient_dz; // Z translation with possible negation + roll/pitch induced shift

    // Build pose (translation only for now; orientation reserved for future)
    auto_pose_ = Pose(offset, Eigen::Quaterniond::Identity());

    // Apply only if magnitude exceeds threshold (jitter guard)
    double mag = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
    if (mag > auto_cfg.apply_threshold_mm) {
        // Direct positional application: external controller/IK will refine if needed
        Point3D new_pos = leg_.getCurrentTipPositionGlobal();
        new_pos.x += offset.x;
        new_pos.y += offset.y;
        new_pos.z += offset.z;
        leg_.setCurrentTipPositionGlobal(new_pos);
    }
}