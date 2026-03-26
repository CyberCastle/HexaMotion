/**
 * @file swing_coxa_orientation_test.cpp
 * @brief Detects whether SWING trajectory generation (target_tip_pose + Bezier nodes)
 *        ignores the current coxa angle (hypothesis: coxa=0° is assumed when computing the target).
 *
 * Methodology:
 *  - Set up a LegStepper with identity and known default_tip_pose.
 *  - Apply a linear velocity (stride) to force a target in swing.
 *  - For several coxa angles (-20, 0, +20 degrees) modify the leg joint
 *    before invoking updateTipPositionIterative() in SWING state (iteration 1).
 *  - Record:
 *      * Initial current_tip_pose (post coxa rotation)
 *      * Frozen target_tip_pose_
 *      * swing_2_nodes_[4] (swing end node)
 *      * Planar direction and magnitude base->target and base->final_swing
 *  - Compute OpenSHC reference: raw_target = default_tip_pose + 0.5 * stride_vector
 *  - If base->target vectors are identical for all angles (angular difference ~0)
 *    it indicates that the current coxa angle was NOT explicitly incorporated into the target computation.
 */

#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct CoxaAngleCase {
    double coxa_deg;
};

static double planarAngle(const Point3D &v) { return std::atan2(v.y, v.x); }
static double planarNorm(const Point3D &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

int main() {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Swing Coxa Orientation Influence Test ===\n";

    // Minimum parameters consistent with AGENTS.md
    Parameters params{};
    params.hexagon_radius = 160.0;
    params.coxa_length = 45.0;
    params.femur_length = 90.0;
    params.tibia_length = 150.0;
    params.default_height_offset = -params.tibia_length;
    params.robot_height = 150.0;
    params.time_delta = 1.0 / 50.0;
    params.standing_height = 120.0;
    params.enable_workspace_constrain = false;

    RobotModel model(params);

    // Common StepCycle configuration
    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 4;
    cycle.stance_period_ = 2;
    cycle.swing_period_ = 2;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 2;
    cycle.swing_start_ = 2;
    cycle.swing_end_ = 4;

    // Accumulated data per leg
    struct LegSummary {
        int leg;
        double max_delta_target_deg;
        double max_delta_swing_end_deg;
    };
    std::vector<LegSummary> leg_summaries;

    // Iterate over all 6 legs
    for (int leg_index = 0; leg_index < 6; ++leg_index) {
        Leg leg(leg_index, model);

        // Build radial identity for the leg using its base theta
        double base_theta = model.getLegBaseAngleOffset(leg_index);                         // radians
        double planar_r = params.hexagon_radius + params.coxa_length + params.femur_length; // same reasoning as stride test
        Point3D identity_tip(planar_r * std::cos(base_theta), planar_r * std::sin(base_theta), params.default_height_offset);

        LegStepper stepper(leg_index, identity_tip, leg, model);
        stepper.setStepCycle(cycle);

        // Velocities that generate stride (identical for all, projected via stride update)
        Point3D linear_vel(60.0, 20.0, 0.0);
        double angular_vel = 0.3; // yaw
        stepper.setDesiredVelocity(linear_vel, angular_vel);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));

        std::vector<CoxaAngleCase> cases = {{-20.0}, {0.0}, {20.0}};
        struct Result {
            double coxa_deg;
            double target_dir;
            double swing_end_dir;
            double target_norm;
            double swing_end_norm;
            Point3D target;
            Point3D swing_end;
            Point3D tip_before;
        };
        std::vector<Result> results;

        // raw_target reference for this leg
        double stance_ratio = double(cycle.stance_period_) / double(cycle.period_);
        Point3D radius(identity_tip.x, identity_tip.y, 0.0);
        Point3D angular_component(-angular_vel * radius.y, angular_vel * radius.x, 0.0);
        Point3D stride_total = (linear_vel + angular_component) * (stance_ratio / cycle.frequency_);
        Point3D stride_half = stride_total * 0.5;
        Point3D reference_raw_target = identity_tip + stride_half;

        for (auto cs : cases) {
            JointAngles ja = leg.getJointAngles();
            ja.coxa = cs.coxa_deg;
            leg.setJointAngles(ja);
            stepper.setCurrentTipPose(leg.getCurrentTipPositionGlobal());
            stepper.setStepState(STEP_SWING);
            stepper.setPhase(cycle.swing_start_);
            stepper.updateTipPositionIterative(1, params.time_delta, false, false);
            Point3D target = stepper.getTargetTipPose();
            Point3D swing_end = stepper.getSwing2ControlNode(4);
            Point3D base = model.getLegBasePosition(leg_index);
            Point3D base_to_target(target.x - base.x, target.y - base.y, 0.0);
            Point3D base_to_swing_end(swing_end.x - base.x, swing_end.y - base.y, 0.0);
            results.push_back(Result{cs.coxa_deg, planarAngle(base_to_target), planarAngle(base_to_swing_end), planarNorm(base_to_target), planarNorm(base_to_swing_end), target, swing_end, leg.getCurrentTipPositionGlobal()});
        }

        // Per-leg report
        std::cout << "\n[Leg " << leg_index << "] base_theta(deg)=" << math_utils::radiansToDegrees(base_theta)
                  << " raw_target=(" << reference_raw_target.x << ", " << reference_raw_target.y << ")" << "\n";
        const double DEG = math_utils::radiansToDegrees(1.0);
        if (!results.empty()) {
            double base_dir = results[0].target_dir;
            std::cout << "  Coxa(deg) | dTarget(deg) | dSwingEnd(deg) | target_norm | swing_norm\n";
            for (auto &r : results) {
                double dtheta_target = (r.target_dir - base_dir) * DEG;
                double dtheta_swing_end = (r.swing_end_dir - base_dir) * DEG;
                std::cout << "  " << std::setw(8) << r.coxa_deg
                          << " | " << std::setw(11) << dtheta_target
                          << " | " << std::setw(13) << dtheta_swing_end
                          << " | " << std::setw(11) << r.target_norm
                          << " | " << std::setw(10) << r.swing_end_norm
                          << "\n"
                          << "    target=(" << r.target.x << ", " << r.target.y << ")"
                          << " swing_end=(" << r.swing_end.x << ", " << r.swing_end.y << ")"
                          << "\n";
            }
        }

        double max_delta_target_deg = 0.0, max_delta_swing_end_deg = 0.0;
        for (size_t i = 1; i < results.size(); ++i) {
            double dt = std::fabs(math_utils::radiansToDegrees(results[i].target_dir - results[0].target_dir));
            double ds = std::fabs(math_utils::radiansToDegrees(results[i].swing_end_dir - results[0].swing_end_dir));
            if (dt > max_delta_target_deg)
                max_delta_target_deg = dt;
            if (ds > max_delta_swing_end_deg)
                max_delta_swing_end_deg = ds;
        }
        leg_summaries.push_back(LegSummary{leg_index, max_delta_target_deg, max_delta_swing_end_deg});
    }

    // Global summary
    const double THRESH = 1e-3; // 0.001 deg
    std::cout << "\n=== Global Summary ===\n";
    for (auto &ls : leg_summaries) {
        std::cout << "Leg " << ls.leg
                  << ": dTargetMax=" << ls.max_delta_target_deg
                  << " deg, dSwingEndMax=" << ls.max_delta_swing_end_deg
                  << " deg ->"
                  << (ls.max_delta_target_deg <= THRESH ? " target INV" : " target VAR")
                  << (ls.max_delta_swing_end_deg <= THRESH ? ", swing INV" : ", swing VAR")
                  << "\n";
    }
    bool all_invariant = true;
    for (auto &ls : leg_summaries) {
        if (ls.max_delta_target_deg > THRESH || ls.max_delta_swing_end_deg > THRESH) {
            all_invariant = false;
            break;
        }
    }
    if (all_invariant)
        std::cout << "Conclusion: target_tip_pose and swing_end are invariant to coxa angle in all legs.\n";
    else
        std::cout << "Conclusion: Variation detected in at least one leg.\n";
    std::cout << "Threshold (deg): " << THRESH << "\n";

    return 0;
}