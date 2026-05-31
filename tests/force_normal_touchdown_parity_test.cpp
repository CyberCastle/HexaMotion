/**
 * @file force_normal_touchdown_parity_test.cpp
 * @brief Validates the OpenSHC-aligned forceNormalTouchdown node-separation formula (§4.bis-D2).
 *
 * After the fix, the secondary-swing control-node separation must equal
 *   sep = stride_vector * (-(stance_delta_t / time_delta)) * 0.25 * (time_delta / swing_delta_t)
 * and the reshaped node swing_2_nodes_[2] must equal target_tip_pose - 2 * sep, matching OpenSHC's
 * generateSecondarySwingControlNodes spacing (not the previous simplified -1/stance_iterations factor).
 */

#include "leg_stepper.h"
#include "robot_model.h"
#include <cmath>
#include <iostream>

static Parameters makeTestParams() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    p.enable_workspace_constrain = false; // keep raw nodes to test the formula directly
    return p;
}

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

static bool approxEqual(const Point3D &a, const Point3D &b, double eps = 1e-6) {
    return std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps && std::fabs(a.z - b.z) < eps;
}

int main() {
    std::cout << "=== forceNormalTouchdown Parity Test (\u00a74.bis-D2) ===\n";

    Parameters params = makeTestParams();
    RobotModel model(params);

    Leg leg(0, model);
    double base_theta = model.getLegBaseAngleOffset(0);
    double planar_r = params.hexagon_radius + params.coxa_length + params.femur_length;
    Point3D identity_tip(planar_r * std::cos(base_theta), planar_r * std::sin(base_theta), params.default_height_offset);
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 4;
    cycle.stance_period_ = 2;
    cycle.swing_period_ = 2;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 2;
    cycle.swing_start_ = 2;
    cycle.swing_end_ = 4;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.setDesiredVelocity(Point3D(80.0, 30.0, 0.0), 0.0);
    stepper.calculateSwingTiming(params.time_delta);

    // Drive one swing iteration to populate stride/target/nodes.
    stepper.setCurrentTipPose(leg.getCurrentTipPositionGlobal());
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(cycle.swing_start_);
    stepper.updateTipPositionIterative(1, params.time_delta, false, false);

    // Now force a vertical touchdown reshape and verify the node spacing formula.
    stepper.testForceNormalTouchdown();

    Point3D stride = stepper.getStrideVector();
    Point3D target = stepper.getTargetTipPose();
    double stance_dt = stepper.getStanceDeltaT();
    double swing_dt = stepper.getSwingDeltaT();

    Point3D final_tip_velocity = stride * (-(stance_dt / params.time_delta));
    Point3D expected_sep = final_tip_velocity * 0.25 * (params.time_delta / swing_dt);
    Point3D expected_node2 = target - expected_sep * 2.0;

    Point3D actual_node2 = stepper.getSwing2ControlNode(2);

    std::cout << "stride=(" << stride.x << "," << stride.y << ") stance_dt=" << stance_dt
              << " swing_dt=" << swing_dt << "\n";
    std::cout << "expected_node2=(" << expected_node2.x << "," << expected_node2.y << "," << expected_node2.z
              << ") actual_node2=(" << actual_node2.x << "," << actual_node2.y << "," << actual_node2.z << ")\n";

    check(std::fabs(expected_sep.x) > 1e-9 || std::fabs(expected_sep.y) > 1e-9,
          "node separation is non-trivial (stride applied)");
    check(approxEqual(expected_node2, actual_node2, 1e-4),
          "swing_2_nodes_[2] == target - 2*sep (OpenSHC formula)");

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
