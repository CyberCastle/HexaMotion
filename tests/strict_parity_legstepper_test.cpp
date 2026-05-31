/**
 * @file strict_parity_legstepper_test.cpp
 * @brief Validates the master strict_openshc_parity switch in LegStepper (§1 / item 2.1).
 *
 * When strict_openshc_parity == true, HexaMotion reproduces OpenSHC's LegStepper trajectory pipeline
 * verbatim: NO stride/target freezing, NO hybrid anti-drift, NO phase-end snap, NO in-gait workspace
 * clamping and NO swing-end Z snapping — even if the fine-grained flags request them.
 *
 * Behavioural discriminator used here: under default (extensions on) the swing target FREEZES after
 * the first swing iteration, so a mid-swing velocity change does NOT move the target. Under parity the
 * target is recomputed live every iteration, so the same velocity change DOES move the target.
 */

#include "leg_stepper.h"
#include "robot_model.h"
#include <cmath>
#include <iostream>

static Parameters baseParams() {
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

static StepCycle makeCycle() {
    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 8;
    cycle.stance_period_ = 4;
    cycle.swing_period_ = 4;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 4;
    cycle.swing_start_ = 4;
    cycle.swing_end_ = 8;
    return cycle;
}

// Returns the planar shift of the swing target between two iterations when the velocity command is
// changed between them. Frozen (default) -> ~0; live (parity) -> non-zero.
static double targetShiftOnVelocityChange(bool parity) {
    Parameters params = baseParams();
    params.strict_openshc_parity = parity;
    // Fine-grained flags requested ON to prove parity overrides them.
    params.enable_phase_end_snap = true;
    params.enable_workspace_constrain = true;

    RobotModel model(params);
    Leg leg(0, model);
    double base_theta = model.getLegBaseAngleOffset(0);
    double planar_r = params.hexagon_radius + params.coxa_length + params.femur_length;
    Point3D identity_tip(planar_r * std::cos(base_theta), planar_r * std::sin(base_theta), params.default_height_offset);
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle = makeCycle();
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.calculateSwingTiming(params.time_delta);
    stepper.setCurrentTipPose(leg.getCurrentTipPositionGlobal());
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(cycle.swing_start_);
    stepper.beginSwingPhase();

    stepper.setDesiredVelocity(Point3D(60.0, 0.0, 0.0), 0.0);
    stepper.updateTipPositionIterative(1, params.time_delta, false, false);
    Point3D t1 = stepper.getTargetTipPose();

    // Change the command mid-swing.
    stepper.setDesiredVelocity(Point3D(-60.0, 40.0, 0.0), 0.0);
    stepper.updateTipPositionIterative(2, params.time_delta, false, false);
    Point3D t2 = stepper.getTargetTipPose();

    double dx = t2.x - t1.x, dy = t2.y - t1.y;
    return std::sqrt(dx * dx + dy * dy);
}

int main() {
    std::cout << "=== Strict OpenSHC Parity LegStepper Test (item 2.1) ===\n";

    double shift_default = targetShiftOnVelocityChange(false);
    double shift_parity = targetShiftOnVelocityChange(true);

    std::cout << "target shift (default, extensions on) = " << shift_default << " mm\n";
    std::cout << "target shift (strict parity)          = " << shift_parity << " mm\n";

    check(shift_default < 1e-6, "default: swing target FROZEN after iteration 1 (no shift)");
    check(shift_parity > 1.0, "parity: swing target recomputed LIVE (shifts with velocity)");

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
