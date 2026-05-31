/**
 * @file leg_stepper_accessor_test.cpp
 * @brief Coverage for LegStepper getters/setters and safety helpers ([J] in TODO_TEST_COVERAGE).
 *
 * The existing trajectory/force-normal-touchdown/strict-parity tests exercise the
 * core trajectory pipeline. This test fills the remaining gaps: the gait-parameter
 * setters/getters, stance-span modifier, phase/step-state accessors, the workspace
 * safety helpers (constrainToWorkspace, calculateSafeTarget, calculateSafeStride,
 * validateTargetTipPose, validateCurrentTrajectory) and the phase-transition helpers
 * (beginSwingPhase, beginStancePhase, iteratePhase, updateStride).
 */

#include "leg.h"
#include "leg_stepper.h"
#include "robot_model.h"
#include <cmath>
#include <iostream>

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

static Parameters makeParams() {
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

int main() {
    std::cout << "=== LegStepper Accessor Coverage Test ([J]) ===\n";

    Parameters params = makeParams();
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
    stepper.setDesiredVelocity(Point3D(60.0, 20.0, 0.0), 0.0);
    stepper.calculateSwingTiming(params.time_delta);

    // Identity/index accessors.
    check(stepper.getLegIndex() == 0, "getLegIndex");

    // Gait-parameter setters/getters.
    stepper.setSwingWidth(12.0);
    stepper.setStepClearanceHeight(25.0);
    stepper.setStepDepth(8.0);
    stepper.setStanceSpanModifier(0.3);
    check(std::fabs(stepper.getSwingWidth() - 12.0) < 1e-9, "swing width setter/getter");
    check(std::fabs(stepper.getStepClearanceHeight() - 25.0) < 1e-9, "step clearance setter/getter");
    check(std::fabs(stepper.getStepDepth() - 8.0) < 1e-9, "step depth setter/getter");
    check(std::fabs(stepper.getStanceSpanModifier() - 0.3) < 1e-9, "stance span modifier setter/getter");

    // StepCycle round-trip.
    const StepCycle &rc = stepper.getStepCycle();
    check(rc.period_ == cycle.period_, "step cycle round-trip");

    // Phase/step-state accessors.
    stepper.setPhase(cycle.swing_start_);
    check(stepper.getPhase() == cycle.swing_start_, "phase setter/getter");
    stepper.setStepState(STEP_SWING);
    check(stepper.getStepState() == STEP_SWING, "step state setter/getter");
    stepper.setStepProgress(0.5);
    check(std::fabs(stepper.getStepProgress() - 0.5) < 1e-9, "step progress setter/getter");
    stepper.setPhaseOffset(2);
    check(stepper.getPhaseOffset() == 2, "phase offset setter/getter");

    // Walk-plane normal accessor.
    Point3D wpn = stepper.getWalkPlaneNormal();
    check(std::fabs(wpn.z - 1.0) < 1e-9, "walk plane normal accessor");

    // Drive one swing iteration to populate stride/target/nodes/velocity.
    stepper.setCurrentTipPose(leg.getCurrentTipPositionGlobal());
    stepper.updateTipPositionIterative(1, params.time_delta, false, false);

    Point3D stride = stepper.getStrideVector();
    check(std::isfinite(stride.x) && std::isfinite(stride.y), "stride vector finite after iteration");
    Point3D tip_vel = stepper.getCurrentTipVelocity();
    check(std::isfinite(tip_vel.x), "current tip velocity finite");
    check(std::isfinite(stepper.getDesiredAngularVelocity()), "desired angular velocity accessor");
    Point3D dlv = stepper.getDesiredLinearVelocity();
    check(std::isfinite(dlv.x), "desired linear velocity accessor");

    // Timing accessors.
    check(stepper.getSwingIterations() > 0, "swing iterations positive");
    check(stepper.getStanceIterations() > 0, "stance iterations positive");
    check(stepper.getSwingDeltaT() > 0.0, "swing delta-t positive");
    check(stepper.getStanceDeltaT() > 0.0, "stance delta-t positive");

    // Workspace safety helpers.
    Point3D safe_stride = stepper.calculateSafeStride(Point3D(500.0, 0.0, 0.0));
    check(std::isfinite(safe_stride.x), "calculateSafeStride finite");
    Point3D reachable_target = identity_tip;
    Point3D constrained = stepper.constrainToWorkspace(reachable_target);
    check(std::isfinite(constrained.x), "constrainToWorkspace finite");
    Point3D safe_target = stepper.calculateSafeTarget(reachable_target);
    check(std::isfinite(safe_target.x), "calculateSafeTarget finite");
    bool valid_target = stepper.validateTargetTipPose(reachable_target);
    check(valid_target, "validateTargetTipPose accepts reachable target");
    bool traj_ok = stepper.validateCurrentTrajectory();
    (void)traj_ok; // Result depends on live trajectory; call exercises the path.

    // Phase-transition helpers.
    stepper.updateStride();
    stepper.beginSwingPhase();
    stepper.beginStancePhase();
    stepper.updateStepStateFromPhase();
    int before = stepper.getPhase();
    stepper.iteratePhase();
    check(stepper.getPhase() == (before + 1) % cycle.period_, "iteratePhase increments and wraps");

    // Tip-pose accessors.
    check(std::isfinite(stepper.getCurrentTipPose().x), "current tip pose accessor");
    check(std::isfinite(stepper.getDefaultTipPose().x), "default tip pose accessor");
    check(std::isfinite(stepper.getIdentityTipPose().x), "identity tip pose accessor");
    check(std::isfinite(stepper.getTargetTipPose().x), "target tip pose accessor");

    if (g_failures == 0) {
        std::cout << "\nALL TESTS PASSED\n";
        return 0;
    }
    std::cout << "\n"
              << g_failures << " CHECK(S) FAILED\n";
    return 1;
}
