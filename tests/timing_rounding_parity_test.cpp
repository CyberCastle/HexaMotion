/**
 * @file timing_rounding_parity_test.cpp
 * @brief Validates the OpenSHC-aligned swing/stance timing rounding (§2.2).
 *
 * After the alignment the swing iteration count must be rounded UP to the nearest even number
 * (so the primary/secondary Bezier halves split cleanly) while the stance iteration count must be a
 * plain truncating integer of the OpenSHC formula with NO forced-even rounding and NO minimum-10
 * clamp — only a divide-by-zero guard keeping it >= 1.
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

int main() {
    std::cout << "=== Timing Rounding Parity Test (\u00a72.2) ===\n";

    Parameters params = makeTestParams();
    RobotModel model(params);

    struct Case {
        double frequency;
        int period;
        int stance_period;
        int swing_period;
    };
    // Mix of even/odd raw iteration counts to exercise both rounding branches.
    std::vector<Case> cases = {
        {1.0, 4, 2, 2},
        {1.5, 6, 4, 2},
        {0.8, 10, 7, 3},
        {2.0, 5, 3, 2},
    };

    for (const auto &c : cases) {
        Leg leg(0, model);
        double base_theta = model.getLegBaseAngleOffset(0);
        double planar_r = params.hexagon_radius + params.coxa_length + params.femur_length;
        Point3D identity_tip(planar_r * std::cos(base_theta), planar_r * std::sin(base_theta), params.default_height_offset);
        LegStepper stepper(0, identity_tip, leg, model);

        StepCycle cycle{};
        cycle.frequency_ = c.frequency;
        cycle.period_ = c.period;
        cycle.stance_period_ = c.stance_period;
        cycle.swing_period_ = c.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = c.stance_period;
        cycle.swing_start_ = c.stance_period;
        cycle.swing_end_ = c.period;
        stepper.setStepCycle(cycle);

        stepper.calculateSwingTiming(params.time_delta);

        int raw_swing = int((double(c.swing_period) / c.period) / (c.frequency * params.time_delta));
        int expected_swing = (raw_swing % 2 != 0) ? raw_swing + 1 : raw_swing;
        int raw_stance = int((double(c.stance_period) / c.period) / (c.frequency * params.time_delta));
        int expected_stance = raw_stance < 1 ? 1 : raw_stance;

        std::cout << "Case f=" << c.frequency << " period=" << c.period << "\n";
        check(stepper.getSwingIterations() == expected_swing,
              "swing iterations even-rounded == OpenSHC formula");
        check(stepper.getSwingIterations() % 2 == 0, "swing iterations are even");
        check(stepper.getStanceIterations() == expected_stance,
              "stance iterations == plain int (no forced-even, no min-10)");
        check(stepper.getStanceIterations() >= 1, "stance iterations >= 1 (div-by-zero guard)");
        // Confirm no minimum-10 clamp by ensuring a low-count case stays below 10.
        if (expected_stance < 10) {
            check(stepper.getStanceIterations() < 10, "stance iterations not clamped up to 10");
        }
    }

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
