/**
 * @file cartesian_velocity_controller_test.cpp
 * @brief Dedicated coverage for CartesianVelocityController (TODO_TEST_COVERAGE [A]).
 *
 * Exercises the public API and, through updateServoSpeeds(), the private
 * scaling/compensation/workspace-constraint paths that previously had 0% coverage:
 *   - updateServoSpeeds / getServoSpeed / getLegServoSpeeds
 *   - calculateLinearVelocityScale / calculateAngularVelocityScale (via updateServoSpeeds)
 *   - calculateGaitSpeedAdjustment / calculateLegSpeedCompensation (via updateServoSpeeds)
 *   - applyWorkspaceConstraints (via updateServoSpeeds)
 *   - getCurrentVelocityMagnitude / setVelocityControlEnabled / isVelocityControlEnabled
 *   - setVelocityScaling / setGaitSpeedModifiers / resetToDefaults
 */

#include "../src/cartesian_velocity_controller.h"
#include "../src/robot_model.h"
#include <cassert>
#include <cmath>
#include <iostream>

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
    std::cout << "=== CartesianVelocityController Test ===\n";

    RobotModel model(makeParams());
    CartesianVelocityController controller(model);

    // Velocity control is enabled by default.
    assert(controller.isVelocityControlEnabled());

    // Drive several gait types so calculateGaitSpeedAdjustment covers every branch.
    const GaitType gaits[] = {TRIPOD_GAIT, WAVE_GAIT, RIPPLE_GAIT, METACHRONAL_GAIT};
    for (GaitType gait : gaits) {
        bool ok = controller.updateServoSpeeds(80.0, -20.0, 0.3, gait);
        assert(ok);
        // Each leg/joint should now have a valid effective speed within bounds.
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const auto &speeds = controller.getLegServoSpeeds(leg);
            double eff = speeds.coxa.getEffectiveSpeed();
            assert(eff >= SERVO_SPEED_MIN - 1e-9 && eff <= SERVO_SPEED_MAX + 1e-9);
            for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
                double s = controller.getServoSpeed(leg, joint);
                assert(s > 0.0);
            }
        }
    }

    // Magnitude combines the last commanded linear velocity (sqrt(80^2 + 20^2))
    // with a weighted angular contribution, so it must be at least the linear part.
    double linear_mag = std::sqrt(80.0 * 80.0 + 20.0 * 20.0);
    assert(controller.getCurrentVelocityMagnitude() >= linear_mag - 1.0);

    // Pure angular command (zero linear) covers the angular-only scaling branch.
    assert(controller.updateServoSpeeds(0.0, 0.0, 1.0, TRIPOD_GAIT));

    // Zero command covers the minimum-speed-ratio branch.
    assert(controller.updateServoSpeeds(0.0, 0.0, 0.0, TRIPOD_GAIT));

    // Custom scaling and gait modifiers should be accepted and re-applied.
    CartesianVelocityController::VelocityScaling scaling;
    scaling.linear_velocity_scale = 0.02;
    scaling.angular_velocity_scale = 0.5;
    scaling.enable_adaptive_scaling = false; // exercise non-adaptive compensation path
    controller.setVelocityScaling(scaling);

    CartesianVelocityController::GaitSpeedModifiers modifiers;
    modifiers.tripod_speed_factor = 1.2;
    controller.setGaitSpeedModifiers(modifiers);
    assert(controller.updateServoSpeeds(50.0, 50.0, 0.1, TRIPOD_GAIT));

    // Disable velocity control: updateServoSpeeds should short-circuit gracefully.
    controller.setVelocityControlEnabled(false);
    assert(!controller.isVelocityControlEnabled());
    controller.updateServoSpeeds(80.0, 0.0, 0.0, TRIPOD_GAIT);

    // Re-enable and reset to defaults.
    controller.setVelocityControlEnabled(true);
    controller.resetToDefaults();
    double s = controller.getServoSpeed(0, 0);
    assert(s > 0.0);

    std::cout << "\nALL TESTS PASSED\n";
    return 0;
}
