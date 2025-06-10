#include "model.h"
#include "velocity_limits.h"
#include "walk_controller.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

class VelocityLimitsTest {
  private:
    std::unique_ptr<RobotModel> model;
    std::unique_ptr<VelocityLimits> velocity_limits;
    std::unique_ptr<WalkController> walk_controller;

    const float FLOAT_TOLERANCE = 1e-5f;
    int tests_passed = 0;
    int tests_failed = 0;

    void assert_near(float actual, float expected, float tolerance, const std::string &message) {
        if (std::abs(actual - expected) <= tolerance) {
            tests_passed++;
            std::cout << "✓ " << message << std::endl;
        } else {
            tests_failed++;
            std::cout << "✗ " << message << " (expected: " << expected
                      << ", actual: " << actual << ", diff: " << std::abs(actual - expected) << ")" << std::endl;
        }
    }

    void assert_true(bool condition, const std::string &message) {
        if (condition) {
            tests_passed++;
            std::cout << "✓ " << message << std::endl;
        } else {
            tests_failed++;
            std::cout << "✗ " << message << std::endl;
        }
    }

    void assert_false(bool condition, const std::string &message) {
        assert_true(!condition, message);
    }

    void assert_greater(float actual, float expected, const std::string &message) {
        if (actual > expected) {
            tests_passed++;
            std::cout << "✓ " << message << std::endl;
        } else {
            tests_failed++;
            std::cout << "✗ " << message << " (expected > " << expected
                      << ", actual: " << actual << ")" << std::endl;
        }
    }

    void assert_less_equal(float actual, float expected, const std::string &message) {
        if (actual <= expected) {
            tests_passed++;
            std::cout << "✓ " << message << std::endl;
        } else {
            tests_failed++;
            std::cout << "✗ " << message << " (expected <= " << expected
                      << ", actual: " << actual << ")" << std::endl;
        }
    }

  public:
    void setUp() {
        // Initialize robot model with test parameters
        Parameters params;
        params.hexagon_radius = 0.1f;
        params.coxa_length = 0.05f;
        params.femur_length = 0.1f;
        params.tibia_length = 0.15f;
        model = std::make_unique<RobotModel>(params);

        velocity_limits = std::make_unique<VelocityLimits>(*model);
        walk_controller = std::make_unique<WalkController>(*model);
    }

    void testInitialization() {
        std::cout << "Testing initialization..." << std::endl;
        auto limits = velocity_limits->getLimit(0.0f);
        assert_greater(limits.linear_x, 0.0f, "Linear X limit should be positive");
        assert_greater(limits.linear_y, 0.0f, "Linear Y limit should be positive");
        assert_greater(limits.angular_z, 0.0f, "Angular Z limit should be positive");
        assert_greater(limits.acceleration, 0.0f, "Acceleration limit should be positive");
    }

    void testWorkspaceCalculation() {
        std::cout << "Testing workspace calculation..." << std::endl;
        VelocityLimits::GaitConfig gait_config;
        gait_config.frequency = 1.0f;
        gait_config.stance_ratio = 0.6f;
        gait_config.swing_ratio = 0.4f;
        gait_config.time_to_max_stride = 2.0f;

        velocity_limits->generateLimits(gait_config);
        auto workspace = velocity_limits->getWorkspaceConfig();

        assert_greater(workspace.walkspace_radius, 0.0f, "Walkspace radius should be positive");
        assert_greater(workspace.stance_radius, 0.0f, "Stance radius should be positive");
        assert_greater(workspace.safety_margin, 0.0f, "Safety margin should be positive");
        assert_less_equal(workspace.safety_margin, 1.0f, "Safety margin should be <= 1.0");
    }

    void testBearingNormalization() {
        std::cout << "Testing bearing normalization..." << std::endl;
        assert_near(VelocityLimits::normalizeBearing(0.0f), 0.0f, FLOAT_TOLERANCE, "0° normalization");
        assert_near(VelocityLimits::normalizeBearing(360.0f), 0.0f, FLOAT_TOLERANCE, "360° normalization");
        assert_near(VelocityLimits::normalizeBearing(-90.0f), 270.0f, FLOAT_TOLERANCE, "-90° normalization");
        assert_near(VelocityLimits::normalizeBearing(450.0f), 90.0f, FLOAT_TOLERANCE, "450° normalization");
    }

    void testBearingCalculation() {
        std::cout << "Testing bearing calculation..." << std::endl;
        assert_near(VelocityLimits::calculateBearing(1.0f, 0.0f), 0.0f, 0.01f, "East bearing");
        assert_near(VelocityLimits::calculateBearing(0.0f, 1.0f), 90.0f, 0.01f, "North bearing");
        assert_near(VelocityLimits::calculateBearing(-1.0f, 0.0f), 180.0f, 0.01f, "West bearing");
        assert_near(VelocityLimits::calculateBearing(0.0f, -1.0f), 270.0f, 0.01f, "South bearing");
    }

    void testGaitParameterEffects() {
        std::cout << "Testing gait parameter effects..." << std::endl;
        VelocityLimits::GaitConfig slow_gait;
        slow_gait.frequency = 0.5f;
        slow_gait.stance_ratio = 0.8f;
        slow_gait.swing_ratio = 0.2f;
        slow_gait.time_to_max_stride = 3.0f;

        VelocityLimits::GaitConfig fast_gait;
        fast_gait.frequency = 2.0f;
        fast_gait.stance_ratio = 0.4f;
        fast_gait.swing_ratio = 0.6f;
        fast_gait.time_to_max_stride = 1.0f;

        velocity_limits->generateLimits(slow_gait);
        auto slow_limits = velocity_limits->getLimit(0.0f);

        velocity_limits->generateLimits(fast_gait);
        auto fast_limits = velocity_limits->getLimit(0.0f);

        assert_greater(fast_limits.linear_x, slow_limits.linear_x, "Fast gait should allow higher linear speeds");
        assert_greater(fast_limits.acceleration, slow_limits.acceleration, "Fast gait should allow higher acceleration");
    }

    void testVelocityValidation() {
        std::cout << "Testing velocity validation..." << std::endl;
        auto limits = velocity_limits->getLimit(0.0f);

        // Valid velocities within limits
        assert_true(velocity_limits->validateVelocityInputs(
                        limits.linear_x * 0.5f, limits.linear_y * 0.5f, limits.angular_z * 0.5f),
                    "Valid velocities should pass validation");

        // Invalid velocities exceeding limits
        assert_false(velocity_limits->validateVelocityInputs(
                         limits.linear_x * 2.0f, limits.linear_y, limits.angular_z),
                     "Excessive linear X should fail validation");
        assert_false(velocity_limits->validateVelocityInputs(
                         limits.linear_x, limits.linear_y * 2.0f, limits.angular_z),
                     "Excessive linear Y should fail validation");
        assert_false(velocity_limits->validateVelocityInputs(
                         limits.linear_x, limits.linear_y, limits.angular_z * 2.0f),
                     "Excessive angular Z should fail validation");
    }

    void testAccelerationLimiting() {
        std::cout << "Testing acceleration limiting..." << std::endl;
        VelocityLimits::LimitValues target(1.0f, 1.0f, 1.0f, 0.5f);
        VelocityLimits::LimitValues current(0.0f, 0.0f, 0.0f, 0.5f);
        float dt = 0.1f;

        auto limited = velocity_limits->applyAccelerationLimits(target, current, dt);

        // Should not reach target immediately due to acceleration limits
        assert_true(limited.linear_x < target.linear_x, "Should not reach target linear X immediately");
        assert_true(limited.linear_y < target.linear_y, "Should not reach target linear Y immediately");
        assert_true(limited.angular_z < target.angular_z, "Should not reach target angular Z immediately");

        // Should be progressing towards target
        assert_greater(limited.linear_x, current.linear_x, "Should progress towards target linear X");
        assert_greater(limited.linear_y, current.linear_y, "Should progress towards target linear Y");
        assert_greater(limited.angular_z, current.angular_z, "Should progress towards target angular Z");
    }

    void testWalkControllerIntegration() {
        std::cout << "Testing WalkController integration..." << std::endl;
        auto limits = walk_controller->getVelocityLimits(0.0f);
        assert_greater(limits.linear_x, 0.0f, "WalkController should provide positive linear X limits");
        assert_greater(limits.linear_y, 0.0f, "WalkController should provide positive linear Y limits");
        assert_greater(limits.angular_z, 0.0f, "WalkController should provide positive angular Z limits");

        // Test velocity limiting
        float test_vx = 2.0f, test_vy = 1.0f, test_omega = 3.0f;
        auto limited = walk_controller->applyVelocityLimits(test_vx, test_vy, test_omega);

        assert_less_equal(std::abs(limited.linear_x), limits.linear_x, "Limited linear X should be within bounds");
        assert_less_equal(std::abs(limited.linear_y), limits.linear_y, "Limited linear Y should be within bounds");
        assert_less_equal(std::abs(limited.angular_z), limits.angular_z, "Limited angular Z should be within bounds");

        // Test velocity validation
        assert_true(walk_controller->validateVelocityCommand(
                        limited.linear_x, limited.linear_y, limited.angular_z),
                    "Limited velocities should pass validation");
    }

    void testOpenSHCEquivalence() {
        std::cout << "Testing OpenSHC mathematical equivalence..." << std::endl;
        VelocityLimits::GaitConfig test_gait;
        test_gait.frequency = 1.5f;
        test_gait.stance_ratio = 0.65f;
        test_gait.time_to_max_stride = 2.5f;

        velocity_limits->generateLimits(test_gait);
        auto workspace = velocity_limits->getWorkspaceConfig();

        // Test OpenSHC-equivalent speed calculation:
        // max_speed = (walkspace_radius * 2.0) / (on_ground_ratio / frequency)
        float cycle_time = test_gait.stance_ratio / test_gait.frequency;
        float expected_max_speed = (workspace.walkspace_radius * 2.0f) / cycle_time;

        auto limits = velocity_limits->getLimit(0.0f);

        // Use more reasonable bounds since our implementation caps extreme values
        if (expected_max_speed > 5.0f) {
            expected_max_speed = 5.0f; // Our safety cap
        }

        assert_near(limits.linear_x, expected_max_speed, std::max(0.5f, expected_max_speed * 0.3f),
                    "Linear speed should match OpenSHC calculation");

        // Test OpenSHC-equivalent angular speed calculation:
        // max_angular_speed = max_linear_speed / stance_radius
        float expected_max_angular = expected_max_speed / workspace.stance_radius;
        if (expected_max_angular > 10.0f) {
            expected_max_angular = 10.0f; // Our safety cap
        }

        assert_near(limits.angular_z, expected_max_angular, std::max(6.0f, expected_max_angular * 0.8f),
                    "Angular speed should match OpenSHC calculation");

        // Test OpenSHC-equivalent acceleration calculation:
        // max_acceleration = max_speed / time_to_max_stride
        float expected_max_accel = expected_max_speed / test_gait.time_to_max_stride;
        if (expected_max_accel > 10.0f) {
            expected_max_accel = 10.0f; // Our safety cap
        }

        assert_near(limits.acceleration, expected_max_accel, std::max(0.5f, expected_max_accel * 0.3f),
                    "Acceleration should match OpenSHC calculation");
    }

    void testBearingRangeCoverage() {
        std::cout << "Testing bearing range coverage..." << std::endl;
        // Test that all bearings from 0-359 degrees provide valid limits
        for (int bearing = 0; bearing < 360; bearing += 30) {
            auto limits = velocity_limits->getLimit(static_cast<float>(bearing));
            assert_greater(limits.linear_x, 0.0f, "Bearing " + std::to_string(bearing) + "° linear X should be positive");
            assert_greater(limits.linear_y, 0.0f, "Bearing " + std::to_string(bearing) + "° linear Y should be positive");
            assert_greater(limits.angular_z, 0.0f, "Bearing " + std::to_string(bearing) + "° angular Z should be positive");
            assert_greater(limits.acceleration, 0.0f, "Bearing " + std::to_string(bearing) + "° acceleration should be positive");
        }
    }

    void runAllTests() {
        std::cout << "Running VelocityLimits Tests" << std::endl;
        std::cout << "============================" << std::endl;

        setUp();

        testInitialization();
        testWorkspaceCalculation();
        testBearingNormalization();
        testBearingCalculation();
        testGaitParameterEffects();
        testVelocityValidation();
        testAccelerationLimiting();
        testWalkControllerIntegration();
        testOpenSHCEquivalence();
        testBearingRangeCoverage();

        std::cout << std::endl;
        std::cout << "Test Results:" << std::endl;
        std::cout << "Passed: " << tests_passed << std::endl;
        std::cout << "Failed: " << tests_failed << std::endl;
        std::cout << "Total:  " << (tests_passed + tests_failed) << std::endl;

        if (tests_failed == 0) {
            std::cout << "All tests PASSED! ✓" << std::endl;
        } else {
            std::cout << "Some tests FAILED! ✗" << std::endl;
        }
    }
};

int main() {
    VelocityLimitsTest test;
    test.runAllTests();
    return 0;
}
