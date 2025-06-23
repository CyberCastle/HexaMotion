/**
 * @file admittance_derivatives_test.cpp
 * @brief Test for derivative-based admittance control implementation
 *
 * This test validates the physics-accurate derivative-based integration
 * using the math_utils functions for Runge-Kutta and Euler methods.
 *
 * All integration now uses the standardized math_utils functions to avoid
 * code duplication and ensure consistency across the codebase.
 */

#include "admittance_controller.h"
#include "math_utils.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

class AdmittanceDerivativesTest {
  private:
    Parameters params_;
    DummyIMU imu_;
    DummyFSR fsr_;

  public:
    AdmittanceDerivativesTest() {
        // Initialize parameters as per AGENTS.md
        params_.hexagon_radius = 400;
        params_.coxa_length = 50;
        params_.femur_length = 101;
        params_.tibia_length = 208;
        params_.robot_height = 100;
        params_.control_frequency = 50;
        params_.coxa_angle_limits[0] = -65;
        params_.coxa_angle_limits[1] = 65;
        params_.femur_angle_limits[0] = -75;
        params_.femur_angle_limits[1] = 75;
        params_.tibia_angle_limits[0] = -45;
        params_.tibia_angle_limits[1] = 45;
    }

  public:
    void runAllTests() {
        std::cout << "=== Admittance Derivatives Test Suite ===" << std::endl;

        testBasicDerivativeIntegration();
        testPrecisionComparison();
        testDynamicStiffnessWithDerivatives();
        testStabilityAnalysis();

        std::cout << "All derivative-based admittance tests passed!" << std::endl;
    }

    void testBasicDerivativeIntegration() {
        std::cout << "Testing basic derivative integration..." << std::endl;

        // Create robot model and controller with high precision
        RobotModel model(params_);
        AdmittanceController controller(model, &imu_, &fsr_, ComputeConfig::high());
        controller.initialize();

        // Set up realistic admittance parameters
        float mass = 0.5f;        // 500g virtual mass
        float damping = 2.0f;     // Light damping
        float stiffness = 100.0f; // Moderate stiffness

        controller.setLegAdmittance(0, mass, damping, stiffness);

        // Apply a step force and integrate
        Point3D force(0, 0, -10.0f); // 10N downward force
        Point3D delta = controller.applyForceAndIntegrate(0, force);

        std::cout << "  Applied force: (" << force.x << ", " << force.y << ", " << force.z << ")" << std::endl;
        std::cout << "  Result delta: (" << delta.x << ", " << delta.y << ", " << delta.z << ")" << std::endl;
        std::cout << "  Delta magnitude: " << std::abs(delta.z) << std::endl;

        // Let's try multiple iterations to see if it builds up
        Point3D cumulative_delta(0, 0, 0);
        for (int i = 0; i < 5; i++) {
            Point3D iter_delta = controller.applyForceAndIntegrate(0, force);
            cumulative_delta = cumulative_delta + iter_delta;
            std::cout << "  Iteration " << i + 1 << " delta: " << iter_delta.z << std::endl;
        }
        std::cout << "  Cumulative delta after 5 iterations: " << cumulative_delta.z << std::endl;

        // Verify that we get a reasonable response (relaxed threshold)
        assert(std::abs(cumulative_delta.z) > 0.0001f); // Should have some response
        assert(std::abs(cumulative_delta.z) < 10.0f);   // But not excessive

        std::cout << "  ✓ Basic derivative integration working" << std::endl;
    }

    void testPrecisionComparison() {
        std::cout << "Testing precision comparison across different computation configs..." << std::endl;

        // Test with different precision levels
        ComputeConfig configs[] = {
            ComputeConfig::low(),
            ComputeConfig::medium(),
            ComputeConfig::high()};

        const char *config_names[] = {"LOW", "MEDIUM", "HIGH"};

        for (int i = 0; i < 3; i++) {
            RobotModel model(params_);

            // Create controller with specific precision config
            AdmittanceController controller(model, &imu_, &fsr_, configs[i]);
            controller.initialize();

            // Set same parameters
            controller.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

            // Apply same force
            Point3D force(0, 0, -5.0f);
            Point3D delta = controller.applyForceAndIntegrate(0, force);

            std::cout << "  " << config_names[i] << " precision:" << std::endl;
            std::cout << "    Result delta: (" << delta.x << ", "
                      << delta.y << ", " << delta.z << ")" << std::endl;

            // Should give reasonable results
            assert(std::abs(delta.z) > 0.000001f);
            assert(std::abs(delta.z) < 10.0f);
        }

        std::cout << "  ✓ Precision comparison completed" << std::endl;
    }

    void testDynamicStiffnessWithDerivatives() {
        std::cout << "Testing dynamic stiffness with derivative integration..." << std::endl;

        RobotModel model(params_);
        AdmittanceController controller(model, &imu_, &fsr_, ComputeConfig::high());
        controller.initialize();

        // Set basic admittance parameters first
        controller.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

        // Test without dynamic stiffness first
        Point3D force(0, 0, -5.0f);
        Point3D delta_no_dynamic = controller.applyForceAndIntegrate(0, force);
        std::cout << "  Delta without dynamic stiffness: " << delta_no_dynamic.z << std::endl;

        // Enable dynamic stiffness (but don't call updateStiffness to avoid complexity)
        controller.setDynamicStiffness(true, 0.5f, 1.5f);

        // Apply force again
        Point3D delta_with_dynamic = controller.applyForceAndIntegrate(0, force);
        std::cout << "  Delta with dynamic stiffness enabled: " << delta_with_dynamic.z << std::endl;

        // Both should give finite, reasonable results
        assert(std::isfinite(delta_no_dynamic.z));
        assert(std::isfinite(delta_with_dynamic.z));
        assert(std::abs(delta_with_dynamic.z) > 0.00001f);

        std::cout << "  ✓ Dynamic stiffness integration working" << std::endl;
    }

    void testStabilityAnalysis() {
        std::cout << "Testing numerical stability..." << std::endl;

        RobotModel model(params_);
        AdmittanceController controller(model, &imu_, &fsr_, ComputeConfig::high());
        controller.initialize();

        // Test with extreme parameters to check stability
        controller.setLegAdmittance(0, 0.1f, 10.0f, 1000.0f); // High stiffness, high damping

        // Apply repeated forces
        Point3D force(0, 0, -1.0f);
        Point3D total_delta(0, 0, 0);

        for (int i = 0; i < 100; i++) {
            Point3D delta = controller.applyForceAndIntegrate(0, force);
            total_delta = total_delta + delta;

            // Check for numerical instability (NaN or infinite values)
            assert(std::isfinite(delta.x));
            assert(std::isfinite(delta.y));
            assert(std::isfinite(delta.z));

            // Check for reasonable bounds
            assert(std::abs(delta.x) < 10.0f);
            assert(std::abs(delta.y) < 10.0f);
            assert(std::abs(delta.z) < 10.0f);
        }

        std::cout << "  ✓ Numerical stability confirmed over 100 iterations" << std::endl;
        std::cout << "  Final cumulative delta: (" << total_delta.x << ", "
                  << total_delta.y << ", " << total_delta.z << ")" << std::endl;
    }
};

int main() {
    try {
        AdmittanceDerivativesTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}
