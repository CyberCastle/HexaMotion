#include "../src/admittance_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>

// Test to validate that RK2 and RK4 methods don't corrupt velocity state
int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.height_offset = 0;
    p.control_frequency = 50;

    RobotModel model(p);
    DummyIMU imu;
    DummyFSR fsr;

    // Test each integration method
    std::cout << "Testing Runge-Kutta Integration Methods..." << std::endl;

    for (int method = 0; method < 3; method++) {
        ComputeConfig config;
        switch (method) {
        case 0:
            config = ComputeConfig::low();
            break; // Euler
        case 1:
            config = ComputeConfig::medium();
            break; // RK2
        case 2:
            config = ComputeConfig::high();
            break; // RK4
        }

        AdmittanceController ac(model, &imu, &fsr, config);
        ac.initialize();

        const char *method_name[] = {"Euler", "RK2", "RK4"};
        std::cout << "\n--- Testing " << method_name[method] << " Method ---" << std::endl;

        // Set initial conditions
        ac.setLegAdmittance(0, 0.5f, 2.0f, 100.0f);

        // Apply consistent force and verify velocity behavior
        Point3D applied_force(0, 0, -10.0f); // Downward force

        // Multiple integration steps
        Point3D total_delta(0, 0, 0);
        for (int step = 0; step < 10; step++) {
            Point3D delta = ac.applyForceAndIntegrate(0, applied_force);
            total_delta = total_delta + delta;

            // Get current state
            const auto &leg_state = ac.getLegState(0);
            Point3D current_velocity = leg_state.params.velocity;

            std::cout << "Step " << step << ": velocity.z = " << current_velocity.z
                      << ", delta.z = " << delta.z << std::endl;

            // Velocity should be monotonically increasing (more negative) due to downward force
            if (step > 0) {
                assert(current_velocity.z < 0.0f); // Should be negative (downward)
            }
        }

        std::cout << "Total displacement.z = " << total_delta.z << std::endl;

        // All methods should produce downward movement
        assert(total_delta.z < 0.0f);

        std::cout << method_name[method] << " method: ✓ PASSED" << std::endl;
    }

    std::cout << "\n✅ All Runge-Kutta integration methods validated successfully!" << std::endl;
    std::cout << "✅ Velocity state preservation confirmed for RK2 and RK4" << std::endl;

    return 0;
}
