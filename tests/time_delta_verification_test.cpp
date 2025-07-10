#include "../src/leg_poser.h"
#include "../src/robot_model.h"
#include "../src/leg.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "=== Time Delta Verification Test ===" << std::endl;

    // Test different control frequencies
    std::vector<double> test_frequencies = {25.0, 50.0, 100.0, 200.0};

    for (double frequency : test_frequencies) {
        std::cout << "\n--- Testing Control Frequency: " << frequency << " Hz ---" << std::endl;

        // Initialize robot model with specific frequency
        Parameters p{};
        p.hexagon_radius = 200;
        p.coxa_length = 50;
        p.femur_length = 101;
        p.tibia_length = 208;
        p.robot_height = 120;
        p.control_frequency = frequency;
        p.coxa_angle_limits[0] = -65;
        p.coxa_angle_limits[1] = 65;
        p.femur_angle_limits[0] = -75;
        p.femur_angle_limits[1] = 75;
        p.tibia_angle_limits[0] = -45;
        p.tibia_angle_limits[1] = 45;

        RobotModel model(p);

        // Create leg object
        Leg leg(0, model);
        leg.initialize(model, Pose::Identity());
        leg.updateForwardKinematics(model);

        // Create LegPoser
        LegPoser leg_poser(0, leg, model);

        // Calculate expected time_delta
        double expected_time_delta = 1.0 / frequency;
        double expected_iterations = 2.0 / expected_time_delta; // For 2 second step

        std::cout << "Expected time_delta: " << expected_time_delta << "s" << std::endl;
        std::cout << "Expected iterations for 2s step: " << expected_iterations << std::endl;

        // Test stepToPosition to see actual iterations used
        Point3D target_pos(50, 0, -120);
        double step_height = 30;
        double step_time = 2.0;

        int iterations = 0;
        bool step_complete = false;
        int max_iterations = 1000; // Safety limit

        while (!step_complete && iterations < max_iterations) {
            step_complete = leg_poser.stepToPosition(target_pos, step_height, step_time);
            iterations++;
        }

        std::cout << "Actual iterations used: " << iterations << std::endl;
        std::cout << "Actual time_delta (calculated): " << (step_time / iterations) << "s" << std::endl;

        // Verify the calculation is correct
        double actual_time_delta = step_time / iterations;
        double error = std::abs(actual_time_delta - expected_time_delta);
        double error_percent = (error / expected_time_delta) * 100.0;

        std::cout << "Error: " << error << "s (" << error_percent << "%)" << std::endl;

        bool test_passed = error_percent < 5.0; // Within 5% tolerance
        std::cout << "Test result: " << (test_passed ? "PASSED" : "FAILED") << std::endl;

        if (!test_passed) {
            std::cout << "ERROR: Time delta calculation is incorrect!" << std::endl;
            return 1;
        }
    }

    std::cout << "\n=== All Tests Passed ===" << std::endl;
    std::cout << "Time delta calculation is working correctly for all tested frequencies!" << std::endl;
    std::cout << "- 25 Hz: time_delta = 0.04s" << std::endl;
    std::cout << "- 50 Hz: time_delta = 0.02s" << std::endl;
    std::cout << "- 100 Hz: time_delta = 0.01s" << std::endl;
    std::cout << "- 200 Hz: time_delta = 0.005s" << std::endl;

    return 0;
}