#include "math_utils.h"
#include "model.h"
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    // Configure test parameters
    Parameters params;
    params.hexagon_radius = 400.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 100.0f;
    params.tibia_length = 159.0f;

    // Angle limits for joints
    params.coxa_angle_limits[0] = -180.0f;
    params.coxa_angle_limits[1] = 180.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -180.0f;
    params.tibia_angle_limits[1] = 180.0f;

    // Initialize DH parameters
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            for (int k = 0; k < 4; ++k) {
                params.dh_parameters[l][j][k] = 0.0f;
            }
        }
    }

    RobotModel model(params);

    std::cout << "=== Comprehensive IK-FK Consistency Test ===" << std::endl;
    std::cout << "Testing realistic joint configurations..." << std::endl;

    // Test a range of realistic joint configurations
    std::vector<JointAngles> realistic_configs = {
        // Basic poses
        {0.0f, -30.0f, 60.0f},  // Mild bend
        {0.0f, -45.0f, 90.0f},  // Standard bend (was our original test)
        {0.0f, -60.0f, 120.0f}, // Deep bend

        // Angled poses
        {30.0f, -30.0f, 60.0f},  // Side + mild bend
        {45.0f, -45.0f, 90.0f},  // Side + standard bend
        {-30.0f, -30.0f, 60.0f}, // Other side + mild bend
        {-45.0f, -45.0f, 90.0f}, // Other side + standard bend

        // Extended poses
        {0.0f, -15.0f, 30.0f},   // Extended forward
        {90.0f, -15.0f, 30.0f},  // Extended side
        {180.0f, -15.0f, 30.0f}, // Extended back
        {-90.0f, -15.0f, 30.0f}, // Extended other side

        // Retracted poses
        {0.0f, -75.0f, 150.0f},  // Retracted forward
        {90.0f, -75.0f, 150.0f}, // Retracted side

        // Edge cases within limits
        {0.0f, -89.0f, 179.0f},   // Near joint limits
        {179.0f, -89.0f, 179.0f}, // Multiple limits
    };

    int total_tests = realistic_configs.size();
    int passed_tests = 0;
    float total_error = 0.0f;
    float max_error = 0.0f;
    float error_threshold = 5.0f; // 5mm tolerance

    std::cout << "\nTesting " << total_tests << " configurations..." << std::endl;

    for (int i = 0; i < realistic_configs.size(); i++) {
        JointAngles config = realistic_configs[i];

        // Forward kinematics to get target
        Point3D target = model.forwardKinematics(0, config);

        // Inverse kinematics back to joint angles
        JointAngles ik_result = model.inverseKinematics(0, target);

        // Forward kinematics to verify
        Point3D fk_verify = model.forwardKinematics(0, ik_result);

        // Calculate error
        float error = sqrt(pow(target.x - fk_verify.x, 2) +
                           pow(target.y - fk_verify.y, 2) +
                           pow(target.z - fk_verify.z, 2));

        bool passed = error < error_threshold;
        if (passed)
            passed_tests++;

        total_error += error;
        max_error = std::max(max_error, error);

        // Detailed output for failing cases
        if (!passed || error > 1.0f) {
            std::cout << "\nTest " << (i + 1) << ": ("
                      << config.coxa << "°, " << config.femur << "°, " << config.tibia << "°)" << std::endl;
            std::cout << "  Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
            std::cout << "  IK result: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;
            std::cout << "  FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;
            std::cout << "  Error: " << error << "mm " << (passed ? "✓" : "✗") << std::endl;
        } else {
            std::cout << "Test " << (i + 1) << ": " << error << "mm ✓" << std::endl;
        }
    }

    float success_rate = (float)passed_tests / total_tests * 100.0f;
    float avg_error = total_error / total_tests;

    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Tests passed: " << passed_tests << "/" << total_tests << std::endl;
    std::cout << "Success rate: " << success_rate << "%" << std::endl;
    std::cout << "Average error: " << avg_error << "mm" << std::endl;
    std::cout << "Maximum error: " << max_error << "mm" << std::endl;

    if (success_rate >= 95.0f) {
        std::cout << "✅ Excellent! IK-FK consistency is very good." << std::endl;
    } else if (success_rate >= 85.0f) {
        std::cout << "✅ Good! IK-FK consistency is acceptable." << std::endl;
    } else if (success_rate >= 70.0f) {
        std::cout << "⚠️  Fair. Some edge cases need attention." << std::endl;
    } else {
        std::cout << "❌ Poor. IK algorithm needs improvement." << std::endl;
    }

    return success_rate >= 85.0f ? 0 : 1;
}
