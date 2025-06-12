#include "Arduino.h"
#include "HexaModel.h"
#include <iostream>

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

    // Initialize DH parameters to zero (will use default)
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            for (int k = 0; k < 4; ++k) {
                params.dh_parameters[l][j][k] = 0.0f;
            }
        }
    }

    RobotModel model(params);

    std::cout << "=== DLS IK-FK Consistency Test ===" << std::endl;
    std::cout << "Testing multiple challenging configurations..." << std::endl;

    // Test configurations
    JointAngles test_configs[] = {
        {0.0f, -45.0f, 90.0f},   // Original failing case
        {30.0f, -30.0f, 60.0f},  // Moderate angles
        {-45.0f, 45.0f, -90.0f}, // Negative angles
        {0.0f, -60.0f, 120.0f},  // More extreme bending
        {90.0f, 0.0f, 0.0f},     // Pure rotation
        {0.0f, 0.0f, 0.0f},      // Straight leg
        {-30.0f, -75.0f, 135.0f} // Near limits
    };

    int num_tests = sizeof(test_configs) / sizeof(test_configs[0]);
    int passed = 0;
    float max_error = 0.0f;
    float total_error = 0.0f;

    for (int i = 0; i < num_tests; i++) {
        JointAngles original = test_configs[i];

        // Forward kinematics
        Point3D target = model.forwardKinematics(0, original);

        // Inverse kinematics
        JointAngles ik_result = model.inverseKinematics(0, target);

        // Verify with forward kinematics
        Point3D fk_verify = model.forwardKinematics(0, ik_result);

        // Calculate error
        float error = sqrt(pow(target.x - fk_verify.x, 2) +
                           pow(target.y - fk_verify.y, 2) +
                           pow(target.z - fk_verify.z, 2));

        total_error += error;
        if (error > max_error)
            max_error = error;

        bool success = error < 5.0f; // 5mm tolerance
        if (success)
            passed++;

        std::cout << "\nTest " << (i + 1) << ": ";
        std::cout << "(" << original.coxa << "°, " << original.femur << "°, " << original.tibia << "°)";
        std::cout << " -> (" << target.x << ", " << target.y << ", " << target.z << ")";
        std::cout << "\nIK: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)";
        std::cout << "\nFK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")";
        std::cout << "\nError: " << error << "mm " << (success ? "✓" : "✗") << std::endl;
    }

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Tests passed: " << passed << "/" << num_tests << std::endl;
    std::cout << "Success rate: " << (passed * 100.0f / num_tests) << "%" << std::endl;
    std::cout << "Average error: " << (total_error / num_tests) << "mm" << std::endl;
    std::cout << "Maximum error: " << max_error << "mm" << std::endl;

    if (passed == num_tests) {
        std::cout << "\n🎉 All tests passed! DLS IK implementation is working correctly." << std::endl;
    } else {
        std::cout << "\n⚠️  Some tests failed. Further investigation needed." << std::endl;
    }

    return (passed == num_tests) ? 0 : 1;
}
