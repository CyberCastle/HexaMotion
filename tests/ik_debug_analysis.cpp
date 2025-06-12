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

    std::cout << "=== Debugging Failed IK Cases ===" << std::endl;

    // Test the failing cases
    JointAngles failing_configs[] = {
        {-45.0f, 45.0f, -90.0f}, // Test 3
        {90.0f, 0.0f, 0.0f}      // Test 5
    };

    for (int i = 0; i < 2; i++) {
        JointAngles config = failing_configs[i];
        std::cout << "\n=== Analyzing Config " << (i + 1) << ": ("
                  << config.coxa << "°, " << config.femur << "°, " << config.tibia << "°) ===" << std::endl;

        // Check if original config is within joint limits
        bool within_limits = model.checkJointLimits(0, config);
        std::cout << "Original config within limits: " << (within_limits ? "YES" : "NO") << std::endl;

        // Forward kinematics
        Point3D target = model.forwardKinematics(0, config);
        std::cout << "FK result: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        // Check workspace reachability
        float distance_from_base = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
        float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
        float min_reach = abs(params.femur_length - params.tibia_length);
        std::cout << "Distance from base: " << distance_from_base << "mm" << std::endl;
        std::cout << "Max reach: " << max_reach << "mm, Min reach: " << min_reach << "mm" << std::endl;
        std::cout << "Within workspace: " << (distance_from_base <= max_reach && distance_from_base >= min_reach ? "YES" : "NO") << std::endl;

        // Try IK
        JointAngles ik_result = model.inverseKinematics(0, target);
        std::cout << "IK result: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;

        // Check if IK result is within limits
        bool ik_within_limits = model.checkJointLimits(0, ik_result);
        std::cout << "IK result within limits: " << (ik_within_limits ? "YES" : "NO") << std::endl;

        // Verify with FK
        Point3D fk_verify = model.forwardKinematics(0, ik_result);
        std::cout << "FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;

        float error = sqrt(pow(target.x - fk_verify.x, 2) +
                           pow(target.y - fk_verify.y, 2) +
                           pow(target.z - fk_verify.z, 2));
        std::cout << "Error: " << error << "mm" << std::endl;
    }

    std::cout << "\n=== Testing Edge Cases for DLS Robustness ===" << std::endl;

    // Test some positions that should work well
    Point3D test_positions[] = {
        {600.0f, 100.0f, -100.0f}, // Moderate reach
        {500.0f, 200.0f, 0.0f},    // Different angle
        {450.0f, 0.0f, -50.0f}     // Straight ahead
    };

    for (int i = 0; i < 3; i++) {
        Point3D target = test_positions[i];
        std::cout << "\nTesting position: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        JointAngles ik_result = model.inverseKinematics(0, target);
        Point3D fk_verify = model.forwardKinematics(0, ik_result);

        float error = sqrt(pow(target.x - fk_verify.x, 2) +
                           pow(target.y - fk_verify.y, 2) +
                           pow(target.z - fk_verify.z, 2));

        std::cout << "IK: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;
        std::cout << "FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;
        std::cout << "Error: " << error << "mm " << (error < 5.0f ? "✓" : "✗") << std::endl;
    }

    return 0;
}
