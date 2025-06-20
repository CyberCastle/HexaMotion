#include "HexaModel.h"
#include "math_utils.h"
#include <iostream>

int main() {
    // Configure test parameters (same as DLS validation test)
    Parameters params;
    params.hexagon_radius = 400.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 101.0f; // Updated to match other tests
    params.tibia_length = 208.0f; // Updated to match other tests

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

    std::cout << "=== Analyzing Test 3 Failure ===" << std::endl;

    // The failing configuration
    JointAngles config(-45.0f, 45.0f, -90.0f);
    std::cout << "Original config: (" << config.coxa << "°, " << config.femur << "°, " << config.tibia << "°)" << std::endl;

    // Check if this configuration is valid
    bool within_limits = model.checkJointLimits(0, config);
    std::cout << "Within joint limits: " << (within_limits ? "YES" : "NO") << std::endl;

    // Forward kinematics to get target
    Point3D target = model.forwardKinematics(0, config);
    std::cout << "Target position: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

    // Check workspace bounds
    float base_x = 400.0f; // Leg 0 base
    float base_y = 0.0f;
    float local_x = target.x - base_x;
    float local_y = target.y - base_y;
    float local_z = target.z;
    float distance = sqrt(local_x * local_x + local_y * local_y + local_z * local_z);

    float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float min_reach = abs(params.femur_length - params.tibia_length);

    std::cout << "Local position relative to leg base: (" << local_x << ", " << local_y << ", " << local_z << ")" << std::endl;
    std::cout << "Distance from leg base: " << distance << "mm" << std::endl;
    std::cout << "Max theoretical reach: " << max_reach << "mm" << std::endl;
    std::cout << "Min theoretical reach: " << min_reach << "mm" << std::endl;
    std::cout << "Within reach bounds: " << (distance <= max_reach && distance >= min_reach ? "YES" : "NO") << std::endl;

    if (distance > max_reach) {
        std::cout << "⚠️  Target is " << (distance - max_reach) << "mm beyond max reach" << std::endl;
    }

    // Try IK
    std::cout << "\n=== IK Analysis ===" << std::endl;
    JointAngles ik_result = model.inverseKinematics(0, target);
    std::cout << "IK result: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;

    bool ik_within_limits = model.checkJointLimits(0, ik_result);
    std::cout << "IK result within limits: " << (ik_within_limits ? "YES" : "NO") << std::endl;

    // Verify with FK
    Point3D fk_verify = model.forwardKinematics(0, ik_result);
    std::cout << "FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;

    float error = sqrt(pow(target.x - fk_verify.x, 2) +
                       pow(target.y - fk_verify.y, 2) +
                       pow(target.z - fk_verify.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    // This configuration may have multiple solutions - let's see if the original is correct
    std::cout << "\n=== Solution Equivalence Check ===" << std::endl;

    // Check if the original and IK result reach the same position
    Point3D original_fk = model.forwardKinematics(0, config);
    Point3D ik_fk = model.forwardKinematics(0, ik_result);

    float position_diff = sqrt(pow(original_fk.x - ik_fk.x, 2) +
                               pow(original_fk.y - ik_fk.y, 2) +
                               pow(original_fk.z - ik_fk.z, 2));

    std::cout << "Original FK: (" << original_fk.x << ", " << original_fk.y << ", " << original_fk.z << ")" << std::endl;
    std::cout << "IK FK: (" << ik_fk.x << ", " << ik_fk.y << ", " << ik_fk.z << ")" << std::endl;
    std::cout << "Position difference: " << position_diff << "mm" << std::endl;

    // This is likely a case where there are multiple valid solutions
    if (position_diff < 5.0f) {
        std::cout << "✅ IK found a valid alternative solution!" << std::endl;
    } else {
        std::cout << "❌ IK failed to find correct solution" << std::endl;
    }

    return 0;
}
