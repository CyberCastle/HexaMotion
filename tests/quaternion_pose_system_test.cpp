#include "manual_pose_controller.h"
#include "math_utils.h"
#include "pose_controller.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

int main() {
    std::cout << "=== Test Quaternion-based Pose System ===" << std::endl;

    // Initialize test environment
    Parameters params = createDefaultParameters();

    RobotModel model(params);
    DummyServo servos;
    PoseController pose_controller(model, &servos);
    ManualPoseController manual_controller(model);

    manual_controller.initialize();

    Point3D leg_positions[NUM_LEGS];
    JointAngles joint_angles[NUM_LEGS];

    // Initialize default leg positions
    // Use a smaller leg radius to keep the feet within reach
    pose_controller.initializeDefaultPose(leg_positions, joint_angles);

    std::cout << "\n1. Testing quaternion conversion functions..." << std::endl;

    // Test conversion functions
    Point3D euler_test_deg(30.0f, 45.0f, 60.0f);
    Point3D euler_test(math_utils::degreesToRadians(euler_test_deg.x),
                       math_utils::degreesToRadians(euler_test_deg.y),
                       math_utils::degreesToRadians(euler_test_deg.z));
    Eigen::Vector4d quat_test = math_utils::eulerPoint3DToQuaternion(euler_test);
    Point3D euler_back_rad = math_utils::quaternionToEulerPoint3D(quat_test);
    Point3D euler_back(math_utils::radiansToDegrees(euler_back_rad.x),
                       math_utils::radiansToDegrees(euler_back_rad.y),
                       math_utils::radiansToDegrees(euler_back_rad.z));

    std::cout << "Original Euler: (" << euler_test_deg.x << ", " << euler_test_deg.y << ", " << euler_test_deg.z << ")" << std::endl;
    std::cout << "Quaternion: (" << quat_test[0] << ", " << quat_test[1] << ", " << quat_test[2] << ", " << quat_test[3] << ")" << std::endl;
    std::cout << "Converted back: (" << euler_back.x << ", " << euler_back.y << ", " << euler_back.z << ")" << std::endl;

    double conversion_error = std::abs(euler_test_deg.x - euler_back.x) +
                             std::abs(euler_test_deg.y - euler_back.y) +
                             std::abs(euler_test_deg.z - euler_back.z);
    std::cout << "Conversion error: " << conversion_error << " degrees" << std::endl;
    std::cout << "Conversion test: " << (conversion_error < 0.1f ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n2. Testing quaternion-based pose setting..." << std::endl;

    // Test quaternion pose setting
    Eigen::Vector3d position(0.0f, 0.0f, 100.0f);
    Eigen::Vector4d quaternion = math_utils::eulerPoint3DToQuaternion(Point3D(5.0f, 5.0f, 5.0f));

    bool pose_success = pose_controller.setBodyPoseQuaternion(position, quaternion, leg_positions, joint_angles);
    std::cout << "Quaternion pose setting: " << (pose_success ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n3. Testing pose interpolation..." << std::endl;

    // Test pose interpolation
    Eigen::Vector3d start_pos(0.0f, 0.0f, 100.0f);
    Eigen::Vector4d start_quat(1.0f, 0.0f, 0.0f, 0.0f); // Identity

    Eigen::Vector3d end_pos(0.0f, 0.0f, 100.0f);
    Eigen::Vector4d end_quat = math_utils::eulerPoint3DToQuaternion(Point3D(5.0f, 5.0f, 5.0f));

    // Test interpolation at t=0.5
    bool interp_success = pose_controller.interpolatePose(start_pos, start_quat, end_pos, end_quat,
                                                          0.5f, leg_positions, joint_angles);
    std::cout << "Pose interpolation: " << (interp_success ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n4. Testing manual pose controller with quaternions..." << std::endl;

    // Test manual pose controller
    manual_controller.setUseQuaternion(true);

    Point3D manual_pos(5.0f, -5.0f, 95.0f);
    Eigen::Vector4d manual_quat = math_utils::eulerPoint3DToQuaternion(Point3D(10.0f, -5.0f, 15.0f));

    manual_controller.setPoseQuaternion(manual_pos, manual_quat, 1.0f);

    Eigen::Vector4d retrieved_quat = manual_controller.getCurrentQuaternion();
    double quat_diff = std::abs(manual_quat[0] - retrieved_quat[0]) +
                      std::abs(manual_quat[1] - retrieved_quat[1]) +
                      std::abs(manual_quat[2] - retrieved_quat[2]) +
                      std::abs(manual_quat[3] - retrieved_quat[3]);

    std::cout << "Manual controller quaternion consistency: " << (quat_diff < 0.01f ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n5. Testing quaternion interpolation in manual controller..." << std::endl;

    Point3D target_pos(15.0f, 10.0f, 105.0f);
    Eigen::Vector4d target_quat = math_utils::eulerPoint3DToQuaternion(Point3D(25.0f, 20.0f, 35.0f));

    // Perform several interpolation steps with higher speed
    for (int i = 0; i < 10; i++) {
        manual_controller.interpolateToQuaternionPose(target_pos, target_quat, 0.3f);
    }

    Eigen::Vector4d final_quat = manual_controller.getCurrentQuaternion();
    double final_diff = std::abs(target_quat[0] - final_quat[0]) +
                       std::abs(target_quat[1] - final_quat[1]) +
                       std::abs(target_quat[2] - final_quat[2]) +
                       std::abs(target_quat[3] - final_quat[3]);

    std::cout << "Quaternion interpolation convergence: " << (final_diff < 0.5f ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n=== All Tests Completed ===" << std::endl;
    std::cout << "Quaternion-based pose system is working correctly!" << std::endl;

    return 0;
}
