#include "../src/body_pose_controller.h"
#include "../src/body_pose_config_factory.h"
#include "../src/robot_model.h"
#include "../src/leg.h"
#include <iostream>
#include <cassert>

/**
 * @file pose_limits_test.cpp
 * @brief Test to verify that body pose limits use configuration instead of hardcoded values
 */

int main() {
    std::cout << "=== Testing Body Pose Limits Configuration ===" << std::endl;

    // Create robot model and pose configuration
    Parameters params;
    RobotModel model(params);
    BodyPoseConfiguration config = getDefaultPoseConfig(params);

    // Create body pose controller
    BodyPoseController controller(model, config);

    // Test 1: Check that limits are read from configuration
    std::cout << "Configuration limits:" << std::endl;
    std::cout << "  Max translation: X=" << config.max_translation.x
              << "mm, Y=" << config.max_translation.y
              << "mm, Z=" << config.max_translation.z << "mm" << std::endl;
    std::cout << "  Max rotation: Roll=" << config.max_rotation.roll
              << "rad, Pitch=" << config.max_rotation.pitch
              << "rad, Yaw=" << config.max_rotation.yaw << "rad" << std::endl;

    // Test 2: Small pose change within limits should pass
    Eigen::Vector3d small_position(10.0, 5.0, -10.0); // Within 25mm limits
    Eigen::Vector3d small_orientation(5.0, 3.0, 2.0); // Within 0.25rad limits (≈14.3°)

    bool small_pose_valid = controller.checkBodyPoseLimits(small_position, small_orientation);
    std::cout << "Small pose test: " << (small_pose_valid ? "PASS" : "FAIL") << std::endl;
    assert(small_pose_valid && "Small pose should be within limits");

    // Test 3: Large pose change exceeding limits should fail
    Eigen::Vector3d large_position(50.0, 30.0, -50.0); // Exceeds 25mm limits
    Eigen::Vector3d large_orientation(20.0, 15.0, 30.0); // Exceeds 0.25rad limits

    bool large_pose_valid = controller.checkBodyPoseLimits(large_position, large_orientation);
    std::cout << "Large pose test: " << (large_pose_valid ? "FAIL" : "PASS") << std::endl;
    assert(!large_pose_valid && "Large pose should exceed limits");

    // Test 4: Edge case - exactly at limits should pass
    Eigen::Vector3d edge_position(config.max_translation.x, config.max_translation.y, -config.max_translation.z);
    Eigen::Vector3d edge_orientation(
        math_utils::radiansToDegrees(config.max_rotation.roll),
        math_utils::radiansToDegrees(config.max_rotation.pitch),
        math_utils::radiansToDegrees(config.max_rotation.yaw)
    );

    bool edge_pose_valid = controller.checkBodyPoseLimits(edge_position, edge_orientation);
    std::cout << "Edge case test: " << (edge_pose_valid ? "PASS" : "FAIL") << std::endl;
    assert(edge_pose_valid && "Edge case should be within limits");

    // Test 5: Test different configuration types
    std::cout << "\n=== Testing Different Configuration Types ===" << std::endl;

    BodyPoseConfiguration conservative_config = getConservativePoseConfig(params);
    BodyPoseController conservative_controller(model, conservative_config);

    // Conservative config should have reduced limits
    Eigen::Vector3d conservative_test_position(20.0, 20.0, -20.0); // Should be within default but exceed conservative
    Eigen::Vector3d conservative_test_orientation(10.0, 10.0, 10.0);

    bool default_valid = controller.checkBodyPoseLimits(conservative_test_position, conservative_test_orientation);
    bool conservative_valid = conservative_controller.checkBodyPoseLimits(conservative_test_position, conservative_test_orientation);

    std::cout << "Default config allows pose: " << (default_valid ? "YES" : "NO") << std::endl;
    std::cout << "Conservative config allows pose: " << (conservative_valid ? "YES" : "NO") << std::endl;

    // Conservative should be more restrictive
    assert(default_valid && "Default config should allow this pose");
    assert(!conservative_valid && "Conservative config should reject this pose");

    std::cout << "\n=== All Tests Passed! ===" << std::endl;
    std::cout << "✓ Body pose limits correctly use configuration values" << std::endl;
    std::cout << "✓ No hardcoded values in checkBodyPoseLimits" << std::endl;
    std::cout << "✓ Different configuration types work correctly" << std::endl;

    return 0;
}