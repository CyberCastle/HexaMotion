#include "../src/pose_config_factory.h"
#include "../src/pose_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

/* Context (from AGENTS.md):
# AGENT Instructions

This file defines the guidelines for contributing to HexaMotion.

## Objective

This library provides locomotion control for a hexapod robot based on the Arduino Giga R1. The robot body forms a hexagon with legs spaced 60° apart, each leg having three joints for 3DOF. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

**New Feature**: HexaMotion now includes **OpenSHC-style dynamic pose configuration** that uses parameter-based configuration from pose_config.h instead of hardcoded values. This provides flexibility for different operational modes (default, conservative, high_speed) and matches OpenSHC's approach to stance positioning and body clearance.

## Test Parameters

robot height: 120 mm
robot weight: 6.5 Kg
body hexagon radius: 200 mm.
coxa length: 50 mm
femur length: 101 mm
tibia length: 208 mm

Use the following `Parameters` configuration in the test files:

```cpp
Parameters p{};
p.hexagon_radius = 200;
p.coxa_length = 50;
p.femur_length = 101;
p.tibia_length = 208;
p.robot_height = 120;
p.robot_weight = 6.5;
p.control_frequency = 50;
p.coxa_angle_limits[0] = -65;
p.coxa_angle_limits[1] = 65;
p.femur_angle_limits[0] = -75;
p.femur_angle_limits[1] = 75;
p.tibia_angle_limits[0] = -45;
p.tibia_angle_limits[1] = 45;
```
*/

int main() {
    // Test parameters with robot weight for OpenSHC-style configuration
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 120;
    p.robot_weight = 6.5; // Required for OpenSHC-style pose limit calculations
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    DummyServo servos;

    // Create default configuration using factory (proper approach)
    PoseConfiguration default_config = getDefaultPoseConfig(p);
    PoseController pc(model, &servos, default_config);

    // Test OpenSHC-style configuration modes
    std::cout << "=== Testing OpenSHC-style Pose Configuration ===" << std::endl;

    // Test current default configuration
    const auto &current_default = pc.getPoseConfig();
    std::cout << "Default config - Body clearance: " << current_default.body_clearance * 1000.0f << "mm, "
              << "Swing height: " << current_default.swing_height * 1000.0f << "mm" << std::endl;
    std::cout << "Max translation: X=" << current_default.max_translation.x * 1000.0f << "mm, "
              << "Y=" << current_default.max_translation.y * 1000.0f << "mm, "
              << "Z=" << current_default.max_translation.z * 1000.0f << "mm" << std::endl;

    // Test conservative configuration using factory
    PoseConfiguration conservative_config = getConservativePoseConfig(p);
    pc.setPoseConfig(conservative_config);
    const auto &current_conservative = pc.getPoseConfig();
    std::cout << "Conservative config - Body clearance: " << current_conservative.body_clearance * 1000.0f << "mm, "
              << "Swing height: " << current_conservative.swing_height * 1000.0f << "mm" << std::endl;

    // Test high-speed configuration using factory
    PoseConfiguration high_speed_config = getHighSpeedPoseConfig(p);
    pc.setPoseConfig(high_speed_config);
    const auto &current_high_speed = pc.getPoseConfig();
    std::cout << "High-speed config - Body clearance: " << current_high_speed.body_clearance * 1000.0f << "mm, "
              << "Swing height: " << current_high_speed.swing_height * 1000.0f << "mm" << std::endl;

    // Reset to default for pose tests using factory
    pc.setPoseConfig(getDefaultPoseConfig(p));

    // Disable smooth trajectory for deterministic tests
    pc.configureSmoothTrajectory(false);

    Point3D legs[NUM_LEGS];
    JointAngles joints[NUM_LEGS];

    std::cout << "\n=== Testing OpenSHC-style Pose Calculations ===" << std::endl;

    // Test default pose using OpenSHC-style configuration
    pc.initializeDefaultPose(legs, joints);
    std::cout << "Default Pose Leg Positions (using OpenSHC config):" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "Leg " << i << ": X=" << legs[i].x << "mm, Y=" << legs[i].y << "mm, Z=" << legs[i].z << "mm" << std::endl;
    }

    // Test standing pose
    if (!pc.setStandingPose(legs, joints)) {
        std::cerr << "Error: setStandingPose failed" << std::endl;
        return -1;
    }
    std::cout << "\nStanding Pose Joint Angles (degrees):" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "Leg " << i << ": Coxa=" << joints[i].coxa << ", Femur=" << joints[i].femur << ", Tibia=" << joints[i].tibia << std::endl;
    }
    // Test body pose with limits (OpenSHC-style limit checking)
    std::cout << "\n=== Testing OpenSHC-style Pose Limits ===" << std::endl;

    Eigen::Vector3f small_translation(5.0f, 5.0f, 0.0f); // 5mm translation
    Eigen::Vector3f small_rotation(2.0f, 2.0f, 2.0f);    // 2 degree rotation

    if (pc.setBodyPose(small_translation, small_rotation, legs, joints)) {
        std::cout << "Small body pose change: SUCCESS (within limits)" << std::endl;
    } else {
        std::cout << "Small body pose change: FAILED (unexpected)" << std::endl;
    }

    // Test pose that should exceed limits
    Eigen::Vector3f large_translation(100.0f, 100.0f, 100.0f); // 100mm translation (should exceed limits)
    Eigen::Vector3f large_rotation(30.0f, 30.0f, 30.0f);       // 30 degree rotation (should exceed limits)

    if (!pc.setBodyPose(large_translation, large_rotation, legs, joints)) {
        std::cout << "Large body pose change: CORRECTLY REJECTED (exceeds limits)" << std::endl;
    } else {
        std::cout << "Large body pose change: INCORRECTLY ACCEPTED (should exceed limits)" << std::endl;
    }

    std::cout << "\npose_controller_test executed successfully with OpenSHC-style configuration" << std::endl;
    return 0;
}
