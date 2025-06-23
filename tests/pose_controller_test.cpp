#include "../src/pose_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

/* Context (from AGENTS.md):
# AGENT Instructions

This file defines the guidelines for contributing to HexaMotion.

## Objective

This library provides locomotion control for a hexapod robot based on the Arduino Giga R1. The robot body forms a hexagon with legs spaced 60° apart, each leg having three joints for 3DOF. It includes inverse kinematics using DH parameters and Jacobians, orientation and pose control, gait planning and error handling. The interfaces `IIMUInterface`, `IFSRInterface` and `IServoInterface` must be implemented to connect the IMU, FSR sensors and smart servos.

**New Feature**: HexaMotion now includes **smooth trajectory interpolation** that uses current servo positions as starting points for pose calculations, equivalent to OpenSHC's smooth movement approach. This is the default behavior and provides natural, smooth robot movements that optimize from the current state rather than theoretical positions.

## Test Parameters

robot height: 100 mm
robot weight: 6.5 Kg
body hexagon radius: 400 mm.
coxa length: 50 mm
femur length: 101 mm
tibia length: 208 mm

Use the following `Parameters` configuration in the test files:

```cpp
Parameters p{};
p.hexagon_radius = 400;
p.coxa_length = 50;
p.femur_length = 101;
p.tibia_length = 208;
p.robot_height = 100;
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
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 100;
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
    PoseController pc(model, &servos);
    // Disable smooth trajectory for deterministic tests
    pc.configureSmoothTrajectory(false);
    Point3D legs[NUM_LEGS];
    JointAngles joints[NUM_LEGS];
    pc.initializeDefaultPose(legs, joints, p.hexagon_radius, p.robot_height);
    if (!pc.setStandingPose(legs, joints, p.robot_height)) {
        std::cerr << "Error: setStandingPose failed" << std::endl;
        return -1;
    }
    std::cout << "Standing Pose Joint Angles (degrees):" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "Leg " << i << ": Coxa=" << joints[i].coxa << ", Femur=" << joints[i].femur << ", Tibia=" << joints[i].tibia << std::endl;
    }
    if (!pc.setCrouchPose(legs, joints, p.robot_height)) {
        std::cerr << "Error: setCrouchPose failed" << std::endl;
        return -1;
    }
    std::cout << "Crouch Pose Joint Angles (degrees):" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "Leg " << i << ": Coxa=" << joints[i].coxa << ", Femur=" << joints[i].femur << ", Tibia=" << joints[i].tibia << std::endl;
    }
    std::cout << "pose_controller_test executed successfully" << std::endl;
    return 0;
}
