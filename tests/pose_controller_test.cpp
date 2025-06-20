#include "../src/pose_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 90;
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
    assert(pc.setStandingPose(legs, joints, p.robot_height));
    assert(pc.setCrouchPose(legs, joints, p.robot_height));
    std::cout << "pose_controller_test executed successfully" << std::endl;
    return 0;
}
