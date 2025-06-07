#include <cassert>
#include "test_stubs.h"
#include "../include/pose_controller.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 100;
    p.coxa_length = 30;
    p.femur_length = 50;
    p.tibia_length = 70;
    p.robot_height = 100;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -180; p.coxa_angle_limits[1] = 180;
    p.femur_angle_limits[0] = -180; p.femur_angle_limits[1] = 180;
    p.tibia_angle_limits[0] = -180; p.tibia_angle_limits[1] = 180;

    RobotModel model(p);
    DummyServo servos;
    PoseController pc(model, &servos);
    Point3D legs[NUM_LEGS];
    JointAngles joints[NUM_LEGS];
    pc.initializeDefaultPose(legs, joints, p.hexagon_radius, p.robot_height);
    assert(pc.setStandingPose(legs, joints, p.robot_height));
    assert(pc.setCrouchPose(legs, joints, p.robot_height));
    return 0;
}
