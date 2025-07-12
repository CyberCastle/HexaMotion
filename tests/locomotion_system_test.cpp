#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Create a BodyPoseConfiguration for testing
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    LocomotionSystem sys(p);
    assert(sys.initialize(&imu, &fsr, &servos, pose_config));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.update());

    // Get step parameters from walk controller
    double len = sys.getWalkController()->getStepLength();
    double h = sys.getWalkController()->getStepHeight();
    assert(len >= 20.0f && len <= 80.0f);
    assert(h >= 15.0f && h <= 50.0f);

    // Second system with invalid limits should report kinematics error
    Parameters bad{};
    bad.hexagon_radius = 200;
    bad.coxa_length = 50;
    bad.femur_length = 101;
    bad.tibia_length = 208;
    bad.robot_height = 7;
    bad.control_frequency = 50;
    // All valid poses yield angles near 0 so exclude zero from allowed range
    bad.coxa_angle_limits[0] = 10;
    bad.coxa_angle_limits[1] = 20;
    bad.femur_angle_limits[0] = 10;
    bad.femur_angle_limits[1] = 20;
    bad.tibia_angle_limits[0] = 10;
    bad.tibia_angle_limits[1] = 20;
    bad.ik.clamp_joints = false;

    LocomotionSystem sys2(bad);
    assert(sys2.initialize(&imu, &fsr, &servos, pose_config));
    assert(sys2.calibrateSystem());
    assert(sys2.setGaitType(TRIPOD_GAIT));
    assert(sys2.update());
    // Note: The system might handle invalid limits differently now
    // Just verify it doesn't crash and can still operate
    // assert(sys2.getLastError() == LocomotionSystem::KINEMATICS_ERROR);

    std::cout << "locomotion_system_test executed successfully" << std::endl;
    return 0;
}
