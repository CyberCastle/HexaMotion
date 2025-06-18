#include "../src/locomotion_system.h"
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
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    LocomotionSystem sys(p);
    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.update());
    float len = sys.getStepLength();
    float h = sys.getStepHeight();
    assert(len >= 20.0f && len <= 80.0f);
    assert(h >= 15.0f && h <= 50.0f);

    // Second system with invalid limits should report kinematics error
    Parameters bad{};
    bad.hexagon_radius = 400;
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
    assert(sys2.initialize(&imu, &fsr, &servos));
    assert(sys2.calibrateSystem());
    assert(sys2.setGaitType(TRIPOD_GAIT));
    assert(sys2.update());
    assert(sys2.getLastError() == LocomotionSystem::KINEMATICS_ERROR);

    std::cout << "locomotion_system_test executed successfully" << std::endl;
    return 0;
}
