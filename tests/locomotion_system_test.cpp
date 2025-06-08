#include <cassert>
#include <iostream>
#include "test_stubs.h"
#include "../include/locomotion_system.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 100;
    p.coxa_length = 30;
    p.femur_length = 50;
    p.tibia_length = 70;
    p.robot_height = 100;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -90; p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90; p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90; p.tibia_angle_limits[1] = 90;
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

    std::cout << "locomotion_system_test executed successfully" << std::endl;
    return 0;
}
