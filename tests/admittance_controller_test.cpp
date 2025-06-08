#include <cassert>
#include <iostream>
#include "test_stubs.h"
#include "../include/admittance_controller.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 7;
    p.control_frequency = 50;

    RobotModel model(p);
    DummyIMU imu;
    DummyFSR fsr;
    AdmittanceController ac(model, &imu, &fsr);
    Eigen::Vector3f target(0, 0, 0);
    Eigen::Vector3f current(0, 0, 0);
    assert(ac.maintainOrientation(target, current, 0.1f));
    LegState states[NUM_LEGS]{};
    Point3D legs[NUM_LEGS]{};
    assert(ac.checkStability(legs, states));
    std::cout << "admittance_controller_test executed successfully" << std::endl;
    return 0;
}
