#include "../src/HexaModel.h" // for LegState enum
#include "../src/admittance_controller.h"
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

    RobotModel model(p);
    DummyIMU imu;
    DummyFSR fsr;
    AdmittanceController ac(model, &imu, &fsr);
    Point3D target(0, 0, 0);
    Point3D current(0, 0, 0);
    assert(ac.maintainOrientation(target, current, 0.1f));
    LegState states[NUM_LEGS];
    Point3D legs[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        states[i] = STANCE_PHASE;
        legs[i] = Point3D(0, 0, 0);
    }
    assert(ac.checkStability(legs, states));
    std::cout << "admittance_controller_test executed successfully" << std::endl;
    return 0;
}
