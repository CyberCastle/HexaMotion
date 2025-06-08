#include <cassert>
#include <iostream>
#include "test_stubs.h"
#include "../include/walk_controller.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 7;
    p.control_frequency = 50;

    RobotModel model(p);
    WalkController wc(model);
    assert(wc.setGaitType(TRIPOD_GAIT));
    wc.updateGaitPhase(0.5f);
    LegState states[NUM_LEGS]{};
    float offsets[NUM_LEGS]{};
    Point3D traj = wc.footTrajectory(0, 0.2f, 20, 50, 0.5f, 0.5f, p.robot_height, offsets, states, nullptr, nullptr);
    (void)traj;
    std::cout << "walk_controller_test executed successfully" << std::endl;
    return 0;
}
