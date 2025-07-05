#include "../src/walk_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 120;
    p.height_offset = 0;
    p.control_frequency = 50;

    RobotModel model(p);

    // Create leg objects for testing (similar to LocomotionSystem)
    Leg test_legs[NUM_LEGS] = {
        Leg(0, p), Leg(1, p), Leg(2, p),
        Leg(3, p), Leg(4, p), Leg(5, p)
    };

    WalkController wc(model, test_legs);
    assert(wc.setGaitType(TRIPOD_GAIT));
    wc.updateGaitPhase(0.5f);
    LegState states[NUM_LEGS]{};
    double offsets[NUM_LEGS]{};
    Point3D traj = wc.footTrajectory(0, 0.2f, 20, 50, 0.5f, 0.5f, p.robot_height, offsets, states, nullptr, nullptr);
    (void)traj;
    std::cout << "walk_controller_test executed successfully" << std::endl;
    return 0;
}
