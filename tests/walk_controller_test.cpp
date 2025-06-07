#include <cassert>
#include "test_stubs.h"
#include "../include/walk_controller.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 100;
    p.coxa_length = 30;
    p.femur_length = 50;
    p.tibia_length = 70;
    p.robot_height = 100;
    p.control_frequency = 50;

    RobotModel model(p);
    WalkController wc(model);
    assert(wc.setGaitType(TRIPOD_GAIT));
    wc.updateGaitPhase(0.5f);
    LegState states[NUM_LEGS]{};
    float offsets[NUM_LEGS]{};
    Point3D traj = wc.footTrajectory(0, 0.2f, 20, 50, 0.5f, 0.5f, p.robot_height, offsets, states, nullptr, nullptr);
    (void)traj;
    return 0;
}
