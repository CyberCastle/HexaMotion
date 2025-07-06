#include "walk_controller.h"
#include "leg.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 120;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    Leg legs[NUM_LEGS] = {Leg(0, p), Leg(1, p), Leg(2, p), Leg(3, p), Leg(4, p), Leg(5, p)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(model, Pose::Identity());
    }

    WalkController wc(model, legs);
    wc.init();
    assert(wc.getWalkState() == WALK_STOPPED);

    // Validate gait switching and timing parameters
    assert(wc.setGaitType(WAVE_GAIT));
    double stance = 0, swing = 0, freq = 0;
    wc.getGaitTimingParameters(WAVE_GAIT, stance, swing, freq);
    assert(std::abs(wc.getStanceDuration() - stance) < 1e-6);
    assert(std::abs(wc.getSwingDuration() - swing) < 1e-6);
    assert(std::abs(wc.getCycleFrequency() - freq) < 1e-6);
    assert(wc.getCurrentGait() == WAVE_GAIT);

    // Validate step parameter setters
    assert(wc.setStepParameters(40.0, 60.0));
    assert(std::abs(wc.getStepHeight() - 40.0) < 1e-6);
    double len = wc.getStepLength();
    assert(len >= 20.0 && len <= 80.0);

    // Generate walkspace and verify values
    wc.generateWalkspace();
    auto ws = wc.getWalkspace();
    assert(!ws.empty());
    for (const auto &kv : ws) {
        assert(kv.second > 0.0);
    }

    // Odometry calculation
    wc.updateWalk(Point3D(30.0, 0.0, 0.0), 0.0);
    Point3D odom = wc.calculateOdometry(0.1);
    std::cout << "Odometry: (" << odom.x << ", " << odom.y << ")" << std::endl;
    assert(std::abs(odom.x) > 0.0);

    // Foot trajectory generation
    LegState states[NUM_LEGS]{};
    double offsets[NUM_LEGS]{};
    Point3D traj = wc.footTrajectory(0, 0.5, wc.getStepHeight(), wc.getStepLength(),
                                    wc.getStanceDuration(), wc.getSwingDuration(),
                                    p.robot_height, offsets, states, nullptr, nullptr);
    assert(!std::isnan(traj.x) && !std::isnan(traj.y) && !std::isnan(traj.z));

    // Metachronal wave direction adaptation
    assert(wc.setGaitType(METACHRONAL_GAIT));
    for (int i = 0; i < 10; ++i) {
        wc.updateWalk(Point3D(-20.0, 0.0, 0.0), 0.0);
    }
    wc.updateMetachronalPattern();
    auto stepper = wc.getLegStepper(0);
    assert(stepper);
    std::cout << "Phase offset L0: " << stepper->getPhaseOffset() << std::endl;
    assert(stepper->getPhaseOffset() > 0.5);

    std::cout << "walk_controller_validation_test passed" << std::endl;
    return 0;
}
