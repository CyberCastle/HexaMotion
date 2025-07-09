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
    Leg legs[NUM_LEGS] = {Leg(0, model), Leg(1, model), Leg(2, model), Leg(3, model), Leg(4, model), Leg(5, model)};
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(model, Pose::Identity());
    }

    WalkController wc(model, legs);
    wc.init();
    assert(wc.getWalkState() == WALK_STOPPED);

    // Validate gait switching using gait names
    assert(wc.setGaitByName("wave_gait"));
    double stance = wc.getStanceDuration();
    double swing = wc.getSwingDuration();
    double freq = wc.getCycleFrequency();
    assert(stance > 0.0 && stance < 1.0);
    assert(swing > 0.0 && swing < 1.0);
    assert(freq > 0.0);
    assert(wc.getCurrentGaitName() == "wave_gait");

    // Validate step parameters from gait configuration
    double step_height = wc.getStepHeight();
    double step_length = wc.getStepLength();
    assert(step_height > 0.0);
    assert(step_length > 0.0);

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

    // Get leg stepper and validate phase offset
    auto stepper = wc.getLegStepper(0);
    assert(stepper);
    double phase_offset = stepper->getPhaseOffset();
    std::cout << "Phase offset L0: " << phase_offset << std::endl;
    assert(phase_offset >= 0.0 && phase_offset <= 1.0);

    // Test metachronal gait
    assert(wc.setGaitByName("metachronal_gait"));
    for (int i = 0; i < 10; ++i) {
        wc.updateWalk(Point3D(-20.0, 0.0, 0.0), 0.0);
    }
    auto metachronal_stepper = wc.getLegStepper(0);
    assert(metachronal_stepper);
    std::cout << "Metachronal phase offset L0: " << metachronal_stepper->getPhaseOffset() << std::endl;
    assert(metachronal_stepper->getPhaseOffset() >= 0.0 && metachronal_stepper->getPhaseOffset() <= 1.0);

    // Test available gait names
    auto available_gaits = wc.getAvailableGaitNames();
    assert(!available_gaits.empty());
    std::cout << "Available gaits: ";
    for (const auto& gait : available_gaits) {
        std::cout << gait << " ";
    }
    std::cout << std::endl;

    // Test velocity limits
    auto limits = wc.getVelocityLimits();
    assert(limits.linear_x > 0.0);
    assert(limits.angular_z > 0.0);

    std::cout << "walk_controller_validation_test passed" << std::endl;
    return 0;
}
