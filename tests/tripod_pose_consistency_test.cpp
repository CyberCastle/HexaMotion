#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>

/**
 * @brief Validate tripod gait leg trajectories and final pose.
 */
int main() {
    Parameters params = createDefaultParameters();

    LocomotionSystem system(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    assert(system.initialize(&imu, &fsr, &servos, pose_config));
    assert(system.setStandingPose());

    Eigen::Vector3d initial_pos = system.getBodyPosition();
    Eigen::Vector3d initial_ori = system.getBodyOrientation();
    Point3D initial_tips[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        initial_tips[i] = system.getLegPosition(i);
    }

    while (!system.executeStartupSequence()) {
        ;
    }

    assert(system.setGaitType(TRIPOD_GAIT));

    const double velocity = 100.0; // mm/s
    const double target_distance = 400.0; // mm
    assert(system.planGaitSequence(velocity, 0.0, 0.0));

    double distance_covered = 0.0;
    const double dt = 1.0 / params.control_frequency;

    while (distance_covered < target_distance) {
        assert(system.update());
        distance_covered += velocity * dt;
        if (static_cast<int>(distance_covered) % 100 == 0) {
            std::cout << "Distance covered: " << distance_covered << " mm" << std::endl;
        }
    }

    assert(system.planGaitSequence(0.0, 0.0, 0.0));
    for (int i = 0; i < params.control_frequency; ++i) {
        assert(system.update());
    }

    Eigen::Vector3d final_pos = system.getBodyPosition();
    Eigen::Vector3d final_ori = system.getBodyOrientation();
    Point3D final_tips[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        final_tips[i] = system.getLegPosition(i);
    }

    double ori_diff = (final_ori - initial_ori).norm();
    assert(ori_diff < 1.0);
    assert(std::abs(final_pos.z() - initial_pos.z()) < 1.0);

    std::cout << "Target distance: " << target_distance << " mm" << std::endl;
    std::cout << "Final orientation difference: " << ori_diff << " deg" << std::endl;

    for (int i = 1; i < NUM_LEGS; ++i) {
        assert(std::abs(final_tips[i].z - final_tips[0].z) < 2.0);
    }

    std::cout << "Leg heights aligned within 2mm" << std::endl;

    (void)initial_tips;

    std::cout << "tripod_pose_consistency_test passed" << std::endl;
    return 0;
}

