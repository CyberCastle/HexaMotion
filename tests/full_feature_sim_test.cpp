#include "../include/locomotion_system.h"
#include "../include/state_controller.h"
#include "../include/manual_pose_controller.h"
#include "../include/imu_auto_pose.h"
#include "../include/admittance_controller.h"
#include "../include/precision_config.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>

/**
 * @brief Full feature simulation test.
 *
 * Exercises gait changes, precision settings, admittance control,
 * manual/auto pose functions and state transitions while walking a
 * simulated distance of 4 meters.
 */
int main() {
    std::cout << "=== Full Feature Simulation Test ===" << std::endl;

    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 90;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    // Hardware stubs
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Locomotion system
    LocomotionSystem sys(p);
    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(400.0f));

    // State controller configuration
    StateMachineConfig state_cfg{};
    state_cfg.enable_startup_sequence = false;
    state_cfg.transition_timeout = 10.0f;
    StateController controller(sys, state_cfg);
    assert(controller.initialize());
    controller.requestSystemState(SYSTEM_OPERATIONAL);
    controller.requestRobotState(ROBOT_RUNNING);
    controller.setDesiredVelocity(Eigen::Vector2f(400.0f, 0.0f), 0.0f);

    // Independent modules for additional features
    RobotModel model(p);
    ManualPoseController pose_ctrl(model);
    pose_ctrl.initialize();
    pose_ctrl.setPoseMode(ManualPoseController::POSE_TRANSLATION);
    pose_ctrl.processInput(20.0f, 0.0f, 0.0f);

    IMUAutoPose auto_pose(model, &imu, pose_ctrl, ComputeConfig::medium());
    auto_pose.initialize();
    auto_pose.setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
    auto_pose.update(0.02f);

    AdmittanceController adm_ctrl(model, &imu, &fsr, ComputeConfig::high());
    adm_ctrl.initialize();
    adm_ctrl.setDynamicStiffness(true, 0.5f, 1.5f);
    Point3D forces[NUM_LEGS];
    Point3D deltas[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        forces[i] = Point3D(1.0f, 0.0f, -2.0f);
    }
    adm_ctrl.updateAllLegs(forces, deltas);

    // Simulate 4 meters of walking
    float distance = 4000.0f;
    float dt = 1.0f / p.control_frequency;
    int steps = static_cast<int>((distance / 400.0f) / dt);

    for (int s = 0; s < steps; ++s) {
        if (s == steps / 3)
            controller.changeGait(WAVE_GAIT);
        if (s == (2 * steps) / 3)
            controller.changeGait(RIPPLE_GAIT);

        auto_pose.update(dt);
        controller.update(dt);
        sys.update();
    }

    std::cout << "Final gait: " << controller.getWalkState() << std::endl;
    std::cout << "full_feature_sim_test executed successfully" << std::endl;
    return 0;
}
