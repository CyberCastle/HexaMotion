/**
 * @file publication_accessor_test.cpp
 * @brief Smoke tests for LocomotionSystem output/publication accessor APIs.
 *
 * Tests getDesiredVelocityCommand, getDesiredBodyPoseCommand, getWalkspaceInfo,
 * getRotationPoseError, getDesiredJointStates, and getLegStateInfo.
 *
 * Validates that:
 *  - Each getter returns plausible data after system initialization and walking.
 *  - getDesiredVelocityCommand reflects the commanded velocity.
 *  - getWalkspaceInfo returns positive radii after workspace generation.
 *  - getDesiredJointStates fills arrays without errors.
 *  - getLegStateInfo returns valid per-leg data for all 6 legs.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

static constexpr double FORWARD_VEL = 60.0;
static constexpr int WALK_STEPS = 300;

int main() {
    std::cout << "=== Publication Accessor Test ===" << std::endl;

    Parameters params = createDefaultParameters();
    LocomotionSystem sys(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "FAIL: initialization" << std::endl;
        return 1;
    }
    sys.setStandingPose();

    GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
    sys.setGaitConfiguration(gait);
    sys.walkForward(FORWARD_VEL);
    sys.startWalking();

    StateController *sc = sys.getStateController();
    int startup_iters = 0;
    while (sc->getRobotState() != ROBOT_RUNNING && startup_iters < 5000) {
        sys.update();
        startup_iters++;
    }
    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "FAIL: did not reach RUNNING state" << std::endl;
        return 1;
    }

    // Run some walk cycles to populate state
    for (int i = 0; i < WALK_STEPS; ++i) {
        sys.update();
    }

    // --- getDesiredVelocityCommand ---
    {
        auto vel = sys.getDesiredVelocityCommand();
        // We commanded walkForward(60.0), so linear_x should be positive
        if (vel.linear_x <= 0.0) {
            std::cerr << "FAIL: getDesiredVelocityCommand linear_x should be positive (got "
                      << vel.linear_x << ")" << std::endl;
            return 1;
        }
        std::cout << "PASS: getDesiredVelocityCommand (linear_x=" << vel.linear_x
                  << ", linear_y=" << vel.linear_y
                  << ", angular_z=" << vel.angular_z << ")" << std::endl;
    }

    // --- getDesiredBodyPoseCommand ---
    {
        auto pose = sys.getDesiredBodyPoseCommand();
        // Just verify the struct is populated without crashing; values depend on pose mode
        std::cout << "PASS: getDesiredBodyPoseCommand (pos_z=" << pose.position_z
                  << ", roll=" << pose.roll << ")" << std::endl;
    }

    // --- getWalkspaceInfo ---
    {
        auto ws = sys.getWalkspaceInfo();
        if (ws.average_radius < 0.0) {
            std::cerr << "FAIL: getWalkspaceInfo average_radius is negative" << std::endl;
            return 1;
        }
        if (ws.min_radius > ws.max_radius) {
            std::cerr << "FAIL: getWalkspaceInfo min_radius > max_radius" << std::endl;
            return 1;
        }
        std::cout << "PASS: getWalkspaceInfo (avg=" << ws.average_radius
                  << ", min=" << ws.min_radius
                  << ", max=" << ws.max_radius << ")" << std::endl;
    }

    // --- getRotationPoseError ---
    {
        auto err = sys.getRotationPoseError();
        // Just verify access doesn't crash and values are finite
        bool finite = std::isfinite(err.absement_error[0]) &&
                      std::isfinite(err.position_error[0]) &&
                      std::isfinite(err.velocity_error[0]);
        if (!finite) {
            std::cerr << "FAIL: getRotationPoseError returned non-finite values" << std::endl;
            return 1;
        }
        std::cout << "PASS: getRotationPoseError" << std::endl;
    }

    // --- getDesiredJointStates ---
    {
        JointAngles positions[NUM_LEGS];
        JointAngles velocities[NUM_LEGS];
        JointAngles efforts[NUM_LEGS];

        if (!sys.getDesiredJointStates(positions, velocities, efforts)) {
            std::cerr << "FAIL: getDesiredJointStates returned false" << std::endl;
            return 1;
        }

        // Verify joint values are finite for all legs
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!std::isfinite(positions[i].coxa) || !std::isfinite(positions[i].femur) ||
                !std::isfinite(positions[i].tibia)) {
                std::cerr << "FAIL: getDesiredJointStates leg " << i
                          << " has non-finite position" << std::endl;
                return 1;
            }
        }
        std::cout << "PASS: getDesiredJointStates" << std::endl;
    }

    // --- getLegStateInfo (all 6 legs) ---
    {
        for (int i = 0; i < NUM_LEGS; ++i) {
            auto info = sys.getLegStateInfo(i);
            // Verify tip positions are finite
            if (!std::isfinite(info.walker_tip_pose.x) || !std::isfinite(info.walker_tip_pose.y) ||
                !std::isfinite(info.walker_tip_pose.z)) {
                std::cerr << "FAIL: getLegStateInfo leg " << i
                          << " has non-finite walker_tip_pose" << std::endl;
                return 1;
            }
            // step_progress should be in [0, 1]
            if (info.step_progress < 0.0 || info.step_progress > 1.0 + 1e-6) {
                std::cerr << "FAIL: getLegStateInfo leg " << i
                          << " step_progress out of range: " << info.step_progress << std::endl;
                return 1;
            }
        }
        std::cout << "PASS: getLegStateInfo (all 6 legs)" << std::endl;
    }

    std::cout << "\n=== ALL PUBLICATION ACCESSOR TESTS PASSED ===" << std::endl;
    return 0;
}
