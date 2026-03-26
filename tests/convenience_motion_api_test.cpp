/**
 * @file convenience_motion_api_test.cpp
 * @brief Smoke tests for LocomotionSystem convenience motion APIs: walkBackward, walkSideways.
 *
 * Validates that:
 *  - walkBackward() sets negative X velocity and returns true after initialization.
 *  - walkSideways(right=true) sets negative Y velocity.
 *  - walkSideways(right=false) sets positive Y velocity.
 *  - The system can execute several update cycles without error after each command.
 *  - Both methods return false before initialization.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

static constexpr double VELOCITY = 80.0;
static constexpr int WALK_STEPS = 200;

int main() {
    std::cout << "=== Convenience Motion API Test ===" << std::endl;

    // --- Pre-init failure ---
    {
        Parameters params = createDefaultParameters();
        LocomotionSystem sys(params);
        if (sys.walkBackward(VELOCITY)) {
            std::cerr << "FAIL: walkBackward should fail before init" << std::endl;
            return 1;
        }
        if (sys.walkSideways(VELOCITY, true)) {
            std::cerr << "FAIL: walkSideways should fail before init" << std::endl;
            return 1;
        }
        std::cout << "PASS: pre-init failure checks" << std::endl;
    }

    // --- walkBackward ---
    {
        Parameters params = createDefaultParameters();
        LocomotionSystem sys(params);
        DummyIMU imu;
        DummyFSR fsr;
        DummyServo servos;
        BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

        if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
            std::cerr << "FAIL: initialization failed" << std::endl;
            return 1;
        }
        if (!sys.setStandingPose()) {
            std::cerr << "FAIL: standing pose failed" << std::endl;
            return 1;
        }

        GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
        sys.setGaitConfiguration(gait);

        if (!sys.walkBackward(VELOCITY)) {
            std::cerr << "FAIL: walkBackward returned false" << std::endl;
            return 1;
        }

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

        // Velocity command is populated after update pipeline runs
        auto vel = sys.getDesiredVelocityCommand();
        if (vel.linear_x >= 0.0) {
            std::cerr << "FAIL: walkBackward did not set negative linear_x (got "
                      << vel.linear_x << ")" << std::endl;
            return 1;
        }
        if (std::abs(vel.linear_y) > 1e-6 || std::abs(vel.angular_z) > 1e-6) {
            std::cerr << "FAIL: walkBackward set unexpected lateral/angular velocity" << std::endl;
            return 1;
        }

        for (int i = 0; i < WALK_STEPS; ++i) {
            sys.update();
        }

        std::cout << "PASS: walkBackward" << std::endl;
    }

    // --- walkSideways (right) ---
    {
        Parameters params = createDefaultParameters();
        LocomotionSystem sys(params);
        DummyIMU imu;
        DummyFSR fsr;
        DummyServo servos;
        BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

        if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
            std::cerr << "FAIL: initialization failed" << std::endl;
            return 1;
        }
        sys.setStandingPose();

        GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
        sys.setGaitConfiguration(gait);

        if (!sys.walkSideways(VELOCITY, true)) {
            std::cerr << "FAIL: walkSideways(right) returned false" << std::endl;
            return 1;
        }

        sys.startWalking();
        StateController *sc = sys.getStateController();
        int startup_iters = 0;
        while (sc->getRobotState() != ROBOT_RUNNING && startup_iters < 5000) {
            sys.update();
            startup_iters++;
        }

        // Check velocity after update pipeline propagates the command
        auto vel = sys.getDesiredVelocityCommand();
        // right_direction=true => lateral_velocity = -velocity => linear_y < 0
        if (vel.linear_y >= 0.0) {
            std::cerr << "FAIL: walkSideways(right) did not set negative linear_y (got "
                      << vel.linear_y << ")" << std::endl;
            return 1;
        }

        for (int i = 0; i < WALK_STEPS; ++i) {
            sys.update();
        }

        std::cout << "PASS: walkSideways(right)" << std::endl;
    }

    // --- walkSideways (left) ---
    {
        Parameters params = createDefaultParameters();
        LocomotionSystem sys(params);
        DummyIMU imu;
        DummyFSR fsr;
        DummyServo servos;
        BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

        if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
            std::cerr << "FAIL: initialization failed" << std::endl;
            return 1;
        }
        sys.setStandingPose();

        GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
        sys.setGaitConfiguration(gait);

        if (!sys.walkSideways(VELOCITY, false)) {
            std::cerr << "FAIL: walkSideways(left) returned false" << std::endl;
            return 1;
        }

        sys.startWalking();
        StateController *sc = sys.getStateController();
        int startup_iters = 0;
        while (sc->getRobotState() != ROBOT_RUNNING && startup_iters < 5000) {
            sys.update();
            startup_iters++;
        }

        // Check velocity after update pipeline propagates the command
        auto vel = sys.getDesiredVelocityCommand();
        // right_direction=false => lateral_velocity = +velocity => linear_y > 0
        if (vel.linear_y <= 0.0) {
            std::cerr << "FAIL: walkSideways(left) did not set positive linear_y (got "
                      << vel.linear_y << ")" << std::endl;
            return 1;
        }

        for (int i = 0; i < WALK_STEPS; ++i) {
            sys.update();
        }

        std::cout << "PASS: walkSideways(left)" << std::endl;
    }

    std::cout << "\n=== ALL CONVENIENCE MOTION API TESTS PASSED ===" << std::endl;
    return 0;
}
