/**
 * @file runtime_api_test.cpp
 * @brief Consolidated test suite.
 *
 * This single translation unit folds together the following sub-tests
 * without losing any coverage. Each sub-test keeps its original assertions
 * and entry function; their bodies are wrapped in a dedicated namespace to
 * avoid symbol collisions, and this file's main() runs them in sequence and
 * aggregates the exit status:
 *   - run_runtime_parameter_setter()
 *   - run_convenience_motion_api()
 *   - run_publication_accessor()
 */

#include "../src/body_pose_config_factory.h"
#include "../src/cartesian_velocity_controller.h"
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

// ===========================================================================
// Sub-test: run_runtime_parameter_setter (from runtime_parameter_setter_test.cpp)
// ===========================================================================
namespace cm_runtime_parameter_setter_test {
/**
 * @file runtime_parameter_setter_test.cpp
 * @brief Smoke tests for LocomotionSystem runtime parameter setter APIs.
 *
 * Tests setStepFrequency, setSwingHeight, setSwingWidth, setStepDepth,
 * setVirtualMass, setVirtualDamping, setForceGain, setVelocityScaling,
 * setGaitSpeedModifiers, and getCurrentServoSpeed.
 *
 * Validates that:
 *  - Each setter returns true after proper initialization.
 *  - Admittance setters (mass, damping, forceGain) clamp within documented ranges.
 *  - Gait setters apply to the current gait configuration.
 *  - getCurrentServoSpeed returns >= 0 for valid indices.
 *  - The system can execute update cycles after each setter call.
 */

static bool initSystem(LocomotionSystem &sys, DummyIMU &imu, DummyFSR &fsr, DummyServo &servos,
                       Parameters &params) {
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);
    if (!sys.initialize(&imu, &fsr, &servos, pose_config))
        return false;
    if (!sys.setStandingPose())
        return false;
    GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
    sys.setGaitConfiguration(gait);
    sys.walkForward(50.0);
    sys.startWalking();
    StateController *sc = sys.getStateController();
    int iters = 0;
    while (sc->getRobotState() != ROBOT_RUNNING && iters < 5000) {
        sys.update();
        iters++;
    }
    return sc->getRobotState() == ROBOT_RUNNING;
}

int run_runtime_parameter_setter() {
    std::cout << "=== Runtime Parameter Setter Test ===" << std::endl;

    Parameters params = createDefaultParameters();
    LocomotionSystem sys(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    if (!initSystem(sys, imu, fsr, servos, params)) {
        std::cerr << "FAIL: system initialization" << std::endl;
        return 1;
    }

    // setStepFrequency
    if (!sys.setStepFrequency(1.0)) {
        std::cerr << "FAIL: setStepFrequency(1.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 50; ++i)
        sys.update();
    std::cout << "PASS: setStepFrequency" << std::endl;

    // setSwingHeight
    if (!sys.setSwingHeight(40.0)) {
        std::cerr << "FAIL: setSwingHeight(40.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 50; ++i)
        sys.update();
    std::cout << "PASS: setSwingHeight" << std::endl;

    // setSwingWidth
    if (!sys.setSwingWidth(20.0)) {
        std::cerr << "FAIL: setSwingWidth(20.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 50; ++i)
        sys.update();
    std::cout << "PASS: setSwingWidth" << std::endl;

    // setStepDepth
    if (!sys.setStepDepth(10.0)) {
        std::cerr << "FAIL: setStepDepth(10.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 50; ++i)
        sys.update();
    std::cout << "PASS: setStepDepth" << std::endl;

    // setVirtualMass
    if (!sys.setVirtualMass(1.5)) {
        std::cerr << "FAIL: setVirtualMass(1.5) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 10; ++i)
        sys.update();
    std::cout << "PASS: setVirtualMass" << std::endl;

    // setVirtualDamping
    if (!sys.setVirtualDamping(5.0)) {
        std::cerr << "FAIL: setVirtualDamping(5.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 10; ++i)
        sys.update();
    std::cout << "PASS: setVirtualDamping" << std::endl;

    // setForceGain
    if (!sys.setForceGain(2.0)) {
        std::cerr << "FAIL: setForceGain(2.0) returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 10; ++i)
        sys.update();
    std::cout << "PASS: setForceGain" << std::endl;

    // setVelocityScaling
    CartesianVelocityController::VelocityScaling scaling;
    scaling.linear_velocity_scale = 1.2;
    scaling.angular_velocity_scale = 0.8;
    if (!sys.setVelocityScaling(scaling)) {
        std::cerr << "FAIL: setVelocityScaling returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 10; ++i)
        sys.update();
    std::cout << "PASS: setVelocityScaling" << std::endl;

    // setGaitSpeedModifiers
    CartesianVelocityController::GaitSpeedModifiers modifiers;
    modifiers.tripod_speed_factor = 1.1;
    if (!sys.setGaitSpeedModifiers(modifiers)) {
        std::cerr << "FAIL: setGaitSpeedModifiers returned false" << std::endl;
        return 1;
    }
    for (int i = 0; i < 10; ++i)
        sys.update();
    std::cout << "PASS: setGaitSpeedModifiers" << std::endl;

    // getCurrentServoSpeed
    double speed = sys.getCurrentServoSpeed(0, 0);
    if (speed < 0.0) {
        std::cerr << "FAIL: getCurrentServoSpeed returned negative value: " << speed << std::endl;
        return 1;
    }
    std::cout << "PASS: getCurrentServoSpeed (leg0/joint0 = " << speed << ")" << std::endl;

    std::cout << "\n=== ALL RUNTIME PARAMETER SETTER TESTS PASSED ===" << std::endl;
    return 0;
}
} // namespace cm_runtime_parameter_setter_test

// ===========================================================================
// Sub-test: run_convenience_motion_api (from convenience_motion_api_test.cpp)
// ===========================================================================
namespace cm_convenience_motion_api_test {
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

static constexpr double VELOCITY = 80.0;
static constexpr int WALK_STEPS = 200;

int run_convenience_motion_api() {
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
} // namespace cm_convenience_motion_api_test

// ===========================================================================
// Sub-test: run_publication_accessor (from publication_accessor_test.cpp)
// ===========================================================================
namespace cm_publication_accessor_test {
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

static constexpr double FORWARD_VEL = 60.0;
static constexpr int WALK_STEPS = 300;

int run_publication_accessor() {
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
} // namespace cm_publication_accessor_test

int main() {
    int rc = 0;

    std::cout << "\n========== runtime parameter setter ==========\n";
    rc |= cm_runtime_parameter_setter_test::run_runtime_parameter_setter();

    std::cout << "\n========== convenience motion api ==========\n";
    rc |= cm_convenience_motion_api_test::run_convenience_motion_api();

    std::cout << "\n========== publication accessor ==========\n";
    rc |= cm_publication_accessor_test::run_publication_accessor();

    std::cout << "\n[runtime_api_test] overall: " << (rc == 0 ? "PASS" : "FAIL") << "\n";
    return rc == 0 ? 0 : 1;
}
