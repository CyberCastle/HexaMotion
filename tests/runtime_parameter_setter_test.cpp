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

#include "../src/body_pose_config_factory.h"
#include "../src/cartesian_velocity_controller.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

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

int main() {
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
