/**
 * @file manual_leg_toggle_dual_test.cpp
 * @brief Validates concurrent dual manual-leg toggling restored for OpenSHC parity (§2.3).
 *
 * OpenSHC supports two independent leg-selection slots (primary + secondary) so up to
 * MAX_MANUAL_LEGS legs can transition WALKING<->MANUAL concurrently. HexaMotion now exposes this via
 * LocomotionSystem::togglePrimaryLegState / toggleSecondaryLegState (routing to StateController's
 * primary/secondary slots). This test drives two legs into MANUAL at the same time and back.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <iostream>

static int g_failures = 0;
static void check(bool cond, const std::string &msg) {
    if (!cond) {
        std::cout << "  [FAIL] " << msg << "\n";
        ++g_failures;
    } else {
        std::cout << "  [ OK ] " << msg << "\n";
    }
}

int main() {
    std::cout << "=== Manual Leg Toggle Dual Test (\u00a72.3) ===\n";

    Parameters params = createDefaultParameters();
    LocomotionSystem sys(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "FAIL: initialization failed\n";
        return 1;
    }
    sys.setStandingPose();
    GaitConfiguration gait = createGaitConfig(TRIPOD_GAIT, params);
    sys.setGaitConfiguration(gait);

    StateController *sc = sys.getStateController();

    // Reach RUNNING.
    sys.startWalking();
    int iters = 0;
    while (sc->getRobotState() != ROBOT_RUNNING && iters < 5000) {
        sys.update();
        ++iters;
    }
    check(sc->getRobotState() == ROBOT_RUNNING, "robot reached RUNNING state");

    const int legA = 0; // primary slot
    const int legB = 3; // secondary slot

    // Request both toggles concurrently (must succeed on independent slots).
    check(sys.togglePrimaryLegState(legA), "primary toggle request accepted");
    check(sys.toggleSecondaryLegState(legB), "secondary toggle request accepted");

    // Drive update cycles until both legs reach MANUAL.
    iters = 0;
    while ((sc->getLegState(legA) != LEG_MANUAL || sc->getLegState(legB) != LEG_MANUAL) && iters < 20000) {
        sys.update();
        ++iters;
    }

    check(sc->getLegState(legA) == LEG_MANUAL, "leg A transitioned WALKING -> MANUAL");
    check(sc->getLegState(legB) == LEG_MANUAL, "leg B transitioned WALKING -> MANUAL");
    check(sc->getManualLegCount() == 2, "two legs are manual concurrently (== MAX_MANUAL_LEGS)");

    // Toggle both back to WALKING.
    check(sys.togglePrimaryLegState(legA), "primary toggle-back request accepted");
    check(sys.toggleSecondaryLegState(legB), "secondary toggle-back request accepted");

    iters = 0;
    while ((sc->getLegState(legA) != LEG_WALKING || sc->getLegState(legB) != LEG_WALKING) && iters < 20000) {
        sys.update();
        ++iters;
    }

    check(sc->getLegState(legA) == LEG_WALKING, "leg A returned MANUAL -> WALKING");
    check(sc->getLegState(legB) == LEG_WALKING, "leg B returned MANUAL -> WALKING");
    check(sc->getManualLegCount() == 0, "manual leg count back to zero");

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nTESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
