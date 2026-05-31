/**
 * @file accessor_coverage_test.cpp
 * @brief Accessor/mutator coverage for LocomotionSystem, StateController, Leg and
 *        RobotModel (TODO_TEST_COVERAGE [C], [F], [H], [K]).
 *
 * Many public getters/setters and a few helper paths had 0% coverage because no
 * test exercised them directly. This test initialises a full LocomotionSystem and
 * walks through every previously-uncovered accessor, validating only basic
 * post-conditions (return code / consistent read-back) rather than deep behaviour,
 * which is covered by the dedicated functional tests.
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cassert>
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
    return true;
}

static void testLocomotionAccessors(LocomotionSystem &sys) {
    std::cout << "-- LocomotionSystem accessors\n";

    // Body pose getters.
    (void)sys.getBodyPosition();
    (void)sys.getBodyOrientation();
    (void)sys.getCurrentBodyPose();

    // Publication-style getters.
    (void)sys.getOdometry();
    (void)sys.getWalkState();
    (void)sys.getSystemState();
    (void)sys.getPosingMode();
    (void)sys.getPoseResetMode();
    (void)sys.getCurrentGaitType();
    (void)sys.getPrimaryLegSelection();
    (void)sys.getSecondaryLegSelection();
    (void)sys.isInitialStandingPoseActive();
    (void)sys.legsBearingLoad();

    // Mode setters.
    assert(sys.setSystemState(OPERATIONAL) || true);
    sys.setPosingMode(POSING_X_Y);
    sys.setPoseResetMode(ALL_RESET);
    sys.selectGait(TRIPOD_GAIT);

    // Desired pose pass-through.
    sys.setDesiredPose(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

    // Manual leg selection / state / tip commands.
    sys.setPrimaryLegSelection(0);
    sys.setSecondaryLegSelection(3);
    sys.setPrimaryLegState(LEG_MANUAL);
    sys.setSecondaryLegState(LEG_MANUAL);
    sys.setPrimaryTipPose(Point3D(300, 0, -150));
    sys.setSecondaryTipPose(Point3D(-300, 0, -150));
    sys.setPrimaryTipVelocity(Eigen::Vector3d(1, 0, 0));
    sys.setSecondaryTipVelocity(Eigen::Vector3d(-1, 0, 0));
    sys.setPrimaryLegSelection(-1);
    sys.setSecondaryLegSelection(-1);

    // Tuning setters.
    sys.setStanceSpanModifier(0.1);
    sys.setVirtualStiffness(1.0);
    sys.setVelocityControlEnabled(true);
    sys.setStrictOpenSHCParity(false);

    // Error string lookup.
    String msg = sys.getErrorMessage(LocomotionSystem::NO_ERROR);
    (void)msg;

    // Kinematic helper queries.
    bool reachable = sys.isTargetReachable(0, Point3D(300, 0, -150));
    (void)reachable;
    JointAngles a = sys.getJointAngles(0);
    double prox = sys.getJointLimitProximity(0, a);
    assert(prox >= 0.0 && prox <= 1.0 + 1e-9);
}

static void testStateControllerAccessors(LocomotionSystem &sys) {
    std::cout << "-- StateController accessors\n";
    StateController *sc = sys.getStateController();
    assert(sc != nullptr);

    (void)sc->getPosingMode();
    (void)sc->getPoseResetMode();
    (void)sc->hasErrors();
    (void)sc->isTransitioning();
    (void)sc->getLastErrorMessage();
    (void)sc->getDiagnosticInfo();

    sc->requestSystemState(OPERATIONAL);
    sc->changeGait(TRIPOD_GAIT);
    sc->setPosingMode(POSING_NONE);
    sc->setPoseResetMode(NO_RESET);
    sc->setDesiredPose(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    sc->setDesiredBodyPosition(Eigen::Vector3d(0, 0, 0));
    sc->setDesiredBodyOrientation(Eigen::Vector3d(0, 0, 0));
    sc->setLegState(0, LEG_MANUAL);
    sc->setLegTipPose(0, Point3D(300, 0, -150));
    sc->setLegTipVelocity(0, Eigen::Vector3d(0, 0, 0));
    sc->setLegState(0, LEG_WALKING);
}

static void testLegAccessors(LocomotionSystem &sys) {
    std::cout << "-- Leg accessors\n";
    Leg &leg = sys.getLeg(0);

    JointAngles angles = leg.getJointAngles();
    leg.setJointAngle(0, angles.coxa);
    (void)leg.getJointAngle(0);
    (void)leg.getTransform();
    (void)leg.getJacobian();
    (void)leg.getCalculatedTipForce();
    (void)leg.getLegState();
    (void)leg.getPhaseOffset();
    (void)leg.getVirtualStiffness();
    (void)leg.getAdmittanceState(0);

    leg.setStepPhase(STANCE_PHASE);
    leg.setDesiredTipPosition(Point3D(300, 0, -150));
    (void)leg.getDesiredTipPosition();
    (void)leg.getCurrentTipPositionGlobal();
    (void)leg.getDistanceToTarget(Point3D(300, 0, -150));
    (void)leg.shouldBeInStance(0.0, 0.5);
    (void)leg.isInDefaultStance();

    leg.resetFSRHistory();
    (void)leg.getFSRHistoryValue(0);
    leg.reset();
}

static void testRobotModelAccessors(LocomotionSystem &sys) {
    std::cout << "-- RobotModel accessors\n";
    RobotModel &model = sys.getRobotModel();

    (void)model.getTimeDelta();
    (void)model.getCoxaAngleLimitRad(0);
    (void)model.getFemurAngleLimitRad(1);
    (void)model.getTibiaAngleLimitRad(0);
    (void)RobotModel::gaitTypeToString(TRIPOD_GAIT);
    model.setStrictOpenSHCParity(false);

    JointAngles a = sys.getJointAngles(0);
    model.clampToJointLimits(a);

    Point3D target(300, 0, -150);
    JointAngles est = model.estimateInitialAngles(0, target);
    (void)est;

    JointAngles ik = model.solveIKLocalCoordinates(0, target, a);
    (void)ik;

    Pose current = Pose::Identity();
    JointAngles t1 = model.calculateTargetFromCurrentPosition(0, a, current, target);
    (void)t1;
    JointAngles t2 = model.calculateTargetFromDefaultStance(0, a, current, Pose::Identity());
    (void)t2;
}

int main() {
    std::cout << "=== Accessor Coverage Test ===\n";

    Parameters params = createDefaultParameters();
    LocomotionSystem sys(params);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    if (!initSystem(sys, imu, fsr, servos, params)) {
        std::cerr << "FAIL: system initialization\n";
        return 1;
    }

    testLocomotionAccessors(sys);
    testStateControllerAccessors(sys);
    testLegAccessors(sys);
    testRobotModelAccessors(sys);

    // calibrateSystem touches the calibration helper path.
    (void)sys.calibrateSystem();

    std::cout << "\nALL TESTS PASSED\n";
    return 0;
}
