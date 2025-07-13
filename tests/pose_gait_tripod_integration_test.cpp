#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>

struct TestReport {
    int total{0};
    int passed{0};
    int failed{0};
};

static void printDiagnostics(const LocomotionSystem &sys) {
    std::cout << "--- Leg diagnostics ---" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles q = leg.getJointAngles();
        Point3D p = leg.getCurrentTipPositionGlobal();
        StepPhase ph = leg.getStepPhase();
        std::cout << "Leg " << i
                  << " phase=" << (ph == STANCE_PHASE ? "STANCE" : "SWING")
                  << " pos(" << p.x << ", " << p.y << ", " << p.z << ")"
                  << " angles(" << q.coxa << ", " << q.femur << ", " << q.tibia
                  << ")" << std::endl;
    }
    std::cout << "-----------------------" << std::endl;
}

static void addResult(TestReport &rep, bool ok, const std::string &msg,
                      const LocomotionSystem &sys) {
    rep.total++;
    if (ok) {
        rep.passed++;
    } else {
        rep.failed++;
        std::cout << "Test failed: " << msg << std::endl;
        printDiagnostics(sys);
    }
}

static void validateGroupSync(const LocomotionSystem &sys, TestReport &rep) {
    bool a_stance = sys.getLegState(0) == STANCE_PHASE &&
                    sys.getLegState(2) == STANCE_PHASE &&
                    sys.getLegState(4) == STANCE_PHASE;
    bool b_stance = sys.getLegState(1) == STANCE_PHASE &&
                    sys.getLegState(3) == STANCE_PHASE &&
                    sys.getLegState(5) == STANCE_PHASE;
    bool ok = (a_stance && !b_stance) || (b_stance && !a_stance);
    addResult(rep, ok, "Tripod synchronization", sys);
}

static void validateSwingHeights(const LocomotionSystem &sys, TestReport &rep) {
    std::vector<int> swing;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLegState(i) == SWING_PHASE) swing.push_back(i);
    }
    addResult(rep, swing.size() == 3, "Three legs should be in swing", sys);
    if (swing.size() == 3) {
        double z0 = sys.getLeg(swing[0]).getCurrentTipPositionGlobal().z;
        bool equal = true;
        for (int i = 1; i < 3; ++i) {
            double zi = sys.getLeg(swing[i]).getCurrentTipPositionGlobal().z;
            if (std::abs(zi - z0) > 1.0) equal = false;
            addResult(rep, zi > -145.0, "Swing leg height", sys);
        }
        addResult(rep, equal, "Swing legs equal height", sys);
    }
}

static void validateCoxaSymmetry(const LocomotionSystem &sys, TestReport &rep) {
    const int pairs[3][2] = {{0,3},{1,4},{2,5}};
    for (const auto &pr : pairs) {
        double c1 = sys.getLeg(pr[0]).getJointAngles().coxa;
        double c2 = sys.getLeg(pr[1]).getJointAngles().coxa;
        bool ok = std::abs(c1 + c2) <= 5.0;
        addResult(rep, ok, "Coxa symmetry " + std::to_string(pr[0]) + "-" +
                               std::to_string(pr[1]), sys);
    }
}

static void validateTrajectorySimilarity(const LocomotionSystem &sys,
                                         TestReport &rep) {
    std::vector<int> swing;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLegState(i) == SWING_PHASE) swing.push_back(i);
    }
    if (swing.size() == 3) {
        Point3D p0 = sys.getLeg(swing[0]).getCurrentTipPositionGlobal();
        bool ok = true;
        for (int i = 1; i < 3; ++i) {
            Point3D pi = sys.getLeg(swing[i]).getCurrentTipPositionGlobal();
            double dx = pi.x - p0.x;
            double dy = pi.y - p0.y;
            double dz = pi.z - p0.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 5.0) ok = false;
        }
        addResult(rep, ok, "Swing leg trajectories similar", sys);
    }
}

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    assert(sys.initialize(&imu, &fsr, &servos, pose_config));
    assert(sys.setStandingPose());

    Eigen::Vector3d pos = sys.getBodyPosition();
    pos.z() = -150.0;
    assert(sys.setBodyPoseImmediate(pos, sys.getBodyOrientation()));

    TestReport rep{};
    addResult(rep, std::abs(sys.getBodyPosition().z() + 150.0) <= 2.0,
              "Initial body height", sys);

    int startup_iterations = 0;
    const int max_startup_iterations = 1000;
    while (!sys.executeStartupSequence() && startup_iterations < max_startup_iterations) {
        sys.update();
        ++startup_iterations;
    }
    addResult(rep, startup_iterations < max_startup_iterations, "Startup sequence timeout", sys);

    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(100.0));

    const double distance = 1000.0; // mm
    double dt = 1.0 / p.control_frequency;
    int cycles = static_cast<int>(distance / (100.0 * dt));

    std::vector<StepPhase> prev_phase(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) prev_phase[i] = sys.getLegState(i);

    int max_cycles = cycles * 10;
    int step = 0;
    for (; step < cycles && step < max_cycles; ++step) {
        assert(sys.update());
        bool phase_change = false;
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase ph = sys.getLegState(i);
            if (ph != prev_phase[i]) phase_change = true;
            prev_phase[i] = ph;
        }
        if (phase_change) {
            validateGroupSync(sys, rep);
            validateSwingHeights(sys, rep);
            validateCoxaSymmetry(sys, rep);
            validateTrajectorySimilarity(sys, rep);
        }
    }
    addResult(rep, step >= cycles, "Gait execution timeout", sys);

    sys.planGaitSequence(0.0, 0.0, 0.0);
    for (int i = 0; i < p.control_frequency; ++i) sys.update();

    addResult(rep, std::abs(sys.getBodyPosition().z() + 150.0) <= 2.0,
              "Final body height", sys);

    std::cout << "Total tests: " << rep.total << " Passed: " << rep.passed
              << " Failed: " << rep.failed << std::endl;
    return rep.failed == 0 ? 0 : 1;
}

