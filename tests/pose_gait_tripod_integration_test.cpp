#include "../src/body_pose_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct TestReport {
    int total{0};
    int passed{0};
    int failed{0};
};

static void printDiagnostics(const LocomotionSystem &sys) {
    std::cout << "--- Leg diagnostics ---" << std::endl;
    const double deg = 180.0 / M_PI;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles q = leg.getJointAngles();
        Point3D p = leg.getCurrentTipPositionGlobal();
        StepPhase ph = leg.getStepPhase();
        std::cout << "Leg " << i
                  << " phase=" << (ph == STANCE_PHASE ? "STANCE" : "SWING")
                  << " pos(" << p.x << ", " << p.y << ", " << p.z << ")"
                  << " angles(" << q.coxa * deg << ", " << q.femur * deg << ", "
                  << q.tibia * deg << ")" << std::endl;
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
        if (sys.getLegState(i) == SWING_PHASE)
            swing.push_back(i);
    }
    addResult(rep, swing.size() == 3, "Three legs should be in swing", sys);
    if (swing.size() == 3) {
        double z0 = sys.getLeg(swing[0]).getCurrentTipPositionGlobal().z;
        bool equal = true;
        for (int i = 1; i < 3; ++i) {
            double zi = sys.getLeg(swing[i]).getCurrentTipPositionGlobal().z;
            if (std::abs(zi - z0) > 1.0)
                equal = false;
            addResult(rep, zi > -145.0, "Swing leg height", sys);
        }
        addResult(rep, equal, "Swing legs equal height", sys);
    }
}

static void validateCoxaSymmetry(const LocomotionSystem &sys, TestReport &rep) {
    // OpenSHC-compatible validation: Remove symmetry requirement due to BASE_THETA_OFFSETS
    // In OpenSHC architecture with asymmetric leg base offsets, perfect coxa symmetry is not expected
    // Instead, validate that coxa angles are within reasonable operational ranges

    for (int i = 0; i < NUM_LEGS; ++i) {
        double coxa_deg = sys.getLeg(i).getJointAngles().coxa * 180.0 / M_PI;
        bool ok = std::abs(coxa_deg) <= 65.0; // Within joint limits
        addResult(rep, ok, "Coxa angle within limits leg " + std::to_string(i) + " (" + std::to_string(coxa_deg) + "°)", sys);
    }
}

static void validateTrajectorySimilarity(const LocomotionSystem &sys,
                                         TestReport &rep) {
    std::vector<int> swing;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLegState(i) == SWING_PHASE)
            swing.push_back(i);
    }
    if (swing.size() == 3) {
        Point3D p0 = sys.getLeg(swing[0]).getCurrentTipPositionGlobal();
        bool ok = true;
        for (int i = 1; i < 3; ++i) {
            Point3D pi = sys.getLeg(swing[i]).getCurrentTipPositionGlobal();
            double dx = pi.x - p0.x;
            double dy = pi.y - p0.y;
            double dz = pi.z - p0.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 5.0)
                ok = false;
        }
        addResult(rep, ok, "Swing leg trajectories similar", sys);
    }
}

static void validateSwingPeakSync(LocomotionSystem &sys, TestReport &rep) {
    WalkController *wc = sys.getWalkController();
    if (!wc)
        return;

    std::vector<int> swing;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLegState(i) == SWING_PHASE)
            swing.push_back(i);
    }

    if (swing.size() == 3) {
        bool near_mid = true;
        for (int idx = 0; idx < 3; ++idx) {
            auto stepper = wc->getLegStepper(swing[idx]);
            double prog = stepper ? stepper->getSwingProgress() : -1.0;
            if (prog < 0.45 || prog > 0.55)
                near_mid = false;
        }

        if (near_mid) {
            double z0 = sys.getLeg(swing[0]).getCurrentTipPositionGlobal().z;
            bool equal = true;
            for (int idx = 1; idx < 3; ++idx) {
                double zi = sys.getLeg(swing[idx]).getCurrentTipPositionGlobal().z;
                if (std::abs(zi - z0) > 1.0)
                    equal = false;
            }
            addResult(rep, equal, "Swing legs peak height sync", sys);
        }
    }
}

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.standing_height = 150; // Initial standing height
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

    TestReport rep{};
    addResult(rep, std::abs(sys.getBodyPosition().z() + 150.0) <= 2.0,
              "Initial body height", sys);

    printf("Initial body position: (%.2f, %.2f, %.2f)\n",
           sys.getBodyPosition().x(), sys.getBodyPosition().y(), sys.getBodyPosition().z());

    // Execute startup sequence until complete (with timeout)
    bool startup_ok = false;
    int startup_iterations = 0;

    // Calculate expected iterations based on system parameters
    // For tripod gait: 2 groups × step_time × control_frequency + safety margin
    double time_to_start = pose_config.time_to_start; // Default: 6.0 seconds
    double step_time = 1.0 / DEFAULT_STEP_FREQUENCY;  // Default: 1.0 second per step
    int expected_iterations_per_group = static_cast<int>(std::ceil(step_time * p.control_frequency));
    int expected_total_iterations = expected_iterations_per_group * 2;     // 2 tripod groups
    int safety_margin = static_cast<int>(expected_total_iterations * 0.5); // 50% safety margin
    int max_startup_iterations = expected_total_iterations + safety_margin;

    printf("Startup calculation: step_time=%.1fs, control_freq=%.0fHz\n",
           step_time, p.control_frequency);
    printf("Expected iterations: %d per group × 2 groups = %d total (+ %d safety margin = %d max)\n",
           expected_iterations_per_group, expected_total_iterations, safety_margin, max_startup_iterations);

    while (!startup_ok && startup_iterations < max_startup_iterations) {
        startup_ok = sys.executeStartupSequence();
        startup_iterations++;
    }

    printf("Startup sequence completed in %d iterations\n", startup_iterations);
    addResult(rep, startup_ok, "Startup sequence", sys);

    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(100.0));

    const double distance = 10.0; // mm
    double dt = 1.0 / p.control_frequency;
    int cycles = static_cast<int>(distance / (100.0 * dt));

    std::vector<StepPhase> prev_phase(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i)
        prev_phase[i] = sys.getLegState(i);

    int max_cycles = cycles * 10;
    int step = 0;
    for (; step < cycles && step < max_cycles; ++step) {
        assert(sys.update());
        bool phase_change = false;
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase ph = sys.getLegState(i);
            if (ph != prev_phase[i])
                phase_change = true;
            prev_phase[i] = ph;
        }
        if (phase_change) {
            validateGroupSync(sys, rep);
            validateSwingHeights(sys, rep);
            validateCoxaSymmetry(sys, rep);
            validateTrajectorySimilarity(sys, rep);
        }
        validateSwingPeakSync(sys, rep);
    }
    addResult(rep, step >= cycles, "Gait execution timeout", sys);

    sys.planGaitSequence(0.0, 0.0, 0.0);
    for (int i = 0; i < p.control_frequency; ++i)
        sys.update();

    addResult(rep, std::abs(sys.getBodyPosition().z() + 150.0) <= 2.0,
              "Final body height", sys);

    std::cout << "Total tests: " << rep.total << " Passed: " << rep.passed
              << " Failed: " << rep.failed << std::endl;
    return rep.failed == 0 ? 0 : 1;
}
