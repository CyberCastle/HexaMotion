/**
 * @file bezier_transition_all_legs_test.cpp
 * @brief Visualizes Bezier curves during pack, unpack and standing pose transitions for ALL six legs.
 *
 * This test extends bezier_transition_single_leg_test by showing all legs simultaneously during
 * the full PACKED -> READY -> RUNNING -> READY -> PACKED state machine cycle. Each phase prints:
 * - Per-iteration joint angles for every leg (in degrees)
 * - Tip positions in global coordinates
 * - Symmetry verification between opposite leg pairs
 *
 * Requires start_up_sequence=true so the state machine uses Bezier-based startup/shutdown.
 *
 * Bezier curves exercised:
 * 1. **Unpack (PACKED -> READY):** Cubic Bezier per joint via LegPoser::transitionConfiguration()
 * 2. **Startup (READY -> RUNNING):** Dual quartic Bezier via executeSequenceInternal()
 * 3. **Shutdown (RUNNING -> READY):** Reverse of startup H/V transitions
 * 4. **Pack (READY -> PACKED):** Cubic Bezier per joint via transitionConfiguration()
 *
 * @author HexaMotion Team
 * @version 1.0
 * @date 2025
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

static const char *LEG_NAMES[NUM_LEGS] = {"AR(0)", "BR(1)", "CR(2)", "CL(3)", "BL(4)", "AL(5)"};

static double toDeg(double rad) { return math_utils::radiansToDegrees(rad); }

static const char *robotStateName(RobotState s) {
    switch (s) {
    case ROBOT_UNKNOWN:
        return "UNKNOWN";
    case ROBOT_PACKED:
        return "PACKED";
    case ROBOT_READY:
        return "READY";
    case ROBOT_RUNNING:
        return "RUNNING";
    default:
        return "?";
    }
}

/** Print compact header for the all-legs table. */
static void printAllLegsHeader(const char *phase_name) {
    std::cout << "\n=== " << phase_name << " ===" << std::endl;
    std::cout << std::left << std::setw(6) << "Iter";
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "| " << std::setw(8) << LEG_NAMES[i]
                  << std::setw(8) << "Coxa"
                  << std::setw(8) << "Femur"
                  << std::setw(8) << "Tibia ";
    }
    std::cout << "| State" << std::endl;
    std::cout << std::string(6 + NUM_LEGS * 35 + 10, '-') << std::endl;
}

/** Print one row with all legs' joint angles. */
static void printAllLegsRow(int iter, const LocomotionSystem &sys, RobotState rs) {
    std::cout << std::fixed << std::setprecision(1)
              << std::left << std::setw(6) << iter;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles a = leg.getJointAngles();
        std::cout << "| " << std::setw(8) << ""
                  << std::setw(8) << toDeg(a.coxa)
                  << std::setw(8) << toDeg(a.femur)
                  << std::setw(8) << toDeg(a.tibia);
    }
    std::cout << "| " << robotStateName(rs) << std::endl;
}

/** Print detailed tip positions for all legs. */
static void printAllLegsTipPositions(int iter, const LocomotionSystem &sys) {
    std::cout << "  Tip positions at iter " << iter << ":" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D tip = sys.getLeg(i).getCurrentTipPositionGlobal();
        std::cout << "    " << LEG_NAMES[i] << ": ("
                  << std::fixed << std::setprecision(2)
                  << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }
}

/** Validate symmetry between opposite leg pairs after a transition. */
static bool validateOppositeSymmetry(const LocomotionSystem &sys, const char *phase_name) {
    const int pairs[][2] = {{0, 5}, {1, 4}, {2, 3}};
    const char *pair_names[] = {"AR/AL", "BR/BL", "CR/CL"};
    constexpr double ANGLE_TOLERANCE_DEG = 1.0;
    bool all_ok = true;

    std::cout << "\n  Symmetry check (" << phase_name << "):" << std::endl;
    for (int p = 0; p < 3; ++p) {
        int a = pairs[p][0], b = pairs[p][1];
        JointAngles ja = sys.getLeg(a).getJointAngles();
        JointAngles jb = sys.getLeg(b).getJointAngles();

        double femur_diff = std::abs(toDeg(ja.femur) - toDeg(jb.femur));
        double tibia_diff = std::abs(toDeg(ja.tibia) - toDeg(jb.tibia));

        bool ok = (femur_diff < ANGLE_TOLERANCE_DEG) && (tibia_diff < ANGLE_TOLERANCE_DEG);
        std::cout << "    " << pair_names[p]
                  << ": femur_diff=" << std::fixed << std::setprecision(2) << femur_diff
                  << " deg, tibia_diff=" << tibia_diff << " deg "
                  << (ok ? "OK" : "WARN") << std::endl;
        if (!ok)
            all_ok = false;
    }
    return all_ok;
}

int main() {
    std::cout << "============================================================================" << std::endl;
    std::cout << "   BEZIER TRANSITION VISUALIZATION TEST — ALL LEGS (6 legs simultaneous)" << std::endl;
    std::cout << "============================================================================" << std::endl;

    // ── 1. Initialize ──────────────────────────────────────────────────────────
    Parameters p = createDefaultParameters();
    enableConfiguredPackedUnpackedPoses(p);

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    // Enable Bezier-based startup/shutdown sequences (not direct mode)
    pose_config.start_up_sequence = true;

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    StateController *sc = sys.getStateController();
    assert(sc != nullptr);

    // Show initial state
    std::cout << "\nInitial state: " << robotStateName(sc->getRobotState()) << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = sys.getLeg(i).getJointAngles();
        std::cout << "  " << LEG_NAMES[i]
                  << ": Coxa=" << std::fixed << std::setprecision(1) << toDeg(a.coxa)
                  << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
    }

    // Configure gait (needed by walk controller internals) but do NOT set any velocity.
    // Walking is not required — we only exercise state transitions with Bezier curves.
    // Without velocity, the walk controller stays in WALK_STOPPED, allowing instant shutdown.
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    sys.setGaitConfiguration(tripod_gait);
    sys.startWalking(); // requests ROBOT_RUNNING (triggers PACKED->READY->RUNNING)

    // ── 2. FULL STARTUP: PACKED -> READY -> RUNNING ────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASES 1+2: PACKED -> READY (Cubic) -> RUNNING (Quartic)" << std::endl;
    std::cout << "===================================================================" << std::endl;

    printAllLegsHeader("STARTUP CYCLE — All Legs");

    int total_iters = 0;
    int unpack_iters = 0;
    int startup_iters = 0;
    const int MAX_ITERS = 1000;
    RobotState prev_state = sc->getRobotState();
    bool unpack_done = false;
    bool startup_done = false;

    while (total_iters < MAX_ITERS) {
        sys.update();
        total_iters++;

        RobotState cur = sc->getRobotState();

        if (!unpack_done && cur == ROBOT_PACKED)
            unpack_iters++;
        if (prev_state == ROBOT_PACKED && cur != ROBOT_PACKED && !unpack_done) {
            unpack_iters++;
            unpack_done = true;
            std::cout << "  >>> Unpack done at iteration " << total_iters
                      << " (" << unpack_iters << " iters)" << std::endl;
            validateOppositeSymmetry(sys, "post-unpack");
        }
        if (unpack_done && !startup_done && cur != ROBOT_RUNNING)
            startup_iters++;
        if (prev_state != ROBOT_RUNNING && cur == ROBOT_RUNNING && !startup_done) {
            startup_iters++;
            startup_done = true;
            std::cout << "  >>> Startup done at iteration " << total_iters
                      << " (" << startup_iters << " iters)" << std::endl;
        }

        // Print every 5 iters or on state change
        if (total_iters <= 5 || total_iters % 5 == 0 || cur != prev_state) {
            printAllLegsRow(total_iters, sys, cur);
        }
        if (total_iters % 25 == 0) {
            printAllLegsTipPositions(total_iters, sys);
        }

        if (cur == ROBOT_RUNNING)
            break;
        prev_state = cur;
    }

    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Did not reach RUNNING after " << MAX_ITERS << " iters." << std::endl;
        return 1;
    }

    std::cout << "\nRUNNING state reached. Standing pose:" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles a = leg.getJointAngles();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        std::cout << "  " << LEG_NAMES[i]
                  << ": angles=(" << std::setprecision(1)
                  << toDeg(a.coxa) << ", " << toDeg(a.femur) << ", " << toDeg(a.tibia) << ")"
                  << " tip=(" << std::setprecision(2) << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }
    validateOppositeSymmetry(sys, "standing/running");

    // ── 3. SHUTDOWN (RUNNING -> READY) ─────────────────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASE 3: SHUTDOWN (RUNNING -> READY) — Quartic Bezier" << std::endl;
    std::cout << "===================================================================" << std::endl;

    // Request READY directly — walk controller is already WALK_STOPPED
    // because no velocity was ever commanded.
    sc->requestRobotState(ROBOT_READY);

    int shutdown_iters = 0;

    printAllLegsHeader("SHUTDOWN — All Legs");

    int shutdown_iters_loop = 0;
    const int MAX_SHUTDOWN = 800;
    prev_state = sc->getRobotState();

    while (shutdown_iters_loop < MAX_SHUTDOWN) {
        sys.update();
        shutdown_iters_loop++;

        RobotState cur = sc->getRobotState();
        if (shutdown_iters_loop <= 5 || shutdown_iters_loop % 10 == 0 || cur != prev_state) {
            printAllLegsRow(shutdown_iters_loop, sys, cur);
        }

        if (cur == ROBOT_READY) {
            std::cout << "  >>> Shutdown done at iteration " << shutdown_iters_loop << std::endl;
            shutdown_iters = shutdown_iters_loop;
            break;
        }
        prev_state = cur;
    }

    if (sc->getRobotState() != ROBOT_READY) {
        std::cout << "WARNING: Shutdown did not complete after " << shutdown_iters_loop << " iters." << std::endl;
        shutdown_iters = shutdown_iters_loop;
    }

    // ── 4. PACK (READY -> PACKED) ──────────────────────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASE 4: PACK (READY -> PACKED) — Cubic Bezier" << std::endl;
    std::cout << "===================================================================" << std::endl;

    sc->requestRobotState(ROBOT_PACKED);

    int pack_iters = 0;

    printAllLegsHeader("PACK — All Legs");

    int pack_iters_loop = 0;
    const int MAX_PACK = 500;
    prev_state = sc->getRobotState();

    while (pack_iters_loop < MAX_PACK) {
        sys.update();
        pack_iters_loop++;

        RobotState cur = sc->getRobotState();
        if (pack_iters_loop <= 5 || pack_iters_loop % 10 == 0 || cur != prev_state) {
            printAllLegsRow(pack_iters_loop, sys, cur);
        }

        if (cur == ROBOT_PACKED) {
            std::cout << "  >>> Pack done at iteration " << pack_iters_loop << std::endl;
            validateOppositeSymmetry(sys, "post-pack");
            pack_iters = pack_iters_loop;
            break;
        }
        prev_state = cur;
    }

    if (pack_iters_loop >= MAX_PACK) {
        std::cout << "WARNING: Pack did not complete after " << pack_iters_loop << " iters." << std::endl;
        pack_iters = pack_iters_loop;
    }

    // ── 5. SUMMARY ─────────────────────────────────────────────────────────────
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "              BEZIER TRANSITION SUMMARY — ALL LEGS" << std::endl;
    std::cout << "============================================================================" << std::endl;
    std::cout << std::left
              << std::setw(30) << "Phase"
              << std::setw(22) << "Bezier Type"
              << std::setw(14) << "Iterations"
              << "Method" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    std::cout << std::setw(30) << "1. Unpack (PACKED->READY)"
              << std::setw(22) << "Cubic per-joint"
              << std::setw(14) << unpack_iters
              << "transitionConfiguration()" << std::endl;
    std::cout << std::setw(30) << "2. Startup (READY->RUNNING)"
              << std::setw(22) << "Dual quartic"
              << std::setw(14) << startup_iters
              << "stepToPosition()" << std::endl;
    std::cout << std::setw(30) << "3. Shutdown (RUNNING->READY)"
              << std::setw(22) << "Dual quartic"
              << std::setw(14) << shutdown_iters
              << "stepToPosition()" << std::endl;
    std::cout << std::setw(30) << "4. Pack (READY->PACKED)"
              << std::setw(22) << "Cubic per-joint"
              << std::setw(14) << pack_iters
              << "transitionConfiguration()" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    int total = unpack_iters + startup_iters + shutdown_iters + pack_iters;
    std::cout << std::setw(30) << "TOTAL"
              << std::setw(22) << ""
              << std::setw(14) << total
              << std::endl;
    std::cout << "============================================================================" << std::endl;

    // Final state
    std::cout << "\nFinal state: " << robotStateName(sc->getRobotState()) << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = sys.getLeg(i).getJointAngles();
        std::cout << "  " << LEG_NAMES[i]
                  << ": Coxa=" << std::fixed << std::setprecision(1) << toDeg(a.coxa)
                  << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
    }

    bool success = (sc->getRobotState() == ROBOT_PACKED);
    if (success) {
        std::cout << "\nTEST PASSED" << std::endl;
    } else {
        std::cerr << "\nTEST FAILED: Final state = " << robotStateName(sc->getRobotState())
                  << " (expected PACKED)." << std::endl;
    }

    return success ? 0 : 1;
}
