/**
 * @file bezier_transition_single_leg_test.cpp
 * @brief Visualizes Bézier curves during pack, unpack and standing pose transitions for a single leg.
 *
 * This test exercises the full LocomotionSystem state machine through PACKED → READY → RUNNING
 * transitions, displaying detailed per-iteration joint angle data for Leg 0 (AR) to observe:
 *
 * 1. **Unpack (PACKED → READY):** Cubic Bézier interpolation via LegPoser::transitionConfiguration()
 *    from packed joints to unpacked joints.
 * 2. **Startup Sequence (READY → RUNNING):** Uses stepToPosition() quartic Bézier inside
 *    BodyPoseController::executeSequenceInternal() for multi-step H/V transitions.
 * 3. **Shutdown (RUNNING → READY):** Reverse startup via executeShutdownSequence().
 * 4. **Pack (READY → PACKED):** Cubic Bézier via transitionConfiguration() from
 *    unpacked back to packed joints.
 *
 * Requires start_up_sequence=true in BodyPoseConfiguration so the state machine uses
 * the full Bézier-based startup/shutdown paths (not direct mode).
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

static double toDeg(double rad) { return math_utils::radiansToDegrees(rad); }

/** Print column header for the iteration table. */
static void printTableHeader(const char *phase_name) {
    std::cout << "\n=== " << phase_name << " ===" << std::endl;
    std::cout << std::left
              << std::setw(6) << "Iter"
              << std::setw(10) << "Coxa(d)"
              << std::setw(10) << "Femur(d)"
              << std::setw(10) << "Tibia(d)"
              << std::setw(30) << "Tip Position (x, y, z)"
              << "RobotState" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
}

/** State name helper. */
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

/** Print a single iteration row for leg 0. */
static void printLegRow(int iter, const Leg &leg, RobotState rs) {
    JointAngles a = leg.getJointAngles();
    Point3D tip = leg.getCurrentTipPositionGlobal();
    std::cout << std::fixed << std::setprecision(2)
              << std::left
              << std::setw(6) << iter
              << std::setw(10) << toDeg(a.coxa)
              << std::setw(10) << toDeg(a.femur)
              << std::setw(10) << toDeg(a.tibia);
    std::cout << "(" << std::setw(8) << tip.x << ", " << std::setw(8) << tip.y << ", " << std::setw(8) << tip.z << ")  ";
    std::cout << robotStateName(rs) << std::endl;
}

int main() {
    std::cout << "============================================================================" << std::endl;
    std::cout << "     BEZIER TRANSITION VISUALIZATION TEST — SINGLE LEG (Leg 0 / AR)" << std::endl;
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

    // Show initial joint angles for leg 0
    {
        const Leg &leg = sys.getLeg(0);
        JointAngles a = leg.getJointAngles();
        std::cout << "\nInitial joint angles (Leg 0):" << std::endl;
        std::cout << "  Coxa:  " << toDeg(a.coxa) << " deg" << std::endl;
        std::cout << "  Femur: " << toDeg(a.femur) << " deg" << std::endl;
        std::cout << "  Tibia: " << toDeg(a.tibia) << " deg" << std::endl;
        std::cout << "  RobotState: " << robotStateName(sc->getRobotState()) << std::endl;
    }

    // ── 2. FULL CYCLE: PACKED → READY → RUNNING ────────────────────────────────
    // Configure gait (needed by walk controller internals) but do NOT set any velocity.
    // Walking is not required for this test — we only exercise state transitions
    // that use Bezier curves (pack/unpack/startup/shutdown). Without velocity,
    // the walk controller stays in WALK_STOPPED, allowing instant shutdown.
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    sys.setGaitConfiguration(tripod_gait);
    sys.startWalking(); // requests ROBOT_RUNNING (triggers PACKED->READY->RUNNING)

    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 1+2: PACKED -> READY (Cubic Bezier) -> RUNNING (Quartic Bezier)" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    printTableHeader("STARTUP CYCLE — Leg 0 Joint Angles per Iteration");

    int total_startup_iters = 0;
    int unpack_iters = 0;
    int startup_iters = 0;
    const int MAX_ITERS = 1000;
    RobotState prev_state = sc->getRobotState();
    bool unpack_detected = false;
    bool startup_detected = false;

    while (total_startup_iters < MAX_ITERS) {
        sys.update();
        total_startup_iters++;

        RobotState cur_state = sc->getRobotState();

        // Track phase transitions
        if (!unpack_detected && cur_state == ROBOT_PACKED) {
            unpack_iters++;
        }
        if (prev_state == ROBOT_PACKED && cur_state != ROBOT_PACKED && !unpack_detected) {
            unpack_iters++;
            unpack_detected = true;
            std::cout << "  >>> Unpack completed at iteration " << total_startup_iters
                      << " (cubic Bezier, " << unpack_iters << " iters)" << std::endl;
        }
        if (unpack_detected && !startup_detected && cur_state != ROBOT_RUNNING) {
            startup_iters++;
        }
        if (prev_state != ROBOT_RUNNING && cur_state == ROBOT_RUNNING && !startup_detected) {
            startup_iters++;
            startup_detected = true;
            std::cout << "  >>> Startup completed at iteration " << total_startup_iters
                      << " (quartic Bezier, " << startup_iters << " iters)" << std::endl;
        }

        // Print every N iterations to keep output manageable
        if (total_startup_iters <= 10 || total_startup_iters % 5 == 0 || cur_state != prev_state) {
            printLegRow(total_startup_iters, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_RUNNING) {
            break;
        }

        prev_state = cur_state;
    }

    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Failed to reach RUNNING state after " << MAX_ITERS << " iterations." << std::endl;
        return 1;
    }

    {
        const Leg &leg = sys.getLeg(0);
        JointAngles a = leg.getJointAngles();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        std::cout << "\nRUNNING state reached. Leg 0 final:" << std::endl;
        std::cout << "  Angles: Coxa=" << toDeg(a.coxa) << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
        std::cout << "  Tip: (" << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }

    // ── 3. SHUTDOWN (RUNNING → READY): Quartic Bézier via stepToPosition ──
    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 3: SHUTDOWN (RUNNING -> READY) — Quartic Bezier" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    // Request READY directly — walk controller is already WALK_STOPPED
    // because no velocity was ever commanded.
    sc->requestRobotState(ROBOT_READY);

    int shutdown_iters = 0;

    printTableHeader("SHUTDOWN — Leg 0 Joint Angles per Iteration");

    int shutdown_iters_loop = 0;
    const int MAX_SHUTDOWN_ITERS = 800;
    prev_state = sc->getRobotState();

    while (shutdown_iters_loop < MAX_SHUTDOWN_ITERS) {
        sys.update();
        shutdown_iters_loop++;

        RobotState cur_state = sc->getRobotState();

        // Print every N iterations or on state change
        if (shutdown_iters_loop <= 10 || shutdown_iters_loop % 5 == 0 || cur_state != prev_state) {
            printLegRow(shutdown_iters_loop, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_READY) {
            std::cout << "  >>> Shutdown completed at iteration " << shutdown_iters_loop << std::endl;
            shutdown_iters = shutdown_iters_loop;
            break;
        }
        prev_state = cur_state;
    }

    if (sc->getRobotState() != ROBOT_READY) {
        std::cout << "WARNING: Shutdown did not reach READY after " << shutdown_iters_loop << " iterations." << std::endl;
        shutdown_iters = shutdown_iters_loop;
    }

    // ── 4. PACK (READY → PACKED): Cubic Bézier via transitionConfiguration ──
    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 4: PACK (READY -> PACKED) — Cubic Bezier" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    sc->requestRobotState(ROBOT_PACKED);

    int pack_iters = 0;

    printTableHeader("PACK — Leg 0 Joint Angles per Iteration");

    int pack_iters_loop = 0;
    const int MAX_PACK_ITERS = 500;
    prev_state = sc->getRobotState();

    while (pack_iters_loop < MAX_PACK_ITERS) {
        sys.update();
        pack_iters_loop++;

        RobotState cur_state = sc->getRobotState();

        if (pack_iters_loop <= 10 || pack_iters_loop % 5 == 0 || cur_state != prev_state) {
            printLegRow(pack_iters_loop, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_PACKED) {
            std::cout << "  >>> Pack completed at iteration " << pack_iters_loop << std::endl;
            const Leg &leg = sys.getLeg(0);
            JointAngles a = leg.getJointAngles();
            std::cout << "  Final packed angles: Coxa=" << toDeg(a.coxa)
                      << ", Femur=" << toDeg(a.femur)
                      << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
            pack_iters = pack_iters_loop;
            break;
        }
        prev_state = cur_state;
    }

    if (pack_iters_loop >= MAX_PACK_ITERS) {
        std::cout << "WARNING: Pack did not complete within " << pack_iters_loop << " iterations." << std::endl;
        pack_iters = pack_iters_loop;
    }

    // ── 5. SUMMARY ─────────────────────────────────────────────────────────────
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "                    BEZIER TRANSITION SUMMARY (Single Leg)" << std::endl;
    std::cout << "============================================================================" << std::endl;
    std::cout << "Phase                     | Bezier Type       | Iterations | Method" << std::endl;
    std::cout << "--------------------------+-------------------+------------+-------------------------" << std::endl;
    std::cout << std::left
              << std::setw(26) << "Unpack (PACKED->READY)"
              << "| " << std::setw(18) << "Cubic per-joint"
              << "| " << std::setw(11) << unpack_iters
              << "| transitionConfiguration()" << std::endl;
    std::cout << std::setw(26) << "Startup (READY->RUNNING)"
              << "| " << std::setw(18) << "Dual quartic"
              << "| " << std::setw(11) << startup_iters
              << "| stepToPosition()" << std::endl;
    std::cout << std::setw(26) << "Shutdown (RUNNING->READY)"
              << "| " << std::setw(18) << "Dual quartic"
              << "| " << std::setw(11) << shutdown_iters
              << "| stepToPosition()" << std::endl;
    std::cout << std::setw(26) << "Pack (READY->PACKED)"
              << "| " << std::setw(18) << "Cubic per-joint"
              << "| " << std::setw(11) << pack_iters
              << "| transitionConfiguration()" << std::endl;
    std::cout << "--------------------------+-------------------+------------+-------------------------" << std::endl;
    int total = unpack_iters + startup_iters + shutdown_iters + pack_iters;
    std::cout << std::setw(26) << "TOTAL"
              << "| " << std::setw(18) << ""
              << "| " << std::setw(11) << total
              << "|" << std::endl;
    std::cout << "============================================================================" << std::endl;

    // Final verdict
    bool success = (sc->getRobotState() == ROBOT_PACKED);
    if (success) {
        std::cout << "\nTEST PASSED" << std::endl;
    } else {
        std::cerr << "\nTEST FAILED: Final state = " << robotStateName(sc->getRobotState())
                  << " (expected PACKED)." << std::endl;
    }

    return success ? 0 : 1;
}
