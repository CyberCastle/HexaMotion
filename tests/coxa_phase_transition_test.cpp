/**
 * @file coxa_phase_transition_test.cpp
 * @brief Test equivalent to tripod_walk_visualization_test focused on coxas.
 *
 * This test is equivalent to tripod_walk_visualization_test but shows
 * only the movement of COXA joints during swing/stance phase transitions
 * in tripod gait.
 *
 * The test:
 * 1. Initializes the LocomotionSystem the same as tripod_walk_visualization_test
 * 2. Runs the full startup sequence
 * 3. Monitors only coxa angles during transitions
 * 4. Uses timing dynamically derived from StepCycle (iterations per phase computed)
 * 5. Finishes when all legs complete the required transitions
 *
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// --------------------------------------------------------------------------------------
// Default parameters (can be overridden by CLI)
// --------------------------------------------------------------------------------------
static double g_test_velocity = 300.0;        // mm/s
static int g_required_swing_transitions = 20; // STANCE->SWING transitions per leg
// Default needs to accommodate the configured transitions.
// With StepCycle period=104, 20 transitions need ~20*104=2080 steps.
static int g_max_steps = 2600;                    // Safety limit
static bool g_show_only_phase_transitions = true; // Compact mode by default
static double g_sym_threshold_stance_deg = 3.0;   // |sum(delta)| maximum allowed in STANCE
static double g_sym_threshold_swing_deg = 4.0;    // |sum(delta)| maximum allowed in SWING (more tolerant)

// Global metric accumulators (maximum absolute values observed)
static double g_max_abs_sum_stance_pair[3] = {0, 0, 0}; // pairs (0,5) (1,4) (2,3)
static double g_max_abs_sum_swing_pair[3] = {0, 0, 0};
static int g_sym_violations_stance = 0;
static int g_sym_violations_swing = 0;

static void printHelpAndExit() {
    std::cout << "Usage: ./coxa_phase_transition_test [options]\n"
              << "  --transitions N    Number of STANCE->SWING transitions per leg (default 20)\n"
              << "  --velocity V       Linear velocity mm/s (default 300)\n"
              << "  --max-steps M      Max simulation steps (default 2600)\n"
              << "  --full             Show ALL iterations\n"
              << "  --phases-only      Phase transitions only (default)\n"
              << "  --help             This help\n";
    std::exit(0);
}

static void parseArgs(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto needVal = [&](const char *flag) {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << std::endl;
                std::exit(1);
            }
        };
        if (a == "--help") {
            printHelpAndExit();
        } else if (a == "--transitions") {
            needVal("--transitions");
            g_required_swing_transitions = std::atoi(argv[++i]);
        } else if (a == "--velocity") {
            needVal("--velocity");
            g_test_velocity = std::atof(argv[++i]);
        } else if (a == "--max-steps") {
            needVal("--max-steps");
            g_max_steps = std::atoi(argv[++i]);
        } else if (a == "--full") {
            g_show_only_phase_transitions = false;
        } else if (a == "--phases-only") {
            g_show_only_phase_transitions = true;
        } else if (a == "--sym-thr-stance") {
            needVal("--sym-thr-stance");
            g_sym_threshold_stance_deg = std::atof(argv[++i]);
        } else if (a == "--sym-thr-swing") {
            needVal("--sym-thr-swing");
            g_sym_threshold_swing_deg = std::atof(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << a << std::endl;
            printHelpAndExit();
        }
    }
    if (g_required_swing_transitions < 1)
        g_required_swing_transitions = 1;
    if (g_max_steps < 200)
        g_max_steps = 200; // minimum safety
}

/**
 * @brief Prints the test header.
 */
static void printTestHeader() {
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "                      TRIPOD GAIT COXA MOVEMENT VALIDATION TEST" << std::endl;
    std::cout << "=======================================================================================================" << std::endl;
    if (g_show_only_phase_transitions) {
        std::cout << "Mode: TRANSITIONS ONLY (initial state + S->W / W->S)." << std::endl;
    } else {
        std::cout << "Mode: ALL ITERATIONS (full detail)." << std::endl;
    }
    std::cout << "With OpenSHC timing: Each phase (stance/swing) uses dynamic normalized iterations (not fixed at 52)." << std::endl;
    std::cout << "Goal: " << g_required_swing_transitions << " STANCE->SWING transitions per leg." << std::endl;
    std::cout << "Estimated duration (approx): ~" << (g_required_swing_transitions * 104) << " steps (reference only)." << std::endl;
    std::cout << "Velocidad: " << g_test_velocity << " mm/s" << std::endl;

    std::cout << std::left << std::setw(8) << "Step"
              << std::setw(6) << "AR"
              << std::setw(6) << "BR"
              << std::setw(6) << "CR"
              << std::setw(6) << "CL"
              << std::setw(6) << "BL"
              << std::setw(6) << "AL"
              << std::setw(12) << "Phases"
              << "Transitions + Metrics" << std::endl;
    std::cout << "       (Coxa angles in degrees)                                    R(S/W)=Radius Stance/Swing  Sym=Symmetry(sum,diff)" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Prints the coxa states of all legs in one step.
 * @param sys The LocomotionSystem instance.
 * @param step The current step number.
 * @param transition_counts Array with transition counts for each leg.
 * @param leg_phase_iterations Array with current phase iterations for each leg.
 * @param swing_iterations_per_cycle Expected swing iterations per cycle.
 * @param stance_iterations_per_cycle Expected stance iterations per cycle.
 */
static void printCoxaStates(const LocomotionSystem &sys, int step, const int transition_counts[NUM_LEGS],
                            const int leg_phase_iterations[NUM_LEGS], int swing_iterations_per_cycle,
                            int stance_iterations_per_cycle,
                            const double stance_start_coxa_rad[NUM_LEGS],
                            const double initial_coxa_rad[NUM_LEGS]) {

    std::cout << std::left << std::setw(8) << step;

    // Print coxa angles for each leg
    const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    double coxa_deg[NUM_LEGS];               // Absolute coxa angle (deg)
    double coxa_delta_initial_deg[NUM_LEGS]; // Delta from initial baseline (deg)
    double coxa_delta_stance_deg[NUM_LEGS];  // Delta from stance start (deg)
    double arc_mm[NUM_LEGS];
    double radius_mm[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles angles = leg.getJointAngles();
        double coxa_angle = angles.coxa; // rad absolute
        coxa_deg[i] = math_utils::radiansToDegrees(coxa_angle);
        coxa_delta_initial_deg[i] = math_utils::radiansToDegrees(coxa_angle - initial_coxa_rad[i]);
        coxa_delta_stance_deg[i] = math_utils::radiansToDegrees(coxa_angle - stance_start_coxa_rad[i]);
        // Planar radius from the leg base to current tip (to estimate theoretical tangential arc)
        Point3D base = leg.getBasePosition();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        double dx = tip.x - base.x;
        double dy = tip.y - base.y;
        double r = std::sqrt(dx * dx + dy * dy);
        radius_mm[i] = r;
        double delta_since_stance = coxa_angle - stance_start_coxa_rad[i]; // rad within the current stance phase (approx)
        arc_mm[i] = r * delta_since_stance;                                // approximate arc length
        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << coxa_deg[i];
    }

    // Print current phases (compact)
    std::cout << std::setw(12);
    std::string phases = "";
    for (int i = 0; i < NUM_LEGS; ++i) {
        StepPhase phase = sys.getLeg(i).getStepPhase();
        phases += (phase == STANCE_PHASE ? "S" : "W");
    }
    std::cout << phases;

    // Print transition counts
    std::cout << " ";
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << LEG_NAMES[i] << ":" << transition_counts[i] << " ";
    }

    // Add additional metrics on the same line
    // Average radii of legs in stance/swing
    double avg_radius_stance = 0, avg_radius_swing = 0;
    int stance_count = 0, swing_count = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        StepPhase phase = sys.getLeg(i).getStepPhase();
        if (phase == STANCE_PHASE) {
            avg_radius_stance += radius_mm[i];
            stance_count++;
        } else {
            avg_radius_swing += radius_mm[i];
            swing_count++;
        }
    }
    if (stance_count > 0)
        avg_radius_stance /= stance_count;
    if (swing_count > 0)
        avg_radius_swing /= swing_count;

    // Symmetry metrics for opposite pairs (0,5) (1,4) (2,3)
    // Note: The original metrics used absolute angles; even though opposite offsets now cancel out,
    // we prefer using delta-based metrics relative to the initial angle (baseline) and only display them
    // when BOTH legs are in the same STANCE phase to isolate trajectory deviations.
    auto pairMetricsAbs = [&](int a, int b) {
        double sum = coxa_deg[a] + coxa_deg[b];
        double diff = coxa_deg[a] - coxa_deg[b];
        return std::make_pair(sum, diff);
    };
    auto pairMetricsDelta = [&](int a, int b) {
        double sum = coxa_delta_initial_deg[a] + coxa_delta_initial_deg[b];
        double diff = coxa_delta_initial_deg[a] - coxa_delta_initial_deg[b];
        return std::make_pair(sum, diff);
    };

    auto p05_abs = pairMetricsAbs(0, 5);
    auto p14_abs = pairMetricsAbs(1, 4);
    auto p23_abs = pairMetricsAbs(2, 3);

    // Delta (baseline) metrics – stance and swing evaluated separately
    auto bothStance = [&](int a, int b) {
        return sys.getLeg(a).getStepPhase() == STANCE_PHASE && sys.getLeg(b).getStepPhase() == STANCE_PHASE;
    };
    auto bothSwing = [&](int a, int b) {
        return sys.getLeg(a).getStepPhase() == SWING_PHASE && sys.getLeg(b).getStepPhase() == SWING_PHASE;
    };
    std::string p05_delta_str = "--";
    std::string p14_delta_str = "--";
    std::string p23_delta_str = "--";
    std::string p05_delta_swing_str = "--";
    std::string p14_delta_swing_str = "--";
    std::string p23_delta_swing_str = "--";
    if (bothStance(0, 5)) {
        auto m = pairMetricsDelta(0, 5);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p05_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[0] = std::max(g_max_abs_sum_stance_pair[0], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    if (bothStance(1, 4)) {
        auto m = pairMetricsDelta(1, 4);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p14_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[1] = std::max(g_max_abs_sum_stance_pair[1], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    if (bothStance(2, 3)) {
        auto m = pairMetricsDelta(2, 3);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p23_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[2] = std::max(g_max_abs_sum_stance_pair[2], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    // Swing symmetry tracking
    if (bothSwing(0, 5)) {
        auto m = pairMetricsDelta(0, 5);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p05_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[0] = std::max(g_max_abs_sum_swing_pair[0], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }
    if (bothSwing(1, 4)) {
        auto m = pairMetricsDelta(1, 4);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p14_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[1] = std::max(g_max_abs_sum_swing_pair[1], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }
    if (bothSwing(2, 3)) {
        auto m = pairMetricsDelta(2, 3);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p23_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[2] = std::max(g_max_abs_sum_swing_pair[2], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }

    std::cout << " R(S/W):" << std::fixed << std::setprecision(0) << avg_radius_stance << "/" << avg_radius_swing
              << " AbsSym05:" << std::setprecision(1) << p05_abs.first << "," << p05_abs.second
              << " 14:" << p14_abs.first << "," << p14_abs.second
              << " 23:" << p23_abs.first << "," << p23_abs.second
              << " dSym05:" << p05_delta_str
              << " d14:" << p14_delta_str
              << " d23:" << p23_delta_str
              << " dSymW05:" << p05_delta_swing_str
              << " dW14:" << p14_delta_swing_str
              << " dW23:" << p23_delta_swing_str;

    std::cout << std::endl;
}

/**
 * @brief Checks if all legs have completed the required transitions.
 * @param transition_counts Array with transition counts for each leg.
 * @return True if the test goal is met, false otherwise.
 */
static bool allLegsCompletedTransitions(const int transition_counts[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (transition_counts[i] < g_required_swing_transitions) {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv) {
    parseArgs(argc, argv);
    std::cout << "=== Coxa Phase Transition Test (Equivalent to Tripod Walk Visualization) ===" << std::endl;

    // 1. Basic initialization
    Parameters p = createDefaultParameters();
    p.max_velocity = 1000.0; // mm/s (high limit to avoid interference)
    enableConfiguredPackedUnpackedPoses(p);
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    pose_config.start_up_sequence = true;

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    // 2. Start in Standing Pose
    if (!sys.setStandingPose()) {
        std::cerr << "ERROR: Failed to set standing pose." << std::endl;
        return 1;
    }
    std::cout << "Robot in standing pose. All legs in STANCE phase." << std::endl;

    // Verify initial pose (coxa only)
    std::cout << "\nINITIAL POSE VERIFICATION (coxa angles only):" << std::endl;
    std::cout << "Leg    Phase  Coxa (degrees)" << std::endl;
    std::cout << "-----------------------------" << std::endl;
    const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles angles = leg.getJointAngles();
        StepPhase phase = leg.getStepPhase();

        std::cout << std::left << std::setw(7) << LEG_NAMES[i]
                  << std::setw(7) << (phase == STANCE_PHASE ? "S" : "W")
                  << std::fixed << std::setprecision(2) << math_utils::radiansToDegrees(angles.coxa) << std::endl;
    }
    std::cout << "-----------------------------\n"
              << std::endl;

    // 3. Configure and start Tripod Gait (identical to tripod_walk_visualization_test)
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);

    double leg_reach = RobotModel::computeStandingHorizontalReach(p);
    std::cout << "Leg reach (horizontal) = " << leg_reach << " mm" << std::endl;
    tripod_gait.step_length = leg_reach * 2;
    if (!sys.setGaitConfiguration(tripod_gait)) {
        std::cerr << "ERROR: Failed to set gait type." << std::endl;
        return 1;
    }

    sys.walkForward(g_test_velocity);
    if (!sys.startWalking()) {
        std::cerr << "ERROR: Failed to start walking (startup sequence)." << std::endl;
        return 1;
    }

    // Run startup sequence
    std::cout << "Running startup sequence..." << std::endl;

    int startup_sequence_attempts = 0;
    const int MAX_STARTUP_SEQUENCE_ATTEMPTS = 2000;

    while (sys.getRobotState() != ROBOT_RUNNING && startup_sequence_attempts < MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        sys.update();
        startup_sequence_attempts++;

        if (startup_sequence_attempts % 25 == 0) {
            std::cout << "Startup attempt " << startup_sequence_attempts
                      << "  Progreso=" << sys.getStartupProgressPercent() << "%" << std::endl;
        }
    }

    if (sys.getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Startup sequence failed after " << startup_sequence_attempts << " attempts." << std::endl;
        return 1;
    }
    std::cout << "Startup sequence completed after " << startup_sequence_attempts << " attempts." << std::endl;

    std::cout << "Starting coxa movement analysis..." << std::endl;
    printTestHeader();

    // 4. Main simulation loop with timing verification
    std::cout << "=== SYNCHRONIZATION VERIFICATION WITH trajectory_tip_position_test ===" << std::endl;

    auto first_leg_stepper = sys.getWalkController()->getLegStepper(0);
    if (!first_leg_stepper) {
        std::cerr << "ERROR: Could not get the LegStepper." << std::endl;
        return 1;
    }

    StepCycle actual_step_cycle = first_leg_stepper->getStepCycle();
    double time_delta = sys.getRobotModel().getTimeDelta();

    // Use exactly the same formula as trajectory_tip_position_test
    int swing_iterations_per_cycle = (int)((double(actual_step_cycle.swing_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int stance_iterations_per_cycle = (int)((double(actual_step_cycle.stance_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int total_iterations_per_cycle = swing_iterations_per_cycle + stance_iterations_per_cycle;

    std::cout << "Active StepCycle:" << std::endl;
    std::cout << "  swing_iterations_per_cycle: " << swing_iterations_per_cycle << std::endl;
    std::cout << "  stance_iterations_per_cycle: " << stance_iterations_per_cycle << std::endl;
    std::cout << "  total_iterations_per_cycle: " << total_iterations_per_cycle << std::endl;

    // Internal coherence validation: derived iterations must sum to the period
    if (swing_iterations_per_cycle + stance_iterations_per_cycle != actual_step_cycle.period_) {
        std::cerr << "ERROR: Timing inconsistency: swing(" << swing_iterations_per_cycle << ") + stance("
                  << stance_iterations_per_cycle << ") != period(" << actual_step_cycle.period_ << ")" << std::endl;
        return 1; // Fail the test: inconsistent internal state
    }

    std::cout << "Derived iterations: swing=" << swing_iterations_per_cycle << ", stance=" << stance_iterations_per_cycle << std::endl;
    if (swing_iterations_per_cycle != stance_iterations_per_cycle) {
        std::cout << "INFO: swing and stance differ; this is allowed if phases have different durations." << std::endl;
    }
    std::cout << "(Previous reference of 52 removed; now validated against actual StepCycle)." << std::endl;

    // DEBUG: Show offset multipliers of the tripod gait
    std::cout << "\n=== DEBUG: Tripod Gait Offset Multipliers ===" << std::endl;
    auto gait_config = sys.getWalkController()->getCurrentGaitConfig();
    const char *leg_names[] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    std::cout << "Offset multipliers:" << std::endl;
    for (int i = 0; i < 6; i++) {
        int offset = gait_config.offsets.getForLegIndex(i);
        std::cout << "  " << leg_names[i] << ": " << offset << std::endl;
    }
    std::cout << std::endl;

    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Phase iteration counter per leg
    int leg_phase_iterations[NUM_LEGS] = {0};
    StepPhase leg_current_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        leg_current_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Show initial state
    // Track stance start to estimate yaw arc: initialized at the initial pose
    double stance_start_coxa_rad[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        stance_start_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
    }

    // Capture initial coxa baseline (for delta symmetry)
    double initial_coxa_rad[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        initial_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
    }

    printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);

    while (step < g_max_steps) {
        // Update system
        if (!sys.update()) {
            std::cerr << "ERROR: System update failed at step " << step << std::endl;
            return 1;
        }

        // Check transitions and detect any phase change
        bool phase_transition_occurred = false;

        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();

            // Detect STANCE -> SWING transitions
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
            }
            // Detect SWING -> STANCE transitions
            else if (previous_phases[i] == SWING_PHASE && current_phase == STANCE_PHASE) {
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
                // Reset stance start reference for this leg
                stance_start_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
            }
            // If in the same phase, increment counter
            else if (leg_current_phases[i] == current_phase) {
                leg_phase_iterations[i]++;
            }
            // If phase changed without being detected above
            else {
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
            }

            previous_phases[i] = current_phase;
        }

        if (g_show_only_phase_transitions) {
            if (phase_transition_occurred) {
                printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);
            }
        } else {
            printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);
        }

        // Check completion
        if (allLegsCompletedTransitions(transition_counts)) {
            std::cout << "\nSUCCESS: All legs completed " << g_required_swing_transitions << " swing transitions." << std::endl;
            break;
        }

        step++;
    }

    if (step == g_max_steps) {
        std::cerr << "\nERROR: Test reached maximum steps (" << g_max_steps << ") before completion." << std::endl;
        return 1;
    }

    // 5. Stop and return to stance
    std::cout << "\nStopping walking and returning to standing pose..." << std::endl;
    if (!sys.stopWalking()) {
        std::cerr << "ERROR: Failed to initiate stop walking." << std::endl;
        return 1;
    }

    // Run update loop to let StateController orchestrate the shutdown
    int shutdown_attempts = 0;
    const int MAX_SHUTDOWN_ATTEMPTS = 2000;
    while (shutdown_attempts < MAX_SHUTDOWN_ATTEMPTS && sys.getRobotState() == ROBOT_RUNNING) {
        sys.update();
        shutdown_attempts++;
    }
    if (sys.getRobotState() == ROBOT_READY) {
        std::cout << "Shutdown completed after " << shutdown_attempts << " iterations." << std::endl;
    } else {
        std::cerr << "ERROR: Shutdown did not complete after " << shutdown_attempts << " iterations." << std::endl;
        return 1;
    }

    // Final summary
    std::cout << "\n=== FINAL TEST SUMMARY ===" << std::endl;
    std::cout << "Total steps executed: " << step << std::endl;
    std::cout << "STANCE->SWING transitions completed:" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "  " << LEG_NAMES[i] << ": " << transition_counts[i] << "/" << g_required_swing_transitions << std::endl;
    }

    bool success = allLegsCompletedTransitions(transition_counts);

    // Evaluate global symmetry PASS/FAIL
    bool symmetry_ok = (g_sym_violations_stance == 0 && g_sym_violations_swing == 0);
    if (!symmetry_ok) {
        std::cout << "\n[SYMMETRY] Violations detected:" << std::endl;
        if (g_sym_violations_stance)
            std::cout << "  STANCE: violations=" << g_sym_violations_stance << " (max abs sum pairs: ["
                      << g_max_abs_sum_stance_pair[0] << ", " << g_max_abs_sum_stance_pair[1] << ", " << g_max_abs_sum_stance_pair[2] << "]) threshold=" << g_sym_threshold_stance_deg << "°" << std::endl;
        if (g_sym_violations_swing)
            std::cout << "  SWING: violations=" << g_sym_violations_swing << " (max abs sum pairs: ["
                      << g_max_abs_sum_swing_pair[0] << ", " << g_max_abs_sum_swing_pair[1] << ", " << g_max_abs_sum_swing_pair[2] << "]) threshold=" << g_sym_threshold_swing_deg << "°" << std::endl;
    } else {
        std::cout << "\n[SYMMETRY] PASS: STANCE max=[" << g_max_abs_sum_stance_pair[0] << ", " << g_max_abs_sum_stance_pair[1] << ", " << g_max_abs_sum_stance_pair[2]
                  << "] SWING max=[" << g_max_abs_sum_swing_pair[0] << ", " << g_max_abs_sum_swing_pair[1] << ", " << g_max_abs_sum_swing_pair[2]
                  << "]" << std::endl;
    }
    success = success && symmetry_ok;
    std::cout << "\nResult: " << (success ? "SUCCESS" : "FAIL") << std::endl;
    std::cout << "Test finished." << std::endl;

    return success ? 0 : 1;
}
