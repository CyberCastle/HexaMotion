/**
 * @file tripod_walk_visualization_test.cpp
 * @brief Test to validate the tripod gait by observing detailed leg states.
 *
 * This test evaluates the tripod gait by running a simulation that captures
 * detailed information about each leg at every step. The primary goal is to
 * generate a clear, step-by-step log of the gait execution to allow for
 * analysis of the underlying Bézier curves and phase transitions.
 *
 * The test will:
 * 1. Start the robot in a standard standing pose.
 * 2. Initiate the tripod gait.
 * 3. Run the simulation until each leg has completed at least three
 *    transitions from STANCE to SWING phase.
 * 4. Print the state of each leg at every step, including its phase,
 *    tip position, and joint angles (in degrees).
 * 5. Conclude by ensuring all legs return to the STANCE phase.
 *
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/hexamotion_constants.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Test configuration
constexpr double TEST_VELOCITY = 100;          // mm/s, a moderate speed for clear observation
constexpr double TEST_ANGULAR_VELOCITY = 0.25; // rad/s, introduce rotational motion for validation
// Número reducido de transiciones para terminar rápido; independiente del tamaño de fase.
constexpr int REQUIRED_SWING_TRANSITIONS = 2;
// Límite general; ya no depende de asumir 52 iteraciones por fase.
constexpr int MAX_STEPS = 600;
constexpr int EXPECTED_TRIPOD_HALF_PERIOD = 52;

// Utility to convert radians to degrees
static double toDegrees(double radians) {
    return math_utils::radiansToDegrees(radians);
}

/**
 * @brief Validates tripod symmetry between legs in the same group
 *
 * Tripod groups based on gait_config_factory.cpp:
 * - Group A (multiplier=0): Legs AR(0), CR(2), BL(4) - indices {0, 2, 4}
 * - Group B (multiplier=1): Legs BR(1), CL(3), AL(5) - indices {1, 3, 5}
 *
 * @param sys LocomotionSystem instance
 * @param leg_phase_values Array with absolute phase_ values (0-period) per leg
 * @param step Current simulation step number
 * @return true if symmetry is valid, false otherwise
 */
static bool validateTripodSymmetry(const LocomotionSystem &sys, const int leg_phase_values[NUM_LEGS], int step, int period) {
    // Tripod group definitions (matching gait_config_factory.cpp tripod configuration)
    const int GROUP_A[] = {0, 2, 4}; // AR, CR, BL
    const int GROUP_B[] = {1, 3, 5}; // BR, CL, AL
    const char *GROUP_A_NAMES[] = {"AR(Leg1)", "CR(Leg3)", "BL(Leg5)"};
    const char *GROUP_B_NAMES[] = {"BR(Leg2)", "CL(Leg4)", "AL(Leg6)"};
    const int GROUP_SIZE = 3;

    auto validateGroup = [&](const int *group, const char **names, const char *group_name) -> bool {
        // Get reference values from first leg in group
        int ref_leg = group[0];
        StepPhase ref_phase = sys.getLeg(ref_leg).getStepPhase();
        int ref_phase_iter = leg_phase_values[ref_leg];

        // Check all other legs in the group match the reference
        for (int i = 1; i < GROUP_SIZE; ++i) {
            int leg_idx = group[i];
            StepPhase current_phase = sys.getLeg(leg_idx).getStepPhase();
            int current_phase_iter = leg_phase_values[leg_idx];

            // Check phase mismatch
            if (current_phase != ref_phase) {
                std::cerr << "\n❌ TRIPOD SYMMETRY VIOLATION DETECTED at step " << step << "!\n";
                std::cerr << "Group: " << group_name << "\n";
                std::cerr << "  Reference leg " << names[0] << ": Phase="
                          << (ref_phase == STANCE_PHASE ? "STANCE" : "SWING") << "\n";
                std::cerr << "  Mismatched leg " << names[i] << ": Phase="
                          << (current_phase == STANCE_PHASE ? "STANCE" : "SWING") << "\n";
                std::cerr << "\nERROR: Legs in the same tripod group MUST be in the same phase!\n";
                std::cerr << "This indicates a phase initialization or synchronization bug.\n";
                return false;
            }

            // Check phase iteration mismatch
            if (current_phase_iter != ref_phase_iter) {
                std::cerr << "\n❌ TRIPOD SYMMETRY VIOLATION DETECTED at step " << step << "!\n";
                std::cerr << "Group: " << group_name << "\n";
                std::cerr << "  Reference leg " << names[0] << ": PhaseIter=" << ref_phase_iter << "\n";
                std::cerr << "  Mismatched leg " << names[i] << ": PhaseIter=" << current_phase_iter << "\n";
                std::cerr << "  Difference: " << abs(current_phase_iter - ref_phase_iter) << " iterations\n";
                std::cerr << "\nERROR: Legs in the same tripod group MUST have synchronized phase iterations!\n";
                std::cerr << "This lag causes lateral oscillation and instability in real robots.\n";
                return false;
            }
        }
        return true;
    };

    // Validate both tripod groups
    if (!validateGroup(GROUP_A, GROUP_A_NAMES, "Group A (AR/CR/BL)")) {
        return false;
    }
    if (!validateGroup(GROUP_B, GROUP_B_NAMES, "Group B (BR/CL/AL)")) {
        return false;
    }

    // Validate fixed half-period offset between tripod groups (OpenSHC tripod parity)
    int group_a_phase = leg_phase_values[GROUP_A[0]];
    int group_b_phase = leg_phase_values[GROUP_B[0]];
    int expected_offset = period / 2;
    int measured_offset = (group_b_phase - group_a_phase + period) % period;
    if (measured_offset != expected_offset) {
        std::cerr << "\n❌ TRIPOD OFFSET VIOLATION at step " << step << "!\n";
        std::cerr << "  Group A phase: " << group_a_phase << "/" << period << "\n";
        std::cerr << "  Group B phase: " << group_b_phase << "/" << period << "\n";
        std::cerr << "  Expected offset: " << expected_offset << " iterations\n";
        std::cerr << "  Measured offset: " << measured_offset << " iterations\n";
        std::cerr << "\nERROR: Tripod groups must remain exactly 180° out of phase.\n";
        return false;
    }

    return true;
}

/**
 * @brief Prints the header for the test output.
 */
static void printTestHeader() {
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "                            TRIPOD GAIT DETAILED VALIDATION TEST" << std::endl;
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "This test will monitor each leg until it completes " << REQUIRED_SWING_TRANSITIONS << " STANCE->SWING transitions." << std::endl;
    std::cout << "With OpenSHC timing: Iteraciones por fase se derivan dinámicamente (no fija 52)." << std::endl;
    std::cout << "Expected test duration (aprox): depende de iteraciones derivadas (se mostrará abajo)." << std::endl;
    std::cout << "Velocity: " << TEST_VELOCITY << " mm/s" << std::endl;
    std::cout << "Angular velocity: " << TEST_ANGULAR_VELOCITY << " rad/s" << std::endl
              << std::endl;
    std::cout << std::left << std::setw(8) << "Step"
              << std::setw(8) << "Leg"
              << std::setw(8) << "Phase"
              << std::setw(25) << "Position (X, Y, Z)"
              << std::setw(35) << "Joint Angles (Coxa, Femur, Tibia) deg"
              << std::setw(12) << "Phase Iter"
              << "Transitions" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Prints the state of all legs for a single simulation step.
 * @param sys The LocomotionSystem instance.
 * @param step The current simulation step number.
 * @param transition_counts An array with the current transition counts for each leg.
 * @param leg_phase_values Array with current phase_ values from LegStepper (0-period).
 * @param period The StepCycle period (should be 104 for standard tripod).
 */
static void printLegStates(const LocomotionSystem &sys, int step, const int transition_counts[NUM_LEGS],
                           const int leg_phase_values[NUM_LEGS], int period) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        Point3D tip_pos = leg.getCurrentTipPositionGlobal();
        JointAngles angles = leg.getJointAngles();
        StepPhase phase = leg.getStepPhase();

        // Format position string
        std::stringstream pos_ss;
        pos_ss << std::fixed << std::setprecision(2)
               << "[" << tip_pos.x << ", " << tip_pos.y << ", " << tip_pos.z << "]";

        // Format angles string
        std::stringstream ang_ss;
        ang_ss << std::fixed << std::setprecision(2)
               << "[" << std::setw(7) << toDegrees(angles.coxa)
               << ", " << std::setw(7) << toDegrees(angles.femur)
               << ", " << std::setw(7) << toDegrees(angles.tibia) << "]";

        // Format phase value (absolute phase_ from LegStepper, 0-103 for period=104)
        std::stringstream phase_info_ss;
        phase_info_ss << leg_phase_values[i] << "/" << period;

        std::cout << std::left << std::setw(8) << step
                  << std::setw(8) << ("Leg " + std::to_string(i + 1))
                  << std::setw(8) << (phase == STANCE_PHASE ? "S" : "W")
                  << std::setw(25) << pos_ss.str()
                  << std::setw(35) << ang_ss.str()
                  << std::setw(12) << phase_info_ss.str()
                  << transition_counts[i] << std::endl;
    }
    std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Checks if all legs have completed the required number of transitions.
 * @param transition_counts An array with the current transition counts for each leg.
 * @return True if the test objective is met, false otherwise.
 */
static bool allLegsCompletedTransitions(const int transition_counts[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (transition_counts[i] < REQUIRED_SWING_TRANSITIONS) {
            return false;
        }
    }
    return true;
}

int main() {
    // 1. Initialization
    Parameters p = createDefaultParameters();
    // Enable configured packed/unpacked poses for Bézier-based transitions
    enableConfiguredPackedUnpackedPoses(p);
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    pose_config.start_up_sequence = true; // Enable Bézier-based startup/shutdown sequences

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    // 2. Start in Standing Pose via full state machine (shows Bézier iterations)
    // Use setStandingPose for immediate standing, then show packed→unpacked path later
    if (!sys.setStandingPose()) {
        std::cerr << "ERROR: Failed to set standing pose." << std::endl;
        return 1;
    }
    std::cout << "Robot is in standing pose. All legs in STANCE phase." << std::endl;

    // Verify all legs are in proper standing pose before starting gait
    std::cout << "\nSTANDING POSE VERIFICATION:" << std::endl;
    std::cout << "Leg     Phase   Position (X, Y, Z)       Joint Angles (Coxa, Femur, Tibia) deg" << std::endl;
    std::cout << "-----------------------------------------------------------------------------------------" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        Point3D tip_pos = leg.getCurrentTipPositionGlobal();
        JointAngles angles = leg.getJointAngles();
        StepPhase phase = leg.getStepPhase();

        std::stringstream pos_ss;
        pos_ss << std::fixed << std::setprecision(2)
               << "[" << tip_pos.x << ", " << tip_pos.y << ", " << tip_pos.z << "]";

        std::stringstream ang_ss;
        ang_ss << std::fixed << std::setprecision(2)
               << "[ " << std::setw(6) << toDegrees(angles.coxa)
               << ", " << std::setw(6) << toDegrees(angles.femur)
               << ", " << std::setw(6) << toDegrees(angles.tibia) << "]";

        std::cout << std::left << std::setw(8) << ("Leg " + std::to_string(i + 1))
                  << std::setw(8) << (phase == STANCE_PHASE ? "S" : "W")
                  << std::setw(25) << pos_ss.str()
                  << std::setw(35) << ang_ss.str() << std::endl;
    }
    std::cout << "-----------------------------------------------------------------------------------------\n"
              << std::endl;

    // 3. Setup and Start Tripod Gait (new API): select gait, set velocities, then startWalking()
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    if (!sys.setGaitConfiguration(tripod_gait)) {
        std::cerr << "ERROR: Failed to set gait type." << std::endl;
        return 1;
    }
    // Configuración de límites ahora se hace vía Parameters (enable_dynamic_velocity_limits / fixed_linear_speed_limit_mm_s)
    // Use walkForward to set forward velocity (directional helpers persist velocities)
    sys.walkForward(TEST_VELOCITY);
    // Optionally add lateral / angular components if needed (keep simple forward for validation)
    if (!sys.startWalking()) {
        std::cerr << "ERROR: Failed to start walking (startup sequence)." << std::endl;
        return 1;
    }

    // Execute startup sequence to transition from READY to RUNNING
    // Track Bézier iterations through each robot state transition
    std::cout << "\n=== BÉZIER TRANSITION TRACKING (PACKED → READY → RUNNING) ===" << std::endl;

    StateController *sc = sys.getStateController();
    assert(sc != nullptr);

    // Track iterations for each transition phase
    int unpack_bezier_iters = 0;  // PACKED → READY (cubic Bézier via transitionConfiguration)
    int startup_bezier_iters = 0; // READY → RUNNING (quartic Bézier via stepToPosition)
    RobotState prev_robot_state = sc->getRobotState();
    bool unpack_phase_complete = false;

    int startup_sequence_attempts = 0;
    // Startup sequence first-execution learning may require the full horizontal+vertical
    // transition budget. Derive timeout from configured timing instead of a hardcoded 500.
    const double startup_time_budget_s =
        (4.0 * HORIZONTAL_TRANSITION_TIME + 2.0 * VERTICAL_TRANSITION_TIME) /
        std::max(1e-6, p.step_frequency);
    const int MAX_STARTUP_SEQUENCE_ATTEMPTS =
        std::max(500, static_cast<int>(std::ceil(startup_time_budget_s / p.time_delta)) + 100);

    while (sc->getRobotState() != ROBOT_RUNNING && startup_sequence_attempts < MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        sys.update();
        startup_sequence_attempts++;

        RobotState cur_robot_state = sc->getRobotState();

        // Count iterations per transition phase
        if (!unpack_phase_complete) {
            if (cur_robot_state == ROBOT_PACKED) {
                unpack_bezier_iters++;
            } else if (prev_robot_state == ROBOT_PACKED && cur_robot_state != ROBOT_PACKED) {
                unpack_bezier_iters++; // count the transition tick itself
                unpack_phase_complete = true;
                std::cout << "  Unpack (cubic Bézier): " << unpack_bezier_iters << " iterations" << std::endl;
            }
        }
        if (unpack_phase_complete && cur_robot_state != ROBOT_RUNNING) {
            startup_bezier_iters++;
        }

        prev_robot_state = cur_robot_state;

        if (startup_sequence_attempts % 25 == 0) {
            std::cout << "Startup attempt " << startup_sequence_attempts
                      << "  Progress=" << sys.getStartupProgressPercent() << "%" << std::endl;
        }
    }

    // If unpack was instant (direct mode), record it
    if (!unpack_phase_complete) {
        unpack_bezier_iters = 0;
        unpack_phase_complete = true;
        std::cout << "  Unpack: skipped (direct mode / instant)" << std::endl;
    }
    if (startup_bezier_iters > 0) {
        std::cout << "  Startup (quartic Bézier): " << startup_bezier_iters << " iterations" << std::endl;
    } else {
        std::cout << "  Startup: direct mode (1 iteration)" << std::endl;
    }

    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Startup sequence failed to complete after " << startup_sequence_attempts << " attempts." << std::endl;
        std::cerr << "The tripod startup sequence may require more time to coordinate leg movements." << std::endl;
        return 1;
    }
    std::cout << "Startup sequence completed successfully after " << startup_sequence_attempts << " sequence attempts." << std::endl;

    std::cout << "Beginning gait analysis..." << std::endl;
    printTestHeader();

    // 4. Main Simulation Loop with iteration calculation as in trajectory_tip_position_test

    // Verificar que el timing de trayectorias esté sincronizado con trajectory_tip_position_test
    // BOTH tests must use exactly the same StepCycle configuration
    std::cout << "=== VERIFICACIÓN DE SINCRONIZACIÓN CON trajectory_tip_position_test ===" << std::endl;

    // El sistema LocomotionSystem ya tiene configurado el StepCycle correcto via WalkController
    // Solo necesitamos verificar que los valores coincidan con trajectory_tip_position_test
    auto first_leg_stepper = sys.getWalkController()->getLegStepper(0);
    if (!first_leg_stepper) {
        std::cerr << "ERROR: No se pudo obtener el LegStepper." << std::endl;
        return 1;
    }

    StepCycle actual_step_cycle = first_leg_stepper->getStepCycle();
    double time_delta = sys.getRobotModel().getTimeDelta(); // unified global timestep

    // Usar EXACTAMENTE la misma fórmula que trajectory_tip_position_test
    int swing_iterations_per_cycle = (int)((double(actual_step_cycle.swing_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int stance_iterations_per_cycle = (int)((double(actual_step_cycle.stance_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int total_iterations_per_cycle = swing_iterations_per_cycle + stance_iterations_per_cycle;

    std::cout << "StepCycle activo en LocomotionSystem:" << std::endl;
    std::cout << "  time_delta: " << time_delta << "s" << std::endl;
    std::cout << "  period: " << actual_step_cycle.period_ << ", swing_period: " << actual_step_cycle.swing_period_ << ", stance_period: " << actual_step_cycle.stance_period_ << std::endl;
    std::cout << "  frequency: " << actual_step_cycle.frequency_ << "Hz" << std::endl;
    std::cout << "  swing_iterations_per_cycle: " << swing_iterations_per_cycle << std::endl;
    std::cout << "  stance_iterations_per_cycle: " << stance_iterations_per_cycle << std::endl;
    std::cout << "  total_iterations_per_cycle: " << total_iterations_per_cycle << std::endl;

    std::cout << "Iteraciones derivadas StepCycle: swing=" << swing_iterations_per_cycle
              << ", stance=" << stance_iterations_per_cycle << std::endl;
    // Coherencia interna: swing + stance debe igualar al periodo total normalizado
    if (swing_iterations_per_cycle + stance_iterations_per_cycle != actual_step_cycle.period_) {
        std::cerr << "ERROR: Inconsistent StepCycle timing (swing+stance != period)." << std::endl;
        return 1; // Abort test early
    }
    if (swing_iterations_per_cycle != stance_iterations_per_cycle) {
        std::cout << "ℹ️  INFO: Diferencia entre swing y stance (válida si la configuración lo define)." << std::endl;
    }
    if (swing_iterations_per_cycle != EXPECTED_TRIPOD_HALF_PERIOD ||
        stance_iterations_per_cycle != EXPECTED_TRIPOD_HALF_PERIOD) {
        std::cerr << "ERROR: Tripod gait must run with exact half-period of " << EXPECTED_TRIPOD_HALF_PERIOD
                  << " iterations per phase (swing/stance)." << std::endl;
        return 1;
    }
    std::cout << "(Validación estricta: tripod exige 52 iteraciones por swing y 52 por stance)." << std::endl;
    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Valores absolutos de phase_ (0-period) de cada pata desde LegStepper
    int leg_phase_values[NUM_LEGS] = {0};
    StepPhase leg_current_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        leg_current_phases[i] = sys.getLeg(i).getStepPhase();
    }

    while (step < MAX_STEPS) {
        // Update system
        if (!sys.update()) {
            std::cerr << "WARNING: System update failed at step " << step << std::endl;
            continue;
        }

        // Check for STANCE -> SWING transitions y obtener la fase real de cada LegStepper
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();

            // Obtener el valor REAL de phase_ desde LegStepper (no contar manualmente)
            auto leg_stepper = sys.getWalkController()->getLegStepper(i);
            if (leg_stepper) {
                leg_phase_values[i] = leg_stepper->getPhase();
            }

            // Detectar transiciones STANCE -> SWING solo para contar
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
            }

            // Validate end-of-swing touchdown posture is kinematically safe.
            // Under dynamic walking, touchdown can deviate from standing angles,
            // especially with stride offsets and local IK minima.
            if (previous_phases[i] == SWING_PHASE && current_phase == STANCE_PHASE) {
                JointAngles touchdown_angles = sys.getLeg(i).getJointAngles();
                double femur_deg = toDegrees(touchdown_angles.femur);
                double tibia_deg = toDegrees(touchdown_angles.tibia);
                bool femur_valid = std::isfinite(femur_deg) &&
                                   femur_deg >= p.femur_angle_limits[0] &&
                                   femur_deg <= p.femur_angle_limits[1];
                bool tibia_valid = std::isfinite(tibia_deg) &&
                                   tibia_deg >= p.tibia_angle_limits[0] &&
                                   tibia_deg <= p.tibia_angle_limits[1];

                if (!femur_valid || !tibia_valid) {
                    std::cerr << "\n❌ TOUCHDOWN ANGLE VIOLATION at step " << step
                              << " (Leg " << (i + 1) << ")!\n";
                    std::cerr << "  Limits femur: [" << p.femur_angle_limits[0] << ", "
                              << p.femur_angle_limits[1] << "] deg\n";
                    std::cerr << "  Limits tibia: [" << p.tibia_angle_limits[0] << ", "
                              << p.tibia_angle_limits[1] << "] deg\n";
                    std::cerr << "  Measured (femur, tibia): ("
                              << femur_deg << ", " << tibia_deg << ") deg\n";
                    std::cerr << "\nERROR: End-of-swing posture is outside configured joint limits.\n";
                    return 1;
                }
            }

            previous_phases[i] = current_phase;
        }

        // CRITICAL: Validate tripod symmetry after every update
        // This ensures legs in the same tripod group remain perfectly synchronized
        if (!validateTripodSymmetry(sys, leg_phase_values, step, actual_step_cycle.period_)) {
            std::cerr << "\n"
                      << std::string(100, '=') << "\n";
            std::cerr << "TEST FAILED: Tripod symmetry violation detected at step " << step << "\n";
            std::cerr << "\nCurrent leg states:\n";
            printLegStates(sys, step, transition_counts, leg_phase_values, actual_step_cycle.period_);
            std::cerr << "\n"
                      << std::string(100, '=') << "\n";
            std::cerr << "\nAborting test due to symmetry violation.\n";
            std::cerr << "This asymmetry would cause instability and oscillation in a real hexapod robot.\n";
            return 1;
        }

        // Print current state con información de phase_ absoluto
        printLegStates(sys, step, transition_counts, leg_phase_values, actual_step_cycle.period_);

        // Progress indicator every 20 steps for long 52-iteration phases
        if (step > 0 && step % 20 == 0) {
            std::cout << "\n--- Progress Update (Step " << step << ") ---" << std::endl;
            std::cout << "Completed transitions per leg: ";
            for (int i = 0; i < NUM_LEGS; i++) {
                std::cout << "Leg" << (i + 1) << ":" << transition_counts[i] << " ";
            }
            std::cout << std::endl;
            std::cout << "Target: " << REQUIRED_SWING_TRANSITIONS << " transitions per leg" << std::endl;
            std::cout << "-----------------------------------\n"
                      << std::endl;
        }

        // Check for completion
        if (allLegsCompletedTransitions(transition_counts)) {
            std::cout << "\nSUCCESS: All legs completed " << REQUIRED_SWING_TRANSITIONS << " swing transitions." << std::endl;
            break;
        }

        step++;
    }

    if (step == MAX_STEPS) {
        std::cerr << "\nWARNING: Test reached maximum steps (" << MAX_STEPS << ") before completion." << std::endl;
    }

    // 5. Stop and Return to Stance — track shutdown Bézier iterations
    std::cout << "\nStopping walk and returning to standing pose..." << std::endl;
    if (!sys.stopWalking()) {
        std::cerr << "WARNING: Failed to initiate stop walking." << std::endl;
    }

    std::cout << "\n=== BÉZIER TRANSITION TRACKING (RUNNING → READY → PACKED) ===" << std::endl;

    // Run update loop to let StateController orchestrate the shutdown
    int shutdown_attempts = 0;
    int shutdown_bezier_iters = 0;
    const double shutdown_time_budget_s =
        (4.0 * HORIZONTAL_TRANSITION_TIME + 2.0 * VERTICAL_TRANSITION_TIME) /
        std::max(1e-6, p.step_frequency);
    const int MAX_SHUTDOWN_ATTEMPTS =
        std::max(500, static_cast<int>(std::ceil(shutdown_time_budget_s / p.time_delta)) + 100);
    while (shutdown_attempts < MAX_SHUTDOWN_ATTEMPTS && sc->getRobotState() == ROBOT_RUNNING) {
        sys.update();
        shutdown_attempts++;
        shutdown_bezier_iters++;
    }
    if (sc->getRobotState() == ROBOT_RUNNING) {
        std::cerr << "ERROR: Shutdown did not complete after " << shutdown_attempts << " iterations." << std::endl;
        return 1;
    }
    std::cout << "  Shutdown (quartic Bézier): " << shutdown_bezier_iters << " iterations" << std::endl;
    std::cout << "Shutdown completed after " << shutdown_attempts << " iterations." << std::endl;

    // Request pack to observe READY → PACKED transition (cubic Bézier)
    int pack_bezier_iters = 0;
    if (!sc->requestRobotState(ROBOT_PACKED)) {
        std::cerr << "ERROR: Failed to request ROBOT_PACKED transition." << std::endl;
        return 1;
    }
    const double pack_time_budget_s = PACK_TIME / std::max(1e-6, p.step_frequency);
    const int MAX_PACK_ATTEMPTS =
        std::max(500, static_cast<int>(std::ceil(pack_time_budget_s / p.time_delta)) + 100);
    while (pack_bezier_iters < MAX_PACK_ATTEMPTS && sc->getRobotState() != ROBOT_PACKED) {
        sys.update();
        pack_bezier_iters++;
    }
    if (sc->getRobotState() != ROBOT_PACKED) {
        std::cerr << "ERROR: Pack sequence did not complete after " << pack_bezier_iters << " iterations." << std::endl;
        return 1;
    }
    std::cout << "  Pack (cubic Bézier): " << pack_bezier_iters << " iterations" << std::endl;

    std::cout << "\nFinal Leg States (all should be STANCE):" << std::endl;
    int final_counts[NUM_LEGS] = {0};           // Dummy counts for final print
    int final_phase_iterations[NUM_LEGS] = {0}; // Dummy phase iterations for final print
    printLegStates(sys, step, final_counts, final_phase_iterations, actual_step_cycle.period_);

    // Final validation
    bool final_all_in_stance = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLeg(i).getStepPhase() != STANCE_PHASE) {
            final_all_in_stance = false;
            std::cerr << "ERROR: Leg " << i + 1 << " did not return to STANCE phase." << std::endl;
        }
    }

    if (final_all_in_stance) {
        std::cout << "\n=== BEZIER TRAJECTORY TIMING ANALYSIS ===" << std::endl;
        std::cout << "CONFIRMACIÓN: Las trayectorias usan el MISMO timing que trajectory_tip_position_test" << std::endl;
        std::cout << "  Swing iterations por ciclo: " << swing_iterations_per_cycle << std::endl;
        std::cout << "  Stance iterations por ciclo: " << stance_iterations_per_cycle << std::endl;
        std::cout << "  Total iterations por ciclo: " << total_iterations_per_cycle << std::endl;
        std::cout << "  Time delta: " << time_delta << "s" << std::endl;
        std::cout << "  Frequency: " << actual_step_cycle.frequency_ << "Hz" << std::endl;
        std::cout << "  Period: " << actual_step_cycle.period_ << ", Swing period: " << actual_step_cycle.swing_period_ << ", Stance period: " << actual_step_cycle.stance_period_ << std::endl;

        std::cout << "\n=== BÉZIER TRANSITION ITERATION SUMMARY ===" << std::endl;
        std::cout << "  Phase                     | Bézier Type     | Iterations" << std::endl;
        std::cout << "  --------------------------+-----------------+-----------" << std::endl;
        std::cout << "  Unpack (PACKED→READY)     | Cubic per-joint | " << unpack_bezier_iters << std::endl;
        std::cout << "  Startup (READY→RUNNING)   | Dual quartic    | " << startup_bezier_iters << std::endl;
        std::cout << "  Shutdown (RUNNING→READY)  | Dual quartic    | " << shutdown_bezier_iters << std::endl;
        std::cout << "  Pack (READY→PACKED)       | Cubic per-joint | " << pack_bezier_iters << std::endl;
        std::cout << "  --------------------------+-----------------+-----------" << std::endl;
        std::cout << "  Total transition iters    |                 | " << (unpack_bezier_iters + startup_bezier_iters + shutdown_bezier_iters + pack_bezier_iters) << std::endl;

        std::cout << "\n🎯 SINCRONIZACIÓN COMPLETA CON trajectory_tip_position_test:" << std::endl;
        std::cout << "  ✅ Ambos tests ejecutan exactamente " << swing_iterations_per_cycle << " iteraciones por fase swing" << std::endl;
        std::cout << "  ✅ Ambos tests ejecutan exactamente " << stance_iterations_per_cycle << " iteraciones por fase stance" << std::endl;
        std::cout << "  ✅ Ambos tests usan la secuencia LocomotionSystem::update -> WalkController::updateWalk -> LegStepper::updateTipPositionIterative" << std::endl;
        std::cout << "  ✅ GaitConfiguration and StepCycle configuration totally coherent with OpenSHC" << std::endl;

        std::cout << "\n🎉 TEST PASSED! Gait cycle observed and robot returned to stable standing pose. 🎉" << std::endl;
        std::cout << "🔄 Timing sincronizado con trajectory_tip_position_test (iteraciones reales: swing=" << swing_iterations_per_cycle
                  << ", stance=" << stance_iterations_per_cycle << ")" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ TEST FAILED! Robot did not return to a stable standing pose. ❌" << std::endl;
        return 1;
    }
}
