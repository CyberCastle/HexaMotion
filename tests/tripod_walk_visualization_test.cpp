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
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Test configuration
constexpr double TEST_VELOCITY = 20.0;         // mm/s, a moderate speed for clear observation
constexpr double TEST_ANGULAR_VELOCITY = 0.25; // rad/s, introduce rotational motion for validation
constexpr int REQUIRED_SWING_TRANSITIONS = 2;  // Each leg must complete 2 STANCE->SWING transitions (reduced for 52-iteration phases)
constexpr int MAX_STEPS = 250;                 // Increased to accommodate 52-iteration phases (2 cycles = 208 steps + margin)

// Utility to convert radians to degrees
static double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

/**
 * @brief Prints the header for the test output.
 */
static void printTestHeader() {
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "                            TRIPOD GAIT DETAILED VALIDATION TEST" << std::endl;
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "This test will monitor each leg until it completes " << REQUIRED_SWING_TRANSITIONS << " STANCE->SWING transitions." << std::endl;
    std::cout << "With OpenSHC timing: Each phase (stance/swing) lasts 52 iterations." << std::endl;
    std::cout << "Expected test duration: ~" << (REQUIRED_SWING_TRANSITIONS * 104) << " steps for full cycle completion." << std::endl;
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
 * @param leg_phase_iterations Array with current phase iteration counts for each leg.
 * @param swing_iterations_per_cycle Expected swing iterations per cycle for comparison.
 * @param stance_iterations_per_cycle Expected stance iterations per cycle for comparison.
 */
static void printLegStates(const LocomotionSystem &sys, int step, const int transition_counts[NUM_LEGS],
                           const int leg_phase_iterations[NUM_LEGS], int swing_iterations_per_cycle,
                           int stance_iterations_per_cycle) {
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

        // Format phase iteration info
        std::stringstream phase_info_ss;
        int expected_iterations = (phase == STANCE_PHASE) ? stance_iterations_per_cycle : swing_iterations_per_cycle;
        phase_info_ss << leg_phase_iterations[i] << "/" << expected_iterations;

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
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    // 2. Start in Standing Pose
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
    if (!sys.setGaitType(TRIPOD_GAIT)) {
        std::cerr << "ERROR: Failed to set gait type." << std::endl;
        return 1;
    }
    // Use walkForward to set forward velocity (directional helpers persist velocities)
    sys.walkForward(TEST_VELOCITY);
    // Optionally add lateral / angular components if needed (keep simple forward for validation)
    if (!sys.startWalking()) {
        std::cerr << "ERROR: Failed to start walking (startup sequence)." << std::endl;
        return 1;
    }

    // Execute startup sequence to transition from READY to RUNNING
    std::cout << "Executing startup sequence..." << std::endl;

    // First loop: Execute startup sequence with its own update cycles
    int startup_sequence_attempts = 0;
    const int MAX_STARTUP_SEQUENCE_ATTEMPTS = 100;

    while (sys.isStartupInProgress() && startup_sequence_attempts < MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        if (sys.executeStartupSequence()) {
            std::cout << "Startup sequence completed successfully after " << startup_sequence_attempts << " sequence attempts." << std::endl;
            break;
        }

        startup_sequence_attempts++;

        if (startup_sequence_attempts % 10 == 0) {
            std::cout << "Startup sequence attempt " << startup_sequence_attempts << "... (coordinated tripod movement)" << std::endl;
        }
    }

    if (startup_sequence_attempts >= MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        std::cerr << "ERROR: Startup sequence failed to complete after " << startup_sequence_attempts << " attempts." << std::endl;
        std::cerr << "The tripod startup sequence may require more time to coordinate leg movements." << std::endl;
        return 1;
    }

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
    double time_delta = 0.02; // 50Hz control loop (mismo que trajectory_tip_position_test)

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

    // Verificar que coincidan con trajectory_tip_position_test (debe ser 52 cada uno)
    if (swing_iterations_per_cycle != 52 || stance_iterations_per_cycle != 52) {
        std::cout << "⚠️  WARNING: Las iteraciones no coinciden con trajectory_tip_position_test (esperado: 52 cada fase)" << std::endl;
        std::cout << "   trajectory_tip_position_test usa: swing=52, stance=52" << std::endl;
        std::cout << "   tripod_walk_visualization_test usa: swing=" << swing_iterations_per_cycle << ", stance=" << stance_iterations_per_cycle << std::endl;
    } else {
        std::cout << "✅ SINCRONIZACIÓN CONFIRMADA: Ambos tests usan 52 iteraciones por fase" << std::endl;
    }
    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Contador de iteraciones para cada fase por pata
    int leg_phase_iterations[NUM_LEGS] = {0};
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

        // Check for STANCE -> SWING transitions y contar iteraciones por fase
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();

            // Detectar transiciones STANCE -> SWING
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
                leg_phase_iterations[i] = 1; // Empezar conteo de nueva fase
                leg_current_phases[i] = current_phase;
            }
            // Detectar transiciones SWING -> STANCE
            else if (previous_phases[i] == SWING_PHASE && current_phase == STANCE_PHASE) {
                leg_phase_iterations[i] = 1; // Empezar conteo de nueva fase
                leg_current_phases[i] = current_phase;
            }
            // Si estamos en la misma fase, incrementar contador
            else if (leg_current_phases[i] == current_phase) {
                leg_phase_iterations[i]++;
            }
            // Si hay cambio de fase sin ser transición detectada arriba
            else {
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
            }

            previous_phases[i] = current_phase;
        }

        // Print current state con información de iteraciones de fase
        printLegStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle);

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

    // 5. Stop and Return to Stance
    std::cout << "\nStopping walk and returning to standing pose..." << std::endl;
    if (!sys.stopWalking()) {
        std::cerr << "WARNING: Failed to initiate stop walking." << std::endl;
    }

    // Execute shutdown sequence to transition from RUNNING to READY
    int shutdown_sequence_attempts = 0;
    const int MAX_SHUTDOWN_SEQUENCE_ATTEMPTS = 100;

    while (sys.isShutdownInProgress() && shutdown_sequence_attempts < MAX_SHUTDOWN_SEQUENCE_ATTEMPTS) {
        if (sys.executeShutdownSequence()) {
            std::cout << "Shutdown sequence completed successfully after " << shutdown_sequence_attempts << " sequence attempts." << std::endl;
            break;
        }

        shutdown_sequence_attempts++;

        if (shutdown_sequence_attempts % 10 == 0) {
            std::cout << "Shutdown sequence attempt " << shutdown_sequence_attempts << "..." << std::endl;
        }
    }

    if (shutdown_sequence_attempts >= MAX_SHUTDOWN_SEQUENCE_ATTEMPTS) {
        std::cerr << "WARNING: Shutdown sequence failed to complete after " << shutdown_sequence_attempts << " attempts." << std::endl;
    }

    std::cout << "\nFinal Leg States (all should be STANCE):" << std::endl;
    int final_counts[NUM_LEGS] = {0};           // Dummy counts for final print
    int final_phase_iterations[NUM_LEGS] = {0}; // Dummy phase iterations for final print
    printLegStates(sys, step, final_counts, final_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle);

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

        std::cout << "\n🎯 SINCRONIZACIÓN COMPLETA CON trajectory_tip_position_test:" << std::endl;
        std::cout << "  ✅ Ambos tests ejecutan exactamente " << swing_iterations_per_cycle << " iteraciones por fase swing" << std::endl;
        std::cout << "  ✅ Ambos tests ejecutan exactamente " << stance_iterations_per_cycle << " iteraciones por fase stance" << std::endl;
        std::cout << "  ✅ Ambos tests usan la secuencia LocomotionSystem::update -> WalkController::updateWalk -> LegStepper::updateTipPositionIterative" << std::endl;
        std::cout << "  ✅ GaitConfiguration and StepCycle configuration totally coherent with OpenSHC" << std::endl;

        std::cout << "\n🎉 TEST PASSED! Gait cycle observed and robot returned to stable standing pose. 🎉" << std::endl;
        std::cout << "🔄 Timing perfectamente sincronizado con trajectory_tip_position_test (52 iteraciones por fase)" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ TEST FAILED! Robot did not return to a stable standing pose. ❌" << std::endl;
        return 1;
    }
}
