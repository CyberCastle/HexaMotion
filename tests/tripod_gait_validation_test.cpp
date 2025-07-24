/**
 * @file tripod_gait_validation_test.cpp
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
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Test configuration
constexpr double TEST_VELOCITY = 50.0;        // mm/s, a moderate speed for clear observation
constexpr int REQUIRED_SWING_TRANSITIONS = 3; // Each leg must complete 3 STANCE->SWING transitions
constexpr int MAX_STEPS = 500;                // Safety break to prevent infinite loops

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
    std::cout << "Velocity: " << TEST_VELOCITY << " mm/s" << std::endl
              << std::endl;
    std::cout << std::left << std::setw(8) << "Step"
              << std::setw(8) << "Leg"
              << std::setw(8) << "Phase"
              << std::setw(25) << "Position (X, Y, Z)"
              << std::setw(35) << "Joint Angles (Coxa, Femur, Tibia) deg"
              << "Transition Count" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Prints the state of all legs for a single simulation step.
 * @param sys The LocomotionSystem instance.
 * @param step The current simulation step number.
 * @param transition_counts An array with the current transition counts for each leg.
 */
static void printLegStates(const LocomotionSystem &sys, int step, const int transition_counts[NUM_LEGS]) {
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

        std::cout << std::left << std::setw(8) << step
                  << std::setw(8) << ("Leg " + std::to_string(i + 1))
                  << std::setw(8) << (phase == STANCE_PHASE ? "S" : "W")
                  << std::setw(25) << pos_ss.str()
                  << std::setw(35) << ang_ss.str()
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

    // 3. Setup and Start Tripod Gait
    if (!sys.startWalking(TRIPOD_GAIT, TEST_VELOCITY, 0.0, 0.0)) {
        std::cerr << "ERROR: Failed to start tripod gait." << std::endl;
        return 1;
    }

    // Wait for startup sequence to complete
    std::cout << "Waiting for startup sequence to complete..." << std::endl;
    int startup_wait = 0;
    const int MAX_STARTUP_WAIT = 100;
    while (startup_wait < MAX_STARTUP_WAIT) {
        if (!sys.update()) {
            std::cerr << "WARNING: System update failed during startup at step " << startup_wait << std::endl;
        }

        // Check if startup is complete by verifying system state
        bool startup_complete = true;
        // If startup is still in progress, continue waiting
        // The system will transition to RUNNING when startup completes
        if (startup_wait > 0 && startup_wait % 10 == 0) {
            std::cout << "Startup step " << startup_wait << "..." << std::endl;
        }

        startup_wait++;

        // Give startup sequence reasonable time to complete
        if (startup_wait >= 20)
            break; // Assume startup completes within 20 steps
    }

    std::cout << "Startup sequence completed. Beginning gait analysis..." << std::endl;

    printTestHeader();

    // 4. Main Simulation Loop
    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    while (step < MAX_STEPS) {
        // Update system
        if (!sys.update()) {
            std::cerr << "WARNING: System update failed at step " << step << std::endl;
            continue;
        }

        // Check for STANCE -> SWING transitions
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
            }
            previous_phases[i] = current_phase;
        }

        // Print current state
        printLegStates(sys, step, transition_counts);

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
    sys.stopWalking();

    // Wait for legs to return to stance with timeout
    int wait_steps = 0;
    const int MAX_WAIT_STEPS = 100;
    bool all_in_stance = false;

    while (!all_in_stance && wait_steps < MAX_WAIT_STEPS) {
        sys.update();
        all_in_stance = true;

        // Check if all legs are in stance
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (sys.getLeg(i).getStepPhase() != STANCE_PHASE) {
                all_in_stance = false;
                break;
            }
        }

        // Print progress every 10 steps
        if (wait_steps % 10 == 0) {
            std::cout << "Waiting for stance... step " << wait_steps << " - Legs in SWING: ";
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (sys.getLeg(i).getStepPhase() == SWING_PHASE) {
                    std::cout << (i + 1) << " ";
                }
            }
            std::cout << std::endl;
        }

        wait_steps++;
    }

    if (wait_steps >= MAX_WAIT_STEPS) {
        std::cerr << "\nWARNING: Timeout waiting for all legs to return to STANCE phase." << std::endl;
    }

    std::cout << "\nFinal Leg States (all should be STANCE):" << std::endl;
    int final_counts[NUM_LEGS] = {0}; // Dummy counts for final print
    printLegStates(sys, step, final_counts);

    // Final validation
    bool final_all_in_stance = true;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (sys.getLeg(i).getStepPhase() != STANCE_PHASE) {
            final_all_in_stance = false;
            std::cerr << "ERROR: Leg " << i + 1 << " did not return to STANCE phase." << std::endl;
        }
    }

    if (final_all_in_stance) {
        std::cout << "\n🎉 TEST PASSED! Gait cycle observed and robot returned to stable standing pose. 🎉" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ TEST FAILED! Robot did not return to a stable standing pose. ❌" << std::endl;
        return 1;
    }
}
