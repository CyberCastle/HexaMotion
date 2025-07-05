/**
 * @file tripod_gait_validation_test.cpp
 * @brief Simplified tripod gait validation test focusing on coherence, synchronization, and symmetry
 *
 * This test validates the core tripod gait characteristics:
 * - Coherence: Consistent gait pattern throughout the simulation
 * - Synchronization: Proper phase relationships between leg groups
 * - Symmetry: Balanced movement between left and right sides
 *
 * @author HexaMotion Team
 * @version 1.0
 * @date 2024
 */

#include "robot_model.h"
#include "../src/locomotion_system.h"
#include "../src/body_pose_config_factory.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

// Test configuration
constexpr int VALIDATION_STEPS = 100;  // Reduced for faster testing
constexpr double TEST_VELOCITY = 200.0; // mm/s
constexpr double TEST_DISTANCE = 400.0; // mm

// Tripod gait validation structure
struct TripodValidation {
    std::vector<StepPhase> group_a_phases;  // Legs 1, 3, 5
    std::vector<StepPhase> group_b_phases;  // Legs 2, 4, 6
    std::vector<double> gait_phases;
    std::vector<bool> symmetry_checks;
    int coherence_violations;
    int sync_violations;
    int symmetry_violations;
};

static void printTestHeader() {
    std::cout << "=========================================" << std::endl;
    std::cout << "TRIPOD GAIT VALIDATION TEST" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Validating: Coherence, Synchronization, Symmetry" << std::endl;
    std::cout << "Steps: " << VALIDATION_STEPS << " | Velocity: " << TEST_VELOCITY << " mm/s" << std::endl;
    std::cout << "Distance: " << TEST_DISTANCE << " mm" << std::endl;
    std::cout << "=========================================" << std::endl << std::endl;
}

static void printGaitPattern(const LocomotionSystem& sys, int step, double phase) {
    std::cout << "Step " << std::setw(3) << step << " (Phase: " << std::fixed << std::setprecision(2) << phase << "): ";

    // Print leg states
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg& leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';
        std::cout << "L" << (leg + 1) << ":" << symbol << " ";
    }

    // Print tripod groups
    std::cout << " | Groups: A[";
    for (int leg : {0, 2, 4}) {
        const Leg& leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        std::cout << (state == STANCE_PHASE ? "S" : "W");
    }
    std::cout << "] B[";
    for (int leg : {1, 3, 5}) {
        const Leg& leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        std::cout << (state == STANCE_PHASE ? "S" : "W");
    }
    std::cout << "]" << std::endl;
}

static bool validateCoherence(const TripodValidation& validation) {
    std::cout << "\n=== COHERENCE VALIDATION ===" << std::endl;

    // Check that each group maintains consistent pattern
    bool group_a_consistent = true;
    bool group_b_consistent = true;

    for (size_t i = 1; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] != validation.group_a_phases[i-1]) {
            group_a_consistent = false;
            break;
        }
    }

    for (size_t i = 1; i < validation.group_b_phases.size(); ++i) {
        if (validation.group_b_phases[i] != validation.group_b_phases[i-1]) {
            group_b_consistent = false;
            break;
        }
    }

    std::cout << "Group A (L1,L3,L5) consistency: " << (group_a_consistent ? "✓ PASS" : "✗ FAIL") << std::endl;
    std::cout << "Group B (L2,L4,L6) consistency: " << (group_b_consistent ? "✓ PASS" : "✗ FAIL") << std::endl;

    return group_a_consistent && group_b_consistent;
}

static bool validateSynchronization(const TripodValidation& validation) {
    std::cout << "\n=== SYNCHRONIZATION VALIDATION ===" << std::endl;

    // Check that groups are always in opposite phases
    int sync_violations = 0;
    for (size_t i = 0; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] == validation.group_b_phases[i]) {
            sync_violations++;
        }
    }

    double sync_percentage = 100.0 * (1.0 - (double)sync_violations / validation.group_a_phases.size());
    std::cout << "Synchronization accuracy: " << std::fixed << std::setprecision(1) << sync_percentage << "%" << std::endl;
    std::cout << "Violations: " << sync_violations << "/" << validation.group_a_phases.size() << std::endl;

    bool passed = sync_percentage >= 95.0; // Allow 5% tolerance
    std::cout << "Synchronization: " << (passed ? "✓ PASS" : "✗ FAIL") << std::endl;

    return passed;
}

static bool validateSymmetry(const TripodValidation& validation) {
    std::cout << "\n=== SYMMETRY VALIDATION ===" << std::endl;

    // Check left-right symmetry
    int symmetry_violations = 0;
    for (bool check : validation.symmetry_checks) {
        if (!check) symmetry_violations++;
    }

    double symmetry_percentage = 100.0 * (1.0 - (double)symmetry_violations / validation.symmetry_checks.size());
    std::cout << "Symmetry accuracy: " << std::fixed << std::setprecision(1) << symmetry_percentage << "%" << std::endl;
    std::cout << "Violations: " << symmetry_violations << "/" << validation.symmetry_checks.size() << std::endl;

    bool passed = symmetry_percentage >= 95.0; // Allow 5% tolerance
    std::cout << "Symmetry: " << (passed ? "✓ PASS" : "✗ FAIL") << std::endl;

    return passed;
}

static bool checkLegSymmetry(const LocomotionSystem& sys) {
    // Check that opposite legs (L1-L4, L2-L5, L3-L6) have opposite phases
    const Leg& leg1 = sys.getLeg(0); // L1
    const Leg& leg4 = sys.getLeg(3); // L4
    const Leg& leg2 = sys.getLeg(1); // L2
    const Leg& leg5 = sys.getLeg(4); // L5
    const Leg& leg3 = sys.getLeg(2); // L3
    const Leg& leg6 = sys.getLeg(5); // L6

    bool symmetry1 = (leg1.getStepPhase() != leg4.getStepPhase());
    bool symmetry2 = (leg2.getStepPhase() != leg5.getStepPhase());
    bool symmetry3 = (leg3.getStepPhase() != leg6.getStepPhase());

    return symmetry1 && symmetry2 && symmetry3;
}

int main() {
    printTestHeader();

    // Initialize robot parameters
    Parameters p = createDefaultParameters();

    // Verify parameters
    assert(p.hexagon_radius == 200.0f);
    assert(p.coxa_length == 50.0f);
    assert(p.femur_length == 101.0f);
    assert(p.tibia_length == 208.0f);
    assert(p.robot_height == 120.0f);
    assert(p.control_frequency == 50.0f);

    // Initialize locomotion system
    LocomotionSystem sys(p);

    // Create mock interfaces
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Create body pose configuration
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    std::cout << "Initializing locomotion system..." << std::endl;

    // Initialize the locomotion system
    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cout << "❌ ERROR: Failed to initialize locomotion system" << std::endl;
        return 1;
    }

    // Set initial standing pose
    if (!sys.setStandingPose()) {
        std::cout << "❌ ERROR: Failed to set standing pose" << std::endl;
        return 1;
    }

    // Execute startup sequence
    while (!sys.executeStartupSequence()) {
        // Simulate control cycle during transition
    }

    // Setup tripod gait
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.planGaitSequence(TEST_VELOCITY, 0.0, 0.0));

    std::cout << "Starting tripod gait validation..." << std::endl << std::endl;

    // Validation data collection
    TripodValidation validation = {};
    validation.coherence_violations = 0;
    validation.sync_violations = 0;
    validation.symmetry_violations = 0;

    // Run simulation and collect data
    for (int step = 0; step < VALIDATION_STEPS; ++step) {
        double phase = static_cast<double>(step) / static_cast<double>(VALIDATION_STEPS);

        // Update locomotion system
        bool update_success = sys.update();
        if (!update_success) {
            std::cout << "⚠️ WARNING: Update failed at step " << step << std::endl;
            continue;
        }

        // Collect phase data for groups
        StepPhase group_a_phase = sys.getLeg(0).getStepPhase(); // L1 represents group A
        StepPhase group_b_phase = sys.getLeg(1).getStepPhase(); // L2 represents group B

        validation.group_a_phases.push_back(group_a_phase);
        validation.group_b_phases.push_back(group_b_phase);
        validation.gait_phases.push_back(phase);

        // Check symmetry
        bool symmetry_ok = checkLegSymmetry(sys);
        validation.symmetry_checks.push_back(symmetry_ok);

        // Print pattern every 10 steps for monitoring
        //if (step % 10 == 0) {
            printGaitPattern(sys, step, phase);
        //}
    }

    std::cout << "\n=========================================" << std::endl;
    std::cout << "VALIDATION RESULTS" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Run validations
    bool coherence_ok = validateCoherence(validation);
    bool sync_ok = validateSynchronization(validation);
    bool symmetry_ok = validateSymmetry(validation);

    // Final summary
    std::cout << "\n=========================================" << std::endl;
    std::cout << "FINAL VALIDATION SUMMARY" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Coherence:     " << (coherence_ok ? "✓ PASS" : "✗ FAIL") << std::endl;
    std::cout << "Synchronization: " << (sync_ok ? "✓ PASS" : "✗ FAIL") << std::endl;
    std::cout << "Symmetry:      " << (symmetry_ok ? "✓ PASS" : "✗ FAIL") << std::endl;

    bool all_passed = coherence_ok && sync_ok && symmetry_ok;

    if (all_passed) {
        std::cout << "\n🎉 ALL VALIDATIONS PASSED! 🎉" << std::endl;
        std::cout << "Tripod gait demonstrates excellent coherence, synchronization, and symmetry." << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ SOME VALIDATIONS FAILED ❌" << std::endl;
        std::cout << "Tripod gait needs improvement in the failed areas." << std::endl;
        return 1;
    }
}