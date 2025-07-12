/**
 * @file tripod_gait_detailed_analysis.cpp
 * @brief Detailed analysis of tripod gait issues with specific diagnostics
 *
 * This test provides detailed analysis of tripod gait problems:
 * - Phase transition analysis
 * - Group synchronization patterns
 * - Symmetry violation details
 * - Recommendations for improvement
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
#include <map>

// Test configuration
constexpr int ANALYSIS_STEPS = 200;  // More steps for better analysis
constexpr double TEST_VELOCITY = 200.0; // mm/s
constexpr double TEST_DISTANCE = 800.0; // mm

// Detailed analysis structure
struct DetailedAnalysis {
    std::vector<StepPhase> all_leg_phases[NUM_LEGS];
    std::vector<double> gait_phases;
    std::vector<std::string> phase_transitions;
    std::map<std::string, int> pattern_frequency;
    int total_transitions;
    int unexpected_transitions;
};

static void printAnalysisHeader() {
    std::cout << "=========================================" << std::endl;
    std::cout << "TRIPOD GAIT DETAILED ANALYSIS" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Analyzing: Phase transitions, patterns, violations" << std::endl;
    std::cout << "Steps: " << ANALYSIS_STEPS << " | Velocity: " << TEST_VELOCITY << " mm/s" << std::endl;
    std::cout << "Distance: " << TEST_DISTANCE << " mm" << std::endl;
    std::cout << "=========================================" << std::endl << std::endl;
}

static void printDetailedPattern(const LocomotionSystem& sys, int step, double phase) {
    std::cout << "Step " << std::setw(3) << step << " (Phase: " << std::fixed << std::setprecision(2) << phase << "): ";

    // Print individual leg states
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg& leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';
        std::cout << "L" << (leg + 1) << ":" << symbol << " ";
    }

    // Print group patterns
    std::cout << " | A[";
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
    std::cout << "]";

    // Print symmetry check
    bool symmetry_ok = true;
    for (int i = 0; i < 3; ++i) {
        const Leg& leg1 = sys.getLeg(i);
        const Leg& leg2 = sys.getLeg(i + 3);
        if (leg1.getStepPhase() == leg2.getStepPhase()) {
            symmetry_ok = false;
            break;
        }
    }
    std::cout << " | Sym: " << (symmetry_ok ? "✓" : "✗");

    std::cout << std::endl;
}

static std::string getPatternString(const LocomotionSystem& sys) {
    std::string pattern;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg& leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        pattern += (state == STANCE_PHASE) ? "S" : "W";
    }
    return pattern;
}

static void analyzePhaseTransitions(const DetailedAnalysis& analysis) {
    std::cout << "\n=== PHASE TRANSITION ANALYSIS ===" << std::endl;

    std::cout << "Total transitions observed: " << analysis.total_transitions << std::endl;
    std::cout << "Unexpected transitions: " << analysis.unexpected_transitions << std::endl;

    if (analysis.total_transitions > 0) {
        double unexpected_rate = 100.0 * analysis.unexpected_transitions / analysis.total_transitions;
        std::cout << "Unexpected transition rate: " << std::fixed << std::setprecision(1) << unexpected_rate << "%" << std::endl;
    }

    std::cout << "\nMost common patterns:" << std::endl;
    int count = 0;
    for (const auto& pattern : analysis.pattern_frequency) {
        if (count < 5) { // Show top 5 patterns
            std::cout << "  " << pattern.first << ": " << pattern.second << " times" << std::endl;
            count++;
        }
    }
}

static void analyzeGroupBehavior(const DetailedAnalysis& analysis) {
    std::cout << "\n=== GROUP BEHAVIOR ANALYSIS ===" << std::endl;

    // Analyze Group A (L1, L3, L5)
    std::cout << "Group A (L1,L3,L5) behavior:" << std::endl;
    for (int leg_idx : {0, 2, 4}) {
        std::cout << "  L" << (leg_idx + 1) << ": ";
        int stance_count = 0, swing_count = 0;
        for (const auto& phase : analysis.all_leg_phases[leg_idx]) {
            if (phase == STANCE_PHASE) stance_count++;
            else swing_count++;
        }
        double stance_percent = 100.0 * stance_count / analysis.all_leg_phases[leg_idx].size();
        std::cout << std::fixed << std::setprecision(1) << stance_percent << "% stance, "
                  << (100.0 - stance_percent) << "% swing" << std::endl;
    }

    // Analyze Group B (L2, L4, L6)
    std::cout << "Group B (L2,L4,L6) behavior:" << std::endl;
    for (int leg_idx : {1, 3, 5}) {
        std::cout << "  L" << (leg_idx + 1) << ": ";
        int stance_count = 0, swing_count = 0;
        for (const auto& phase : analysis.all_leg_phases[leg_idx]) {
            if (phase == STANCE_PHASE) stance_count++;
            else swing_count++;
        }
        double stance_percent = 100.0 * stance_count / analysis.all_leg_phases[leg_idx].size();
        std::cout << std::fixed << std::setprecision(1) << stance_percent << "% stance, "
                  << (100.0 - stance_percent) << "% swing" << std::endl;
    }
}

static void provideRecommendations() {
    std::cout << "\n=== RECOMMENDATIONS ===" << std::endl;
    std::cout << "Based on the analysis, consider the following improvements:" << std::endl;
    std::cout << std::endl;
    std::cout << "1. GAIT SYNCHRONIZATION:" << std::endl;
    std::cout << "   - Ensure Group A (L1,L3,L5) and Group B (L2,L4,L6) are always in opposite phases" << std::endl;
    std::cout << "   - Implement proper phase locking between leg groups" << std::endl;
    std::cout << "   - Add gait phase validation in the walk controller" << std::endl;
    std::cout << std::endl;
    std::cout << "2. SYMMETRY ENFORCEMENT:" << std::endl;
    std::cout << "   - Ensure L1-L4, L2-L5, L3-L6 pairs maintain opposite phases" << std::endl;
    std::cout << "   - Add symmetry checks in the leg coordination logic" << std::endl;
    std::cout << "   - Consider implementing a symmetry correction mechanism" << std::endl;
    std::cout << std::endl;
    std::cout << "3. PHASE COHERENCE:" << std::endl;
    std::cout << "   - Implement consistent phase progression across all legs" << std::endl;
    std::cout << "   - Add phase transition validation" << std::endl;
    std::cout << "   - Consider using a centralized gait phase manager" << std::endl;
    std::cout << std::endl;
    std::cout << "4. TESTING IMPROVEMENTS:" << std::endl;
    std::cout << "   - Add unit tests for individual leg phase transitions" << std::endl;
    std::cout << "   - Implement gait pattern validation in the locomotion system" << std::endl;
    std::cout << "   - Add real-time gait quality monitoring" << std::endl;
}

int main() {
    printAnalysisHeader();

    // Initialize robot parameters
    Parameters p = createDefaultParameters();

    // Verify parameters
    assert(p.hexagon_radius == 200.0f);
    assert(p.coxa_length == 50.0f);
    assert(p.femur_length == 101.0f);
    assert(p.tibia_length == 208.0f);
    assert(p.robot_height == 208.0f);
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

    std::cout << "Starting detailed tripod gait analysis..." << std::endl << std::endl;

    // Analysis data collection
    DetailedAnalysis analysis = {};
    analysis.total_transitions = 0;
    analysis.unexpected_transitions = 0;

    std::string previous_pattern = "";

    // Run simulation and collect detailed data
    for (int step = 0; step < ANALYSIS_STEPS; ++step) {
        double phase = static_cast<double>(step) / static_cast<double>(ANALYSIS_STEPS);

        // Update locomotion system
        bool update_success = sys.update();
        if (!update_success) {
            std::cout << "⚠️ WARNING: Update failed at step " << step << std::endl;
            continue;
        }

        // Collect phase data for all legs
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const Leg& leg_obj = sys.getLeg(leg);
            analysis.all_leg_phases[leg].push_back(leg_obj.getStepPhase());
        }

        analysis.gait_phases.push_back(phase);

        // Track pattern changes
        std::string current_pattern = getPatternString(sys);
        analysis.pattern_frequency[current_pattern]++;

        if (!previous_pattern.empty() && previous_pattern != current_pattern) {
            analysis.total_transitions++;
            analysis.phase_transitions.push_back(previous_pattern + " -> " + current_pattern);

            // Check if transition is unexpected (groups not in opposite phases)
            bool group_a_stance = (current_pattern[0] == 'S' && current_pattern[2] == 'S' && current_pattern[4] == 'S');
            bool group_b_stance = (current_pattern[1] == 'S' && current_pattern[3] == 'S' && current_pattern[5] == 'S');

            if (group_a_stance == group_b_stance) {
                analysis.unexpected_transitions++;
            }
        }
        previous_pattern = current_pattern;

        // Print detailed pattern every 20 steps for monitoring
        if (step % 20 == 0) {
            printDetailedPattern(sys, step, phase);
        }
    }

    std::cout << "\n=========================================" << std::endl;
    std::cout << "DETAILED ANALYSIS RESULTS" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Run detailed analyses
    analyzePhaseTransitions(analysis);
    analyzeGroupBehavior(analysis);
    provideRecommendations();

    // Final summary
    std::cout << "\n=========================================" << std::endl;
    std::cout << "ANALYSIS SUMMARY" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "✓ Detailed analysis completed successfully" << std::endl;
    std::cout << "✓ Pattern frequency analysis performed" << std::endl;
    std::cout << "✓ Phase transition tracking implemented" << std::endl;
    std::cout << "✓ Group behavior analysis completed" << std::endl;
    std::cout << "✓ Recommendations provided for improvement" << std::endl;
    std::cout << "=========================================" << std::endl;

    std::cout << "\n🎯 ANALYSIS COMPLETE - USE RESULTS TO IMPROVE TRIPOD GAIT 🎯" << std::endl;
    return 0;
}