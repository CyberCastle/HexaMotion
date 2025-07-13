/**
 * @file tripod_gait_sim_test.cpp
 * @brief Enhanced tripod gait simulation test with comprehensive validation
 *
 * This test validates tripod gait functionality using the proper HexaMotion
 * architecture with LocomotionSystem as the orchestrator, integrating all
 * validation tests from tripod_gait_validation_test.cpp.
 *
 * Requirements:
 * - Validate initial standing pose equals final pose (body and legs)
 * - Execute all applicable tests every 10 cycles (stance->swing->stance)
 * - Adjust cycles for 100 meters displacement
 * - Provide summary with error, warning, and ok counts
 * - Test passes only if all tests are valid (no warnings)
 *
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 */

#include "../src/body_pose_config_factory.h"
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <Eigen/Dense>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

// Test configuration for 100 meters displacement
constexpr double TARGET_DISTANCE = 5000.0;                            // 5 meters in mm
constexpr double TEST_VELOCITY = 400.0;                               // mm/s
constexpr double TEST_DURATION = TARGET_DISTANCE / TEST_VELOCITY;     // seconds
constexpr int TEST_CYCLES_PER_VALIDATION = 100;                       // Validate every 100 cycles
constexpr double FINAL_POSE_POSITION_TOLERANCE = 5.0;                 // 5mm tolerance for pose comparison
constexpr double FINAL_POSE_ORIENTATION_TOLERANCE = 2 * M_PI / 180.0; // 2 degrees tolerance for orientation

// Tripod gait validation structure
struct TripodValidation {
    std::vector<StepPhase> group_a_phases; // Legs 1, 3, 5
    std::vector<StepPhase> group_b_phases; // Legs 2, 4, 6
    std::vector<double> gait_phases;
    std::vector<bool> symmetry_checks;

    // Position tracking for complete trajectory symmetry validation
    std::vector<Point3D> leg_positions[NUM_LEGS];  // Positions for each leg over time
    std::vector<JointAngles> leg_angles[NUM_LEGS]; // Angles for each leg over time

    int coherence_violations;
    int sync_violations;
    int symmetry_violations;
    int trajectory_consistency_violations;
};

// Test result tracking
struct TestResults {
    int total_tests;
    int passed_tests;
    int warning_tests;
    int failed_tests;

    std::vector<std::string> passed_messages;
    std::vector<std::string> warning_messages;
    std::vector<std::string> failed_messages;

    void reset() {
        total_tests = 0;
        passed_tests = 0;
        warning_tests = 0;
        failed_tests = 0;
        passed_messages.clear();
        warning_messages.clear();
        failed_messages.clear();
    }

    void addResult(bool passed, bool warning, const std::string &message) {
        total_tests++;
        if (passed && !warning) {
            passed_tests++;
            passed_messages.push_back(message);
        } else if (warning) {
            warning_tests++;
            warning_messages.push_back(message);
        } else {
            failed_tests++;
            failed_messages.push_back(message);
        }
    }

    bool allPassed() const {
        return failed_tests == 0 && warning_tests == 0;
    }
};

static void printWelcome() {
    std::cout << "=========================================" << std::endl;
    std::cout << "HEXAPOD TRIPOD GAIT COMPREHENSIVE TEST" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Enhanced simulation with full validation suite" << std::endl;
    std::cout << "Distance: " << (TARGET_DISTANCE / 1000.0) << "m | Velocity: " << TEST_VELOCITY << "mm/s" << std::endl;
    std::cout << "Duration: " << TEST_DURATION << "s | Validation every " << TEST_CYCLES_PER_VALIDATION << " cycles" << std::endl;
    std::cout << "Architecture: LocomotionSystem orchestrator" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;
}

static void printRobotDimensions(const Parameters &p) {
    std::cout << "=========================================" << std::endl;
    std::cout << "           ROBOT DIMENSIONS" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Hexagon Radius:       " << std::setw(6) << p.hexagon_radius << " mm" << std::endl;
    std::cout << "Coxa Length:          " << std::setw(6) << p.coxa_length << " mm" << std::endl;
    std::cout << "Femur Length:         " << std::setw(6) << p.femur_length << " mm" << std::endl;
    std::cout << "Tibia Length:         " << std::setw(6) << p.tibia_length << " mm" << std::endl;
    std::cout << "Robot Height:         " << std::setw(6) << p.robot_height << " mm" << std::endl;
    std::cout << "Standing Height:      " << std::setw(6) << p.standing_height << " mm" << std::endl;
    std::cout << "Control Frequency:    " << std::setw(6) << p.control_frequency << " Hz" << std::endl;
    std::cout << std::endl;

    double max_reach = p.coxa_length + p.femur_length + p.tibia_length;
    double body_diagonal = p.hexagon_radius * 2;

    std::cout << "CALCULATED DIMENSIONS:" << std::endl;
    std::cout << "Max Leg Reach:     " << std::setw(6) << std::setprecision(1) << max_reach << " mm" << std::endl;
    std::cout << "Body Diagonal:     " << std::setw(6) << std::setprecision(1) << body_diagonal << " mm" << std::endl;
    std::cout << "Working Radius:    " << std::setw(6) << std::setprecision(1) << (p.hexagon_radius + max_reach) << " mm" << std::endl;
    std::cout << std::endl;

    std::cout << "JOINT ANGLE LIMITS:" << std::endl;
    std::cout << "Coxa:  " << std::setw(4) << p.coxa_angle_limits[0] << "° to " << std::setw(4) << p.coxa_angle_limits[1] << "°" << std::endl;
    std::cout << "Femur: " << std::setw(4) << p.femur_angle_limits[0] << "° to " << std::setw(4) << p.femur_angle_limits[1] << "°" << std::endl;
    std::cout << "Tibia: " << std::setw(4) << p.tibia_angle_limits[0] << "° to " << std::setw(4) << p.tibia_angle_limits[1] << "°" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;
}

static void printGaitPattern(const LocomotionSystem &sys, int step, double phase) {
    std::cout << "Step " << std::setw(3) << step << " (Phase: " << std::fixed << std::setprecision(2) << phase << "): ";

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg &leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';
        std::cout << "L" << (leg + 1) << ":" << symbol << " ";
    }

    std::cout << " | Groups: A[";
    for (int leg : {0, 2, 4}) {
        const Leg &leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        std::cout << (state == STANCE_PHASE ? "S" : "W");
    }
    std::cout << "] B[";
    for (int leg : {1, 3, 5}) {
        const Leg &leg_obj = sys.getLeg(leg);
        StepPhase state = leg_obj.getStepPhase();
        std::cout << (state == STANCE_PHASE ? "S" : "W");
    }
    std::cout << "]" << std::endl;
}

static bool validateInitialStandingPose(const LocomotionSystem &sys, TestResults &results) {
    std::cout << "\n=== INITIAL STANDING POSE VALIDATION ===" << std::endl;

    // Get initial body pose
    Eigen::Vector3d initial_position = sys.getBodyPosition();
    Eigen::Vector3d initial_orientation = sys.getBodyOrientation();

    std::cout << "Initial body position: [" << std::fixed << std::setprecision(3)
              << initial_position.x() << ", " << initial_position.y() << ", " << initial_position.z() << "] mm" << std::endl;
    std::cout << "Initial body orientation: [" << std::fixed << std::setprecision(6)
              << initial_orientation.x() << ", " << initial_orientation.y() << ", " << initial_orientation.z() << "] rad" << std::endl;

    // Check that all legs are in stance phase initially
    bool all_stance = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg &leg_obj = sys.getLeg(leg);
        if (leg_obj.getStepPhase() != STANCE_PHASE) {
            all_stance = false;
            break;
        }
    }

    if (all_stance) {
        results.addResult(true, false, "Initial standing pose validation: All legs in stance phase");
        std::cout << "✓ Initial standing pose validation: All legs in stance phase" << std::endl;
    } else {
        results.addResult(false, false, "Initial standing pose validation: Not all legs in stance phase");
        std::cout << "✗ Initial standing pose validation: Not all legs in stance phase" << std::endl;
    }

    return all_stance;
}

static bool validateCoherence(const TripodValidation &validation, TestResults &results) {
    std::cout << "\n=== COHERENCE VALIDATION ===" << std::endl;

    bool group_a_coherent = true;
    bool group_b_coherent = true;

    for (size_t i = 0; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] == validation.group_b_phases[i]) {
            group_a_coherent = false;
            group_b_coherent = false;
            break;
        }
    }

    bool proper_alternation = true;
    int transitions = 0;
    for (size_t i = 1; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] != validation.group_a_phases[i - 1]) {
            transitions++;
        }
    }

    proper_alternation = (transitions >= 40 && transitions <= 60);

    bool coherence_ok = group_a_coherent && group_b_coherent && proper_alternation;

    if (coherence_ok) {
        results.addResult(true, false, "Coherence validation: Groups alternating properly");
        std::cout << "✓ Coherence validation: Groups alternating properly" << std::endl;
    } else {
        results.addResult(false, false, "Coherence validation: Groups not alternating properly");
        std::cout << "✗ Coherence validation: Groups not alternating properly" << std::endl;
    }

    return coherence_ok;
}

static bool validateSynchronization(const TripodValidation &validation, TestResults &results) {
    std::cout << "\n=== SYNCHRONIZATION VALIDATION ===" << std::endl;

    int sync_violations = 0;
    for (size_t i = 0; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] == validation.group_b_phases[i]) {
            sync_violations++;
        }
    }

    double sync_percentage = 100.0 * (1.0 - (double)sync_violations / validation.group_a_phases.size());
    bool passed = sync_percentage >= 95.0;

    if (passed) {
        results.addResult(true, false, "Synchronization validation: Groups properly synchronized");
        std::cout << "✓ Synchronization validation: Groups properly synchronized (" << std::fixed << std::setprecision(1) << sync_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Synchronization validation: Groups not properly synchronized");
        std::cout << "✗ Synchronization validation: Groups not properly synchronized (" << std::fixed << std::setprecision(1) << sync_percentage << "%)" << std::endl;
    }

    return passed;
}

// Añadir función para validar altura de las patas
template <typename GetLegFunc>
bool validateLegsHeight(const Parameters &p, GetLegFunc getLeg, int n_legs, const char *context) {
    const double expected_z = -p.standing_height; // Altura esperada de las patas en mm
    const double tol = 2.0;                       // mm
    bool all_ok = true;
    std::cout << "\n=== HEIGHT VALIDATION: " << context << " ===" << std::endl;
    std::cout << "Leg | z (mm) | Status" << std::endl;
    std::cout << "----|--------|-------" << std::endl;
    for (int i = 0; i < n_legs; ++i) {
        double z = getLeg(i).getCurrentTipPositionGlobal().z;
        bool ok = std::abs(z - expected_z) <= tol;
        all_ok = all_ok && ok;
        std::cout << " L" << (i + 1) << " | " << std::fixed << std::setprecision(1) << z << " | " << (ok ? "✓ OK" : "✗ ERROR") << std::endl;
    }
    if (all_ok) {
        std::cout << "✓ All legs at correct height (" << expected_z << ") mm" << std::endl;
    } else {
        std::cout << "✗ Some legs not at correct height (" << expected_z << ") mm" << std::endl;
    }
    return all_ok;
}

static bool validateJointAngleCoherence(const LocomotionSystem &sys, const Parameters &p, TestResults &results) {
    std::cout << "\n=== JOINT ANGLE COHERENCE VALIDATION ===" << std::endl;

    int coherence_violations = 0;
    int total_checks = 0;
    double expected_z = -p.robot_height;
    double tol = 2.0;

    std::cout << "Current leg positions and angles:" << std::endl;
    std::cout << "Leg | Phase  | Position (x,y,z) mm | Coxa° | Femur° | Tibia° | Height | Status" << std::endl;
    std::cout << "----|--------|---------------------|-------|--------|--------|--------|--------" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Leg &leg_obj = sys.getLeg(leg);
        StepPhase phase = leg_obj.getStepPhase();
        JointAngles angles = leg_obj.getJointAngles();
        Point3D tip_pos = leg_obj.getCurrentTipPositionGlobal();

        total_checks++;

        bool violation = false;
        std::string status = "OK";

        if (phase == STANCE_PHASE) {
            if (angles.tibia > 0.2) {
                coherence_violations++;
                violation = true;
                status = "VIOLATION";
            }
        } else if (phase == SWING_PHASE) {
            if (angles.tibia < -0.5) {
                coherence_violations++;
                violation = true;
                status = "VIOLATION";
            }
        }
        bool height_ok = std::abs(tip_pos.z - expected_z) <= tol;

        std::cout << " L" << (leg + 1) << " | " << (phase == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << tip_pos.x << ","
                  << std::setw(6) << tip_pos.y << "," << std::setw(6) << tip_pos.z << " | "
                  << std::setw(5) << (angles.coxa * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles.femur * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles.tibia * 180.0 / M_PI) << " | "
                  << (height_ok ? "✓" : "✗") << " | "
                  << (violation ? "✗ " : "✓ ") << status << std::endl;
    }

    double coherence_percentage = 100.0 * (1.0 - (double)coherence_violations / total_checks);
    bool passed = coherence_percentage >= 80.0;

    if (passed) {
        results.addResult(true, false, "Joint angle coherence validation: Joint angles consistent with phases");
        std::cout << "✓ Joint angle coherence validation: Joint angles consistent with phases (" << std::fixed << std::setprecision(1) << coherence_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Joint angle coherence validation: Joint angles not consistent with phases");
        std::cout << "✗ Joint angle coherence validation: Joint angles not consistent with phases (" << std::fixed << std::setprecision(1) << coherence_percentage << "%)" << std::endl;
    }

    return passed;
}

static bool validateSymmetry(const TripodValidation &validation, TestResults &results) {
    std::cout << "\n=== SYMMETRY VALIDATION ===" << std::endl;

    int proper_alternations = 0;
    for (size_t i = 0; i < validation.group_a_phases.size(); ++i) {
        if (validation.group_a_phases[i] != validation.group_b_phases[i]) {
            proper_alternations++;
        }
    }

    double alternation_percentage = 100.0 * (double)proper_alternations / validation.group_a_phases.size();
    bool passed = alternation_percentage >= 95.0;

    if (passed) {
        results.addResult(true, false, "Symmetry validation: Groups maintaining proper alternation");
        std::cout << "✓ Symmetry validation: Groups maintaining proper alternation (" << std::fixed << std::setprecision(1) << alternation_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Symmetry validation: Groups not maintaining proper alternation");
        std::cout << "✗ Symmetry validation: Groups not maintaining proper alternation (" << std::fixed << std::setprecision(1) << alternation_percentage << "%)" << std::endl;
    }

    return passed;
}

static void printDetailedLegSymmetry(const LocomotionSystem &sys, const Parameters &p) {
    std::cout << "\n=== DETAILED LEG SYMMETRY ANALYSIS ===" << std::endl;

    // Define opposite leg pairs for symmetry validation (180° apart)
    const int opposite_pairs[3][2] = {
        {0, 5}, // Front Right (-30°) <-> Back Left (150°)
        {1, 4}, // Middle Right (-90°) <-> Middle Left (90°)
        {2, 3}  // Back Right (-150°) <-> Front Left (30°)
    };

    const char *pair_names[3] = {"Front Right <-> Back Left", "Middle Right <-> Middle Left", "Back Right <-> Front Left"};
    const double tolerance = 5.0; // 5 degrees tolerance

    double expected_z = -p.robot_height;
    double tol = 2.0;

    std::cout << "Opposite leg pair analysis:" << std::endl;
    std::cout << "Pair | Leg | Phase  | Position (x,y,z) mm | Coxa° | Femur° | Tibia° | Height | Symmetry" << std::endl;
    std::cout << "-----|-----|--------|---------------------|-------|--------|--------|--------|----------" << std::endl;

    for (int pair = 0; pair < 3; ++pair) {
        int leg1 = opposite_pairs[pair][0];
        int leg2 = opposite_pairs[pair][1];

        const Leg &leg1_obj = sys.getLeg(leg1);
        const Leg &leg2_obj = sys.getLeg(leg2);

        JointAngles angles1 = leg1_obj.getJointAngles();
        JointAngles angles2 = leg2_obj.getJointAngles();
        Point3D pos1 = leg1_obj.getCurrentTipPositionGlobal();
        Point3D pos2 = leg2_obj.getCurrentTipPositionGlobal();
        StepPhase phase1 = leg1_obj.getStepPhase();
        StepPhase phase2 = leg2_obj.getStepPhase();

        bool height1_ok = std::abs(pos1.z - expected_z) <= tol;
        bool height2_ok = std::abs(pos2.z - expected_z) <= tol;

        // Calculate symmetry errors
        double coxa_symmetry_error = std::abs(angles1.coxa + angles2.coxa) * 180.0 / M_PI;
        double femur_symmetry_error = std::abs(angles1.femur - angles2.femur) * 180.0 / M_PI;
        double tibia_symmetry_error = std::abs(angles1.tibia - angles2.tibia) * 180.0 / M_PI;

        bool coxa_symmetric = coxa_symmetry_error < tolerance;
        bool femur_symmetric = femur_symmetry_error < tolerance;
        bool tibia_symmetric = tibia_symmetry_error < tolerance;
        bool pair_symmetric = coxa_symmetric && femur_symmetric && tibia_symmetric;

        // Print first leg
        std::cout << " " << pair_names[pair] << " | L" << (leg1 + 1) << " | "
                  << (phase1 == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << pos1.x << ","
                  << std::setw(6) << pos1.y << "," << std::setw(6) << pos1.z << " | "
                  << std::setw(5) << (angles1.coxa * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles1.femur * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles1.tibia * 180.0 / M_PI) << " | "
                  << (height1_ok ? "✓" : "✗") << " | "
                  << (pair_symmetric ? "✓ SYMMETRIC" : "✗ ASYMMETRIC") << std::endl;

        // Print second leg
        std::cout << "     | L" << (leg2 + 1) << " | "
                  << (phase2 == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << pos2.x << ","
                  << std::setw(6) << pos2.y << "," << std::setw(6) << pos2.z << " | "
                  << std::setw(5) << (angles2.coxa * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles2.femur * 180.0 / M_PI) << " | "
                  << std::setw(6) << (angles2.tibia * 180.0 / M_PI) << " | "
                  << (height2_ok ? "✓" : "✗") << " | "
                  << (pair_symmetric ? "✓ SYMMETRIC" : "✗ ASYMMETRIC") << std::endl;

        // Print symmetry details
        std::cout << "     |     |        |                     |       |        |        | "
                  << "Coxa err: " << std::fixed << std::setprecision(1) << coxa_symmetry_error << "° "
                  << (coxa_symmetric ? "✓" : "✗") << ", "
                  << "Femur err: " << femur_symmetry_error << "° "
                  << (femur_symmetric ? "✓" : "✗") << ", "
                  << "Tibia err: " << tibia_symmetry_error << "° "
                  << (tibia_symmetric ? "✓" : "✗") << std::endl;
        std::cout << std::endl;
    }
}

static bool validateTrajectorySymmetry(const TripodValidation &validation, TestResults &results) {
    std::cout << "\n=== TRAJECTORY SYMMETRY VALIDATION ===" << std::endl;

    int symmetry_steps = 0;
    int total_comparable_steps = 0;
    int group_a_consistent_steps = 0;
    int group_b_consistent_steps = 0;

    for (size_t step = 0; step < validation.group_a_phases.size(); ++step) {
        StepPhase group_a_phase = validation.group_a_phases[step];
        StepPhase group_b_phase = validation.group_b_phases[step];

        bool group_a_stance = (group_a_phase == STANCE_PHASE);
        bool group_a_swing = (group_a_phase == SWING_PHASE);
        bool group_b_stance = (group_b_phase == STANCE_PHASE);
        bool group_b_swing = (group_b_phase == SWING_PHASE);

        total_comparable_steps++;

        bool groups_opposite_phases = (group_a_phase != group_b_phase);

        if (groups_opposite_phases) {
            bool group_a_consistent = (group_a_stance || group_a_swing);
            bool group_b_consistent = (group_b_stance || group_b_swing);

            if (group_a_consistent && group_b_consistent) {
                symmetry_steps++;
            }
        }

        if (group_a_stance || group_a_swing) {
            group_a_consistent_steps++;
        }
        if (group_b_stance || group_b_swing) {
            group_b_consistent_steps++;
        }
    }

    double symmetry_percentage = total_comparable_steps > 0 ? (double)symmetry_steps / total_comparable_steps * 100.0 : 0.0;
    double group_a_consistency = (double)group_a_consistent_steps / validation.group_a_phases.size() * 100.0;
    double group_b_consistency = (double)group_b_consistent_steps / validation.group_a_phases.size() * 100.0;

    bool trajectory_symmetry_ok = symmetry_percentage >= 60.0;
    bool group_consistency_ok = group_a_consistency >= 80.0 && group_b_consistency >= 80.0;
    bool passed = trajectory_symmetry_ok && group_consistency_ok;

    if (passed) {
        results.addResult(true, false, "Trajectory symmetry validation: Trajectories properly symmetrical");
        std::cout << "✓ Trajectory symmetry validation: Trajectories properly symmetrical (" << std::fixed << std::setprecision(1) << symmetry_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Trajectory symmetry validation: Trajectories not properly symmetrical");
        std::cout << "✗ Trajectory symmetry validation: Trajectories not properly symmetrical (" << std::fixed << std::setprecision(1) << symmetry_percentage << "%)" << std::endl;
    }

    return passed;
}

static bool validateCoxaAngleSignOpposition(const TripodValidation &validation, TestResults &results) {
    std::cout << "\n=== COXA ANGLE SIGN OPPOSITION VALIDATION ===" << std::endl;

    // Get current coxa angles and phases from the locomotion system
    // We'll use the last step data for detailed display
    if (!validation.group_a_phases.empty()) {
        size_t last_step = validation.group_a_phases.size() - 1;

        std::cout << "Current coxa angles analysis (Step " << last_step << "):" << std::endl;
        std::cout << "Group | Leg | Phase  | Coxa Angle° | Sign | Opposition Analysis" << std::endl;
        std::cout << "------|-----|--------|-------------|------|-------------------" << std::endl;

        // Group A (Legs 0, 2, 4)
        std::cout << "  A   | L1  | " << (validation.group_a_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[0][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[0][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        std::cout << "  A   | L3  | " << (validation.group_a_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[2][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[2][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        std::cout << "  A   | L5  | " << (validation.group_a_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[4][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[4][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        // Group B (Legs 1, 3, 5)
        std::cout << "  B   | L2  | " << (validation.group_b_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[1][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[1][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        std::cout << "  B   | L4  | " << (validation.group_b_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[3][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[3][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        std::cout << "  B   | L6  | " << (validation.group_b_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING ") << " | "
                  << std::fixed << std::setprecision(1) << std::setw(10) << (validation.leg_angles[5][last_step].coxa * 180.0 / M_PI) << " | "
                  << (validation.leg_angles[5][last_step].coxa >= 0 ? "  +  " : "  -  ") << " | " << std::endl;

        // Calculate and display averages
        double group_a_avg_coxa = (validation.leg_angles[0][last_step].coxa +
                                   validation.leg_angles[2][last_step].coxa +
                                   validation.leg_angles[4][last_step].coxa) /
                                  3.0;
        double group_b_avg_coxa = (validation.leg_angles[1][last_step].coxa +
                                   validation.leg_angles[3][last_step].coxa +
                                   validation.leg_angles[5][last_step].coxa) /
                                  3.0;

        std::cout << std::endl;
        std::cout << "Group A average coxa angle: " << std::fixed << std::setprecision(1) << (group_a_avg_coxa * 180.0 / M_PI) << "° "
                  << (group_a_avg_coxa >= 0 ? "(+)" : "(-)") << std::endl;
        std::cout << "Group B average coxa angle: " << std::fixed << std::setprecision(1) << (group_b_avg_coxa * 180.0 / M_PI) << "° "
                  << (group_b_avg_coxa >= 0 ? "(+)" : "(-)") << std::endl;

        bool signs_opposite = (group_a_avg_coxa * group_b_avg_coxa) < 0;
        std::cout << "Sign opposition: " << (signs_opposite ? "✓ OPPOSITE" : "✗ SAME") << std::endl;

        // Show phase opposition
        bool phases_opposite = (validation.group_a_phases[last_step] != validation.group_b_phases[last_step]);
        std::cout << "Phase opposition: " << (phases_opposite ? "✓ OPPOSITE" : "✗ SAME") << std::endl;
        std::cout << "  Group A phase: " << (validation.group_a_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING") << std::endl;
        std::cout << "  Group B phase: " << (validation.group_b_phases[last_step] == STANCE_PHASE ? "STANCE" : "SWING") << std::endl;
    }

    int opposition_steps = 0;
    int total_comparable_steps = 0;

    for (size_t step = 0; step < validation.group_a_phases.size(); ++step) {
        StepPhase group_a_phase = validation.group_a_phases[step];
        StepPhase group_b_phase = validation.group_b_phases[step];

        total_comparable_steps++;

        bool groups_opposite_phases = (group_a_phase != group_b_phase);

        if (groups_opposite_phases) {
            double group_a_avg_coxa = (validation.leg_angles[0][step].coxa +
                                       validation.leg_angles[2][step].coxa +
                                       validation.leg_angles[4][step].coxa) /
                                      3.0;
            double group_b_avg_coxa = (validation.leg_angles[1][step].coxa +
                                       validation.leg_angles[3][step].coxa +
                                       validation.leg_angles[5][step].coxa) /
                                      3.0;

            bool appropriate_pattern = false;
            if (std::abs(group_a_avg_coxa) > 0.1 && std::abs(group_b_avg_coxa) > 0.1) {
                appropriate_pattern = (group_a_avg_coxa * group_b_avg_coxa) < 0;
            } else if (std::abs(group_a_avg_coxa - group_b_avg_coxa) > 0.3) {
                appropriate_pattern = true;
            } else if (std::abs(group_a_avg_coxa) < 0.2 && std::abs(group_b_avg_coxa) < 0.2) {
                appropriate_pattern = true;
            }

            if (appropriate_pattern) {
                opposition_steps++;
            }
        }
    }

    double opposition_percentage = total_comparable_steps > 0 ? (double)opposition_steps / total_comparable_steps * 100.0 : 0.0;

    bool coxa_opposition_ok = opposition_percentage >= 70.0;

    if (coxa_opposition_ok) {
        results.addResult(true, false, "Coxa angle opposition validation: Coxa angles showing proper opposition");
        std::cout << "✓ Coxa angle opposition validation: Coxa angles showing proper opposition (" << std::fixed << std::setprecision(1) << opposition_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Coxa angle opposition validation: Coxa angles not showing proper opposition");
        std::cout << "✗ Coxa angle opposition validation: Coxa angles not showing proper opposition (" << std::fixed << std::setprecision(1) << opposition_percentage << "%)" << std::endl;
    }

    return coxa_opposition_ok;
}

static bool validateBodyPoseStability(const std::vector<Eigen::Vector3d> &body_positions,
                                      const std::vector<Eigen::Vector3d> &body_orientations,
                                      TestResults &results) {
    std::cout << "\n=== BODY POSE STABILITY VALIDATION ===" << std::endl;

    if (body_positions.empty() || body_orientations.empty()) {
        results.addResult(false, false, "Body pose stability validation: No body pose data collected");
        std::cout << "✗ Body pose stability validation: No body pose data collected" << std::endl;
        return false;
    }

    const double position_tolerance = 1e-2;
    const double orientation_tolerance = 1e-2;

    int stable_steps = 0;
    int total_steps = body_positions.size();

    for (size_t i = 1; i < body_positions.size(); ++i) {
        double position_diff = (body_positions[i] - body_positions[i - 1]).norm();
        double orientation_diff = (body_orientations[i] - body_orientations[i - 1]).norm();

        if (position_diff <= position_tolerance && orientation_diff <= orientation_tolerance) {
            stable_steps++;
        }
    }

    double stability_percentage = 100.0 * (double)stable_steps / (total_steps - 1);
    bool passed = stability_percentage >= 90.0;

    if (passed) {
        results.addResult(true, false, "Body pose stability validation: Body pose remains stable during gait");
        std::cout << "✓ Body pose stability validation: Body pose remains stable during gait (" << std::fixed << std::setprecision(1) << stability_percentage << "%)" << std::endl;
    } else {
        results.addResult(false, false, "Body pose stability validation: Body pose not stable during gait");
        std::cout << "✗ Body pose stability validation: Body pose not stable during gait (" << std::fixed << std::setprecision(1) << stability_percentage << "%)" << std::endl;
    }

    return passed;
}

static bool validateFinalPoseConsistency(const LocomotionSystem &sys,
                                         const Eigen::Vector3d &initial_position,
                                         const Eigen::Vector3d &initial_orientation,
                                         TestResults &results) {
    std::cout << "\n=== FINAL POSE CONSISTENCY VALIDATION ===" << std::endl;

    // Get final body pose
    Eigen::Vector3d final_position = sys.getBodyPosition();
    Eigen::Vector3d final_orientation = sys.getBodyOrientation();

    std::cout << "Final body position: [" << std::fixed << std::setprecision(3)
              << final_position.x() << ", " << final_position.y() << ", " << final_position.z() << "] mm" << std::endl;
    std::cout << "Final body orientation: [" << std::fixed << std::setprecision(6)
              << final_orientation.x() << ", " << final_orientation.y() << ", " << final_orientation.z() << "] rad" << std::endl;

    double position_diff = (final_position - initial_position).norm();
    double orientation_diff = (final_orientation - initial_orientation).norm();

    std::cout << "Position difference: " << position_diff << " mm" << std::endl;
    std::cout << "Orientation difference: " << orientation_diff << " rad" << std::endl;

    bool position_consistent = position_diff <= FINAL_POSE_POSITION_TOLERANCE;
    bool orientation_consistent = orientation_diff <= FINAL_POSE_ORIENTATION_TOLERANCE;
    bool all_consistent = position_consistent && orientation_consistent;

    if (all_consistent) {
        results.addResult(true, false, "Final pose consistency validation: Final pose matches initial pose");
        std::cout << "✓ Final pose consistency validation: Final pose matches initial pose" << std::endl;
    } else {
        results.addResult(false, false, "Final pose consistency validation: Final pose does not match initial pose");
        std::cout << "✗ Final pose consistency validation: Final pose does not match initial pose" << std::endl;
    }

    return all_consistent;
}

static void printTestSummary(const TestResults &results) {
    std::cout << "\n=========================================" << std::endl;
    std::cout << "COMPREHENSIVE TEST SUMMARY" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Total Tests: " << results.total_tests << std::endl;
    std::cout << "Passed: " << results.passed_tests << std::endl;
    std::cout << "Warnings: " << results.warning_tests << std::endl;
    std::cout << "Failed: " << results.failed_tests << std::endl;
    std::cout << "=========================================" << std::endl;

    if (!results.passed_messages.empty()) {
        std::cout << "\nPASSED TESTS:" << std::endl;
        for (const auto &msg : results.passed_messages) {
            std::cout << "✓ " << msg << std::endl;
        }
    }

    if (!results.warning_messages.empty()) {
        std::cout << "\nWARNING TESTS:" << std::endl;
        for (const auto &msg : results.warning_messages) {
            std::cout << "⚠ " << msg << std::endl;
        }
    }

    if (!results.failed_messages.empty()) {
        std::cout << "\nFAILED TESTS:" << std::endl;
        for (const auto &msg : results.failed_messages) {
            std::cout << "✗ " << msg << std::endl;
        }
    }

    std::cout << "\n=========================================" << std::endl;
    if (results.allPassed()) {
        std::cout << "🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "Tripod gait demonstrates excellent performance across all validation criteria." << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED ❌" << std::endl;
        std::cout << "Tripod gait needs improvement in the failed areas." << std::endl;
    }
    std::cout << "=========================================" << std::endl;
}

int main() {
    printWelcome();

    // Initialize robot parameters
    Parameters p = createDefaultParameters();

    // Verify parameters
    assert(p.hexagon_radius == 200.0f);
    assert(p.coxa_length == 50.0f);
    assert(p.femur_length == 101.0f);
    assert(p.tibia_length == 208.0f);
    assert(p.robot_height == 208.0f);
    assert(p.control_frequency == 50.0f);

    printRobotDimensions(p);

    // Calculate total steps for 100 meters
    unsigned total_steps = static_cast<unsigned>(TEST_DURATION * p.control_frequency);
    unsigned validation_steps = total_steps / TEST_CYCLES_PER_VALIDATION;

    std::cout << "Simulation Configuration:" << std::endl;
    std::cout << "  Total distance: " << (TARGET_DISTANCE / 1000.0) << " meters" << std::endl;
    std::cout << "  Velocity: " << TEST_VELOCITY << " mm/s" << std::endl;
    std::cout << "  Duration: " << TEST_DURATION << " seconds" << std::endl;
    std::cout << "  Total steps: " << total_steps << std::endl;
    std::cout << "  Validation cycles: " << validation_steps << " (every " << TEST_CYCLES_PER_VALIDATION << " steps)" << std::endl;
    std::cout << std::endl;

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
    // Validar altura de patas en standing pose
    validateLegsHeight(p, [&](int i) { return sys.getLeg(i); }, NUM_LEGS, "STANDING POSE");

    // Execute startup sequence
    while (!sys.executeStartupSequence()) {
        // Simulate control cycle during transition
    }

    // Setup tripod gait
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(TEST_VELOCITY));

    // Store initial pose for final comparison
    Eigen::Vector3d initial_position = sys.getBodyPosition();
    Eigen::Vector3d initial_orientation = sys.getBodyOrientation();

    // Initialize test results
    TestResults results;
    results.reset();

    // Validate initial standing pose
    validateInitialStandingPose(sys, results);

    std::cout << "Starting comprehensive tripod gait simulation..." << std::endl;
    std::cout << "Validating every " << TEST_CYCLES_PER_VALIDATION << " cycles..." << std::endl;
    std::cout << std::endl;

    // Run simulation with periodic validation
    for (unsigned step = 0; step < total_steps; ++step) {
        double phase = static_cast<double>(step) / static_cast<double>(total_steps);
        double distance_covered = phase * TARGET_DISTANCE;

        // Update locomotion system
        bool update_success = sys.update();
        if (!update_success) {
            std::cout << "⚠️ WARNING: Update failed at step " << step << std::endl;
            continue;
        }

        // Perform validation every TEST_CYCLES_PER_VALIDATION steps
        if (step % TEST_CYCLES_PER_VALIDATION == 0) {
            std::cout << "\n--- VALIDATION CYCLE " << (step / TEST_CYCLES_PER_VALIDATION) << " ---" << std::endl;
            std::cout << "Step: " << step << " | Phase: " << std::fixed << std::setprecision(2) << phase << std::endl;
            std::cout << "Distance covered: " << std::fixed << std::setprecision(1) << (distance_covered / 1000.0) << " meters" << std::endl;

            // Collect validation data
            TripodValidation validation = {};
            validation.coherence_violations = 0;
            validation.sync_violations = 0;
            validation.symmetry_violations = 0;
            validation.trajectory_consistency_violations = 0;

            // Initialize trajectory tracking arrays
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                validation.leg_positions[leg].reserve(TEST_CYCLES_PER_VALIDATION);
                validation.leg_angles[leg].reserve(TEST_CYCLES_PER_VALIDATION);
            }

            // Collect data for this validation cycle
            std::vector<Eigen::Vector3d> body_positions;
            std::vector<Eigen::Vector3d> body_orientations;

            for (unsigned cycle_step = 0; cycle_step < TEST_CYCLES_PER_VALIDATION && (step + cycle_step) < total_steps; ++cycle_step) {
                if (step + cycle_step < total_steps) {
                    sys.update();

                    // Store body pose
                    body_positions.push_back(sys.getBodyPosition());
                    body_orientations.push_back(sys.getBodyOrientation());

                    // Collect phase data
                    StepPhase group_a_phase = sys.getLeg(0).getStepPhase();
                    StepPhase group_b_phase = sys.getLeg(1).getStepPhase();

                    validation.group_a_phases.push_back(group_a_phase);
                    validation.group_b_phases.push_back(group_b_phase);
                    validation.gait_phases.push_back(static_cast<double>(cycle_step) / TEST_CYCLES_PER_VALIDATION);

                    // Collect trajectory data
                    for (int leg = 0; leg < NUM_LEGS; ++leg) {
                        const Leg &leg_obj = sys.getLeg(leg);
                        Point3D tip_pos = leg_obj.getCurrentTipPositionGlobal();
                        JointAngles angles = leg_obj.getJointAngles();

                        validation.leg_positions[leg].push_back(tip_pos);
                        validation.leg_angles[leg].push_back(angles);
                    }
                }
            }

            // Run all validation tests
            validateCoherence(validation, results);
            validateSynchronization(validation, results);
            validateJointAngleCoherence(sys, p, results);
            validateSymmetry(validation, results);
            printDetailedLegSymmetry(sys, p);
            validateTrajectorySymmetry(validation, results);
            validateCoxaAngleSignOpposition(validation, results);
            validateBodyPoseStability(body_positions, body_orientations, results);

            // Print current gait pattern
            printGaitPattern(sys, step, phase);
        }
    }

    // Final validation: check that final pose matches initial pose
    validateFinalPoseConsistency(sys, initial_position, initial_orientation, results);

    // Print comprehensive test summary
    printTestSummary(results);

    // Return success only if all tests passed (no failures and no warnings)
    return results.allPassed() ? 0 : 1;
}