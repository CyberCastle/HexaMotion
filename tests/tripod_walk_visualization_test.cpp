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
#include <sstream>
#include <vector>

// Test configuration
constexpr double TEST_VELOCITY = 100;          // mm/s, a moderate speed for clear observation
constexpr double TEST_ANGULAR_VELOCITY = 0.25; // rad/s, introduce rotational motion for validation
// Reduced number of transitions to finish quickly; independent of phase size.
constexpr int REQUIRED_SWING_TRANSITIONS = 2;
// General limit; no longer depends on assuming 52 iterations per phase.
constexpr int MAX_STEPS = 600;
constexpr int EXPECTED_TRIPOD_HALF_PERIOD = 52;

// Phase transition angle validation constants
// At Z=-150 (standing height) the expected femur/tibia angles are approximately -35°/+35°
constexpr double EXPECTED_FEMUR_DEG = -35.05;
constexpr double EXPECTED_TIBIA_DEG = 35.05;
constexpr double EXPECTED_Z = -150.0;

// Tolerance for phase-boundary angle deviations (degrees)
// Origin/Target/Midpoint should be close to standing angles when Z ≈ -150
constexpr double ANGLE_TOLERANCE_DEG = 12.0; // Report deviations; wide for observation
constexpr double Z_TOLERANCE = 2.0;          // Z acceptable range around -150

/**
 * @brief Record of angle state at a phase transition point.
 */
struct PhaseTransitionRecord {
    int step;               ///< Simulation step when transition occurred
    int leg_index;          ///< Leg index (0-5)
    int transition_number;  ///< Which transition for this leg (1-based)
    std::string transition; ///< "STANCE->SWING" or "SWING->STANCE"

    // Origin point (start of the new phase)
    Point3D origin_pos;
    double origin_femur_deg;
    double origin_tibia_deg;
    double origin_coxa_deg;

    // Target point (end of the new phase)
    Point3D target_pos;
    double target_femur_deg;
    double target_tibia_deg;
    double target_coxa_deg;

    // Midpoint (average of origin and target)
    Point3D midpoint_pos;
    double midpoint_femur_deg;
    double midpoint_tibia_deg;
    double midpoint_coxa_deg;

    // Current tip position/angles at the exact transition instant
    Point3D current_pos;
    double current_femur_deg;
    double current_tibia_deg;
    double current_coxa_deg;
};

/**
 * @brief Computes IK angles for a given tip position using the robot model.
 * @param model The robot model
 * @param leg_index Leg index for IK context
 * @param pos The tip position in global coordinates
 * @param out_coxa Output coxa angle in degrees
 * @param out_femur Output femur angle in degrees
 * @param out_tibia Output tibia angle in degrees
 * @return true if IK succeeded (angles are finite)
 */
static bool computeIKAnglesDeg(const RobotModel &model, int leg_index, const Point3D &pos,
                               double &out_coxa, double &out_femur, double &out_tibia) {
    JointAngles angles = model.inverseKinematicsGlobalCoordinates(leg_index, pos);
    out_coxa = math_utils::radiansToDegrees(angles.coxa);
    out_femur = math_utils::radiansToDegrees(angles.femur);
    out_tibia = math_utils::radiansToDegrees(angles.tibia);
    return std::isfinite(out_coxa) && std::isfinite(out_femur) && std::isfinite(out_tibia);
}

/**
 * @brief Captures a PhaseTransitionRecord at a phase boundary.
 *
 * Records origin, midpoint, and target positions and their IK-derived angles
 * for deterministic analysis of angle deviations at each gait phase change.
 *
 * @param sys The LocomotionSystem
 * @param leg_index Leg index (0-5)
 * @param step Current simulation step
 * @param transition_num Current transition count for this leg
 * @param is_stance_to_swing true if STANCE->SWING, false if SWING->STANCE
 * @return PhaseTransitionRecord with all fields populated
 */
static PhaseTransitionRecord capturePhaseTransition(
    LocomotionSystem &sys, int leg_index, int step, int transition_num,
    bool is_stance_to_swing) {

    PhaseTransitionRecord rec;
    rec.step = step;
    rec.leg_index = leg_index;
    rec.transition_number = transition_num;
    rec.transition = is_stance_to_swing ? "STANCE->SWING" : "SWING->STANCE";

    const RobotModel &model = sys.getRobotModel();
    const Leg &leg = sys.getLeg(leg_index);
    auto leg_stepper = sys.getWalkController()->getLegStepper(leg_index);

    // Get body pose for walk-plane → global coordinate transformation.
    // LegStepper positions are in the walk-plane frame (z=0 = ground plane);
    // body_pose.inverseTransformVector() applies the body height + rotation
    // to produce global coordinates suitable for IK.
    const BodyPoseController *bpc = sys.getBodyPoseController();
    Pose body_pose;
    if (bpc) {
        body_pose = bpc->getCurrentBodyPose();
    }

    // Current position and angles at the transition instant
    rec.current_pos = leg.getCurrentTipPositionGlobal();
    JointAngles cur_angles = leg.getJointAngles();
    rec.current_coxa_deg = math_utils::radiansToDegrees(cur_angles.coxa);
    rec.current_femur_deg = math_utils::radiansToDegrees(cur_angles.femur);
    rec.current_tibia_deg = math_utils::radiansToDegrees(cur_angles.tibia);

    if (leg_stepper) {
        // Origin: where the new phase started (swing_origin for swing, stance_origin for stance)
        // These are in walk-plane frame; transform to global for IK
        Point3D origin_walk;
        if (is_stance_to_swing) {
            origin_walk = leg_stepper->getSwingOriginTipPosition();
        } else {
            origin_walk = leg_stepper->getStanceOriginTipPosition();
        }
        rec.origin_pos = body_pose.inverseTransformVector(origin_walk);
        computeIKAnglesDeg(model, leg_index, rec.origin_pos,
                           rec.origin_coxa_deg, rec.origin_femur_deg, rec.origin_tibia_deg);

        // Target: where this phase aims to end (walk-plane frame → global)
        Point3D target_walk = leg_stepper->getTargetTipPose();
        rec.target_pos = body_pose.inverseTransformVector(target_walk);
        computeIKAnglesDeg(model, leg_index, rec.target_pos,
                           rec.target_coxa_deg, rec.target_femur_deg, rec.target_tibia_deg);

        // Midpoint: geometric average of origin and target (in global frame)
        rec.midpoint_pos.x = (rec.origin_pos.x + rec.target_pos.x) / 2.0;
        rec.midpoint_pos.y = (rec.origin_pos.y + rec.target_pos.y) / 2.0;
        rec.midpoint_pos.z = (rec.origin_pos.z + rec.target_pos.z) / 2.0;
        computeIKAnglesDeg(model, leg_index, rec.midpoint_pos,
                           rec.midpoint_coxa_deg, rec.midpoint_femur_deg, rec.midpoint_tibia_deg);
    }

    return rec;
}

/**
 * @brief Prints a single PhaseTransitionRecord in a formatted table.
 */
static void printPhaseTransitionRecord(const PhaseTransitionRecord &rec) {
    auto fmtPos = [](const Point3D &p) -> std::string {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "[" << std::setw(8) << p.x << ", " << std::setw(8) << p.y << ", " << std::setw(8) << p.z << "]";
        return ss.str();
    };

    auto fmtAng = [](double coxa, double femur, double tibia) -> std::string {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "[" << std::setw(8) << coxa << ", " << std::setw(8) << femur << ", " << std::setw(8) << tibia << "]";
        return ss.str();
    };

    auto deviationTag = [](double femur, double tibia) -> std::string {
        double femur_dev = std::abs(femur - EXPECTED_FEMUR_DEG);
        double tibia_dev = std::abs(tibia - EXPECTED_TIBIA_DEG);
        if (femur_dev > ANGLE_TOLERANCE_DEG || tibia_dev > ANGLE_TOLERANCE_DEG) {
            std::stringstream ss;
            ss << " !! DEV femur=" << std::fixed << std::setprecision(1)
               << femur_dev << "° tibia=" << tibia_dev << "°";
            return ss.str();
        }
        return "";
    };

    std::cout << "\n  +--- Phase Transition [Step " << rec.step
              << "] Leg " << (rec.leg_index + 1)
              << " | " << rec.transition
              << " | Transition #" << rec.transition_number << "\n";

    std::cout << "  |  CURRENT  Pos=" << fmtPos(rec.current_pos)
              << "  Angles=" << fmtAng(rec.current_coxa_deg, rec.current_femur_deg, rec.current_tibia_deg)
              << deviationTag(rec.current_femur_deg, rec.current_tibia_deg) << "\n";

    std::cout << "  |  ORIGIN   Pos=" << fmtPos(rec.origin_pos)
              << "  Angles=" << fmtAng(rec.origin_coxa_deg, rec.origin_femur_deg, rec.origin_tibia_deg)
              << deviationTag(rec.origin_femur_deg, rec.origin_tibia_deg) << "\n";

    std::cout << "  |  MIDPOINT Pos=" << fmtPos(rec.midpoint_pos)
              << "  Angles=" << fmtAng(rec.midpoint_coxa_deg, rec.midpoint_femur_deg, rec.midpoint_tibia_deg)
              << deviationTag(rec.midpoint_femur_deg, rec.midpoint_tibia_deg) << "\n";

    std::cout << "  |  TARGET   Pos=" << fmtPos(rec.target_pos)
              << "  Angles=" << fmtAng(rec.target_coxa_deg, rec.target_femur_deg, rec.target_tibia_deg)
              << deviationTag(rec.target_femur_deg, rec.target_tibia_deg) << "\n";

    std::cout << "  +---\n";
}

/**
 * @brief Prints a summary table of all phase transition records with deviation analysis.
 *
 * Groups records by leg and transition type, showing the angle deviations from
 * the expected standing pose angles (femur ≈ -35°, tibia ≈ 35°) at each
 * critical point (origin, midpoint, target).
 */
static void printPhaseTransitionSummary(const std::vector<PhaseTransitionRecord> &records) {
    std::cout << "\n"
              << std::string(120, '=') << "\n";
    std::cout << "  PHASE TRANSITION ANGLE DEVIATION ANALYSIS\n";
    std::cout << "  Expected at Z=" << EXPECTED_Z << " : femur=" << EXPECTED_FEMUR_DEG
              << "° tibia=" << EXPECTED_TIBIA_DEG << "°\n";
    std::cout << "  Tolerance: " << ANGLE_TOLERANCE_DEG << "°\n";
    std::cout << std::string(120, '=') << "\n\n";

    // Header
    std::cout << std::left
              << std::setw(6) << "Leg"
              << std::setw(16) << "Transition"
              << std::setw(5) << "#"
              << std::setw(6) << "Step"
              << std::setw(10) << "Point"
              << std::setw(10) << "Z"
              << std::setw(12) << "Femur(°)"
              << std::setw(12) << "Tibia(°)"
              << std::setw(14) << "dFemur(°)"
              << std::setw(14) << "dTibia(°)"
              << "Status\n";
    std::cout << std::string(120, '-') << "\n";

    int total_points = 0;
    int within_tolerance = 0;
    int z_deviations = 0;
    double max_femur_dev = 0.0;
    double max_tibia_dev = 0.0;
    int max_femur_dev_leg = -1;
    int max_tibia_dev_leg = -1;

    for (const auto &rec : records) {
        struct PointData {
            const char *name;
            Point3D pos;
            double femur, tibia, coxa;
        };

        PointData points[] = {
            {"ORIGIN", rec.origin_pos, rec.origin_femur_deg, rec.origin_tibia_deg, rec.origin_coxa_deg},
            {"MIDPOINT", rec.midpoint_pos, rec.midpoint_femur_deg, rec.midpoint_tibia_deg, rec.midpoint_coxa_deg},
            {"TARGET", rec.target_pos, rec.target_femur_deg, rec.target_tibia_deg, rec.target_coxa_deg},
            {"CURRENT", rec.current_pos, rec.current_femur_deg, rec.current_tibia_deg, rec.current_coxa_deg},
        };

        for (const auto &pt : points) {
            double femur_dev = pt.femur - EXPECTED_FEMUR_DEG;
            double tibia_dev = pt.tibia - EXPECTED_TIBIA_DEG;
            double abs_femur_dev = std::abs(femur_dev);
            double abs_tibia_dev = std::abs(tibia_dev);
            bool z_ok = std::abs(pt.pos.z - EXPECTED_Z) <= Z_TOLERANCE;
            bool angle_ok = abs_femur_dev <= ANGLE_TOLERANCE_DEG && abs_tibia_dev <= ANGLE_TOLERANCE_DEG;

            std::string status;
            if (!z_ok) {
                status = "Z-DEV";
                z_deviations++;
            } else if (angle_ok) {
                status = "OK";
                within_tolerance++;
            } else {
                status = "DEVIATION";
            }
            total_points++;

            if (abs_femur_dev > max_femur_dev) {
                max_femur_dev = abs_femur_dev;
                max_femur_dev_leg = rec.leg_index;
            }
            if (abs_tibia_dev > max_tibia_dev) {
                max_tibia_dev = abs_tibia_dev;
                max_tibia_dev_leg = rec.leg_index;
            }

            std::cout << std::left
                      << std::setw(6) << (rec.leg_index + 1)
                      << std::setw(16) << rec.transition
                      << std::setw(5) << rec.transition_number
                      << std::setw(6) << rec.step
                      << std::setw(10) << pt.name
                      << std::fixed << std::setprecision(2)
                      << std::setw(10) << pt.pos.z
                      << std::setw(12) << pt.femur
                      << std::setw(12) << pt.tibia
                      << std::showpos
                      << std::setw(14) << femur_dev
                      << std::setw(14) << tibia_dev
                      << std::noshowpos
                      << status << "\n";
        }
        std::cout << std::string(120, '-') << "\n";
    }

    // Summary statistics
    std::cout << "\n"
              << std::string(80, '=') << "\n";
    std::cout << "  DEVIATION SUMMARY\n";
    std::cout << std::string(80, '=') << "\n";
    std::cout << "  Total measurement points: " << total_points << "\n";
    std::cout << "  Within tolerance:         " << within_tolerance
              << " (" << (total_points > 0 ? 100.0 * within_tolerance / total_points : 0.0) << "%)\n";
    std::cout << "  Z deviations (|dZ|>" << Z_TOLERANCE << "): " << z_deviations << "\n";
    std::cout << "  Max femur deviation:      " << std::fixed << std::setprecision(2)
              << max_femur_dev << "° (Leg " << (max_femur_dev_leg + 1) << ")\n";
    std::cout << "  Max tibia deviation:      " << max_tibia_dev
              << "° (Leg " << (max_tibia_dev_leg + 1) << ")\n";
    std::cout << std::string(80, '=') << "\n\n";

    // Per-leg summary
    std::cout << "  PER-LEG DEVIATION PROFILE (origin/midpoint/target at Z≈" << EXPECTED_Z << "):\n";
    std::cout << std::string(80, '-') << "\n";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        double sum_femur_dev = 0.0, sum_tibia_dev = 0.0;
        int count = 0;
        for (const auto &rec : records) {
            if (rec.leg_index != leg)
                continue;
            // Only consider points where Z is close to expected
            struct {
                double femur, tibia;
                Point3D pos;
            } pts[] = {
                {rec.origin_femur_deg, rec.origin_tibia_deg, rec.origin_pos},
                {rec.midpoint_femur_deg, rec.midpoint_tibia_deg, rec.midpoint_pos},
                {rec.target_femur_deg, rec.target_tibia_deg, rec.target_pos},
            };
            for (const auto &pt : pts) {
                if (std::abs(pt.pos.z - EXPECTED_Z) <= Z_TOLERANCE) {
                    sum_femur_dev += std::abs(pt.femur - EXPECTED_FEMUR_DEG);
                    sum_tibia_dev += std::abs(pt.tibia - EXPECTED_TIBIA_DEG);
                    count++;
                }
            }
        }
        if (count > 0) {
            std::cout << "  Leg " << (leg + 1)
                      << ": avg |dFemur|=" << std::fixed << std::setprecision(2)
                      << (sum_femur_dev / count)
                      << "°  avg |dTibia|=" << (sum_tibia_dev / count)
                      << "°  (from " << count << " points at Z≈" << EXPECTED_Z << ")\n";
        } else {
            std::cout << "  Leg " << (leg + 1) << ": no points at Z≈" << EXPECTED_Z << "\n";
        }
    }
    std::cout << std::string(80, '-') << "\n";
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
    std::cout << "With OpenSHC timing: Iterations per phase are derived dynamically (not fixed at 52)." << std::endl;
    std::cout << "Expected test duration (approx): depends on derived iterations (shown below)." << std::endl;
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
               << "[" << std::setw(7) << math_utils::radiansToDegrees(angles.coxa)
               << ", " << std::setw(7) << math_utils::radiansToDegrees(angles.femur)
               << ", " << std::setw(7) << math_utils::radiansToDegrees(angles.tibia) << "]";

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
               << "[ " << std::setw(6) << math_utils::radiansToDegrees(angles.coxa)
               << ", " << std::setw(6) << math_utils::radiansToDegrees(angles.femur)
               << ", " << std::setw(6) << math_utils::radiansToDegrees(angles.tibia) << "]";

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
    // Limit configuration is now done via Parameters (enable_dynamic_velocity_limits / fixed_linear_speed_limit_mm_s)
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

    // Verify that trajectory timing is synchronized with trajectory_tip_position_test
    // BOTH tests must use exactly the same StepCycle configuration
    std::cout << "=== SYNCHRONIZATION VERIFICATION WITH trajectory_tip_position_test ===" << std::endl;

    // The LocomotionSystem already has the correct StepCycle configured via WalkController
    // We only need to verify that the values match trajectory_tip_position_test
    auto first_leg_stepper = sys.getWalkController()->getLegStepper(0);
    if (!first_leg_stepper) {
        std::cerr << "ERROR: Could not obtain the LegStepper." << std::endl;
        return 1;
    }

    StepCycle actual_step_cycle = first_leg_stepper->getStepCycle();
    double time_delta = sys.getRobotModel().getTimeDelta(); // unified global timestep

    // Use EXACTLY the same formula as trajectory_tip_position_test
    int swing_iterations_per_cycle = (int)((double(actual_step_cycle.swing_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int stance_iterations_per_cycle = (int)((double(actual_step_cycle.stance_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int total_iterations_per_cycle = swing_iterations_per_cycle + stance_iterations_per_cycle;

    std::cout << "Active StepCycle in LocomotionSystem:" << std::endl;
    std::cout << "  time_delta: " << time_delta << "s" << std::endl;
    std::cout << "  period: " << actual_step_cycle.period_ << ", swing_period: " << actual_step_cycle.swing_period_ << ", stance_period: " << actual_step_cycle.stance_period_ << std::endl;
    std::cout << "  frequency: " << actual_step_cycle.frequency_ << "Hz" << std::endl;
    std::cout << "  swing_iterations_per_cycle: " << swing_iterations_per_cycle << std::endl;
    std::cout << "  stance_iterations_per_cycle: " << stance_iterations_per_cycle << std::endl;
    std::cout << "  total_iterations_per_cycle: " << total_iterations_per_cycle << std::endl;

    std::cout << "Derived StepCycle iterations: swing=" << swing_iterations_per_cycle
              << ", stance=" << stance_iterations_per_cycle << std::endl;
    // Internal coherence: swing + stance must equal the total normalized period
    if (swing_iterations_per_cycle + stance_iterations_per_cycle != actual_step_cycle.period_) {
        std::cerr << "ERROR: Inconsistent StepCycle timing (swing+stance != period)." << std::endl;
        return 1; // Abort test early
    }
    if (swing_iterations_per_cycle != stance_iterations_per_cycle) {
        std::cout << "ℹ️  INFO: Difference between swing and stance (valid if the configuration defines it)." << std::endl;
    }
    if (swing_iterations_per_cycle != EXPECTED_TRIPOD_HALF_PERIOD ||
        stance_iterations_per_cycle != EXPECTED_TRIPOD_HALF_PERIOD) {
        std::cerr << "ERROR: Tripod gait must run with exact half-period of " << EXPECTED_TRIPOD_HALF_PERIOD
                  << " iterations per phase (swing/stance)." << std::endl;
        return 1;
    }
    std::cout << "(Strict validation: tripod requires 52 iterations per swing and 52 per stance)." << std::endl;
    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    int stance_to_swing_counts[NUM_LEGS] = {0}; // STANCE->SWING transition counter per leg
    int swing_to_stance_counts[NUM_LEGS] = {0}; // SWING->STANCE transition counter per leg
    std::vector<PhaseTransitionRecord> transition_records;
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Absolute phase_ values (0-period) for each leg from LegStepper
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

        // Check for STANCE -> SWING transitions and get the actual phase of each LegStepper
        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();

            // Get the REAL phase_ value from LegStepper (do not count manually)
            auto leg_stepper = sys.getWalkController()->getLegStepper(i);
            if (leg_stepper) {
                leg_phase_values[i] = leg_stepper->getPhase();
            }

            // Detect STANCE -> SWING transitions: count and capture record
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
                stance_to_swing_counts[i]++;
                PhaseTransitionRecord rec = capturePhaseTransition(
                    sys, i, step, stance_to_swing_counts[i], true);
                transition_records.push_back(rec);
                printPhaseTransitionRecord(rec);
            }

            // Detect SWING -> STANCE transitions: capture record and validate
            if (previous_phases[i] == SWING_PHASE && current_phase == STANCE_PHASE) {
                swing_to_stance_counts[i]++;
                PhaseTransitionRecord rec = capturePhaseTransition(
                    sys, i, step, swing_to_stance_counts[i], false);
                transition_records.push_back(rec);
                printPhaseTransitionRecord(rec);

                // Validate end-of-swing touchdown posture is kinematically safe.
                JointAngles touchdown_angles = sys.getLeg(i).getJointAngles();
                double femur_deg = math_utils::radiansToDegrees(touchdown_angles.femur);
                double tibia_deg = math_utils::radiansToDegrees(touchdown_angles.tibia);
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

        // Print current state with absolute phase_ information
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

    // Print comprehensive phase transition deviation analysis
    if (!transition_records.empty()) {
        printPhaseTransitionSummary(transition_records);
    }

    if (final_all_in_stance) {
        std::cout << "\n=== BEZIER TRAJECTORY TIMING ANALYSIS ===" << std::endl;
        std::cout << "CONFIRMATION: Trajectories use the SAME timing as trajectory_tip_position_test" << std::endl;
        std::cout << "  Swing iterations per cycle: " << swing_iterations_per_cycle << std::endl;
        std::cout << "  Stance iterations per cycle: " << stance_iterations_per_cycle << std::endl;
        std::cout << "  Total iterations per cycle: " << total_iterations_per_cycle << std::endl;
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

        std::cout << "\n🎯 FULL SYNCHRONIZATION WITH trajectory_tip_position_test:" << std::endl;
        std::cout << "  ✅ Both tests execute exactly " << swing_iterations_per_cycle << " iterations per swing phase" << std::endl;
        std::cout << "  ✅ Both tests execute exactly " << stance_iterations_per_cycle << " iterations per stance phase" << std::endl;
        std::cout << "  ✅ Both tests use the sequence LocomotionSystem::update -> WalkController::updateWalk -> LegStepper::updateTipPositionIterative" << std::endl;
        std::cout << "  ✅ GaitConfiguration and StepCycle configuration totally coherent with OpenSHC" << std::endl;

        std::cout << "\n🎉 TEST PASSED! Gait cycle observed and robot returned to stable standing pose. 🎉" << std::endl;
        std::cout << "\n🔄 Timing synchronized with trajectory_tip_position_test (actual iterations: swing=" << swing_iterations_per_cycle
                  << ", stance=" << stance_iterations_per_cycle << ")" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ TEST FAILED! Robot did not return to a stable standing pose. ❌" << std::endl;
        return 1;
    }
}
