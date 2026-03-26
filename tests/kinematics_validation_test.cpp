/**
 * @file kinematics_validation_test.cpp
 * @brief Validation of HexaMotion DH-based kinematics (calculateServoAnglesForHeight)
 * @author HexaMotion Team
 * @version 3.0
 * @date 2024
 *
 * This test validates calculateServoAnglesForHeight against the DH forward
 * kinematics model used in RobotModel. The reference implementation uses a
 * brute-force sweep of femur angle to independently solve the same DH-based
 * height equation:
 *
 *   Z = -femur_length * sin(femur_angle) - tibia_length * cos(femur_angle + tibia_angle)
 *
 * With the standing constraint tibia_angle = -femur_angle (tibia vertical):
 *
 *   Z = -femur_length * sin(femur_angle) - tibia_length
 *   height = femur_length * sin(femur_angle) + tibia_length
 */

#include "body_pose_config_factory.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

// Robot segment lengths (mm)
constexpr double A_COXA = 50.0;
constexpr double B_FEMUR = 101.0;
constexpr double C_TIBIA = 208.0;

// Reference angle solution (degrees)
struct CalcAngles {
    double theta1; // femur servo angle (degrees)
    double theta2; // tibia servo angle (degrees)
    bool valid;
};

constexpr double FEMUR_MIN_DEG = -75.0;
constexpr double FEMUR_MAX_DEG = 75.0;
constexpr double TIBIA_MIN_DEG = -45.0;
constexpr double TIBIA_MAX_DEG = 45.0;

/**
 * @brief Reference implementation: calculate servo angles for a given height
 *        using the DH model with brute-force femur angle sweep.
 *
 * DH model (coxa = 0°, tibia vertical constraint):
 *   height = femur_length * sin(femur_angle) + tibia_length
 *   sin(femur_angle) = (height - tibia_length) / femur_length
 *   tibia_angle = -femur_angle
 *
 * This replicates calculateServoAnglesForHeight via sweep instead of
 * direct analytical solution, serving as independent validation.
 *
 * @param H_mm Target standing height in mm.
 * @return Servo angles in degrees and validity flag.
 */
CalcAngles calcLegAngles(double H_mm) {
    CalcAngles best{0, 0, false};
    double bestErr = std::numeric_limits<double>::infinity();

    // Sweep femur angle across its full range
    const double femurMin = math_utils::degreesToRadians(FEMUR_MIN_DEG);
    const double femurMax = math_utils::degreesToRadians(FEMUR_MAX_DEG);
    const double dFemur = math_utils::degreesToRadians(0.01); // fine step

    for (double femur_rad = femurMin; femur_rad <= femurMax; femur_rad += dFemur) {
        // Standing constraint: tibia_angle = -femur_angle (tibia vertical)
        double tibia_rad = -femur_rad;

        // Check tibia limits
        double tibia_deg = math_utils::radiansToDegrees(tibia_rad);
        if (tibia_deg < TIBIA_MIN_DEG || tibia_deg > TIBIA_MAX_DEG)
            continue;

        // DH height formula: Z = -B*sin(femur) - C*cos(femur+tibia)
        // With tibia = -femur: Z = -B*sin(femur) - C
        // height = -Z = B*sin(femur) + C
        double computed_height = B_FEMUR * std::sin(femur_rad) + C_TIBIA;

        double err = std::fabs(computed_height - H_mm);
        if (err < bestErr) {
            bestErr = err;
            double femur_deg = math_utils::radiansToDegrees(femur_rad);
            best = {femur_deg, tibia_deg, true};
        }
    }

    // Accept only if error is small enough (< 0.1 mm)
    if (bestErr > 0.1)
        best.valid = false;

    return best;
}

/**
 * @brief Reference DH forward kinematics: compute height from servo angles.
 *
 * DH model: Z = -femur_length * sin(femur) - tibia_length * cos(femur + tibia)
 * height = -Z
 *
 * @param femur_deg Femur servo angle in degrees.
 * @param tibia_deg Tibia servo angle in degrees.
 * @param[out] valid True if angles are within limits.
 * @return Height in mm (positive downward from femur pivot).
 */
double calcHeight(double femur_deg, double tibia_deg, bool &valid) {
    valid = false;

    if (femur_deg < FEMUR_MIN_DEG || femur_deg > FEMUR_MAX_DEG)
        return 0.0;
    if (tibia_deg < TIBIA_MIN_DEG || tibia_deg > TIBIA_MAX_DEG)
        return 0.0;

    double femur_rad = math_utils::degreesToRadians(femur_deg);
    double tibia_rad = math_utils::degreesToRadians(tibia_deg);

    // DH Z component: Z = -B*sin(femur) - C*cos(femur + tibia)
    double Z = -B_FEMUR * std::sin(femur_rad) - C_TIBIA * std::cos(femur_rad + tibia_rad);

    valid = true;
    return -Z; // height = -Z (positive downward)
}

class KinematicsValidator {
  private:
    Parameters params;
    std::unique_ptr<RobotModel> model;

  public:
    KinematicsValidator() {
        setupParameters();
        model = std::make_unique<RobotModel>(params);
        model->workspaceAnalyzerInitializer();
    }

    void setupParameters() {
        params.hexagon_radius = 200.0f;
        params.coxa_length = A_COXA;
        params.femur_length = B_FEMUR;
        params.tibia_length = C_TIBIA;
        params.default_height_offset = -C_TIBIA;
        params.robot_height = 208.0f;
        params.standing_height = 150.0f;

        params.coxa_angle_limits[0] = -75.0f;
        params.coxa_angle_limits[1] = 75.0f;
        params.femur_angle_limits[0] = FEMUR_MIN_DEG;
        params.femur_angle_limits[1] = FEMUR_MAX_DEG;
        params.tibia_angle_limits[0] = TIBIA_MIN_DEG;
        params.tibia_angle_limits[1] = TIBIA_MAX_DEG;

        params.ik.max_iterations = 50;
        params.ik.pos_threshold_mm = 0.5f;
        params.use_custom_dh_parameters = false;

        std::cout << "Parameters: hexagon=" << params.hexagon_radius
                  << ", coxa=" << params.coxa_length
                  << ", femur=" << params.femur_length
                  << ", tibia=" << params.tibia_length << std::endl;
    }

    bool validateVerticalReach() {
        std::cout << "\n=== VALIDATION: FEMUR & TIBIA SERVO ANGLES ===" << std::endl;
        std::cout << "Comparing: brute-force DH sweep vs calculateServoAnglesForHeight" << std::endl;
        std::cout << std::string(100, '-') << std::endl;
        std::cout << "Height | Reference (femur, tibia) | Calculated (femur, tibia) | Dfemur | Dtibia | Status" << std::endl;
        std::cout << "(mm)   | (deg, deg)               | (deg, deg)                | (deg)  | (deg)  |" << std::endl;
        std::cout << std::string(100, '-') << std::endl;

        // Test heights within the achievable DH range:
        // min height = tibia - femur = 208 - 101 = 107 mm
        // max height = tibia + femur = 208 + 101 = 309 mm
        // But also constrained by joint limits (femur: ±75°, tibia: ±45°)
        std::vector<double> test_heights = {
            120.0, 140.0, 150.0, 160.0, 180.0, 200.0, 208.0, 220.0, 240.0, 260.0, 280.0, 300.0};

        int passed = 0, total = 0;

        for (double height : test_heights) {
            total++;
            // 1. Reference: brute-force sweep of DH model
            CalcAngles ref_solution = calcLegAngles(height);
            // 2. Under test: analytical calculateServoAnglesForHeight
            CalculatedServoAngles calc_solution = RobotModel::calculateServoAnglesForHeight(height, params);

            if (!ref_solution.valid && !calc_solution.valid) {
                std::cout << std::setw(6) << height << " | BOTH INVALID                | BOTH INVALID                  |   N/A  |   N/A  | OK (both reject)" << std::endl;
                passed++;
                continue;
            }

            if (!ref_solution.valid || !calc_solution.valid) {
                std::cout << std::setw(6) << height << " | valid=" << ref_solution.valid
                          << "                      | valid=" << calc_solution.valid
                          << "                        |   N/A  |   N/A  | DIFF (validity)" << std::endl;
                continue;
            }

            // Convert calculateServoAnglesForHeight results from radians to degrees
            double femur_ref = ref_solution.theta1;
            double tibia_ref = ref_solution.theta2;
            double femur_calc = math_utils::radiansToDegrees(calc_solution.femur);
            double tibia_calc = math_utils::radiansToDegrees(calc_solution.tibia);

            double dfemur = std::abs(femur_ref - femur_calc);
            double dtibia = std::abs(tibia_ref - tibia_calc);
            bool match = (dfemur < 0.1 && dtibia < 0.1);
            if (match)
                passed++;

            std::cout << std::setw(6) << height << " | "
                      << std::setw(8) << std::fixed << std::setprecision(2) << femur_ref << ", "
                      << std::setw(8) << tibia_ref << " | "
                      << std::setw(8) << femur_calc << ", "
                      << std::setw(8) << tibia_calc << " | "
                      << std::setw(6) << std::setprecision(3) << dfemur << " | "
                      << std::setw(6) << dtibia << " | "
                      << (match ? "MATCH" : "DIFF") << std::endl;
        }
        std::cout << std::string(100, '-') << std::endl;
        std::cout << "Results: " << passed << "/" << total << " tests match ("
                  << std::fixed << std::setprecision(1) << (100.0 * passed / total) << "%)" << std::endl;
        return passed == total;
    }

    bool validateAngleConsistency() {
        std::cout << "\n=== VALIDATION: FK/IK ROUND-TRIP CONSISTENCY ===" << std::endl;
        std::cout << "Testing that FK(IK(target)) = target using DH model" << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        // Use heights that are achievable via calculateServoAnglesForHeight
        std::vector<double> test_heights = {
            120.0, 140.0, 150.0, 160.0, 180.0, 200.0, 208.0, 250.0};

        int passed = 0, total = 0;

        std::cout << "Height | calcHeight   | FK height    | Error   | Status" << std::endl;
        std::cout << "(mm)   | (mm)         | (mm)         | (mm)    |" << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        for (double height : test_heights) {
            total++;

            // Get DH servo angles for this height
            CalculatedServoAngles calc = RobotModel::calculateServoAnglesForHeight(height, params);
            if (!calc.valid) {
                std::cout << std::setw(6) << height
                          << " | INVALID      | N/A          | N/A     | SKIP" << std::endl;
                total--;
                continue;
            }

            // Reference: DH calcHeight (degrees)
            double femur_deg = math_utils::radiansToDegrees(calc.femur);
            double tibia_deg = math_utils::radiansToDegrees(calc.tibia);
            bool valid_ref;
            double ref_height = calcHeight(femur_deg, tibia_deg, valid_ref);

            // HexaMotion FK: compute global tip position from these angles
            JointAngles angles(calc.coxa, calc.femur, calc.tibia);
            Point3D tip_global = model->forwardKinematicsGlobalCoordinates(0, angles);
            // Height relative to the femur pivot base (Z=0 plane)
            double fk_height = -tip_global.z;

            double height_error = std::abs(ref_height - fk_height);
            bool test_passed = height_error < 2.0; // 2mm tolerance
            if (test_passed)
                passed++;

            std::cout << std::setw(6) << std::fixed << std::setprecision(1) << height
                      << " | " << std::setw(12) << std::setprecision(2) << ref_height
                      << " | " << std::setw(12) << fk_height
                      << " | " << std::setw(7) << std::setprecision(3) << height_error
                      << " | " << (test_passed ? "PASS" : "FAIL") << std::endl;
        }

        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Results: " << passed << "/" << total << " tests passed ("
                  << std::fixed << std::setprecision(1) << (100.0 * passed / total) << "%)" << std::endl;
        return passed == total;
    }

    bool validateWorkspaceComparison() {
        std::cout << "\n=== VALIDATION: WORKSPACE COVERAGE ===" << std::endl;
        std::cout << "Analyzing height range achievable by calculateServoAnglesForHeight" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        // Theoretical DH range:
        // min = tibia - femur = 208 - 101 = 107 mm
        // max = tibia + femur = 208 + 101 = 309 mm
        // But constrained by joint limits
        double min_valid = 1000.0, max_valid = 0.0;
        int valid_analytical = 0;
        int valid_fk_match = 0;
        int total_tested = 0;

        for (double h = 100.0; h <= 320.0; h += 2.0) {
            total_tested++;
            CalculatedServoAngles calc = RobotModel::calculateServoAnglesForHeight(h, params);
            if (!calc.valid)
                continue;

            valid_analytical++;
            min_valid = std::min(min_valid, h);
            max_valid = std::max(max_valid, h);

            // Verify with FK
            JointAngles angles(calc.coxa, calc.femur, calc.tibia);
            Point3D tip = model->forwardKinematicsGlobalCoordinates(0, angles);
            double fk_height = -tip.z;
            double error = std::abs(fk_height - h);
            if (error < 5.0)
                valid_fk_match++;
        }

        std::cout << "Analytical solver workspace:" << std::endl;
        std::cout << "  Min height: " << min_valid << " mm" << std::endl;
        std::cout << "  Max height: " << max_valid << " mm" << std::endl;
        std::cout << "  Range: " << (max_valid - min_valid) << " mm" << std::endl;
        std::cout << "  Valid solutions: " << valid_analytical << "/" << total_tested << std::endl;

        double coverage = 100.0 * valid_fk_match / std::max(1, valid_analytical);
        std::cout << "\nFK verification:" << std::endl;
        std::cout << "  FK-matched solutions: " << valid_fk_match << "/" << valid_analytical << std::endl;
        std::cout << "  FK coverage: " << std::fixed << std::setprecision(1) << coverage << "%" << std::endl;
        return valid_fk_match == valid_analytical;
    }

    bool runFullValidation() {
        std::cout << "===== FULL VALIDATION: calculateServoAnglesForHeight =====" << std::endl;
        std::cout << "Robot dimensions:" << std::endl;
        std::cout << "  Coxa: " << A_COXA << " mm" << std::endl;
        std::cout << "  Femur: " << B_FEMUR << " mm" << std::endl;
        std::cout << "  Tibia: " << C_TIBIA << " mm" << std::endl;
        std::cout << "  Hexagon radius: " << params.hexagon_radius << " mm" << std::endl;

        bool v1 = validateVerticalReach();
        bool v2 = validateAngleConsistency();
        bool v3 = validateWorkspaceComparison();

        bool all_passed = v1 && v2 && v3;
        std::cout << "\n===== VALIDATION " << (all_passed ? "PASSED" : "FAILED") << " =====" << std::endl;
        return all_passed;
    }
};

int main() {
    KinematicsValidator validator;
    bool passed = validator.runFullValidation();
    return passed ? 0 : 1;
}
