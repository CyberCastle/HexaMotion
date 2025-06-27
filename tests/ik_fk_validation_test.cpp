/**
 * @file hexamotion_ik_fk_validation_test.cpp
 * @brief Comprehensive test for validating Inverse Kinematics (IK) and Forward Kinematics (FK) in HexaMotion
 *
 * This test validates:
 * 1. IK-FK consistency (Forward then Inverse should return original position)
 * 2. FK-IK consistency (Inverse then Forward should return original angles)
 * 3. Joint limit validation
 * 4. Workspace boundary testing
 * 5. Numerical precision testing
 * 6. All 6 legs independently
 * 7. Error tolerance analysis
 * 8. Performance benchmarking
 */

#include "HexaModel.h"
#include "math_utils.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

class IKFKValidator {
  private:
    RobotModel model;
    float position_tolerance_mm;
    float angle_tolerance_degrees;
    int total_tests;
    int passed_tests;
    int failed_tests;

    struct TestResults {
        float max_position_error;
        float max_angle_error;
        float avg_position_error;
        float avg_angle_error;
        long total_computation_time_us;
        int test_count;
    };

  public:
    IKFKValidator(const Parameters &params,
                  float pos_tol = 1.0f,
                  float angle_tol = 1.0f)
        : model(params),
          position_tolerance_mm(pos_tol),
          angle_tolerance_degrees(angle_tol),
          total_tests(0),
          passed_tests(0),
          failed_tests(0) {}

    void printHeader() {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "    HEXAMOTION INVERSE & FORWARD KINEMATICS VALIDATION TEST" << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "Position Tolerance: " << position_tolerance_mm << " mm" << std::endl;
        std::cout << "Angle Tolerance: " << angle_tolerance_degrees << " degrees" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
    }

    bool testIKFKConsistency(int leg, const Point3D &target_pos, const std::string &test_name) {
        total_tests++;

        auto start_time = std::chrono::high_resolution_clock::now();

        // Step 1: Compute IK for target position
        JointAngles computed_angles = model.inverseKinematics(leg, target_pos);

        // Step 2: Compute FK with computed angles
        Point3D computed_pos = model.forwardKinematics(leg, computed_angles);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // Step 3: Calculate position error
        float position_error = sqrt(pow(target_pos.x - computed_pos.x, 2) +
                                    pow(target_pos.y - computed_pos.y, 2) +
                                    pow(target_pos.z - computed_pos.z, 2));

        // Step 4: Check joint limits
        bool within_limits = model.checkJointLimits(leg, computed_angles);

        bool test_passed = (position_error <= position_tolerance_mm) && within_limits;

        if (test_passed) {
            passed_tests++;
            std::cout << "[PASS] ";
        } else {
            failed_tests++;
            std::cout << "[FAIL] ";
        }

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Leg " << leg << " " << test_name
                  << " | Error: " << position_error << "mm"
                  << " | Limits: " << (within_limits ? "OK" : "EXCEEDED")
                  << " | Time: " << duration.count() << "μs" << std::endl;

        if (!test_passed) {
            std::cout << "    Target:   (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;
            std::cout << "    Computed: (" << computed_pos.x << ", " << computed_pos.y << ", " << computed_pos.z << ")" << std::endl;
            std::cout << "    Angles:   (" << computed_angles.coxa << "°, " << computed_angles.femur << "°, " << computed_angles.tibia << "°)" << std::endl;
        }

        return test_passed;
    }

    bool testFKIKConsistency(int leg, const JointAngles &target_angles, const std::string &test_name) {
        total_tests++;

        auto start_time = std::chrono::high_resolution_clock::now();

        // Step 1: Compute FK for target angles
        Point3D computed_pos = model.forwardKinematics(leg, target_angles);

        // Step 2: Compute IK with computed position
        JointAngles computed_angles = model.inverseKinematics(leg, computed_pos);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // Step 3: Calculate angle errors (considering angle wrapping)
        float coxa_error = fabs(math_utils::normalizeAngle(target_angles.coxa - computed_angles.coxa));
        float femur_error = fabs(math_utils::normalizeAngle(target_angles.femur - computed_angles.femur));
        float tibia_error = fabs(math_utils::normalizeAngle(target_angles.tibia - computed_angles.tibia));
        float max_angle_error = std::max({coxa_error, femur_error, tibia_error});

        // Step 4: Check if original angles were within limits
        bool original_within_limits = model.checkJointLimits(leg, target_angles);
        bool computed_within_limits = model.checkJointLimits(leg, computed_angles);

        bool test_passed = (max_angle_error <= angle_tolerance_degrees) &&
                           original_within_limits && computed_within_limits;

        if (test_passed) {
            passed_tests++;
            std::cout << "[PASS] ";
        } else {
            failed_tests++;
            std::cout << "[FAIL] ";
        }

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Leg " << leg << " " << test_name
                  << " | Max Angle Error: " << max_angle_error << "°"
                  << " | Limits: " << (original_within_limits && computed_within_limits ? "OK" : "EXCEEDED")
                  << " | Time: " << duration.count() << "μs" << std::endl;

        if (!test_passed) {
            std::cout << "    Target:   (" << target_angles.coxa << "°, " << target_angles.femur << "°, " << target_angles.tibia << "°)" << std::endl;
            std::cout << "    Computed: (" << computed_angles.coxa << "°, " << computed_angles.femur << "°, " << computed_angles.tibia << "°)" << std::endl;
            std::cout << "    Position: (" << computed_pos.x << ", " << computed_pos.y << ", " << computed_pos.z << ")" << std::endl;
        }

        return test_passed;
    }

    void runComprehensiveTests() {
        printHeader();

        // Test 1: Standard stance positions for all legs
        std::cout << "\n1. TESTING STANDARD STANCE POSITIONS:" << std::endl;
        std::vector<Point3D> stance_positions = {
            Point3D(250, 0, -150),     // Leg 0 (front right)
            Point3D(125, 217, -150),   // Leg 1 (middle right)
            Point3D(-125, 217, -150),  // Leg 2 (rear right)
            Point3D(-250, 0, -150),    // Leg 3 (rear left)
            Point3D(-125, -217, -150), // Leg 4 (middle left)
            Point3D(125, -217, -150)   // Leg 5 (front left)
        };

        for (int leg = 0; leg < NUM_LEGS; leg++) {
            testIKFKConsistency(leg, stance_positions[leg], "Stance");
        }

        // Test 2: Realistic joint configurations for all legs
        std::cout << "\n2. TESTING REALISTIC JOINT CONFIGURATIONS:" << std::endl;
        std::vector<JointAngles> realistic_configs = {
            JointAngles(0.0f, -30.0f, 60.0f),   // Mild bend
            JointAngles(0.0f, -45.0f, 90.0f),   // Standard bend
            JointAngles(0.0f, -60.0f, 120.0f),  // Deep bend
            JointAngles(30.0f, -30.0f, 60.0f),  // Side + mild bend
            JointAngles(45.0f, -45.0f, 90.0f),  // Side + standard bend
            JointAngles(-30.0f, -30.0f, 60.0f), // Other side + mild bend
            JointAngles(-45.0f, -45.0f, 90.0f), // Other side + standard bend
            JointAngles(0.0f, -15.0f, 30.0f),   // Extended pose
            JointAngles(0.0f, -75.0f, 150.0f),  // High pose
        };

        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (size_t i = 0; i < realistic_configs.size(); i++) {
                testFKIKConsistency(leg, realistic_configs[i], "Config" + std::to_string(i + 1));
            }
        }

        // Test 3: Workspace boundary testing
        std::cout << "\n3. TESTING WORKSPACE BOUNDARIES:" << std::endl;
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Test near minimum reach
            testIKFKConsistency(leg, Point3D(50, 0, -50), "MinReach");

            // Test near maximum reach
            testIKFKConsistency(leg, Point3D(300, 0, -200), "MaxReach");

            // Test various heights
            testIKFKConsistency(leg, Point3D(200, 0, -50), "HighPos");
            testIKFKConsistency(leg, Point3D(200, 0, -250), "LowPos");

            // Test side positions
            testIKFKConsistency(leg, Point3D(150, 150, -150), "SidePos");
            testIKFKConsistency(leg, Point3D(150, -150, -150), "SideNeg");
        }

        // Test 4: Joint limit boundary testing
        std::cout << "\n4. TESTING JOINT LIMIT BOUNDARIES:" << std::endl;
        Parameters params = model.getParams();
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Test near joint limits
            testFKIKConsistency(leg, JointAngles(params.coxa_angle_limits[0] + 5, -45, 90), "CoxaMin");
            testFKIKConsistency(leg, JointAngles(params.coxa_angle_limits[1] - 5, -45, 90), "CoxaMax");
            testFKIKConsistency(leg, JointAngles(0, params.femur_angle_limits[0] + 5, 90), "FemurMin");
            testFKIKConsistency(leg, JointAngles(0, params.femur_angle_limits[1] - 5, 90), "FemurMax");
            testFKIKConsistency(leg, JointAngles(0, -45, params.tibia_angle_limits[0] + 5), "TibiaMin");
            testFKIKConsistency(leg, JointAngles(0, -45, params.tibia_angle_limits[1] - 5), "TibiaMax");
        }

        // Test 5: High precision testing (smaller tolerance)
        std::cout << "\n5. TESTING HIGH PRECISION (0.1mm, 0.1° tolerance):" << std::endl;
        float original_pos_tol = position_tolerance_mm;
        float original_angle_tol = angle_tolerance_degrees;
        position_tolerance_mm = 0.1f;
        angle_tolerance_degrees = 0.1f;

        for (int leg = 0; leg < NUM_LEGS; leg++) {
            testIKFKConsistency(leg, Point3D(200, 0, -150), "HighPrec");
            testFKIKConsistency(leg, JointAngles(0, -45, 90), "HighPrec");
        }

        // Restore original tolerances
        position_tolerance_mm = original_pos_tol;
        angle_tolerance_degrees = original_angle_tol;

        // Test 6: Performance benchmark
        std::cout << "\n6. PERFORMANCE BENCHMARK:" << std::endl;
        auto benchmark_start = std::chrono::high_resolution_clock::now();
        int benchmark_iterations = 1000;

        for (int i = 0; i < benchmark_iterations; i++) {
            int leg = i % NUM_LEGS;
            Point3D test_pos(200 + (i % 100), 0, -150 - (i % 50));
            JointAngles angles = model.inverseKinematics(leg, test_pos);
            Point3D computed_pos = model.forwardKinematics(leg, angles);
        }

        auto benchmark_end = std::chrono::high_resolution_clock::now();
        auto benchmark_duration = std::chrono::duration_cast<std::chrono::microseconds>(benchmark_end - benchmark_start);

        std::cout << "Completed " << benchmark_iterations << " IK+FK cycles in "
                  << benchmark_duration.count() << "μs" << std::endl;
        std::cout << "Average time per IK+FK cycle: "
                  << (float)benchmark_duration.count() / benchmark_iterations << "μs" << std::endl;
    }

    void printSummary() {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "                           TEST SUMMARY" << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "Total Tests:  " << total_tests << std::endl;
        std::cout << "Passed:       " << passed_tests << " (" << std::fixed << std::setprecision(1)
                  << (100.0f * passed_tests / total_tests) << "%)" << std::endl;
        std::cout << "Failed:       " << failed_tests << " (" << std::fixed << std::setprecision(1)
                  << (100.0f * failed_tests / total_tests) << "%)" << std::endl;

        if (failed_tests == 0) {
            std::cout << "\n🎉 ALL TESTS PASSED! IK/FK implementation is validated." << std::endl;
        } else {
            std::cout << "\n⚠️  Some tests failed. Review the implementation or adjust tolerances." << std::endl;
        }
        std::cout << std::string(80, '=') << std::endl;
    }

    bool allTestsPassed() {
        return failed_tests == 0;
    }
};

int main() {
    // Configure robot parameters for HexaMotion
    Parameters params;
    params.hexagon_radius = 200.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 100.0f;
    params.tibia_length = 159.0f;
    params.robot_height = 150.0f;
    params.robot_weight = 5.0f;

    // Set realistic joint limits
    params.coxa_angle_limits[0] = -180.0f;
    params.coxa_angle_limits[1] = 180.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -180.0f;
    params.tibia_angle_limits[1] = 180.0f;

    // Initialize other parameters
    params.center_of_mass = Eigen::Vector3f(0, 0, 0);
    params.imu_calibration_offset = Eigen::Vector3f(0, 0, 0);
    params.fsr_touchdown_threshold = 50.0f;
    params.fsr_liftoff_threshold = 20.0f;
    params.fsr_max_pressure = 1000.0f;
    params.max_velocity = 100.0f;
    params.max_angular_velocity = 45.0f;
    params.stability_margin = 50.0f;
    params.control_frequency = 50.0f;

    // Initialize DH parameters
    params.use_custom_dh_parameters = false;
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            for (int k = 0; k < 4; ++k) {
                params.dh_parameters[l][j][k] = 0.0f;
            }
        }
    }

    // Create validator with reasonable tolerances
    IKFKValidator validator(params, 1.0f, 1.0f); // 1mm position, 1° angle tolerance

    // Run comprehensive validation tests
    validator.runComprehensiveTests();

    // Print summary
    validator.printSummary();

    // Return appropriate exit code
    return validator.allTestsPassed() ? 0 : 1;
}
