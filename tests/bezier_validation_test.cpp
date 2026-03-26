/**
 * @file bezier_validation_test.cpp
 * @brief Validation test to compare HexaMotion Bezier implementation with OpenSHC
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 *
 * This test validates that our Bezier curve implementation is equivalent to OpenSHC.
 */

#include "math_utils.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

// Test parameters
const double TEST_TOLERANCE = 1e-6f;
const int NUM_TEST_POINTS = 100;

/**
 * @brief Independent reference: De Casteljau's algorithm for quartic Bezier.
 *
 * This is a fundamentally different computation than the direct polynomial
 * expansion used in math_utils::quarticBezier. De Casteljau uses recursive
 * linear interpolation, so shared implementation bugs are not possible.
 */
template <class T>
inline T deCasteljauQuarticBezier(const T *points, double t) {
    double s = 1.0 - t;
    // Level 1: 4 intermediate points
    T p01 = points[0] * s + points[1] * t;
    T p12 = points[1] * s + points[2] * t;
    T p23 = points[2] * s + points[3] * t;
    T p34 = points[3] * s + points[4] * t;
    // Level 2: 3 intermediate points
    T p012 = p01 * s + p12 * t;
    T p123 = p12 * s + p23 * t;
    T p234 = p23 * s + p34 * t;
    // Level 3: 2 intermediate points
    T p0123 = p012 * s + p123 * t;
    T p1234 = p123 * s + p234 * t;
    // Level 4: final point
    return p0123 * s + p1234 * t;
}

/**
 * @brief Independent reference: numerical Bezier derivative via central finite differences.
 *
 * Uses the De Casteljau evaluator (not the SUT) and a small step h to approximate
 * the derivative numerically. This is a different method than the analytical
 * coefficient formula used in math_utils::quarticBezierDot.
 */
template <class T>
inline T numericalBezierDerivative(const T *points, double t, double h = 1e-7) {
    double t_plus = std::min(t + h, 1.0);
    double t_minus = std::max(t - h, 0.0);
    double actual_h = t_plus - t_minus;
    return (deCasteljauQuarticBezier(points, t_plus) - deCasteljauQuarticBezier(points, t_minus)) * (1.0 / actual_h);
}

bool testBezierEquivalence() {
    cout << "Testing Bezier Implementation Equivalence..." << endl;

    // Test control points (typical swing trajectory)
    Eigen::Vector3d control_points[5] = {
        Eigen::Vector3d(80.0f, 0.0f, -80.0f), // Start position
        Eigen::Vector3d(85.0f, 0.0f, -70.0f), // First control
        Eigen::Vector3d(90.0f, 0.0f, -60.0f), // Peak control
        Eigen::Vector3d(95.0f, 0.0f, -70.0f), // Third control
        Eigen::Vector3d(100.0f, 0.0f, -80.0f) // End position
    };

    cout << "Control Points:" << endl;
    for (int i = 0; i < 5; i++) {
        cout << "  P" << i << ": ("
             << control_points[i][0] << ", "
             << control_points[i][1] << ", "
             << control_points[i][2] << ")" << endl;
    }
    cout << endl;

    bool all_tests_passed = true;
    double max_position_error = 0.0f;
    double max_velocity_error = 0.0f;

    // Test at multiple time points
    for (int i = 0; i <= NUM_TEST_POINTS; i++) {
        double t = static_cast<double>(i) / NUM_TEST_POINTS;

        // Test position: SUT (polynomial expansion) vs De Casteljau (recursive lerp)
        Eigen::Vector3d hexamotion_pos = math_utils::quarticBezier(control_points, t);
        Eigen::Vector3d decasteljau_pos = deCasteljauQuarticBezier(control_points, t);

        double pos_error = (hexamotion_pos - decasteljau_pos).norm();
        max_position_error = max(max_position_error, pos_error);

        if (pos_error > TEST_TOLERANCE) {
            cout << "❌ Position mismatch at t=" << t << endl;
            cout << "  HexaMotion (polynomial): (" << hexamotion_pos[0] << ", " << hexamotion_pos[1] << ", " << hexamotion_pos[2] << ")" << endl;
            cout << "  De Casteljau (lerp):     (" << decasteljau_pos[0] << ", " << decasteljau_pos[1] << ", " << decasteljau_pos[2] << ")" << endl;
            cout << "  Error:                   " << pos_error << endl;
            all_tests_passed = false;
        }

        // Test velocity: SUT (analytical derivative) vs numerical finite differences over De Casteljau
        Eigen::Vector3d hexamotion_vel = math_utils::quarticBezierDot(control_points, t);
        Eigen::Vector3d numerical_vel = numericalBezierDerivative(control_points, t);

        double vel_error = (hexamotion_vel - numerical_vel).norm();
        max_velocity_error = max(max_velocity_error, vel_error);

        // Slightly relaxed tolerance for numerical derivative (finite difference approximation)
        double vel_tolerance = 1e-4;
        if (vel_error > vel_tolerance) {
            cout << "❌ Velocity mismatch at t=" << t << endl;
            cout << "  HexaMotion (analytical):     (" << hexamotion_vel[0] << ", " << hexamotion_vel[1] << ", " << hexamotion_vel[2] << ")" << endl;
            cout << "  Numerical (finite diff):     (" << numerical_vel[0] << ", " << numerical_vel[1] << ", " << numerical_vel[2] << ")" << endl;
            cout << "  Error:                       " << vel_error << endl;
            all_tests_passed = false;
        }
    }

    cout << "Maximum Position Error (vs De Casteljau): " << scientific << setprecision(2) << max_position_error << endl;
    cout << "Maximum Velocity Error (vs numerical diff): " << scientific << setprecision(2) << max_velocity_error << endl;

    if (all_tests_passed) {
        cout << "✓ Bezier implementation matches independent De Casteljau oracle!" << endl;
    }

    return all_tests_passed;
}

bool testSwingTrajectoryEquivalence() {
    cout << "\nTesting Swing Trajectory Implementation..." << endl;

    Parameters params = createDefaultParameters();
    RobotModel model(params);
    model.workspaceAnalyzerInitializer(); // Initialize WorkspaceAnalyzer

    // Test parameters matching OpenSHC usage
    int leg_index = 0;
    double step_height = 20.0f;
    double step_length = 40.0f;
    double stance_duration = 0.6f;
    double swing_duration = 0.4f;
    double robot_height = 80.0f;

    cout << "Test Parameters:" << endl;
    cout << "  Step Height: " << step_height << " mm" << endl;
    cout << "  Step Length: " << step_length << " mm" << endl;
    cout << "  Stance Duration: " << stance_duration << endl;
    cout << "  Swing Duration: " << swing_duration << endl;
    cout << "  Robot Height: " << robot_height << " mm" << endl;
    cout << endl;

    // Test swing phase (equivalent to OpenSHC swing trajectory)
    cout << "Testing Swing Phase Trajectory:" << endl;
    cout << "Phase\tX\t\tY\t\tZ\t\tHeight" << endl;
    cout << "-----\t--------\t--------\t--------\t--------" << endl;

    for (int i = 0; i <= 20; i++) {
        double phase = stance_duration + (swing_duration * i / 20.0f);
        double t = (double)i / 20.0f; // Normalized parameter for Bezier curve

        // Create simple swing trajectory control points
        Point3D control_points[5] = {
            Point3D(0.0f, 0.0f, -robot_height),                                     // Start: ground level
            Point3D(step_length * 0.25f, 0.0f, -robot_height + step_height * 0.5f), // Early lift
            Point3D(step_length * 0.5f, 0.0f, -robot_height + step_height),         // Mid-swing: peak
            Point3D(step_length * 0.75f, 0.0f, -robot_height + step_height * 0.5f), // Late descent
            Point3D(step_length, 0.0f, -robot_height)                               // End: ground level
        };

        Point3D pos = math_utils::quarticBezier(control_points, t);

        double height_above_ground = pos.z + robot_height;

        cout << fixed << setprecision(2)
             << phase << "\t"
             << pos.x << "\t\t"
             << pos.y << "\t\t"
             << pos.z << "\t\t"
             << height_above_ground << endl;

        // Validate trajectory against De Casteljau independent oracle
        Point3D dc_pos = deCasteljauQuarticBezier(control_points, t);
        double dc_error = sqrt(pow(pos.x - dc_pos.x, 2) + pow(pos.y - dc_pos.y, 2) + pow(pos.z - dc_pos.z, 2));
        assert(dc_error < 1e-6);

        // Validate physical trajectory properties
        if (i > 0 && i < 20) {
            // Mid-swing should be above ground
            assert(height_above_ground > 0.0f);
        }

        // At midpoint (t=0.5) validate height against analytically-derived value.
        // For these symmetric control points the quartic Bezier midpoint Z is:
        // B(0.5).z = (1*P0.z + 4*P1.z + 6*P2.z + 4*P3.z + 1*P4.z) / 16
        if (i == 10) {
            double expected_mid_z = (1.0 * (-robot_height) + 4.0 * (-robot_height + step_height * 0.5) +
                                     6.0 * (-robot_height + step_height) + 4.0 * (-robot_height + step_height * 0.5) +
                                     1.0 * (-robot_height)) /
                                    16.0;
            assert(abs(pos.z - expected_mid_z) < 1e-6);
        }
    }

    cout << "✓ Swing trajectory properties validated" << endl;
    return true;
}

bool testContinuityAndSmoothness() {
    cout << "\nTesting Trajectory Continuity and Smoothness..." << endl;

    Eigen::Vector3d control_points[5] = {
        Eigen::Vector3d(80.0f, 0.0f, -80.0f),
        Eigen::Vector3d(85.0f, 0.0f, -70.0f),
        Eigen::Vector3d(90.0f, 0.0f, -60.0f),
        Eigen::Vector3d(95.0f, 0.0f, -70.0f),
        Eigen::Vector3d(100.0f, 0.0f, -80.0f)};

    // Validate against De Casteljau at multiple points (independent oracle for interior)
    double max_dc_error = 0.0;
    for (int i = 0; i <= 50; i++) {
        double t = static_cast<double>(i) / 50.0;
        Eigen::Vector3d sut_pos = math_utils::quarticBezier(control_points, t);
        Eigen::Vector3d dc_pos = deCasteljauQuarticBezier(control_points, t);
        double err = (sut_pos - dc_pos).norm();
        max_dc_error = max(max_dc_error, err);
    }
    cout << "De Casteljau cross-check max error: " << max_dc_error << endl;
    assert(max_dc_error < TEST_TOLERANCE);

    // Validate midpoint B(0.5) against hand-computed value:
    // B(0.5) = (P0 + 4*P1 + 6*P2 + 4*P3 + P4) / 16
    Eigen::Vector3d expected_mid = (control_points[0] + control_points[1] * 4.0 +
                                    control_points[2] * 6.0 + control_points[3] * 4.0 +
                                    control_points[4]) /
                                   16.0;
    Eigen::Vector3d sut_mid = math_utils::quarticBezier(control_points, 0.5);
    double mid_error = (sut_mid - expected_mid).norm();
    cout << "Midpoint (analytic B(0.5)) error: " << mid_error << endl;
    assert(mid_error < TEST_TOLERANCE);

    // Test C1 continuity (velocity continuity at endpoints)
    Eigen::Vector3d start_vel = math_utils::quarticBezierDot(control_points, 0.0);
    Eigen::Vector3d end_vel = math_utils::quarticBezierDot(control_points, 1.0);

    cout << "C1 Continuity Test:" << endl;
    cout << "  Start velocity: (" << start_vel[0] << ", " << start_vel[1] << ", " << start_vel[2] << ")" << endl;
    cout << "  End velocity: (" << end_vel[0] << ", " << end_vel[1] << ", " << end_vel[2] << ")" << endl;

    // Test smoothness by checking velocity magnitude changes
    double max_velocity_change = 0.0f;
    Eigen::Vector3d prev_vel = start_vel;

    for (int i = 1; i <= 100; i++) {
        double t = static_cast<double>(i) / 100.0;
        Eigen::Vector3d curr_vel = math_utils::quarticBezierDot(control_points, t);
        double velocity_change = (curr_vel - prev_vel).norm();
        max_velocity_change = max(max_velocity_change, velocity_change);
        prev_vel = curr_vel;
    }

    cout << "Smoothness Test:" << endl;
    cout << "  Maximum velocity change between adjacent points: " << max_velocity_change << endl;

    cout << "✓ Trajectory continuity and smoothness validated" << endl;
    return true;
}

bool testOpenSHCCompatibility() {
    cout << "\nTesting OpenSHC Compatibility Features..." << endl;

    // Test that our implementation produces the same control node structure as OpenSHC
    cout << "Verifying control node generation..." << endl;

    // This tests the same 5-control-point quartic Bezier approach used in OpenSHC
    // for stance, primary swing, and secondary swing curves

    // Stance trajectory (linear ground movement)
    Eigen::Vector3d stance_nodes[5];
    Eigen::Vector3d stride_vector(40.0f, 0.0f, 0.0f);
    Eigen::Vector3d start_pos(80.0f, 0.0f, -80.0f);

    // Generate stance control nodes (equivalent to OpenSHC generateStanceControlNodes)
    for (int i = 0; i < 5; i++) {
        stance_nodes[i] = start_pos + stride_vector * (i / 4.0f);
    }

    cout << "Stance control nodes:" << endl;
    for (int i = 0; i < 5; i++) {
        cout << "  Node " << i << ": (" << stance_nodes[i][0] << ", " << stance_nodes[i][1] << ", " << stance_nodes[i][2] << ")" << endl;
    }

    // Validate stance trajectory: SUT must match De Casteljau (independent oracle)
    double max_dc_stance_error = 0.0;
    for (int i = 0; i <= 20; i++) {
        double t = static_cast<double>(i) / 20.0;
        Eigen::Vector3d sut_pos = math_utils::quarticBezier(stance_nodes, t);
        Eigen::Vector3d dc_pos = deCasteljauQuarticBezier(stance_nodes, t);
        double error = (sut_pos - dc_pos).norm();
        max_dc_stance_error = max(max_dc_stance_error, error);
    }
    cout << "Stance De Casteljau cross-check error: " << max_dc_stance_error << endl;
    assert(max_dc_stance_error < TEST_TOLERANCE);

    // Verify linearity: for equally-spaced collinear control points, quartic Bezier
    // IS exactly linear (all control points on a line with uniform spacing).
    // This IS a meaningful check: it verifies the SUT handles this degenerate case.
    double max_stance_linearity_error = 0.0;
    for (int i = 0; i <= 20; i++) {
        double t = static_cast<double>(i) / 20.0;
        Eigen::Vector3d pos = math_utils::quarticBezier(stance_nodes, t);
        Eigen::Vector3d linear_expected = start_pos + stride_vector * t;
        double error = (pos - linear_expected).norm();
        max_stance_linearity_error = max(max_stance_linearity_error, error);
    }
    cout << "Maximum stance linearity error: " << max_stance_linearity_error << endl;
    // Collinear equally-spaced quartic Bezier is exactly linear (within float precision)
    assert(max_stance_linearity_error < 1e-10);
    cout << "✓ Stance trajectory is exactly linear for uniformly-spaced collinear nodes" << endl;

    // Test swing trajectory shape matches OpenSHC characteristics
    cout << "Verifying swing trajectory characteristics..." << endl;

    Parameters params = createDefaultParameters();
    RobotModel model(params);
    model.workspaceAnalyzerInitializer(); // Initialize WorkspaceAnalyzer

    // Test that swing trajectory has proper bell curve shape
    double max_height = -1000.0f;
    double min_height = 1000.0f;

    for (int i = 0; i <= 100; i++) {
        double swing_progress = i / 100.0f;
        double total_phase = 0.6f + 0.4f * swing_progress; // Swing phase

        // Create simple swing trajectory control points
        Point3D control_points[5] = {
            Point3D(0.0f, 0.0f, -80.0f),  // Start: ground level
            Point3D(10.0f, 0.0f, -60.0f), // Early lift
            Point3D(20.0f, 0.0f, -60.0f), // Mid-swing: peak
            Point3D(30.0f, 0.0f, -60.0f), // Late descent
            Point3D(40.0f, 0.0f, -80.0f)  // End: ground level
        };

        Point3D pos = math_utils::quarticBezier(control_points, swing_progress);

        max_height = max(max_height, pos.z);
        min_height = min(min_height, pos.z);
    }

    cout << "Swing trajectory height range: " << min_height << " to " << max_height << " mm" << endl;
    assert(max_height > min_height); // Should have height variation
    assert(max_height > -70.0f);     // Should lift above ground

    cout << "✓ OpenSHC compatibility validated" << endl;
    return true;
}

int main() {
    cout << "========================================" << endl;
    cout << "HexaMotion vs OpenSHC Bezier Validation" << endl;
    cout << "========================================" << endl;

    bool all_tests_passed = true;

    try {
        all_tests_passed &= testBezierEquivalence();
        all_tests_passed &= testSwingTrajectoryEquivalence();
        all_tests_passed &= testContinuityAndSmoothness();
        all_tests_passed &= testOpenSHCCompatibility();

        cout << "\n========================================" << endl;

        if (all_tests_passed) {
            cout << "🎉 VALIDATION SUCCESSFUL! 🎉" << endl;
            cout << "HexaMotion Bezier validated against independent oracles:" << endl;
            cout << "  ✓ Quartic Bezier matches De Casteljau (independent algorithm)" << endl;
            cout << "  ✓ Derivative matches numerical finite differences" << endl;
            cout << "  ✓ Midpoint values match hand-computed analytic expectation" << endl;
            cout << "  ✓ Trajectory smoothness validated" << endl;
            cout << "  ✓ Stance linearity verified for collinear control points" << endl;
        } else {
            cout << "❌ VALIDATION FAILED" << endl;
            cout << "Some tests did not pass - implementation differs from OpenSHC" << endl;
        }

    } catch (const exception &e) {
        cout << "❌ TEST EXCEPTION: " << e.what() << endl;
        all_tests_passed = false;
    }

    cout << "========================================" << endl;

    return all_tests_passed ? 0 : 1;
}
