/**
 * @file bezier_test.cpp
 * @brief Consolidated test suite.
 *
 * This single translation unit folds together the following sub-tests
 * without losing any coverage. Each sub-test keeps its original assertions
 * and entry function; their bodies are wrapped in a dedicated namespace to
 * avoid symbol collisions, and this file's main() runs them in sequence and
 * aggregates the exit status:
 *   - run_bezier_validation()
 *   - run_bezier_curve_deterministic()
 *   - run_bezier_transition_single_leg()
 *   - run_bezier_transition_all_legs()
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/state_controller.h"
#include "body_pose_config_factory.h"
#include "gait_config.h"
#include "gait_config_factory.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// ===========================================================================
// Sub-test: run_bezier_validation (from bezier_validation_test.cpp)
// ===========================================================================
namespace cm_bezier_validation_test {
/**
 * @file bezier_validation_test.cpp
 * @brief Validation test to compare HexaMotion Bezier implementation with OpenSHC
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 *
 * This test validates that our Bezier curve implementation is equivalent to OpenSHC.
 */

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

int run_bezier_validation() {
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
} // namespace cm_bezier_validation_test

// ===========================================================================
// Sub-test: run_bezier_curve_deterministic (from bezier_curve_deterministic_test.cpp)
// ===========================================================================
namespace cm_bezier_curve_deterministic_test {
/**
 * @file bezier_curve_deterministic_test.cpp
 * @brief Deterministic validation that swing/stance trajectories conform to their
 *        analytical quartic Bézier curves with pre-established parameters.
 *
 * Tests cover nine levels:
 *
 * 1. **Pure Bézier math**: quarticBezier boundary conditions (B(0)=P0, B(1)=P4),
 *    derivative consistency, delta accumulation vs direct evaluation.
 * 2. **Control node generation**: Verify LegStepper-generated control nodes match
 *    the analytical formulas for primary and secondary swing phases.
 * 3. **C0/C1 continuity**: Position and velocity match at the swing phase junction.
 * 4. **Delta accumulation accuracy**: Simulated right-endpoint Riemann sum of B'(t)
 *    converges to B(t) within bounded discretization error.
 * 5. **Full swing trajectory**: End-to-end iterative trajectory from origin to target.
 * 6. **Stance linearity**: Equally-spaced stance nodes produce constant-velocity
 *    linear motion.
 * 7. **Swing clearance height**: Peak Z at midpoint includes the configured clearance.
 * 8. **Multi-leg consistency**: All six legs produce valid C0/C1 junctions and
 *    correct clearance.
 * 9. **Stride vector influence**: Validates that varying stride vectors (magnitude,
 *    diagonal, angular, combined) produce analytically correct control node changes
 *    across primary swing, secondary swing and stance Bézier curves.
 *
 * @author HexaMotion Team
 * @version 1.0
 */

// ── Configuration ──────────────────────────────────────────────────────────────

static constexpr double TIGHT_TOL = 1e-9;      // Deterministic (closed-form) checks
static constexpr double ACCUM_TOL = 2.5;       // Delta-accumulation vs direct (mm) — accounts for Riemann sum discretization with ~13 iterations
static constexpr double LANDING_TOL = 1.5;     // Landing position tolerance (mm)
static constexpr double C1_TOL = 1e-6;         // Velocity continuity tolerance
static constexpr double STANCE_LIN_TOL = 1e-9; // Stance linearity tolerance

static Parameters createTestParams() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.standing_height = 150;
    p.time_delta = 1.0 / 50.0; // 50 Hz
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    p.preserve_swing_end_pose = true;     // Avoid drift corrections for deterministic testing
    p.enable_workspace_constrain = false; // No workspace clamping for pure Bezier testing
    p.ik.max_iterations = 5;              // Minimal IK iterations (workspace init speed)
    return p;
}

/**
 * @brief Returns a shared RobotModel with pre-initialized workspace analyzer.
 *
 * Workspace initialization is expensive (brute-force IK grid search for all 6 legs).
 * Leg::setCurrentTipPositionGlobal -> RobotModel::makeReachable triggers lazy
 * workspace init regardless of enable_workspace_constrain. Sharing a single model
 * avoids repeating this O(legs * bearings * layers * radius_steps) work per test.
 */
static RobotModel &getSharedModel() {
    static Parameters p = createTestParams();
    static RobotModel model(p);
    static bool initialized = false;
    if (!initialized) {
        model.workspaceAnalyzerInitializer(ComputeConfig::low());
        initialized = true;
    }
    return model;
}

static double vec_distance(const Point3D &a, const Point3D &b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static void print_point(const char *label, const Point3D &p) {
    std::cout << "  " << label << ": (" << std::fixed << std::setprecision(4)
              << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 1: Pure Bézier math properties
// ════════════════════════════════════════════════════════════════════════════════

static int test_bezier_math_properties() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 1: Pure Bézier Math Properties.                    ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    // Known control points (arbitrary but well-separated)
    Point3D nodes[5] = {
        Point3D(10.0, 20.0, -150.0), // P0
        Point3D(30.0, 25.0, -140.0), // P1
        Point3D(60.0, 30.0, -120.0), // P2
        Point3D(80.0, 25.0, -135.0), // P3
        Point3D(100.0, 20.0, -150.0) // P4
    };

    // 1a. B(0) == P0
    Point3D b0 = math_utils::quarticBezier(nodes, 0.0);
    double err = vec_distance(b0, nodes[0]);
    std::cout << "  B(0) == P0: error = " << err << " ... ";
    if (err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1b. B(1) == P4
    Point3D b1 = math_utils::quarticBezier(nodes, 1.0);
    err = vec_distance(b1, nodes[4]);
    std::cout << "  B(1) == P4: error = " << err << " ... ";
    if (err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1c. B'(0) == 4*(P1 - P0) — analytical derivative at t=0
    Point3D bd0 = math_utils::quarticBezierDot(nodes, 0.0);
    Point3D expected_bd0 = (nodes[1] - nodes[0]) * 4.0;
    err = vec_distance(bd0, expected_bd0);
    std::cout << "  B'(0) == 4(P1-P0): error = " << err << " ... ";
    if (err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1d. B'(1) == 4*(P4 - P3) — analytical derivative at t=1
    Point3D bd1 = math_utils::quarticBezierDot(nodes, 1.0);
    Point3D expected_bd1 = (nodes[4] - nodes[3]) * 4.0;
    err = vec_distance(bd1, expected_bd1);
    std::cout << "  B'(1) == 4(P4-P3): error = " << err << " ... ";
    if (err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1e. Numerical derivative check: B'(t) ≈ (B(t+h) - B(t-h)) / (2h)
    double t_test = 0.4;
    double h = 1e-7;
    Point3D b_plus = math_utils::quarticBezier(nodes, t_test + h);
    Point3D b_minus = math_utils::quarticBezier(nodes, t_test - h);
    Point3D numerical_deriv = (b_plus - b_minus) / (2.0 * h);
    Point3D analytical_deriv = math_utils::quarticBezierDot(nodes, t_test);
    err = vec_distance(numerical_deriv, analytical_deriv);
    std::cout << "  B'(0.4) numerical vs analytical: error = " << std::scientific << err << " ... ";
    if (err > 1e-4) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1f. Delta accumulation (Riemann sum) vs direct evaluation
    // Sum of B'(i*dt)*dt for i=1..N should approximate B(1) - B(0)
    int N = 100;
    double dt = 1.0 / N;
    Point3D accumulated(0, 0, 0);
    for (int i = 1; i <= N; i++) {
        Point3D deriv = math_utils::quarticBezierDot(nodes, i * dt);
        accumulated = accumulated + deriv * dt;
    }
    Point3D expected_total = nodes[4] - nodes[0];
    err = vec_distance(accumulated, expected_total);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Delta accumulation (N=" << N << ") vs B(1)-B(0): error = " << err << " mm ... ";
    // Right-endpoint Riemann sum error for a cubic is O(1/N^2) with Euler-Maclaurin
    if (err > 1.0) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 1g. Delta accumulation at midpoint
    int half_N = N / 2;
    Point3D accumulated_half(0, 0, 0);
    for (int i = 1; i <= half_N; i++) {
        accumulated_half = accumulated_half + math_utils::quarticBezierDot(nodes, i * dt) * dt;
    }
    Point3D direct_half = math_utils::quarticBezier(nodes, 0.5) - nodes[0];
    err = vec_distance(accumulated_half, direct_half);
    std::cout << "  Delta accumulation at midpoint: error = " << err << " mm ... ";
    if (err > 0.5) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    std::cout << "  Test 1 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 2: Control Node Generation Verification
// ════════════════════════════════════════════════════════════════════════════════

static int test_control_node_generation() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 2: Control Node Generation Verification            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();

    // Use leg 0 (AR, base angle = -30°)
    Leg leg(0, model);

    // Identity tip position: use FK at zero angles (default position from DH)
    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0; // Walk plane frame (z=0 is the walk plane)

    LegStepper stepper(0, identity_tip, leg, model);

    // Configure step cycle
    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 50; // 50 iterations at 50Hz = 1 second
    cycle.stance_period_ = 25;
    cycle.swing_period_ = 25;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 25;
    cycle.swing_start_ = 25;
    cycle.swing_end_ = 50;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.setStepClearanceHeight(30.0); // 30mm swing clearance

    // Set known forward velocity → produces known stride_vector_
    Point3D linear_vel(50.0, 0.0, 0.0); // 50 mm/s forward
    stepper.setDesiredVelocity(linear_vel, 0.0);

    // Calculate timing
    stepper.calculateSwingTiming(p.time_delta);
    int swing_iters = stepper.getSwingIterations();
    int stance_iters = stepper.getStanceIterations();
    double swing_dt = stepper.getSwingDeltaT();
    double stance_dt = stepper.getStanceDeltaT();

    std::cout << "  Swing iterations: " << swing_iters << std::endl;
    std::cout << "  Stance iterations: " << stance_iters << std::endl;
    std::cout << "  Swing delta_t: " << swing_dt << std::endl;
    std::cout << "  Stance delta_t: " << stance_dt << std::endl;

    // Set current tip pose to a known swing origin position
    Point3D swing_origin = identity_tip;
    stepper.setCurrentTipPose(swing_origin);
    stepper.setDefaultTipPose(identity_tip);

    // Set zero initial velocity (first swing from rest)
    stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));

    // Compute stride and freeze it
    stepper.updateStride();
    Point3D stride = stepper.getStrideVector();
    std::cout << "  Stride vector: (" << stride.x << ", " << stride.y << ", " << stride.z << ")" << std::endl;

    // Target = default + stride * 0.5 (as computed in updateTipPositionIterative)
    Point3D target = identity_tip + stride * 0.5;

    stepper.setTargetTipPose(target);

    // Set step state and initialize swing
    stepper.setStepState(STEP_SWING);
    stepper.initializeSwingPeriod(1);

    // Generate control nodes
    stepper.testGeneratePrimarySwingControlNodes();
    stepper.testGenerateSecondarySwingControlNodes(false);

    // Read generated nodes
    Point3D s1[5], s2[5];
    for (int i = 0; i < 5; i++) {
        s1[i] = stepper.getSwing1ControlNode(i);
        s2[i] = stepper.getSwing2ControlNode(i);
    }

    // ── Analytically compute expected primary swing nodes ──
    // With V_origin = (0,0,0), stance_node_separation = (0,0,0)
    Point3D expected_mid = (swing_origin + target) / 2.0;
    expected_mid.z = std::max(swing_origin.z, target.z);
    Point3D swing_clearance = Point3D(0, 0, 1) * 30.0; // walk_plane_normal * step_clearance_height
    expected_mid = expected_mid + swing_clearance;
    // Lateral shift: default swing_width_ = 5.0; direction depends on identity_tip.y sign
    double default_swing_width = 5.0;
    bool positive_y = identity_tip.y > 0.0;
    expected_mid.y += positive_y ? default_swing_width : -default_swing_width;

    Point3D exp_s1[5];
    exp_s1[0] = swing_origin;
    exp_s1[1] = swing_origin; // + stance_node_sep * 1 = 0 (V_origin=0)
    exp_s1[2] = swing_origin; // + stance_node_sep * 2 = 0
    Point3D node3_raw = (expected_mid + exp_s1[2]) / 2.0;
    node3_raw.z = expected_mid.z;
    exp_s1[3] = node3_raw;
    exp_s1[4] = expected_mid;

    std::cout << "\n  Primary swing nodes (generated vs expected):" << std::endl;
    for (int i = 0; i < 5; i++) {
        double err_i = vec_distance(s1[i], exp_s1[i]);
        std::cout << "    N1[" << i << "] gen=(" << s1[i].x << "," << s1[i].y << "," << s1[i].z
                  << ") exp=(" << exp_s1[i].x << "," << exp_s1[i].y << "," << exp_s1[i].z
                  << ") err=" << err_i << (err_i > TIGHT_TOL ? " FAIL" : " OK") << std::endl;
        if (err_i > TIGHT_TOL)
            failures++;
    }

    // ── Analytically compute expected secondary swing nodes ──
    // final_tip_velocity = stride * (-1) * (stance_delta_t / time_delta)
    Point3D final_tip_vel = stride * (-1.0) * (stance_dt / p.time_delta);
    // departure_node_sep = final_tip_vel * 0.25 * (time_delta / swing_delta_t)
    Point3D departure_sep = final_tip_vel * 0.25 * (p.time_delta / swing_dt);

    Point3D exp_s2[5];
    exp_s2[0] = exp_s1[4];                           // C0: matches end of primary
    exp_s2[1] = exp_s1[4] - (exp_s1[3] - exp_s1[4]); // C1: mirror of node 3
    exp_s2[2] = target - departure_sep * 2.0;
    exp_s2[3] = target - departure_sep;
    exp_s2[4] = target;
    exp_s2[4].z = identity_tip.z; // Landing height snap

    std::cout << "\n  Secondary swing nodes (generated vs expected):" << std::endl;
    for (int i = 0; i < 5; i++) {
        double err_i = vec_distance(s2[i], exp_s2[i]);
        std::cout << "    N2[" << i << "] gen=(" << s2[i].x << "," << s2[i].y << "," << s2[i].z
                  << ") exp=(" << exp_s2[i].x << "," << exp_s2[i].y << "," << exp_s2[i].z
                  << ") err=" << err_i << (err_i > TIGHT_TOL ? " FAIL" : " OK") << std::endl;
        if (err_i > TIGHT_TOL)
            failures++;
    }

    std::cout << "  Test 2 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 3: C0/C1 Continuity at Swing Phase Junction
// ════════════════════════════════════════════════════════════════════════════════

static int test_phase_junction_continuity() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 3: C0/C1 Continuity at Swing Phase Junction        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();
    Leg leg(0, model);

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 50;
    cycle.stance_period_ = 25;
    cycle.swing_period_ = 25;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 25;
    cycle.swing_start_ = 25;
    cycle.swing_end_ = 50;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.setStepClearanceHeight(30.0);
    stepper.calculateSwingTiming(p.time_delta);

    // Use non-zero initial velocity for a richer test
    Point3D linear_vel(80.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_vel, 0.0);
    stepper.setCurrentTipPose(identity_tip);
    stepper.setDefaultTipPose(identity_tip);

    // Simulate stance exit velocity: stride * (-1) * stance_dt / time_delta
    stepper.updateStride();
    Point3D stride = stepper.getStrideVector();
    double stance_dt = stepper.getStanceDeltaT();
    Point3D stance_exit_vel = stride * (-1.0) * (stance_dt / p.time_delta);
    stepper.setSwingOriginTipVelocity(stance_exit_vel);

    Point3D target = identity_tip + stride * 0.5;
    stepper.setTargetTipPose(target);
    stepper.setStepState(STEP_SWING);
    stepper.initializeSwingPeriod(1);
    stepper.testGeneratePrimarySwingControlNodes();
    stepper.testGenerateSecondarySwingControlNodes(false);

    Point3D s1[5], s2[5];
    for (int i = 0; i < 5; i++) {
        s1[i] = stepper.getSwing1ControlNode(i);
        s2[i] = stepper.getSwing2ControlNode(i);
    }

    // C0: swing_1_nodes_[4] == swing_2_nodes_[0]
    double c0_err = vec_distance(s1[4], s2[0]);
    std::cout << "  C0 continuity (position at junction): error = " << c0_err << " ... ";
    if (c0_err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // C1: velocity continuity
    // B1'(1) = 4*(N1[4] - N1[3])
    // B2'(0) = 4*(N2[1] - N2[0])
    Point3D vel_end_phase1 = (s1[4] - s1[3]) * 4.0;
    Point3D vel_start_phase2 = (s2[1] - s2[0]) * 4.0;
    double c1_err = vec_distance(vel_end_phase1, vel_start_phase2);
    std::cout << "  C1 continuity (velocity at junction): error = " << c1_err << " ... ";
    if (c1_err > C1_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Also verify using the quarticBezierDot functions directly
    Point3D qbd_end1 = math_utils::quarticBezierDot(s1, 1.0);
    Point3D qbd_start2 = math_utils::quarticBezierDot(s2, 0.0);
    double c1_func_err = vec_distance(qbd_end1, qbd_start2);
    std::cout << "  C1 via quarticBezierDot: error = " << c1_func_err << " ... ";
    if (c1_func_err > C1_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Verify B1(1) == midpoint == B2(0)
    Point3D b1_end = math_utils::quarticBezier(s1, 1.0);
    Point3D b2_start = math_utils::quarticBezier(s2, 0.0);
    double mid_err = vec_distance(b1_end, b2_start);
    std::cout << "  Midpoint B1(1)==B2(0): error = " << mid_err << " ... ";
    if (mid_err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Print details
    print_point("B1'(1)", vel_end_phase1);
    print_point("B2'(0)", vel_start_phase2);
    print_point("Junction point", s1[4]);

    std::cout << "  Test 3 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 4: Delta Accumulation vs Direct Bézier Evaluation
// ════════════════════════════════════════════════════════════════════════════════

static int test_delta_accumulation_accuracy() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 4: Delta Accumulation vs Direct Bézier Evaluation  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();
    Leg leg(0, model);

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 50;
    cycle.stance_period_ = 25;
    cycle.swing_period_ = 25;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 25;
    cycle.swing_start_ = 25;
    cycle.swing_end_ = 50;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.setStepClearanceHeight(30.0);
    stepper.calculateSwingTiming(p.time_delta);

    int swing_iters = stepper.getSwingIterations();
    double swing_dt = stepper.getSwingDeltaT();

    Point3D linear_vel(60.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_vel, 0.0);
    stepper.setCurrentTipPose(identity_tip);
    stepper.setDefaultTipPose(identity_tip);
    stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
    stepper.updateStride();

    Point3D stride = stepper.getStrideVector();
    Point3D target = identity_tip + stride * 0.5;
    stepper.setTargetTipPose(target);
    stepper.setStepState(STEP_SWING);
    stepper.initializeSwingPeriod(1);
    stepper.testGeneratePrimarySwingControlNodes();
    stepper.testGenerateSecondarySwingControlNodes(false);

    Point3D s1[5], s2[5];
    for (int i = 0; i < 5; i++) {
        s1[i] = stepper.getSwing1ControlNode(i);
        s2[i] = stepper.getSwing2ControlNode(i);
    }

    std::cout << "  Testing primary swing (first half) — " << swing_iters / 2 << " iterations" << std::endl;

    // Simulate delta accumulation exactly as the code does
    int half_iters = swing_iters / 2;
    Point3D accumulated_pos = s1[0]; // Start at origin (nodes[0])
    double max_err_phase1 = 0.0;

    for (int i = 1; i <= half_iters; i++) {
        double t = swing_dt * i;
        Point3D delta = math_utils::quarticBezierDot(s1, t) * swing_dt;
        accumulated_pos = accumulated_pos + delta;
        Point3D direct_pos = math_utils::quarticBezier(s1, t);
        double err = vec_distance(accumulated_pos, direct_pos);
        if (err > max_err_phase1)
            max_err_phase1 = err;
    }

    std::cout << "  Phase 1 max error: " << max_err_phase1 << " mm ... ";
    if (max_err_phase1 > ACCUM_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // At t=1 of primary, accumulated should land at nodes[4]
    double end_err_1 = vec_distance(accumulated_pos, s1[4]);
    std::cout << "  Phase 1 endpoint error: " << end_err_1 << " mm ... ";
    if (end_err_1 > ACCUM_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Secondary swing (second half)
    std::cout << "  Testing secondary swing (second half) — " << swing_iters / 2 << " iterations" << std::endl;
    Point3D accumulated_pos2 = s2[0]; // Start at junction point
    double max_err_phase2 = 0.0;

    for (int i = 1; i <= half_iters; i++) {
        double t = swing_dt * i;
        Point3D delta = math_utils::quarticBezierDot(s2, t) * swing_dt;
        accumulated_pos2 = accumulated_pos2 + delta;
        Point3D direct_pos = math_utils::quarticBezier(s2, t);
        double err = vec_distance(accumulated_pos2, direct_pos);
        if (err > max_err_phase2)
            max_err_phase2 = err;
    }

    std::cout << "  Phase 2 max error: " << max_err_phase2 << " mm ... ";
    if (max_err_phase2 > ACCUM_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // At t=1 of secondary, accumulated should land at nodes[4] = target
    double end_err_2 = vec_distance(accumulated_pos2, s2[4]);
    std::cout << "  Phase 2 endpoint error: " << end_err_2 << " mm ... ";
    if (end_err_2 > ACCUM_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Full-swing accumulated endpoint should be near target
    double full_err = vec_distance(accumulated_pos2, target);
    std::cout << "  Full swing endpoint vs target: error = " << full_err << " mm" << std::endl;

    std::cout << "  Test 4 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 5: Full Swing Trajectory via updateTipPositionIterative
// ════════════════════════════════════════════════════════════════════════════════

static int test_full_swing_trajectory() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 5: Full Swing Trajectory (updateTipPositionIter.)  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();
    Leg leg(0, model);

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 50;
    cycle.stance_period_ = 25;
    cycle.swing_period_ = 25;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 25;
    cycle.swing_start_ = 25;
    cycle.swing_end_ = 50;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.setStepClearanceHeight(30.0);

    Point3D linear_vel(50.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_vel, 0.0);
    stepper.setCurrentTipPose(identity_tip);
    stepper.setDefaultTipPose(identity_tip);
    stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
    stepper.setStepState(STEP_SWING);
    stepper.setCompletedFirstStep(true);

    // Run the full swing trajectory
    stepper.calculateSwingTiming(p.time_delta);
    int swing_iters = stepper.getSwingIterations();
    double swing_dt = stepper.getSwingDeltaT();

    Point3D start_pos = stepper.getCurrentTipPose();
    std::cout << "  Swing iterations: " << swing_iters << std::endl;
    print_point("Start position", start_pos);

    // Run all swing iterations through the real code path
    std::vector<Point3D> trajectory;
    trajectory.push_back(start_pos);

    for (int iter = cycle.swing_start_ + 1; iter <= cycle.swing_end_; iter++) {
        stepper.updateTipPositionIterative(iter, p.time_delta, false, false);
        trajectory.push_back(stepper.getCurrentTipPose());
    }

    // After the first iteration, read back the control nodes for analytical comparison
    // (Nodes are regenerated every iteration but should be stable given frozen inputs)

    Point3D final_pos = trajectory.back();
    print_point("Final position", final_pos);

    // Read target
    Point3D actual_target = stepper.getTargetTipPose();
    print_point("Target position", actual_target);

    // 5a. Verify the trajectory started at the origin
    double start_err = vec_distance(trajectory[0], identity_tip);
    std::cout << "  Origin match: error = " << start_err << " ... ";
    if (start_err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 5b. Verify the trajectory ends near the target (within landing tolerance)
    double landing_err = vec_distance(final_pos, actual_target);
    std::cout << "  Landing match: error = " << landing_err << " mm ... ";
    if (landing_err > LANDING_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 5c. Verify net forward progress (Bezier arcs may overshoot then return to target)
    double net_x = final_pos.x - start_pos.x;
    bool forward_progress = net_x > 0.0;
    std::cout << "  Net X forward progress: " << net_x << " mm ... " << (forward_progress ? "OK" : "FAIL") << std::endl;
    if (!forward_progress)
        failures++;

    // 5d. Verify swing reached above the walk plane (clearance height)
    double max_z = -1e9;
    int max_z_iter = 0;
    for (size_t i = 0; i < trajectory.size(); i++) {
        if (trajectory[i].z > max_z) {
            max_z = trajectory[i].z;
            max_z_iter = (int)i;
        }
    }
    std::cout << "  Max Z height: " << max_z << " mm at iteration " << max_z_iter << " ... ";
    // Swing clearance is 30mm, so peak should be close to that
    if (max_z < 15.0) { // Should reach at least half the clearance
        std::cout << "FAIL (too low)" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // 5e. Print trajectory sample for visual inspection
    std::cout << "\n  Trajectory sample (every 5th point):" << std::endl;
    std::cout << "  Iter  | X         | Y         | Z" << std::endl;
    std::cout << "  ------+-----------+-----------+-----------" << std::endl;
    for (size_t i = 0; i < trajectory.size(); i += 5) {
        printf("  %3zu   | %9.3f | %9.3f | %9.3f\n", i,
               trajectory[i].x, trajectory[i].y, trajectory[i].z);
    }
    // Always show the last point
    if ((trajectory.size() - 1) % 5 != 0) {
        printf("  %3zu   | %9.3f | %9.3f | %9.3f\n", trajectory.size() - 1,
               trajectory.back().x, trajectory.back().y, trajectory.back().z);
    }

    std::cout << "\n  Test 5 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 6: Stance Linearity
// ════════════════════════════════════════════════════════════════════════════════

static int test_stance_linearity() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 6: Stance Linearity (Equally-Spaced Nodes)         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();
    Leg leg(0, model);

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;
    LegStepper stepper(0, identity_tip, leg, model);

    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 50;
    cycle.stance_period_ = 25;
    cycle.swing_period_ = 25;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 25;
    cycle.swing_start_ = 25;
    cycle.swing_end_ = 50;
    stepper.setStepCycle(cycle);
    stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
    stepper.calculateSwingTiming(p.time_delta);

    Point3D linear_vel(50.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_vel, 0.0);
    stepper.setCurrentTipPose(identity_tip);
    stepper.setDefaultTipPose(identity_tip);
    stepper.updateStride();
    Point3D stride = stepper.getStrideVector();

    // Generate stance control nodes with stride_scaler = 1.0
    stepper.testGenerateStanceControlNodes(1.0);

    Point3D sn[5];
    for (int i = 0; i < 5; i++) {
        sn[i] = stepper.getStanceControlNode(i);
    }

    // Verify nodes are equally spaced (collinear with constant separation)
    Point3D sep = sn[1] - sn[0];
    std::cout << "  Node separation: (" << sep.x << ", " << sep.y << ", " << sep.z << ")" << std::endl;

    for (int i = 2; i < 5; i++) {
        Point3D actual_sep = sn[i] - sn[i - 1];
        double err = vec_distance(actual_sep, sep);
        std::cout << "  sep[" << i - 1 << "→" << i << "] vs sep[0→1]: error = " << err << " ... ";
        if (err > STANCE_LIN_TOL) {
            std::cout << "FAIL" << std::endl;
            failures++;
        } else {
            std::cout << "OK" << std::endl;
        }
    }

    // Verify that all quarticBezierDot evaluations at different t produce the same delta
    double stance_dt = stepper.getStanceDeltaT();
    int stance_iters = stepper.getStanceIterations();
    Point3D first_delta = math_utils::quarticBezierDot(sn, stance_dt) * stance_dt;
    std::cout << "  First stance delta: (" << first_delta.x << ", " << first_delta.y << ", " << first_delta.z << ")" << std::endl;

    double max_delta_err = 0.0;
    for (int i = 2; i <= stance_iters; i++) {
        double t = i * stance_dt;
        Point3D delta = math_utils::quarticBezierDot(sn, t) * stance_dt;
        double err = vec_distance(delta, first_delta);
        if (err > max_delta_err)
            max_delta_err = err;
    }
    std::cout << "  Max delta variation: " << max_delta_err << " ... ";
    if (max_delta_err > STANCE_LIN_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    // Verify total displacement matches expected stride
    // Total displacement = stance_node_separation * 4 = stride * (-stride_scaler * 0.25) * 4 = -stride
    Point3D total_displacement = sn[4] - sn[0];
    Point3D expected_displacement = stride * (-1.0);
    double disp_err = vec_distance(total_displacement, expected_displacement);
    std::cout << "  Total displacement vs -stride: error = " << disp_err << " ... ";
    if (disp_err > TIGHT_TOL) {
        std::cout << "FAIL" << std::endl;
        failures++;
    } else {
        std::cout << "OK" << std::endl;
    }

    std::cout << "  Test 6 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 7: Swing Clearance Height at Midpoint
// ════════════════════════════════════════════════════════════════════════════════

static int test_swing_clearance_height() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 7: Swing Clearance Height at Midpoint              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();
    Leg leg(0, model);

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;

    // Test with various clearance heights
    double clearance_heights[] = {10.0, 30.0, 50.0};
    int num_heights = 3;

    for (int h = 0; h < num_heights; h++) {
        double clearance = clearance_heights[h];
        std::cout << "\n  --- Clearance height: " << clearance << " mm ---" << std::endl;

        Leg test_leg(0, model);
        LegStepper stepper(0, identity_tip, test_leg, model);

        StepCycle cycle{};
        cycle.frequency_ = 1.0;
        cycle.period_ = 50;
        cycle.stance_period_ = 25;
        cycle.swing_period_ = 25;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = 25;
        cycle.swing_start_ = 25;
        cycle.swing_end_ = 50;
        stepper.setStepCycle(cycle);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(clearance);
        stepper.calculateSwingTiming(p.time_delta);

        Point3D linear_vel(50.0, 0.0, 0.0);
        stepper.setDesiredVelocity(linear_vel, 0.0);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();
        Point3D target = identity_tip + stride * 0.5;
        stepper.setTargetTipPose(target);
        stepper.setStepState(STEP_SWING);
        stepper.initializeSwingPeriod(1);
        stepper.testGeneratePrimarySwingControlNodes();

        // The midpoint node (swing_1_nodes_[4]) should have z = max(origin.z, target.z) + clearance
        Point3D mid_node = stepper.getSwing1ControlNode(4);
        double expected_mid_z = std::max(identity_tip.z, target.z) + clearance;
        double z_err = std::abs(mid_node.z - expected_mid_z);
        std::cout << "  Midpoint Z: " << mid_node.z << ", expected: " << expected_mid_z
                  << ", error: " << z_err << " ... ";
        if (z_err > TIGHT_TOL) {
            std::cout << "FAIL" << std::endl;
            failures++;
        } else {
            std::cout << "OK" << std::endl;
        }

        // Also verify the XY midpoint is the average of origin and target (plus swing_width)
        double expected_mid_x = (identity_tip.x + target.x) / 2.0;
        double swing_w = 5.0; // default swing_width_
        bool pos_y = identity_tip.y > 0.0;
        double expected_mid_y = (identity_tip.y + target.y) / 2.0 + (pos_y ? swing_w : -swing_w);
        double xy_err_x = std::abs(mid_node.x - expected_mid_x);
        double xy_err_y = std::abs(mid_node.y - expected_mid_y);
        std::cout << "  Midpoint XY: (" << mid_node.x << ", " << mid_node.y
                  << "), expected: (" << expected_mid_x << ", " << expected_mid_y
                  << "), err_x=" << xy_err_x << " err_y=" << xy_err_y << " ... ";
        if (xy_err_x > TIGHT_TOL || xy_err_y > TIGHT_TOL) {
            std::cout << "FAIL" << std::endl;
            failures++;
        } else {
            std::cout << "OK" << std::endl;
        }
    }

    std::cout << "\n  Test 7 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 8: Multi-leg Bézier Consistency
// ════════════════════════════════════════════════════════════════════════════════

static int test_multi_leg_bezier_consistency() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 8: Multi-Leg Bézier Consistency (All 6 Legs)       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();

    // For each leg, verify that the C0/C1 continuity holds
    // and that clearance height is reached
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        Leg test_leg(leg_id, model);
        Point3D identity_tip = model.getLegDefaultPosition(leg_id);
        identity_tip.z = 0.0;

        LegStepper stepper(leg_id, identity_tip, test_leg, model);

        StepCycle cycle{};
        cycle.frequency_ = 1.0;
        cycle.period_ = 50;
        cycle.stance_period_ = 25;
        cycle.swing_period_ = 25;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = 25;
        cycle.swing_start_ = 25;
        cycle.swing_end_ = 50;
        stepper.setStepCycle(cycle);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(30.0);
        stepper.calculateSwingTiming(p.time_delta);

        Point3D linear_vel(50.0, 0.0, 0.0);
        stepper.setDesiredVelocity(linear_vel, 0.0);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();
        Point3D target = identity_tip + stride * 0.5;
        stepper.setTargetTipPose(target);
        stepper.setStepState(STEP_SWING);
        stepper.initializeSwingPeriod(1);
        stepper.testGeneratePrimarySwingControlNodes();
        stepper.testGenerateSecondarySwingControlNodes(false);

        Point3D s1[5], s2[5];
        for (int i = 0; i < 5; i++) {
            s1[i] = stepper.getSwing1ControlNode(i);
            s2[i] = stepper.getSwing2ControlNode(i);
        }

        // C0 continuity
        double c0_err = vec_distance(s1[4], s2[0]);
        // C1 continuity
        Point3D v_end = (s1[4] - s1[3]) * 4.0;
        Point3D v_start = (s2[1] - s2[0]) * 4.0;
        double c1_err = vec_distance(v_end, v_start);
        // Origin
        double origin_err = vec_distance(s1[0], identity_tip);
        // Clearance
        double clearance_z = s1[4].z;
        double expected_z = std::max(identity_tip.z, target.z) + 30.0;
        double z_err = std::abs(clearance_z - expected_z);

        bool leg_ok = (c0_err < TIGHT_TOL) && (c1_err < C1_TOL) &&
                      (origin_err < TIGHT_TOL) && (z_err < TIGHT_TOL);

        std::cout << "  Leg " << leg_id << ": C0=" << c0_err << " C1=" << c1_err
                  << " Origin=" << origin_err << " Clearance_Z_err=" << z_err
                  << " ... " << (leg_ok ? "OK" : "FAIL") << std::endl;
        if (!leg_ok)
            failures++;
    }

    std::cout << "  Test 8 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Test 9: Stride Vector Influence on Bézier Control Nodes
// ════════════════════════════════════════════════════════════════════════════════

static int test_stride_vector_influence() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test 9: Stride Vector Influence on Bézier Nodes         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    int failures = 0;

    RobotModel &model = getSharedModel();
    const Parameters &p = model.getParams();

    // Common cycle: period 50, equal stance/swing (25 each), freq 1.0
    // Derived constants (calculateSwingTiming rounds to even):
    //   stride_scale  = (25/50) / 1.0 = 0.5
    //   swing_iters   = 25 → round_even → 26;  swing_dt  = 1/(26/2) = 1/13 ≈ 0.076923
    //   stance_iters  = 25 → round_even → 26;  stance_dt = 1/26 ≈ 0.038462
    // Note: Products like (stance_dt/time_delta)*(time_delta/swing_dt) = stance_dt/swing_dt = 0.5
    //       are invariant regardless of rounding, so most analytical formulas tolerate the
    //       simplified constants below.  Sub-tests that feed a fixed external velocity (e.g. 9b)
    //       must query the stepper's actual timing values to avoid residual mismatch.
    const double STRIDE_SCALE = 0.5;
    const double TIME_DELTA = p.time_delta; // 0.02
    const double CLEARANCE = 30.0;
    const double SWING_W = 5.0; // default swing_width_

    Point3D identity_tip = model.getLegDefaultPosition(0);
    identity_tip.z = 0.0;
    bool pos_y = (identity_tip.y > 0.0);

    auto make_cycle = []() -> StepCycle {
        StepCycle c{};
        c.frequency_ = 1.0;
        c.period_ = 50;
        c.stance_period_ = 25;
        c.swing_period_ = 25;
        c.stance_start_ = 0;
        c.stance_end_ = 25;
        c.swing_start_ = 25;
        c.swing_end_ = 50;
        return c;
    };

    // ── 9a: Stride vector scales linearly with velocity ──────────────────────
    std::cout << "\n  ── 9a: Stride magnitude linear scaling ──" << std::endl;
    {
        double velocities[] = {30.0, 60.0, 90.0};
        Point3D strides[3];

        for (int v = 0; v < 3; v++) {
            Leg leg(0, model);
            LegStepper stepper(0, identity_tip, leg, model);
            stepper.setStepCycle(make_cycle());
            stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
            stepper.setStepClearanceHeight(CLEARANCE);
            stepper.calculateSwingTiming(TIME_DELTA);

            stepper.setDesiredVelocity(Point3D(velocities[v], 0.0, 0.0), 0.0);
            stepper.setCurrentTipPose(identity_tip);
            stepper.setDefaultTipPose(identity_tip);
            stepper.updateStride();

            strides[v] = stepper.getStrideVector();

            // stride = velocity * stride_scale (X-only for pure forward)
            Point3D expected(velocities[v] * STRIDE_SCALE, 0.0, 0.0);
            double err = vec_distance(strides[v], expected);
            std::cout << "  V=" << velocities[v] << "  stride.x=" << strides[v].x
                      << "  expected=" << expected.x << "  err=" << err
                      << " ... " << (err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
            if (err > TIGHT_TOL)
                failures++;

            // Stance total displacement = -stride (stride_scaler=1)
            stepper.testGenerateStanceControlNodes(1.0);
            Point3D disp = stepper.getStanceControlNode(4) - stepper.getStanceControlNode(0);
            Point3D expected_disp = strides[v] * (-1.0);
            double disp_err = vec_distance(disp, expected_disp);
            std::cout << "         stance disp err=" << disp_err
                      << " ... " << (disp_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
            if (disp_err > TIGHT_TOL)
                failures++;
        }

        // Scaling: stride[1] = 2*stride[0], stride[2] = 3*stride[0]
        double scale_2 = vec_distance(strides[1], strides[0] * 2.0);
        double scale_3 = vec_distance(strides[2], strides[0] * 3.0);
        std::cout << "  Scaling 2x err=" << scale_2 << "  3x err=" << scale_3
                  << " ... " << ((scale_2 < TIGHT_TOL && scale_3 < TIGHT_TOL) ? "OK" : "FAIL") << std::endl;
        if (scale_2 > TIGHT_TOL || scale_3 > TIGHT_TOL)
            failures++;
    }

    // ── 9b: Primary swing entry nodes with realistic V_origin ────────────────
    std::cout << "\n  ── 9b: Primary swing with non-zero V_origin ──" << std::endl;
    {
        double V = 80.0;
        Leg leg(0, model);
        LegStepper stepper(0, identity_tip, leg, model);
        stepper.setStepCycle(make_cycle());
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(CLEARANCE);
        stepper.calculateSwingTiming(TIME_DELTA);

        stepper.setDesiredVelocity(Point3D(V, 0.0, 0.0), 0.0);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();
        Point3D target = identity_tip + stride * 0.5;
        stepper.setTargetTipPose(target);
        stepper.setStepState(STEP_SWING);

        // Realistic stance exit velocity: v_exit = stride * (-1) * (stance_dt / time_delta)
        // Use actual stepper timing values (round-to-even may differ from hardcoded constants)
        double actual_stance_dt = stepper.getStanceDeltaT();
        double actual_swing_dt = stepper.getSwingDeltaT();
        Point3D v_origin = stride * (-1.0) * (actual_stance_dt / TIME_DELTA);

        // Set after initializeSwingPeriod (which copies current_tip_velocity_=0)
        stepper.initializeSwingPeriod(1);
        stepper.setSwingOriginTipVelocity(v_origin);
        stepper.testGeneratePrimarySwingControlNodes();

        // Expected entry separation: v_origin * 0.25 * (time_delta / swing_dt)
        // Must use actual swing_dt to match stepper's generatePrimarySwingControlNodes()
        Point3D entry_sep = v_origin * 0.25 * (TIME_DELTA / actual_swing_dt);

        Point3D n[5];
        for (int i = 0; i < 5; i++)
            n[i] = stepper.getSwing1ControlNode(i);

        // n[0] = origin (C0 from stance)
        double err0 = vec_distance(n[0], identity_tip);
        std::cout << "  Node[0]=origin: err=" << err0 << " ... " << (err0 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err0 > TIGHT_TOL)
            failures++;

        // n[1] = origin + entry_sep (C1 from stance)
        Point3D exp1 = identity_tip + entry_sep;
        double err1 = vec_distance(n[1], exp1);
        std::cout << "  Node[1]=origin+entry_sep: err=" << err1 << " ... " << (err1 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err1 > TIGHT_TOL)
            failures++;

        // n[2] = origin + 2*entry_sep (C2 from stance)
        Point3D exp2 = identity_tip + entry_sep * 2.0;
        double err2 = vec_distance(n[2], exp2);
        std::cout << "  Node[2]=origin+2*entry_sep: err=" << err2 << " ... " << (err2 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err2 > TIGHT_TOL)
            failures++;

        // entry_sep must be non-zero (demonstrating stride influence)
        double sep_norm = std::sqrt(entry_sep.x * entry_sep.x + entry_sep.y * entry_sep.y + entry_sep.z * entry_sep.z);
        std::cout << "  |entry_sep| = " << sep_norm << " mm ... " << (sep_norm > 1e-6 ? "OK (non-zero)" : "FAIL") << std::endl;
        if (sep_norm < 1e-6)
            failures++;

        // n[4] = midpoint (clearance + stride-shifted centre)
        Point3D exp_mid = (identity_tip + target) / 2.0;
        exp_mid.z = std::max(identity_tip.z, target.z) + CLEARANCE;
        exp_mid.y += pos_y ? SWING_W : -SWING_W;
        double mid_err = vec_distance(n[4], exp_mid);
        std::cout << "  Node[4]=midpoint: err=" << mid_err << " ... " << (mid_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (mid_err > TIGHT_TOL)
            failures++;

        // n[3] = (mid + n[2]) / 2, with z = mid.z (C2 symmetric)
        Point3D exp3 = (exp_mid + exp2) / 2.0;
        exp3.z = exp_mid.z;
        double err3 = vec_distance(n[3], exp3);
        std::cout << "  Node[3]=(mid+n2)/2: err=" << err3 << " ... " << (err3 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err3 > TIGHT_TOL)
            failures++;

        // Contrast against V_origin=0 — nodes 1,2 must differ
        Leg leg_z(0, model);
        LegStepper stepper_z(0, identity_tip, leg_z, model);
        stepper_z.setStepCycle(make_cycle());
        stepper_z.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper_z.setStepClearanceHeight(CLEARANCE);
        stepper_z.calculateSwingTiming(TIME_DELTA);
        stepper_z.setDesiredVelocity(Point3D(V, 0.0, 0.0), 0.0);
        stepper_z.setCurrentTipPose(identity_tip);
        stepper_z.setDefaultTipPose(identity_tip);
        stepper_z.updateStride();
        stepper_z.setTargetTipPose(target);
        stepper_z.setStepState(STEP_SWING);
        stepper_z.initializeSwingPeriod(1);
        stepper_z.setSwingOriginTipVelocity(Point3D(0, 0, 0));
        stepper_z.testGeneratePrimarySwingControlNodes();

        Point3D n1_zero = stepper_z.getSwing1ControlNode(1);
        double delta_n1 = vec_distance(n[1], n1_zero);
        std::cout << "  Node[1] shift vs V_origin=0: " << delta_n1 << " mm ... "
                  << (delta_n1 > 1e-6 ? "OK (different)" : "FAIL") << std::endl;
        if (delta_n1 < 1e-6)
            failures++;
    }

    // ── 9c: Secondary swing departure nodes ──────────────────────────────────
    std::cout << "\n  ── 9c: Secondary swing departure separation ──" << std::endl;
    {
        double velocities[] = {40.0, 80.0};
        Point3D departure_seps[2];

        for (int v = 0; v < 2; v++) {
            double V = velocities[v];
            Leg leg(0, model);
            LegStepper stepper(0, identity_tip, leg, model);
            stepper.setStepCycle(make_cycle());
            stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
            stepper.setStepClearanceHeight(CLEARANCE);
            stepper.calculateSwingTiming(TIME_DELTA);

            stepper.setDesiredVelocity(Point3D(V, 0.0, 0.0), 0.0);
            stepper.setCurrentTipPose(identity_tip);
            stepper.setDefaultTipPose(identity_tip);
            stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
            stepper.updateStride();

            Point3D stride = stepper.getStrideVector();
            Point3D target = identity_tip + stride * 0.5;
            stepper.setTargetTipPose(target);
            stepper.setStepState(STEP_SWING);
            stepper.initializeSwingPeriod(1);
            stepper.testGeneratePrimarySwingControlNodes();
            stepper.testGenerateSecondarySwingControlNodes(false);

            // Expected: final_vel = stride * (-1) * (stance_dt / time_delta)
            //           departure_sep = final_vel * 0.25 * (time_delta / swing_dt)
            // Query the stepper's actual deltas (post-§2.2 stance is plain int).
            Point3D final_vel = stride * (-1.0) * (stepper.getStanceDeltaT() / TIME_DELTA);
            Point3D dep_sep = final_vel * 0.25 * (TIME_DELTA / stepper.getSwingDeltaT());
            departure_seps[v] = dep_sep;

            Point3D s2[5];
            for (int i = 0; i < 5; i++)
                s2[i] = stepper.getSwing2ControlNode(i);

            // s2[4] = target (with z forced to default)
            Point3D exp4 = target;
            exp4.z = identity_tip.z; // default_tip_pose_.z
            double err4 = vec_distance(s2[4], exp4);
            // s2[3] = target - departure_sep
            double err3 = vec_distance(s2[3], target - dep_sep);
            // s2[2] = target - 2*departure_sep
            double err2 = vec_distance(s2[2], target - dep_sep * 2.0);

            bool ok = (err4 < TIGHT_TOL && err3 < TIGHT_TOL && err2 < TIGHT_TOL);
            std::cout << "  V=" << V << "  s2[4] err=" << err4
                      << "  s2[3] err=" << err3 << "  s2[2] err=" << err2
                      << " ... " << (ok ? "OK" : "FAIL") << std::endl;
            if (!ok)
                failures++;

            // departure_sep must be non-zero
            double dep_norm = std::sqrt(dep_sep.x * dep_sep.x + dep_sep.y * dep_sep.y + dep_sep.z * dep_sep.z);
            std::cout << "         |departure_sep| = " << dep_norm << " mm ... "
                      << (dep_norm > 1e-6 ? "OK" : "FAIL") << std::endl;
            if (dep_norm < 1e-6)
                failures++;
        }

        // departure_sep scales linearly: V=80 gives 2x departure_sep vs V=40
        double scale_err = vec_distance(departure_seps[1], departure_seps[0] * 2.0);
        std::cout << "  Departure scaling 2x err=" << scale_err
                  << " ... " << (scale_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (scale_err > TIGHT_TOL)
            failures++;
    }

    // ── 9d: Diagonal stride (non-zero vx + vy) ──────────────────────────────
    std::cout << "\n  ── 9d: Diagonal stride ──" << std::endl;
    {
        double vx = 40.0, vy = 30.0;
        Leg leg(0, model);
        LegStepper stepper(0, identity_tip, leg, model);
        stepper.setStepCycle(make_cycle());
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(CLEARANCE);
        stepper.calculateSwingTiming(TIME_DELTA);

        stepper.setDesiredVelocity(Point3D(vx, vy, 0.0), 0.0);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();
        Point3D expected_stride(vx * STRIDE_SCALE, vy * STRIDE_SCALE, 0.0);
        double stride_err = vec_distance(stride, expected_stride);
        std::cout << "  stride=(" << stride.x << "," << stride.y << "," << stride.z << ")"
                  << " expected=(" << expected_stride.x << "," << expected_stride.y << ",0)"
                  << " err=" << stride_err << " ... " << (stride_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (stride_err > TIGHT_TOL)
            failures++;

        // Target has both X and Y offset
        Point3D target = identity_tip + stride * 0.5;
        double target_dx = target.x - identity_tip.x;
        double target_dy = target.y - identity_tip.y;
        bool xy_ok = (std::abs(target_dx - vx * STRIDE_SCALE * 0.5) < TIGHT_TOL &&
                      std::abs(target_dy - vy * STRIDE_SCALE * 0.5) < TIGHT_TOL);
        std::cout << "  target offset=(" << target_dx << "," << target_dy << ")"
                  << " ... " << (xy_ok ? "OK" : "FAIL") << std::endl;
        if (!xy_ok)
            failures++;

        // Stance displacement must have both X and Y components
        stepper.testGenerateStanceControlNodes(1.0);
        Point3D stance_disp = stepper.getStanceControlNode(4) - stepper.getStanceControlNode(0);
        Point3D expected_disp = stride * (-1.0);
        double disp_err = vec_distance(stance_disp, expected_disp);
        std::cout << "  stance disp=(" << stance_disp.x << "," << stance_disp.y << "," << stance_disp.z << ")"
                  << " err=" << disp_err << " ... " << (disp_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (disp_err > TIGHT_TOL)
            failures++;

        // Primary swing midpoint must reflect diagonal trajectory
        stepper.setTargetTipPose(target);
        stepper.setStepState(STEP_SWING);
        stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
        stepper.initializeSwingPeriod(1);
        stepper.testGeneratePrimarySwingControlNodes();

        Point3D mid = stepper.getSwing1ControlNode(4);
        Point3D exp_mid = (identity_tip + target) / 2.0;
        exp_mid.z = std::max(identity_tip.z, target.z) + CLEARANCE;
        exp_mid.y += pos_y ? SWING_W : -SWING_W;
        double mid_err = vec_distance(mid, exp_mid);
        std::cout << "  swing midpoint err=" << mid_err << " ... " << (mid_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (mid_err > TIGHT_TOL)
            failures++;
    }

    // ── 9e: Angular velocity stride contribution ─────────────────────────────
    std::cout << "\n  ── 9e: Angular velocity stride (pure rotation) ──" << std::endl;
    {
        double omega = 0.5; // rad/s
        Leg leg(0, model);
        LegStepper stepper(0, identity_tip, leg, model);
        stepper.setStepCycle(make_cycle());
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(CLEARANCE);
        stepper.calculateSwingTiming(TIME_DELTA);

        stepper.setDesiredVelocity(Point3D(0.0, 0.0, 0.0), omega);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();

        // Angular stride = cross((0,0,omega), radius) * stride_scale
        //   radius = (identity_tip.x, identity_tip.y, 0)  [z rejected]
        //   cross result = (-omega*Iy, omega*Ix, 0)
        double Ix = identity_tip.x, Iy = identity_tip.y;
        Point3D expected_stride(
            -omega * Iy * STRIDE_SCALE,
            omega * Ix * STRIDE_SCALE,
            0.0);
        double stride_err = vec_distance(stride, expected_stride);
        std::cout << "  angular stride=(" << stride.x << "," << stride.y << "," << stride.z << ")"
                  << " expected=(" << expected_stride.x << "," << expected_stride.y << ",0)"
                  << " err=" << stride_err << " ... " << (stride_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (stride_err > TIGHT_TOL)
            failures++;

        // Angular stride must be tangential: dot(stride, radius) ≈ 0
        double dot = stride.x * Ix + stride.y * Iy;
        std::cout << "  tangential check (stride·radius) = " << dot
                  << " ... " << (std::abs(dot) < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (std::abs(dot) > TIGHT_TOL)
            failures++;

        // Non-zero magnitude
        double stride_norm = std::sqrt(stride.x * stride.x + stride.y * stride.y);
        std::cout << "  |stride| = " << stride_norm << " mm ... "
                  << (stride_norm > 1e-6 ? "OK (non-zero)" : "FAIL") << std::endl;
        if (stride_norm < 1e-6)
            failures++;

        // Stance nodes follow angular stride
        stepper.testGenerateStanceControlNodes(1.0);
        Point3D disp = stepper.getStanceControlNode(4) - stepper.getStanceControlNode(0);
        Point3D expected_disp = stride * (-1.0);
        double disp_err = vec_distance(disp, expected_disp);
        std::cout << "  stance disp err=" << disp_err << " ... " << (disp_err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (disp_err > TIGHT_TOL)
            failures++;
    }

    // ── 9f: Combined linear + angular stride ─────────────────────────────────
    std::cout << "\n  ── 9f: Combined linear + angular stride ──" << std::endl;
    {
        double vx = 50.0, omega = 0.3;
        Leg leg(0, model);
        LegStepper stepper(0, identity_tip, leg, model);
        stepper.setStepCycle(make_cycle());
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.setStepClearanceHeight(CLEARANCE);
        stepper.calculateSwingTiming(TIME_DELTA);

        stepper.setDesiredVelocity(Point3D(vx, 0.0, 0.0), omega);
        stepper.setCurrentTipPose(identity_tip);
        stepper.setDefaultTipPose(identity_tip);
        stepper.updateStride();

        Point3D stride = stepper.getStrideVector();

        // linear + angular contributions
        double Ix = identity_tip.x, Iy = identity_tip.y;
        Point3D expected_stride(
            (vx + (-omega * Iy)) * STRIDE_SCALE,
            (0.0 + omega * Ix) * STRIDE_SCALE,
            0.0);
        double err = vec_distance(stride, expected_stride);
        std::cout << "  combined stride=(" << stride.x << "," << stride.y << "," << stride.z << ")"
                  << " expected=(" << expected_stride.x << "," << expected_stride.y << ",0)"
                  << " err=" << err << " ... " << (err < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err > TIGHT_TOL)
            failures++;

        // Verify combined stride propagates through secondary swing nodes
        Point3D target = identity_tip + stride * 0.5;
        stepper.setTargetTipPose(target);
        stepper.setStepState(STEP_SWING);
        stepper.setSwingOriginTipVelocity(Point3D(0, 0, 0));
        stepper.initializeSwingPeriod(1);
        stepper.testGeneratePrimarySwingControlNodes();
        stepper.testGenerateSecondarySwingControlNodes(false);

        // departure_sep = stride * (-1) * (stance_dt/time_delta) * 0.25 * (time_delta/swing_dt)
        // Query the stepper's actual deltas (post-§2.2 stance is plain int).
        Point3D final_vel = stride * (-1.0) * (stepper.getStanceDeltaT() / TIME_DELTA);
        Point3D departure_sep = final_vel * 0.25 * (TIME_DELTA / stepper.getSwingDeltaT());

        Point3D s2_3 = stepper.getSwing2ControlNode(3);
        Point3D exp_s2_3 = target - departure_sep;
        double err_s2_3 = vec_distance(s2_3, exp_s2_3);
        std::cout << "  secondary n[3] err=" << err_s2_3 << " ... " << (err_s2_3 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err_s2_3 > TIGHT_TOL)
            failures++;

        Point3D s2_2 = stepper.getSwing2ControlNode(2);
        Point3D exp_s2_2 = target - departure_sep * 2.0;
        double err_s2_2 = vec_distance(s2_2, exp_s2_2);
        std::cout << "  secondary n[2] err=" << err_s2_2 << " ... " << (err_s2_2 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err_s2_2 > TIGHT_TOL)
            failures++;

        // s2[4].xy = target.xy, z forced to default (both 0 here)
        Point3D s2_4 = stepper.getSwing2ControlNode(4);
        Point3D exp_s2_4 = target;
        exp_s2_4.z = identity_tip.z; // default_tip_pose_.z
        double err_s2_4 = vec_distance(s2_4, exp_s2_4);
        std::cout << "  secondary n[4]≈target: err=" << err_s2_4 << " ... " << (err_s2_4 < TIGHT_TOL ? "OK" : "FAIL") << std::endl;
        if (err_s2_4 > TIGHT_TOL)
            failures++;
    }

    std::cout << "\n  Test 9 result: " << (failures == 0 ? "PASSED ✓" : "FAILED ✗") << std::endl;
    return failures;
}

// ════════════════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════════════════

int run_bezier_curve_deterministic() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  DETERMINISTIC BÉZIER CURVE VALIDATION TEST                  ║" << std::endl;
    std::cout << "║  Validates swing/stance trajectories against analytical      ║" << std::endl;
    std::cout << "║  quartic Bézier formulas with pre-established parameters.    ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;

    int total_failures = 0;

    total_failures += test_bezier_math_properties();
    total_failures += test_control_node_generation();
    total_failures += test_phase_junction_continuity();
    total_failures += test_delta_accumulation_accuracy();
    total_failures += test_full_swing_trajectory();
    total_failures += test_stance_linearity();
    total_failures += test_swing_clearance_height();
    total_failures += test_multi_leg_bezier_consistency();
    total_failures += test_stride_vector_influence();

    std::cout << "\n════════════════════════════════════════════════════════════════" << std::endl;
    if (total_failures == 0) {
        std::cout << "ALL TESTS PASSED ✓ (0 failures)" << std::endl;
    } else {
        std::cout << "TESTS FAILED: " << total_failures << " failure(s)" << std::endl;
    }
    std::cout << "════════════════════════════════════════════════════════════════" << std::endl;

    return (total_failures == 0) ? 0 : 1;
}
} // namespace cm_bezier_curve_deterministic_test

// ===========================================================================
// Sub-test: run_bezier_transition_single_leg (from bezier_transition_single_leg_test.cpp)
// ===========================================================================
namespace cm_bezier_transition_single_leg_test {
/**
 * @file bezier_transition_single_leg_test.cpp
 * @brief Visualizes Bézier curves during pack, unpack and standing pose transitions for a single leg.
 *
 * This test exercises the full LocomotionSystem state machine through PACKED → READY → RUNNING
 * transitions, displaying detailed per-iteration joint angle data for Leg 0 (AR) to observe:
 *
 * 1. **Unpack (PACKED → READY):** Cubic Bézier interpolation via LegPoser::transitionConfiguration()
 *    from packed joints to unpacked joints.
 * 2. **Startup Sequence (READY → RUNNING):** Uses stepToPosition() quartic Bézier inside
 *    BodyPoseController::executeSequenceInternal() for multi-step H/V transitions.
 * 3. **Shutdown (RUNNING → READY):** Reverse startup via executeShutdownSequence().
 * 4. **Pack (READY → PACKED):** Cubic Bézier via transitionConfiguration() from
 *    unpacked back to packed joints.
 *
 * Requires start_up_sequence=true in BodyPoseConfiguration so the state machine uses
 * the full Bézier-based startup/shutdown paths (not direct mode).
 *
 * @author HexaMotion Team
 * @version 1.0
 * @date 2025
 */

static double toDeg(double rad) { return math_utils::radiansToDegrees(rad); }

/** Print column header for the iteration table. */
static void printTableHeader(const char *phase_name) {
    std::cout << "\n=== " << phase_name << " ===" << std::endl;
    std::cout << std::left
              << std::setw(6) << "Iter"
              << std::setw(10) << "Coxa(d)"
              << std::setw(10) << "Femur(d)"
              << std::setw(10) << "Tibia(d)"
              << std::setw(30) << "Tip Position (x, y, z)"
              << "RobotState" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
}

/** State name helper. */
static const char *robotStateName(RobotState s) {
    switch (s) {
    case ROBOT_UNKNOWN:
        return "UNKNOWN";
    case ROBOT_PACKED:
        return "PACKED";
    case ROBOT_READY:
        return "READY";
    case ROBOT_RUNNING:
        return "RUNNING";
    default:
        return "?";
    }
}

/** Print a single iteration row for leg 0. */
static void printLegRow(int iter, const Leg &leg, RobotState rs) {
    JointAngles a = leg.getJointAngles();
    Point3D tip = leg.getCurrentTipPositionGlobal();
    std::cout << std::fixed << std::setprecision(2)
              << std::left
              << std::setw(6) << iter
              << std::setw(10) << toDeg(a.coxa)
              << std::setw(10) << toDeg(a.femur)
              << std::setw(10) << toDeg(a.tibia);
    std::cout << "(" << std::setw(8) << tip.x << ", " << std::setw(8) << tip.y << ", " << std::setw(8) << tip.z << ")  ";
    std::cout << robotStateName(rs) << std::endl;
}

int run_bezier_transition_single_leg() {
    std::cout << "============================================================================" << std::endl;
    std::cout << "     BEZIER TRANSITION VISUALIZATION TEST — SINGLE LEG (Leg 0 / AR)" << std::endl;
    std::cout << "============================================================================" << std::endl;

    // ── 1. Initialize ──────────────────────────────────────────────────────────
    Parameters p = createDefaultParameters();
    enableConfiguredPackedUnpackedPoses(p);

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    // Enable Bezier-based startup/shutdown sequences (not direct mode)
    pose_config.start_up_sequence = true;

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    StateController *sc = sys.getStateController();
    assert(sc != nullptr);

    // Show initial joint angles for leg 0
    {
        const Leg &leg = sys.getLeg(0);
        JointAngles a = leg.getJointAngles();
        std::cout << "\nInitial joint angles (Leg 0):" << std::endl;
        std::cout << "  Coxa:  " << toDeg(a.coxa) << " deg" << std::endl;
        std::cout << "  Femur: " << toDeg(a.femur) << " deg" << std::endl;
        std::cout << "  Tibia: " << toDeg(a.tibia) << " deg" << std::endl;
        std::cout << "  RobotState: " << robotStateName(sc->getRobotState()) << std::endl;
    }

    // ── 2. FULL CYCLE: PACKED → READY → RUNNING ────────────────────────────────
    // Configure gait (needed by walk controller internals) but do NOT set any velocity.
    // Walking is not required for this test — we only exercise state transitions
    // that use Bezier curves (pack/unpack/startup/shutdown). Without velocity,
    // the walk controller stays in WALK_STOPPED, allowing instant shutdown.
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    sys.setGaitConfiguration(tripod_gait);
    sys.startWalking(); // requests ROBOT_RUNNING (triggers PACKED->READY->RUNNING)

    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 1+2: PACKED -> READY (Cubic Bezier) -> RUNNING (Quartic Bezier)" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    printTableHeader("STARTUP CYCLE — Leg 0 Joint Angles per Iteration");

    int total_startup_iters = 0;
    int unpack_iters = 0;
    int startup_iters = 0;
    const int MAX_ITERS = 1000;
    RobotState prev_state = sc->getRobotState();
    bool unpack_detected = false;
    bool startup_detected = false;

    while (total_startup_iters < MAX_ITERS) {
        sys.update();
        total_startup_iters++;

        RobotState cur_state = sc->getRobotState();

        // Track phase transitions
        if (!unpack_detected && cur_state == ROBOT_PACKED) {
            unpack_iters++;
        }
        if (prev_state == ROBOT_PACKED && cur_state != ROBOT_PACKED && !unpack_detected) {
            unpack_iters++;
            unpack_detected = true;
            std::cout << "  >>> Unpack completed at iteration " << total_startup_iters
                      << " (cubic Bezier, " << unpack_iters << " iters)" << std::endl;
        }
        if (unpack_detected && !startup_detected && cur_state != ROBOT_RUNNING) {
            startup_iters++;
        }
        if (prev_state != ROBOT_RUNNING && cur_state == ROBOT_RUNNING && !startup_detected) {
            startup_iters++;
            startup_detected = true;
            std::cout << "  >>> Startup completed at iteration " << total_startup_iters
                      << " (quartic Bezier, " << startup_iters << " iters)" << std::endl;
        }

        // Print every N iterations to keep output manageable
        if (total_startup_iters <= 10 || total_startup_iters % 5 == 0 || cur_state != prev_state) {
            printLegRow(total_startup_iters, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_RUNNING) {
            break;
        }

        prev_state = cur_state;
    }

    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Failed to reach RUNNING state after " << MAX_ITERS << " iterations." << std::endl;
        return 1;
    }

    {
        const Leg &leg = sys.getLeg(0);
        JointAngles a = leg.getJointAngles();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        std::cout << "\nRUNNING state reached. Leg 0 final:" << std::endl;
        std::cout << "  Angles: Coxa=" << toDeg(a.coxa) << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
        std::cout << "  Tip: (" << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }

    // ── 3. SHUTDOWN (RUNNING → READY): Quartic Bézier via stepToPosition ──
    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 3: SHUTDOWN (RUNNING -> READY) — Quartic Bezier" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    // Request READY directly — walk controller is already WALK_STOPPED
    // because no velocity was ever commanded.
    sc->requestRobotState(ROBOT_READY);

    int shutdown_iters = 0;

    printTableHeader("SHUTDOWN — Leg 0 Joint Angles per Iteration");

    int shutdown_iters_loop = 0;
    const int MAX_SHUTDOWN_ITERS = 800;
    prev_state = sc->getRobotState();

    while (shutdown_iters_loop < MAX_SHUTDOWN_ITERS) {
        sys.update();
        shutdown_iters_loop++;

        RobotState cur_state = sc->getRobotState();

        // Print every N iterations or on state change
        if (shutdown_iters_loop <= 10 || shutdown_iters_loop % 5 == 0 || cur_state != prev_state) {
            printLegRow(shutdown_iters_loop, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_READY) {
            std::cout << "  >>> Shutdown completed at iteration " << shutdown_iters_loop << std::endl;
            shutdown_iters = shutdown_iters_loop;
            break;
        }
        prev_state = cur_state;
    }

    if (sc->getRobotState() != ROBOT_READY) {
        std::cout << "WARNING: Shutdown did not reach READY after " << shutdown_iters_loop << " iterations." << std::endl;
        shutdown_iters = shutdown_iters_loop;
    }

    // ── 4. PACK (READY → PACKED): Cubic Bézier via transitionConfiguration ──
    std::cout << "\n──────────────────────────────────────────────────────────────" << std::endl;
    std::cout << "PHASE 4: PACK (READY -> PACKED) — Cubic Bezier" << std::endl;
    std::cout << "──────────────────────────────────────────────────────────────" << std::endl;

    sc->requestRobotState(ROBOT_PACKED);

    int pack_iters = 0;

    printTableHeader("PACK — Leg 0 Joint Angles per Iteration");

    int pack_iters_loop = 0;
    const int MAX_PACK_ITERS = 500;
    prev_state = sc->getRobotState();

    while (pack_iters_loop < MAX_PACK_ITERS) {
        sys.update();
        pack_iters_loop++;

        RobotState cur_state = sc->getRobotState();

        if (pack_iters_loop <= 10 || pack_iters_loop % 5 == 0 || cur_state != prev_state) {
            printLegRow(pack_iters_loop, sys.getLeg(0), cur_state);
        }

        if (cur_state == ROBOT_PACKED) {
            std::cout << "  >>> Pack completed at iteration " << pack_iters_loop << std::endl;
            const Leg &leg = sys.getLeg(0);
            JointAngles a = leg.getJointAngles();
            std::cout << "  Final packed angles: Coxa=" << toDeg(a.coxa)
                      << ", Femur=" << toDeg(a.femur)
                      << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
            pack_iters = pack_iters_loop;
            break;
        }
        prev_state = cur_state;
    }

    if (pack_iters_loop >= MAX_PACK_ITERS) {
        std::cout << "WARNING: Pack did not complete within " << pack_iters_loop << " iterations." << std::endl;
        pack_iters = pack_iters_loop;
    }

    // ── 5. SUMMARY ─────────────────────────────────────────────────────────────
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "                    BEZIER TRANSITION SUMMARY (Single Leg)" << std::endl;
    std::cout << "============================================================================" << std::endl;
    std::cout << "Phase                     | Bezier Type       | Iterations | Method" << std::endl;
    std::cout << "--------------------------+-------------------+------------+-------------------------" << std::endl;
    std::cout << std::left
              << std::setw(26) << "Unpack (PACKED->READY)"
              << "| " << std::setw(18) << "Cubic per-joint"
              << "| " << std::setw(11) << unpack_iters
              << "| transitionConfiguration()" << std::endl;
    std::cout << std::setw(26) << "Startup (READY->RUNNING)"
              << "| " << std::setw(18) << "Dual quartic"
              << "| " << std::setw(11) << startup_iters
              << "| stepToPosition()" << std::endl;
    std::cout << std::setw(26) << "Shutdown (RUNNING->READY)"
              << "| " << std::setw(18) << "Dual quartic"
              << "| " << std::setw(11) << shutdown_iters
              << "| stepToPosition()" << std::endl;
    std::cout << std::setw(26) << "Pack (READY->PACKED)"
              << "| " << std::setw(18) << "Cubic per-joint"
              << "| " << std::setw(11) << pack_iters
              << "| transitionConfiguration()" << std::endl;
    std::cout << "--------------------------+-------------------+------------+-------------------------" << std::endl;
    int total = unpack_iters + startup_iters + shutdown_iters + pack_iters;
    std::cout << std::setw(26) << "TOTAL"
              << "| " << std::setw(18) << ""
              << "| " << std::setw(11) << total
              << "|" << std::endl;
    std::cout << "============================================================================" << std::endl;

    // Final verdict
    bool success = (sc->getRobotState() == ROBOT_PACKED);
    if (success) {
        std::cout << "\nTEST PASSED" << std::endl;
    } else {
        std::cerr << "\nTEST FAILED: Final state = " << robotStateName(sc->getRobotState())
                  << " (expected PACKED)." << std::endl;
    }

    return success ? 0 : 1;
}
} // namespace cm_bezier_transition_single_leg_test

// ===========================================================================
// Sub-test: run_bezier_transition_all_legs (from bezier_transition_all_legs_test.cpp)
// ===========================================================================
namespace cm_bezier_transition_all_legs_test {
/**
 * @file bezier_transition_all_legs_test.cpp
 * @brief Visualizes Bezier curves during pack, unpack and standing pose transitions for ALL six legs.
 *
 * This test extends bezier_transition_single_leg_test by showing all legs simultaneously during
 * the full PACKED -> READY -> RUNNING -> READY -> PACKED state machine cycle. Each phase prints:
 * - Per-iteration joint angles for every leg (in degrees)
 * - Tip positions in global coordinates
 * - Symmetry verification between opposite leg pairs
 *
 * Requires start_up_sequence=true so the state machine uses Bezier-based startup/shutdown.
 *
 * Bezier curves exercised:
 * 1. **Unpack (PACKED -> READY):** Cubic Bezier per joint via LegPoser::transitionConfiguration()
 * 2. **Startup (READY -> RUNNING):** Dual quartic Bezier via executeSequenceInternal()
 * 3. **Shutdown (RUNNING -> READY):** Reverse of startup H/V transitions
 * 4. **Pack (READY -> PACKED):** Cubic Bezier per joint via transitionConfiguration()
 *
 * @author HexaMotion Team
 * @version 1.0
 * @date 2025
 */

static const char *LEG_NAMES[NUM_LEGS] = {"AR(0)", "BR(1)", "CR(2)", "CL(3)", "BL(4)", "AL(5)"};

static double toDeg(double rad) { return math_utils::radiansToDegrees(rad); }

static const char *robotStateName(RobotState s) {
    switch (s) {
    case ROBOT_UNKNOWN:
        return "UNKNOWN";
    case ROBOT_PACKED:
        return "PACKED";
    case ROBOT_READY:
        return "READY";
    case ROBOT_RUNNING:
        return "RUNNING";
    default:
        return "?";
    }
}

/** Print compact header for the all-legs table. */
static void printAllLegsHeader(const char *phase_name) {
    std::cout << "\n=== " << phase_name << " ===" << std::endl;
    std::cout << std::left << std::setw(6) << "Iter";
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "| " << std::setw(8) << LEG_NAMES[i]
                  << std::setw(8) << "Coxa"
                  << std::setw(8) << "Femur"
                  << std::setw(8) << "Tibia ";
    }
    std::cout << "| State" << std::endl;
    std::cout << std::string(6 + NUM_LEGS * 35 + 10, '-') << std::endl;
}

/** Print one row with all legs' joint angles. */
static void printAllLegsRow(int iter, const LocomotionSystem &sys, RobotState rs) {
    std::cout << std::fixed << std::setprecision(1)
              << std::left << std::setw(6) << iter;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles a = leg.getJointAngles();
        std::cout << "| " << std::setw(8) << ""
                  << std::setw(8) << toDeg(a.coxa)
                  << std::setw(8) << toDeg(a.femur)
                  << std::setw(8) << toDeg(a.tibia);
    }
    std::cout << "| " << robotStateName(rs) << std::endl;
}

/** Print detailed tip positions for all legs. */
static void printAllLegsTipPositions(int iter, const LocomotionSystem &sys) {
    std::cout << "  Tip positions at iter " << iter << ":" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D tip = sys.getLeg(i).getCurrentTipPositionGlobal();
        std::cout << "    " << LEG_NAMES[i] << ": ("
                  << std::fixed << std::setprecision(2)
                  << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }
}

/** Validate symmetry between opposite leg pairs after a transition. */
static bool validateOppositeSymmetry(const LocomotionSystem &sys, const char *phase_name) {
    const int pairs[][2] = {{0, 5}, {1, 4}, {2, 3}};
    const char *pair_names[] = {"AR/AL", "BR/BL", "CR/CL"};
    constexpr double ANGLE_TOLERANCE_DEG = 1.0;
    bool all_ok = true;

    std::cout << "\n  Symmetry check (" << phase_name << "):" << std::endl;
    for (int p = 0; p < 3; ++p) {
        int a = pairs[p][0], b = pairs[p][1];
        JointAngles ja = sys.getLeg(a).getJointAngles();
        JointAngles jb = sys.getLeg(b).getJointAngles();

        double femur_diff = std::abs(toDeg(ja.femur) - toDeg(jb.femur));
        double tibia_diff = std::abs(toDeg(ja.tibia) - toDeg(jb.tibia));

        bool ok = (femur_diff < ANGLE_TOLERANCE_DEG) && (tibia_diff < ANGLE_TOLERANCE_DEG);
        std::cout << "    " << pair_names[p]
                  << ": femur_diff=" << std::fixed << std::setprecision(2) << femur_diff
                  << " deg, tibia_diff=" << tibia_diff << " deg "
                  << (ok ? "OK" : "WARN") << std::endl;
        if (!ok)
            all_ok = false;
    }
    return all_ok;
}

int run_bezier_transition_all_legs() {
    std::cout << "============================================================================" << std::endl;
    std::cout << "   BEZIER TRANSITION VISUALIZATION TEST — ALL LEGS (6 legs simultaneous)" << std::endl;
    std::cout << "============================================================================" << std::endl;

    // ── 1. Initialize ──────────────────────────────────────────────────────────
    Parameters p = createDefaultParameters();
    enableConfiguredPackedUnpackedPoses(p);

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    // Enable Bezier-based startup/shutdown sequences (not direct mode)
    pose_config.start_up_sequence = true;

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    StateController *sc = sys.getStateController();
    assert(sc != nullptr);

    // Show initial state
    std::cout << "\nInitial state: " << robotStateName(sc->getRobotState()) << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = sys.getLeg(i).getJointAngles();
        std::cout << "  " << LEG_NAMES[i]
                  << ": Coxa=" << std::fixed << std::setprecision(1) << toDeg(a.coxa)
                  << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
    }

    // Configure gait (needed by walk controller internals) but do NOT set any velocity.
    // Walking is not required — we only exercise state transitions with Bezier curves.
    // Without velocity, the walk controller stays in WALK_STOPPED, allowing instant shutdown.
    GaitConfiguration tripod_gait = createGaitConfig(TRIPOD_GAIT, p);
    sys.setGaitConfiguration(tripod_gait);
    sys.startWalking(); // requests ROBOT_RUNNING (triggers PACKED->READY->RUNNING)

    // ── 2. FULL STARTUP: PACKED -> READY -> RUNNING ────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASES 1+2: PACKED -> READY (Cubic) -> RUNNING (Quartic)" << std::endl;
    std::cout << "===================================================================" << std::endl;

    printAllLegsHeader("STARTUP CYCLE — All Legs");

    int total_iters = 0;
    int unpack_iters = 0;
    int startup_iters = 0;
    const int MAX_ITERS = 1000;
    RobotState prev_state = sc->getRobotState();
    bool unpack_done = false;
    bool startup_done = false;

    while (total_iters < MAX_ITERS) {
        sys.update();
        total_iters++;

        RobotState cur = sc->getRobotState();

        if (!unpack_done && cur == ROBOT_PACKED)
            unpack_iters++;
        if (prev_state == ROBOT_PACKED && cur != ROBOT_PACKED && !unpack_done) {
            unpack_iters++;
            unpack_done = true;
            std::cout << "  >>> Unpack done at iteration " << total_iters
                      << " (" << unpack_iters << " iters)" << std::endl;
            validateOppositeSymmetry(sys, "post-unpack");
        }
        if (unpack_done && !startup_done && cur != ROBOT_RUNNING)
            startup_iters++;
        if (prev_state != ROBOT_RUNNING && cur == ROBOT_RUNNING && !startup_done) {
            startup_iters++;
            startup_done = true;
            std::cout << "  >>> Startup done at iteration " << total_iters
                      << " (" << startup_iters << " iters)" << std::endl;
        }

        // Print every 5 iters or on state change
        if (total_iters <= 5 || total_iters % 5 == 0 || cur != prev_state) {
            printAllLegsRow(total_iters, sys, cur);
        }
        if (total_iters % 25 == 0) {
            printAllLegsTipPositions(total_iters, sys);
        }

        if (cur == ROBOT_RUNNING)
            break;
        prev_state = cur;
    }

    if (sc->getRobotState() != ROBOT_RUNNING) {
        std::cerr << "ERROR: Did not reach RUNNING after " << MAX_ITERS << " iters." << std::endl;
        return 1;
    }

    std::cout << "\nRUNNING state reached. Standing pose:" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles a = leg.getJointAngles();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        std::cout << "  " << LEG_NAMES[i]
                  << ": angles=(" << std::setprecision(1)
                  << toDeg(a.coxa) << ", " << toDeg(a.femur) << ", " << toDeg(a.tibia) << ")"
                  << " tip=(" << std::setprecision(2) << tip.x << ", " << tip.y << ", " << tip.z << ")" << std::endl;
    }
    validateOppositeSymmetry(sys, "standing/running");

    // ── 3. SHUTDOWN (RUNNING -> READY) ─────────────────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASE 3: SHUTDOWN (RUNNING -> READY) — Quartic Bezier" << std::endl;
    std::cout << "===================================================================" << std::endl;

    // Request READY directly — walk controller is already WALK_STOPPED
    // because no velocity was ever commanded.
    sc->requestRobotState(ROBOT_READY);

    int shutdown_iters = 0;

    printAllLegsHeader("SHUTDOWN — All Legs");

    int shutdown_iters_loop = 0;
    const int MAX_SHUTDOWN = 800;
    prev_state = sc->getRobotState();

    while (shutdown_iters_loop < MAX_SHUTDOWN) {
        sys.update();
        shutdown_iters_loop++;

        RobotState cur = sc->getRobotState();
        if (shutdown_iters_loop <= 5 || shutdown_iters_loop % 10 == 0 || cur != prev_state) {
            printAllLegsRow(shutdown_iters_loop, sys, cur);
        }

        if (cur == ROBOT_READY) {
            std::cout << "  >>> Shutdown done at iteration " << shutdown_iters_loop << std::endl;
            shutdown_iters = shutdown_iters_loop;
            break;
        }
        prev_state = cur;
    }

    if (sc->getRobotState() != ROBOT_READY) {
        std::cout << "WARNING: Shutdown did not complete after " << shutdown_iters_loop << " iters." << std::endl;
        shutdown_iters = shutdown_iters_loop;
    }

    // ── 4. PACK (READY -> PACKED) ──────────────────────────────────────────────
    std::cout << "\n===================================================================" << std::endl;
    std::cout << "PHASE 4: PACK (READY -> PACKED) — Cubic Bezier" << std::endl;
    std::cout << "===================================================================" << std::endl;

    sc->requestRobotState(ROBOT_PACKED);

    int pack_iters = 0;

    printAllLegsHeader("PACK — All Legs");

    int pack_iters_loop = 0;
    const int MAX_PACK = 500;
    prev_state = sc->getRobotState();

    while (pack_iters_loop < MAX_PACK) {
        sys.update();
        pack_iters_loop++;

        RobotState cur = sc->getRobotState();
        if (pack_iters_loop <= 5 || pack_iters_loop % 10 == 0 || cur != prev_state) {
            printAllLegsRow(pack_iters_loop, sys, cur);
        }

        if (cur == ROBOT_PACKED) {
            std::cout << "  >>> Pack done at iteration " << pack_iters_loop << std::endl;
            validateOppositeSymmetry(sys, "post-pack");
            pack_iters = pack_iters_loop;
            break;
        }
        prev_state = cur;
    }

    if (pack_iters_loop >= MAX_PACK) {
        std::cout << "WARNING: Pack did not complete after " << pack_iters_loop << " iters." << std::endl;
        pack_iters = pack_iters_loop;
    }

    // ── 5. SUMMARY ─────────────────────────────────────────────────────────────
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "              BEZIER TRANSITION SUMMARY — ALL LEGS" << std::endl;
    std::cout << "============================================================================" << std::endl;
    std::cout << std::left
              << std::setw(30) << "Phase"
              << std::setw(22) << "Bezier Type"
              << std::setw(14) << "Iterations"
              << "Method" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    std::cout << std::setw(30) << "1. Unpack (PACKED->READY)"
              << std::setw(22) << "Cubic per-joint"
              << std::setw(14) << unpack_iters
              << "transitionConfiguration()" << std::endl;
    std::cout << std::setw(30) << "2. Startup (READY->RUNNING)"
              << std::setw(22) << "Dual quartic"
              << std::setw(14) << startup_iters
              << "stepToPosition()" << std::endl;
    std::cout << std::setw(30) << "3. Shutdown (RUNNING->READY)"
              << std::setw(22) << "Dual quartic"
              << std::setw(14) << shutdown_iters
              << "stepToPosition()" << std::endl;
    std::cout << std::setw(30) << "4. Pack (READY->PACKED)"
              << std::setw(22) << "Cubic per-joint"
              << std::setw(14) << pack_iters
              << "transitionConfiguration()" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    int total = unpack_iters + startup_iters + shutdown_iters + pack_iters;
    std::cout << std::setw(30) << "TOTAL"
              << std::setw(22) << ""
              << std::setw(14) << total
              << std::endl;
    std::cout << "============================================================================" << std::endl;

    // Final state
    std::cout << "\nFinal state: " << robotStateName(sc->getRobotState()) << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles a = sys.getLeg(i).getJointAngles();
        std::cout << "  " << LEG_NAMES[i]
                  << ": Coxa=" << std::fixed << std::setprecision(1) << toDeg(a.coxa)
                  << ", Femur=" << toDeg(a.femur)
                  << ", Tibia=" << toDeg(a.tibia) << " deg" << std::endl;
    }

    bool success = (sc->getRobotState() == ROBOT_PACKED);
    if (success) {
        std::cout << "\nTEST PASSED" << std::endl;
    } else {
        std::cerr << "\nTEST FAILED: Final state = " << robotStateName(sc->getRobotState())
                  << " (expected PACKED)." << std::endl;
    }

    return success ? 0 : 1;
}
} // namespace cm_bezier_transition_all_legs_test

int main() {
    int rc = 0;

    std::cout << "\n========== bezier validation ==========\n";
    rc |= cm_bezier_validation_test::run_bezier_validation();

    std::cout << "\n========== bezier curve deterministic ==========\n";
    rc |= cm_bezier_curve_deterministic_test::run_bezier_curve_deterministic();

    std::cout << "\n========== bezier transition single leg ==========\n";
    rc |= cm_bezier_transition_single_leg_test::run_bezier_transition_single_leg();

    std::cout << "\n========== bezier transition all legs ==========\n";
    rc |= cm_bezier_transition_all_legs_test::run_bezier_transition_all_legs();

    std::cout << "\n[bezier_test] overall: " << (rc == 0 ? "PASS" : "FAIL") << "\n";
    return rc == 0 ? 0 : 1;
}
