#include <iostream>
#include <iomanip>
#include <cmath>
#include "../src/robot_model.h"
#include "../src/hexamotion_constants.h"

using namespace std;

int main() {
    cout << "=== DH Base Position Validation Test ===" << endl << endl;

    // Create robot model with default parameters
    Parameters params;
    params.hexagon_radius = 200.0;
    params.coxa_length = 50.0;
    params.femur_length = 101.0;
    params.tibia_length = 208.0;
    params.robot_height = 208.0;
    params.robot_weight = 6.5;
    params.center_of_mass = Eigen::Vector3d(0.0, 0.0, 0.0);
    params.coxa_angle_limits[0] = -90.0;
    params.coxa_angle_limits[1] = 90.0;
    params.femur_angle_limits[0] = -45.0;
    params.femur_angle_limits[1] = 45.0;
    params.tibia_angle_limits[0] = -90.0;
    params.tibia_angle_limits[1] = 90.0;
    params.max_velocity = 100.0;
    params.max_angular_velocity = 30.0;
    params.stability_margin = 0.1;
    params.control_frequency = 50.0;
    params.use_custom_dh_parameters = false;

    RobotModel model(params);

    cout << "--- Test: DH Base Position vs Analytic Base Position ---" << endl;

    bool all_tests_passed = true;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D dh_base = model.getDHLegBasePosition(leg);
        Point3D analytic_base = model.getAnalyticLegBasePosition(leg);

        double error = sqrt(pow(dh_base.x - analytic_base.x, 2) +
                           pow(dh_base.y - analytic_base.y, 2) +
                           pow(dh_base.z - analytic_base.z, 2));

        cout << "Leg " << leg << ":" << endl;
        cout << "  DH Base:      (" << fixed << setprecision(3) << dh_base.x
             << ", " << dh_base.y << ", " << dh_base.z << ")" << endl;
        cout << "  Analytic Base: (" << fixed << setprecision(3) << analytic_base.x
             << ", " << analytic_base.y << ", " << analytic_base.z << ")" << endl;
        cout << "  Error: " << fixed << setprecision(6) << error << endl;

        if (error > 1e-6) {
            cout << "  ❌ FAILED - Error too large!" << endl;
            all_tests_passed = false;
        } else {
            cout << "  ✓ PASSED" << endl;
        }
        cout << endl;
    }

        cout << "--- Test: DH Base Position with Zero Joint Angles ---" << endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles zero_angles(0.0, 0.0, 0.0);
        Point3D dh_base = model.getDHLegBasePosition(leg);
        Point3D fk_base = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        // With zero joint angles, the FK should include:
        // 1. Base position (from DH row 0)
        // 2. Coxa length in the base direction (from DH row 2)
        // 3. Femur length forward (from DH row 3)
        // 4. Tibia length down (from DH row 3)
        double base_angle = model.getLegBaseAngleOffset(leg);
        double expected_x = dh_base.x + params.coxa_length * cos(base_angle) + params.femur_length * cos(base_angle);
        double expected_y = dh_base.y + params.coxa_length * sin(base_angle) + params.femur_length * sin(base_angle);
        double expected_z = -params.tibia_length; // Full leg extension down

        double error = sqrt(pow(fk_base.x - expected_x, 2) +
                           pow(fk_base.y - expected_y, 2) +
                           pow(fk_base.z - expected_z, 2));

        cout << "Leg " << leg << ":" << endl;
        cout << "  DH Base:      (" << fixed << setprecision(3) << dh_base.x
             << ", " << dh_base.y << ", " << dh_base.z << ")" << endl;
        cout << "  FK Zero:      (" << fixed << setprecision(3) << fk_base.x
             << ", " << fk_base.y << ", " << fk_base.z << ")" << endl;
        cout << "  Expected FK:  (" << fixed << setprecision(3) << expected_x
             << ", " << expected_y << ", " << expected_z << ")" << endl;
        cout << "  Error: " << fixed << setprecision(6) << error << endl;

        if (error > 1e-3) { // Allow slightly larger tolerance for FK calculation
            cout << "  ❌ FAILED - Error too large!" << endl;
            all_tests_passed = false;
        } else {
            cout << "  ✓ PASSED" << endl;
        }
        cout << endl;
    }

    cout << "--- Test: Base Angle Offset Consistency ---" << endl;

    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {
        -30.0 * DEGREES_TO_RADIANS_FACTOR,
        -90.0 * DEGREES_TO_RADIANS_FACTOR,
        -150.0 * DEGREES_TO_RADIANS_FACTOR,
        150.0 * DEGREES_TO_RADIANS_FACTOR,
        90.0 * DEGREES_TO_RADIANS_FACTOR,
        30.0 * DEGREES_TO_RADIANS_FACTOR
    };

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        double dh_angle = model.getLegBaseAngleOffset(leg);
        double expected_angle = BASE_THETA_OFFSETS[leg];

        double error = abs(dh_angle - expected_angle);

        cout << "Leg " << leg << ":" << endl;
        cout << "  DH Angle:     " << fixed << setprecision(6) << dh_angle << " rad" << endl;
        cout << "  Expected:     " << fixed << setprecision(6) << expected_angle << " rad" << endl;
        cout << "  Error: " << fixed << setprecision(6) << error << " rad" << endl;

        if (error > 1e-6) {
            cout << "  ❌ FAILED - Error too large!" << endl;
            all_tests_passed = false;
        } else {
            cout << "  ✓ PASSED" << endl;
        }
        cout << endl;
    }

    if (all_tests_passed) {
        cout << "✓ All DH base position tests passed!" << endl;
        return 0;
    } else {
        cout << "❌ Some tests failed!" << endl;
        return 1;
    }
}