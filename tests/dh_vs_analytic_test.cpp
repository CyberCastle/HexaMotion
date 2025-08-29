#include "analytic_robot_model.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.default_height_offset = -208.0; // Set to -tibia_length for explicit configuration
    p.robot_height = 208;
    p.time_delta = 1.0 / 50.0; // 50 Hz
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    model.workspaceAnalyzerInitializer(); // Inicializar WorkspaceAnalyzer
    AnalyticRobotModel analytic_model(p);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== DH vs Analytic Methods Comparison Test ===" << std::endl;

    // Define BASE_THETA_OFFSETS for the test (in radians)
    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {
        -30.0 * M_PI / 180.0, -90.0 * M_PI / 180.0, -150.0 * M_PI / 180.0,
        30.0 * M_PI / 180.0, 90.0 * M_PI / 180.0, 150.0 * M_PI / 180.0};

    bool ok = true;

    // Test 1: Compare forward kinematics methods
    std::cout << "\n--- Test 1: Forward Kinematics Comparison ---" << std::endl;

    // Test with zero angles
    JointAngles zero_angles(0, 0, 0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D analytic_pos = analytic_model.forwardKinematicsGlobalCoordinatesAnalytic(leg, zero_angles);
        Point3D dh_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        double error = std::sqrt(std::pow(analytic_pos.x - dh_pos.x, 2) +
                                 std::pow(analytic_pos.y - dh_pos.y, 2) +
                                 std::pow(analytic_pos.z - dh_pos.z, 2));

        std::cout << "Leg " << leg << " (zero angles): analytic(" << analytic_pos.x << ", " << analytic_pos.y << ", " << analytic_pos.z
                  << ") DH(" << dh_pos.x << ", " << dh_pos.y << ", " << dh_pos.z << ") error=" << error << std::endl;

        if (error > 1e-6) {
            ok = false;
        }
    }

    // Test with non-zero angles
    JointAngles test_angles(15.0 * M_PI / 180.0, -30.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D analytic_pos = analytic_model.forwardKinematicsGlobalCoordinatesAnalytic(leg, test_angles);
        Point3D dh_pos = model.forwardKinematicsGlobalCoordinates(leg, test_angles);

        double error = std::sqrt(std::pow(analytic_pos.x - dh_pos.x, 2) +
                                 std::pow(analytic_pos.y - dh_pos.y, 2) +
                                 std::pow(analytic_pos.z - dh_pos.z, 2));

        std::cout << "Leg " << leg << " (test angles): analytic(" << analytic_pos.x << ", " << analytic_pos.y << ", " << analytic_pos.z
                  << ") DH(" << dh_pos.x << ", " << dh_pos.y << ", " << dh_pos.z << ") error=" << error << std::endl;

        if (error > 1e-6) {
            ok = false;
        }
    }

    // Test 2: Compare Jacobian methods
    std::cout << "\n--- Test 2: Jacobian Comparison ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Eigen::Matrix3d analytic_jacobian = analytic_model.calculateJacobianAnalytic(leg, test_angles, Point3D());
        Eigen::Matrix3d dh_jacobian = model.calculateJacobian(leg, test_angles, Point3D());

        double max_error = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double error = std::abs(analytic_jacobian(i, j) - dh_jacobian(i, j));
                max_error = std::max(max_error, error);
            }
        }

        std::cout << "Leg " << leg << " Jacobian max_error=" << max_error << std::endl;

        if (max_error > 1e-6) {
            ok = false;
        }
    }

    // Test 3: Compare transform matrices
    std::cout << "\n--- Test 3: Transform Matrix Comparison ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Eigen::Matrix4d analytic_transform = analytic_model.legTransformAnalytic(leg, test_angles);
        Eigen::Matrix4d dh_transform = model.legTransform(leg, test_angles);

        double max_error = 0.0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double error = std::abs(analytic_transform(i, j) - dh_transform(i, j));
                max_error = std::max(max_error, error);
            }
        }

        std::cout << "Leg " << leg << " Transform max_error=" << max_error << std::endl;

        if (max_error > 1e-6) {
            ok = false;
        }
    }

    // Test 4: Compare DH transform building methods
    std::cout << "\n--- Test 4: DH Transform Building Comparison ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::vector<Eigen::Matrix4d> analytic_transforms = analytic_model.buildDHTransformsAnalytic(leg, test_angles);
        std::vector<Eigen::Matrix4d> dh_transforms = model.buildDHTransforms(leg, test_angles);

        double max_error = 0.0;
        for (size_t t = 0; t < analytic_transforms.size(); ++t) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    double error = std::abs(analytic_transforms[t](i, j) - dh_transforms[t](i, j));
                    max_error = std::max(max_error, error);
                }
            }
        }

        std::cout << "Leg " << leg << " DH Transforms max_error=" << max_error << std::endl;

        if (max_error > 1e-6) {
            ok = false;
        }
    }

    // Test 5: Validate DH parameters are correctly used
    std::cout << "\n--- Test 5: DH Parameters Validation ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Check that DH parameters are correctly initialized
        double base_angle = model.getLegBaseAngleOffset(leg);
        double expected_angle = BASE_THETA_OFFSETS[leg];

        double angle_error = std::abs(base_angle - expected_angle);
        std::cout << "Leg " << leg << " base angle: DH=" << base_angle << " expected=" << expected_angle
                  << " error=" << angle_error << std::endl;

        if (angle_error > 1e-6) {
            ok = false;
        }
    }

    // Test 6: Compare leg base position methods
    std::cout << "\n--- Test 6: Leg Base Position Comparison ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D analytic_base_pos = analytic_model.getAnalyticLegBasePosition(leg);
        Point3D dh_base_pos = model.getLegBasePosition(leg);

        double error = std::sqrt(std::pow(analytic_base_pos.x - dh_base_pos.x, 2) +
                                 std::pow(analytic_base_pos.y - dh_base_pos.y, 2) +
                                 std::pow(analytic_base_pos.z - dh_base_pos.z, 2));

        std::cout << "Leg " << leg << " base position: analytic(" << analytic_base_pos.x << ", "
                  << analytic_base_pos.y << ", " << analytic_base_pos.z
                  << ") DH(" << dh_base_pos.x << ", " << dh_base_pos.y << ", " << dh_base_pos.z
                  << ") error=" << error << std::endl;

        if (error > 1e-6) {
            std::cout << "  ⚠️  Base position mismatch for leg " << leg << std::endl;
            ok = false;
        }

        // Verify expected position based on hexagon geometry
        double expected_angle = BASE_THETA_OFFSETS[leg];
        double expected_x = p.hexagon_radius * cos(expected_angle);
        double expected_y = p.hexagon_radius * sin(expected_angle);
        double expected_z = 0.0;

        double analytic_error = std::sqrt(std::pow(analytic_base_pos.x - expected_x, 2) +
                                          std::pow(analytic_base_pos.y - expected_y, 2) +
                                          std::pow(analytic_base_pos.z - expected_z, 2));

        std::cout << "  Expected(" << expected_x << ", " << expected_y << ", " << expected_z
                  << ") analytic_error=" << analytic_error << std::endl;

        if (analytic_error > 1e-6) {
            std::cout << "  ⚠️  Analytic base position doesn't match expected geometry for leg " << leg << std::endl;
            ok = false;
        }

        // Additional validation: verify hexagon properties
        double distance_from_origin = std::sqrt(analytic_base_pos.x * analytic_base_pos.x +
                                                analytic_base_pos.y * analytic_base_pos.y);
        double distance_error = std::abs(distance_from_origin - p.hexagon_radius);

        std::cout << "  Distance from origin=" << distance_from_origin
                  << " (expected=" << p.hexagon_radius << ") error=" << distance_error << std::endl;

        if (distance_error > 1e-6) {
            std::cout << "  ⚠️  Distance from origin incorrect for leg " << leg << std::endl;
            ok = false;
        }

        // Verify angle calculation
        double calculated_angle = atan2(analytic_base_pos.y, analytic_base_pos.x);
        double angle_difference = std::abs(calculated_angle - expected_angle);

        // Handle angle wraparound (difference should be < π)
        if (angle_difference > M_PI) {
            angle_difference = 2.0 * M_PI - angle_difference;
        }

        std::cout << "  Calculated angle=" << calculated_angle * 180.0 / M_PI
                  << "° (expected=" << expected_angle * 180.0 / M_PI
                  << "°) error=" << angle_difference * 180.0 / M_PI << "°" << std::endl;

        if (angle_difference > 1e-6) {
            std::cout << "  ⚠️  Angle calculation incorrect for leg " << leg << std::endl;
            ok = false;
        }
    }

    // Test 7: Verify hexagon symmetry properties
    std::cout << "\n--- Test 7: Hexagon Symmetry Validation ---" << std::endl;

    // Check that opposite legs are symmetric (leg pairs: 0-3, 1-4, 2-5)
    int leg_pairs[3][2] = {{0, 3}, {1, 4}, {2, 5}};

    for (int p_idx = 0; p_idx < 3; ++p_idx) {
        int leg1 = leg_pairs[p_idx][0];
        int leg2 = leg_pairs[p_idx][1];

        Point3D pos1 = analytic_model.getAnalyticLegBasePosition(leg1);
        Point3D pos2 = analytic_model.getAnalyticLegBasePosition(leg2);

        // Opposite legs should be symmetric about origin
        double symmetry_error_x = std::abs(pos1.x + pos2.x);
        double symmetry_error_y = std::abs(pos1.y + pos2.y);
        double symmetry_error_z = std::abs(pos1.z - pos2.z);

        std::cout << "Leg pair (" << leg1 << "," << leg2 << ") symmetry errors: "
                  << "x=" << symmetry_error_x << " y=" << symmetry_error_y
                  << " z=" << symmetry_error_z << std::endl;

        if (symmetry_error_x > 1e-6 || symmetry_error_y > 1e-6 || symmetry_error_z > 1e-6) {
            std::cout << "  ⚠️  Symmetry violation for leg pair (" << leg1 << "," << leg2 << ")" << std::endl;
            ok = false;
        }
    }

    // Test 8: Verify 60-degree spacing between adjacent legs
    std::cout << "\n--- Test 8: Adjacent Leg Spacing Validation ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        int next_leg = (leg + 1) % NUM_LEGS;

        Point3D pos1 = analytic_model.getAnalyticLegBasePosition(leg);
        Point3D pos2 = analytic_model.getAnalyticLegBasePosition(next_leg);

        double angle1 = atan2(pos1.y, pos1.x);
        double angle2 = atan2(pos2.y, pos2.x);

        double angle_diff = angle2 - angle1;

        // Normalize angle difference to [-π, π]
        while (angle_diff > M_PI)
            angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0 * M_PI;

        double expected_diff = -60.0 * M_PI / 180.0; // -60 degrees (clockwise)
        double spacing_error = std::abs(angle_diff - expected_diff);

        std::cout << "Legs " << leg << "->" << next_leg << " angle difference="
                  << angle_diff * 180.0 / M_PI << "° (expected="
                  << expected_diff * 180.0 / M_PI << "°) error="
                  << spacing_error * 180.0 / M_PI << "°" << std::endl;

        if (spacing_error > 1e-6) {
            std::cout << "  ⚠️  Incorrect spacing between legs " << leg << " and " << next_leg << std::endl;
            ok = false;
        }
    }

    if (ok) {
        std::cout << "\n✓ All DH vs Analytic comparison tests passed!" << std::endl;
        std::cout << "✓ DH-based methods are equivalent to analytic methods!" << std::endl;
        std::cout << "✓ Leg base positions match expected hexagon geometry!" << std::endl;
        std::cout << "✓ Hexagon symmetry properties validated!" << std::endl;
        std::cout << "✓ 60-degree leg spacing verified!" << std::endl;
        std::cout << "✓ AnalyticRobotModel::getAnalyticLegBasePosition is mathematically correct!" << std::endl;
        return 0;
    } else {
        std::cerr << "\n✗ Some DH vs Analytic comparison tests failed." << std::endl;
        return 1;
    }
}