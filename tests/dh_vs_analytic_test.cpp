#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include "analytic_robot_model.h"

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    AnalyticRobotModel analytic_model(p);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== DH vs Analytic Methods Comparison Test ===" << std::endl;

    // Define BASE_THETA_OFFSETS for the test (in radians)
    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {
        -30.0 * M_PI / 180.0, -90.0 * M_PI / 180.0, -150.0 * M_PI / 180.0,
        150.0 * M_PI / 180.0, 90.0 * M_PI / 180.0, 30.0 * M_PI / 180.0
    };

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

    if (ok) {
        std::cout << "\n✓ All DH vs Analytic comparison tests passed!" << std::endl;
        std::cout << "✓ DH-based methods are equivalent to analytic methods!" << std::endl;
        return 0;
    } else {
        std::cerr << "\n✗ Some DH vs Analytic comparison tests failed." << std::endl;
        return 1;
    }
}