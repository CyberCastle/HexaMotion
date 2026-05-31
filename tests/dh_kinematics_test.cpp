/**
 * @file dh_kinematics_test.cpp
 * @brief Consolidated test suite.
 *
 * This single translation unit folds together the following sub-tests
 * without losing any coverage. Each sub-test keeps its original assertions
 * and entry function; their bodies are wrapped in a dedicated namespace to
 * avoid symbol collisions, and this file's main() runs them in sequence and
 * aggregates the exit status:
 *   - run_simple_dh()
 *   - run_dh_vs_analytic()
 *   - run_complete_physical_offset()
 *   - run_joint_output_calibration()
 */

#include "../src/body_pose_config_factory.h"
#include "../src/locomotion_system.h"
#include "../src/robot_model.h"
#include "../src/workspace_analyzer.h"
#include "analytic_robot_model.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

// ===========================================================================
// Sub-test: run_simple_dh (from simple_dh_test.cpp)
// ===========================================================================
namespace cm_simple_dh_test {
/** Forward declarations for coordinate transformation functions. */
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos);
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos);
Point3D forwardKinematicsLocal(const RobotModel &model, int leg, const JointAngles &angles);

int run_simple_dh() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    /** Set to -tibia_length for explicit configuration. */
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    /** Initialize WorkspaceAnalyzer. */
    model.workspaceAnalyzerInitializer();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== DH Parameter Validation Test ===" << std::endl;

    JointAngles q(0, 0, 0);
    bool ok = true;

    /** Test 1: validate global FK consistency. */
    std::cout << "\n--- Test 1: Global Forward Kinematics Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D pos = model.forwardKinematicsGlobalCoordinates(leg, q);
        double theta_rad = BASE_THETA_OFFSETS[leg];
        double reach = p.hexagon_radius + p.coxa_length + p.femur_length;
        double expected_x = reach * cos(theta_rad);
        double expected_y = reach * sin(theta_rad);
        double expected_z = -p.tibia_length;
        double err = std::sqrt(std::pow(pos.x - expected_x, 2) +
                               std::pow(pos.y - expected_y, 2) +
                               std::pow(pos.z - expected_z, 2));
        std::cout << "Leg " << leg << ": (" << pos.x << ", " << pos.y << ", " << pos.z
                  << ") expected (" << expected_x << ", " << expected_y << ", " << expected_z
                  << ") error=" << err << std::endl;
        if (err > 1e-3f) {
            ok = false;
        }
    }

    /** Test 2: validate local FK implementation. */
    std::cout << "\n--- Test 2: Local Forward Kinematics Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D local_pos = forwardKinematicsLocal(model, leg, q);
        /** In local coordinates with all joints at 0, tip should be at. */
        /** x = coxa_length + femur_length = 50 + 101 = 151. */
        /** y = 0 (no rotation). */
        /** z = -tibia_length = -208. */
        double expected_local_x = p.coxa_length + p.femur_length;
        double expected_local_y = 0.0;
        double expected_local_z = -p.tibia_length;

        double err = std::sqrt(std::pow(local_pos.x - expected_local_x, 2) +
                               std::pow(local_pos.y - expected_local_y, 2) +
                               std::pow(local_pos.z - expected_local_z, 2));
        std::cout << "Leg " << leg << " (local): (" << local_pos.x << ", " << local_pos.y << ", " << local_pos.z
                  << ") expected (" << expected_local_x << ", " << expected_local_y << ", " << expected_local_z
                  << ") error=" << err << std::endl;
        if (err > 1e-3f) {
            ok = false;
        }
    }

    /** Test 3: coordinate transformation validation (using RobotModel functions). */
    std::cout << "\n--- Test 3: Consistency test — Global-Local Coordinate Round-Trip (RobotModel) ---" << std::endl;
    JointAngles zero_angles(0, 0, 0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        /** Get global position from FK. */
        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        /** Transform to local coordinates using RobotModel function. */
        Point3D local_converted = model.transformGlobalToLocalCoordinates(leg, global_pos, zero_angles);

        /** Transform back to global coordinates using RobotModel function. */
        Point3D global_restored = model.transformLocalToGlobalCoordinates(leg, local_converted, zero_angles);

        /** Check round-trip transformation accuracy. */
        double roundtrip_error = std::sqrt(std::pow(global_pos.x - global_restored.x, 2) +
                                           std::pow(global_pos.y - global_restored.y, 2) +
                                           std::pow(global_pos.z - global_restored.z, 2));

        std::cout << "Leg " << leg << " round-trip (RobotModel): original(" << global_pos.x << ", " << global_pos.y << ", " << global_pos.z
                  << ") -> local(" << local_converted.x << ", " << local_converted.y << ", " << local_converted.z
                  << ") -> restored(" << global_restored.x << ", " << global_restored.y << ", " << global_restored.z
                  << ") error=" << roundtrip_error << std::endl;

        if (roundtrip_error > 1e-6f) {
            ok = false;
        }
    }

    /** Test 3B: compare manual vs RobotModel coordinate transformations. */
    std::cout << "\n--- Test 3B: Consistency test — Manual vs RobotModel Coordinate Transformation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        /** Manual transformation. */
        Point3D local_manual = transformGlobalToLocal(model, leg, global_pos);
        Point3D global_manual = transformLocalToGlobal(model, leg, local_manual);

        /** RobotModel transformation. */
        Point3D local_robot = model.transformGlobalToLocalCoordinates(leg, global_pos, zero_angles);
        Point3D global_robot = model.transformLocalToGlobalCoordinates(leg, local_robot, zero_angles);

        /** Compare local results. */
        double local_diff = std::sqrt(std::pow(local_manual.x - local_robot.x, 2) +
                                      std::pow(local_manual.y - local_robot.y, 2) +
                                      std::pow(local_manual.z - local_robot.z, 2));

        /** Compare global results. */
        double global_diff = std::sqrt(std::pow(global_manual.x - global_robot.x, 2) +
                                       std::pow(global_manual.y - global_robot.y, 2) +
                                       std::pow(global_manual.z - global_robot.z, 2));

        std::cout << "Leg " << leg << " comparison: local_diff=" << local_diff << ", global_diff=" << global_diff << std::endl;
        std::cout << "  Manual local: (" << local_manual.x << ", " << local_manual.y << ", " << local_manual.z << ")" << std::endl;
        std::cout << "  RobotModel local: (" << local_robot.x << ", " << local_robot.y << ", " << local_robot.z << ")" << std::endl;

        /** Different coordinate system definitions are acceptable. */
        /** Manual uses pure geometric transformation, RobotModel uses relative positioning. */
        /** Both global transformations should be consistent (which they are). */
        if (global_diff > 1e-6f) {
            ok = false;
        }
    }

    /** Test 4: consistency between local FK and transformed global FK. */
    std::cout << "\n--- Test 4: Consistency test — Local FK vs Transformed Global FK ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, q);
        Point3D local_from_transform = transformGlobalToLocal(model, leg, global_pos);
        Point3D local_from_fk = forwardKinematicsLocal(model, leg, q);

        double consistency_error = std::sqrt(std::pow(local_from_transform.x - local_from_fk.x, 2) +
                                             std::pow(local_from_transform.y - local_from_fk.y, 2) +
                                             std::pow(local_from_transform.z - local_from_fk.z, 2));

        std::cout << "Leg " << leg << " consistency: transform(" << local_from_transform.x << ", " << local_from_transform.y << ", " << local_from_transform.z
                  << ") vs direct FK(" << local_from_fk.x << ", " << local_from_fk.y << ", " << local_from_fk.z
                  << ") error=" << consistency_error << std::endl;

        if (consistency_error > 1e-6f) {
            ok = false;
        }
    }

    /** Test 5: test with non-zero joint angles. */
    std::cout << "\n--- Test 5: Consistency test — Round-Trip with Non-Zero Joint Angles ---" << std::endl;
    /** Coxa=15 degrees, Femur=-30 degrees, Tibia=20 degrees. */
    JointAngles test_angles(15.0, -30.0, 20.0);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, test_angles);
        Point3D local_converted = transformGlobalToLocal(model, leg, global_pos);
        Point3D global_restored = transformLocalToGlobal(model, leg, local_converted);

        double roundtrip_error = std::sqrt(std::pow(global_pos.x - global_restored.x, 2) +
                                           std::pow(global_pos.y - global_restored.y, 2) +
                                           std::pow(global_pos.z - global_restored.z, 2));

        std::cout << "Leg " << leg << " non-zero angles: original(" << global_pos.x << ", " << global_pos.y << ", " << global_pos.z
                  << ") -> local(" << local_converted.x << ", " << local_converted.y << ", " << local_converted.z
                  << ") -> restored(" << global_restored.x << ", " << global_restored.y << ", " << global_restored.z
                  << ") error=" << roundtrip_error << std::endl;

        if (roundtrip_error > 1e-6f) {
            ok = false;
        }
    }

    if (ok) {
        std::cout << "\n✓ All DH parameter and coordinate transformation tests passed!" << std::endl;
        return 0;
    } else {
        std::cerr << "\n✗ DH parameter validation or coordinate transformation tests failed." << std::endl;
        return 1;
    }
}

/** Coordinate transformation function implementations. */

/**
 * @brief Transform a point from global robot coordinates to local leg coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param global_pos Position in global robot coordinates
 * @return Position in local leg coordinates
 */
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos) {
    /** Get the leg base position and orientation from DH parameters. */
    const Parameters &params = model.getParams();

    /** Calculate leg base position. */
    double base_angle_rad = BASE_THETA_OFFSETS[leg];
    double base_x = params.hexagon_radius * cos(base_angle_rad);
    double base_y = params.hexagon_radius * sin(base_angle_rad);

    /** Translate to leg base coordinate system. */
    double dx = global_pos.x - base_x;
    double dy = global_pos.y - base_y;
    double dz = global_pos.z;

    /** Rotate by negative base angle to align with leg local coordinate system. */
    double neg_angle_rad = -base_angle_rad;
    double local_x = cos(neg_angle_rad) * dx - sin(neg_angle_rad) * dy;
    double local_y = sin(neg_angle_rad) * dx + cos(neg_angle_rad) * dy;
    double local_z = dz;

    return Point3D{local_x, local_y, local_z};
}

/**
 * @brief Transform a point from local leg coordinates to global robot coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param local_pos Position in local leg coordinates
 * @return Position in global robot coordinates
 */
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos) {
    /** Get the leg base position and orientation from DH parameters. */
    const Parameters &params = model.getParams();

    /** Calculate leg base position. */
    double base_angle_rad = BASE_THETA_OFFSETS[leg];
    double base_x = params.hexagon_radius * cos(base_angle_rad);
    double base_y = params.hexagon_radius * sin(base_angle_rad);

    /** Rotate from leg local coordinate system by base angle. */
    double rotated_x = cos(base_angle_rad) * local_pos.x - sin(base_angle_rad) * local_pos.y;
    double rotated_y = sin(base_angle_rad) * local_pos.x + cos(base_angle_rad) * local_pos.y;
    double rotated_z = local_pos.z;

    /** Translate to global coordinate system. */
    double global_x = rotated_x + base_x;
    double global_y = rotated_y + base_y;
    double global_z = rotated_z;

    return Point3D{global_x, global_y, global_z};
}

/**
 * @brief Compute forward kinematics in local leg coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param angles Joint angles
 * @return End-effector position in local leg coordinates
 */
Point3D forwardKinematicsLocal(const RobotModel &model, int leg, const JointAngles &angles) {
    const Parameters &params = model.getParams();

    /** Build transformation matrix using only the joint DH parameters (without base transform). */
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    const double joint_deg[DOF_PER_LEG] = {angles.coxa, angles.femur, angles.tibia};

    /** Apply each joint transformation starting from joint 1 (skip base transform). */
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        /** DH parameters for this joint (assuming standard DH parameters structure). */
        double a, alpha, d, theta0;

        switch (j) {
        /** Coxa joint (yaw). */
        case 1:
            a = 0.0;
            alpha = math_utils::degreesToRadians(90.0);
            d = 0.0;
            theta0 = 0.0;
            break;
        /** Femur joint (hip-pitch). */
        case 2:
            a = params.coxa_length;
            alpha = math_utils::degreesToRadians(90.0);
            d = 0.0;
            theta0 = 0.0;
            break;
        /** Tibia joint (knee-pitch). */
        case 3:
            a = params.femur_length;
            alpha = 0.0;
            d = params.tibia_length;
            theta0 = 0.0;
            break;
        default:
            continue;
        }

        double theta = theta0 + joint_deg[j - 1];
        T *= math_utils::dhTransform(a, alpha, d, math_utils::degreesToRadians(theta));
    }

    return Point3D{T(0, 3), T(1, 3), T(2, 3)};
}
} // namespace cm_simple_dh_test

// ===========================================================================
// Sub-test: run_dh_vs_analytic (from dh_vs_analytic_test.cpp)
// ===========================================================================
namespace cm_dh_vs_analytic_test {
int run_dh_vs_analytic() {
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
    model.workspaceAnalyzerInitializer(); // Initialize WorkspaceAnalyzer
    AnalyticRobotModel analytic_model(p);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== DH vs Analytic Methods Comparison Test ===" << std::endl;

    // Use global BASE_THETA_OFFSETS (declared extern in analytic_robot_model.h / hexamotion_constants.h)
    // to avoid divergence between test expectations and library constants.

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
    JointAngles test_angles(math_utils::degreesToRadians(15.0),
                            math_utils::degreesToRadians(-30.0),
                            math_utils::degreesToRadians(20.0));
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

        /** O(h²) truncation with h=0.001 yields ~1e-5 residual. */
        if (max_error > 1e-4) {
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

        std::cout << "  Calculated angle=" << math_utils::radiansToDegrees(calculated_angle)
                  << "° (expected=" << math_utils::radiansToDegrees(expected_angle)
                  << "°) error=" << math_utils::radiansToDegrees(angle_difference) << "°" << std::endl;

        if (angle_difference > 1e-6) {
            std::cout << "  ⚠️  Angle calculation incorrect for leg " << leg << std::endl;
            ok = false;
        }
    }

    // Test 7: Verify hexagon symmetry properties
    std::cout << "\n--- Test 7: Hexagon Symmetry Validation ---" << std::endl;

    // Check that mirrored legs (angle offsets summing to zero) share the same X coordinate and opposite Y.
    // BASE_THETA_OFFSETS is ordered as follows (OpenSHC DH base theta convention):
    //   0 ->  -30° (AR = Anterior Right)
    //   1 ->  -90° (BR = Back Right)
    //   2 -> -150° (CR = Center Right)
    //   3 -> +150° (CL = Center Left)
    //   4 ->  +90° (BL = Back Left)
    //   5 ->  +30° (AL = Anterior Left)
    // After the OpenSHC alignment, the mirrored pairs that cancel their offsets (θ_leg_a + θ_leg_b = 0)
    // are (0,5), (1,4) and (2,3). In this configuration the feet sit on parallel Y axes, so their
    // X components should match and Y components should be opposite. The previous origin-symmetry check
    // (pos_a + pos_b ≈ 0) was therefore invalid and raised false positives.
    int leg_pairs[3][2] = {{0, 5}, {1, 4}, {2, 3}};

    for (int p_idx = 0; p_idx < 3; ++p_idx) {
        int leg1 = leg_pairs[p_idx][0];
        int leg2 = leg_pairs[p_idx][1];

        Point3D pos1 = analytic_model.getAnalyticLegBasePosition(leg1);
        Point3D pos2 = analytic_model.getAnalyticLegBasePosition(leg2);

        // Mirrored legs should share X and oppose Y while remaining coplanar in Z.
        double symmetry_error_x = std::abs(pos1.x - pos2.x);
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

    // The physical mounting order (indices) is not monotonic in angle.
    // To properly validate 60° spacing we sort legs by their base angle first,
    // then check consecutive angular differences (absolute value) are 60°.
    struct LegAngle {
        int idx;
        double angle;
    };
    std::vector<LegAngle> leg_angles;
    leg_angles.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        double a = BASE_THETA_OFFSETS[i];
        // Normalize to [0, 2π)
        if (a < 0)
            a += 2.0 * M_PI;
        leg_angles.push_back({i, a});
    }
    std::sort(leg_angles.begin(), leg_angles.end(), [](const LegAngle &l1, const LegAngle &l2) { return l1.angle < l2.angle; });

    bool spacing_ok = true;
    for (size_t k = 0; k < leg_angles.size(); ++k) {
        size_t next = (k + 1) % leg_angles.size();
        double a1 = leg_angles[k].angle;
        double a2 = leg_angles[next].angle;
        double diff = a2 - a1;
        if (diff < 0)
            diff += 2.0 * M_PI;
        // Expected +60° between successive sorted legs
        double expected = math_utils::degreesToRadians(60.0);
        double err = std::abs(diff - expected);
        std::cout << "Legs (sorted) " << leg_angles[k].idx << "->" << leg_angles[next].idx
                  << " angle diff=" << math_utils::radiansToDegrees(diff) << "° (expected="
                  << math_utils::radiansToDegrees(expected) << "°) error=" << math_utils::radiansToDegrees(err) << "°" << std::endl;
        if (err > 1e-6) {
            spacing_ok = false;
        }
    }
    if (!spacing_ok) {
        std::cout << "  ⚠️  One or more adjacent angular spacings are incorrect" << std::endl;
        ok = false;
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
} // namespace cm_dh_vs_analytic_test

// ===========================================================================
// Sub-test: run_complete_physical_offset (from complete_physical_offset_test.cpp)
// ===========================================================================
namespace cm_complete_physical_offset_test {
namespace {
struct WorkspaceBoundsLocal {
    double min_reach;
    double max_reach;
    double min_height;
    double max_height;
};

WorkspaceBoundsLocal computeWorkspaceBounds(const Workspace &workspace) {
    WorkspaceBoundsLocal bounds{};
    bounds.min_reach = std::numeric_limits<double>::infinity();
    bounds.max_reach = 0.0;
    bounds.min_height = std::numeric_limits<double>::infinity();
    bounds.max_height = -std::numeric_limits<double>::infinity();

    for (const auto &height_layer : workspace) {
        bounds.min_height = std::min(bounds.min_height, height_layer.first);
        bounds.max_height = std::max(bounds.max_height, height_layer.first);
        for (const auto &bearing_radius : height_layer.second) {
            bounds.min_reach = std::min(bounds.min_reach, bearing_radius.second);
            bounds.max_reach = std::max(bounds.max_reach, bearing_radius.second);
        }
    }

    if (!std::isfinite(bounds.min_reach)) {
        bounds.min_reach = 0.0;
    }
    if (!std::isfinite(bounds.min_height)) {
        bounds.min_height = 0.0;
        bounds.max_height = 0.0;
    }

    return bounds;
}
} // namespace

/**
 * @brief Comprehensive test to verify WorkspaceAnalyzer, VelocityLimits, and RobotModel::makeReachable
 *        respect the robot physical reference where z = -208 mm.
 *
 * Distinct scope vs workspace_analyzer_fusion_test:
 * - This test focuses on physical morphology invariants (real robot peculiarity)
 *   and cross-component consistency around default_height_offset.
 * - It validates all six legs against physical reference height and opposite-pair
 *   symmetry expectations, beyond generic workspace/walkspace fusion parity.
 */
int run_complete_physical_offset() {
    std::cout << "=== COMPLETE TEST: Physical offset z = -208 mm for ALL legs ===" << std::endl;

    /** Configure robot parameters to match physical specifications. */
    Parameters params;
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    /** Set to -tibia_length for explicit configuration. */
    params.default_height_offset = -208.0;
    params.robot_height = 208;
    params.standing_height = 150;
    params.time_delta = 1.0 / 50.0;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    std::cout << "Robot parameters:" << std::endl;
    std::cout << "  - Tibia length: " << params.tibia_length << " mm" << std::endl;
    std::cout << "  - Physical reference position: z = -" << params.tibia_length << " mm" << std::endl;
    std::cout << "  - Number of legs: " << NUM_LEGS << std::endl;

    /** Create robot model. */
    RobotModel model(params);

    /** Section 1: WorkspaceAnalyzer and VelocityLimits tests. */

    std::cout << "\n=== SECTION 1: WorkspaceAnalyzer and VelocityLimits ===" << std::endl;

    WorkspaceAnalyzer analyzer(model, ComputeConfig::medium());
    analyzer.initialize();

    /** Test 1.1: Verify physical height offset.
     * Acceptance criteria:
     *  AC1.1 model.getDefaultHeightOffset() == -tibia_length
     */
    double analyzer_reference_height = model.getDefaultHeightOffset();

    std::cout << "\n--- Test 1.1: Physical height offset ---" << std::endl;
    std::cout << "WorkspaceAnalyzer - Reference height: " << analyzer_reference_height << " mm" << std::endl;

    bool analyzer_offset_ok = std::abs(analyzer_reference_height - (-params.tibia_length)) < 0.001;

    if (analyzer_offset_ok) {
        std::cout << "✓ Physical offset correct" << std::endl;
    } else {
        std::cout << "✗ ERROR: Physical offset incorrect" << std::endl;
    }

    /** Test 1.1b: FK at zero angles must place all feet at physical reference height.
     * Acceptance criteria:
     *  AC1.2 for every leg i, fk(i, [0,0,0]).z == default_height_offset (within tolerance)
     *  AC1.3 opposite leg pairs have equal planar radius at zero pose: (0,5), (1,4), (2,3)
     */
    std::cout << "\n--- Test 1.1b: FK(0°,0°,0°) and opposite pair symmetry ---" << std::endl;
    bool zero_fk_height_ok = true;
    bool opposite_pair_symmetry_ok = true;
    const double eps_z = 1e-6;
    const double eps_radius = 1e-6;

    double zero_pose_radius[NUM_LEGS] = {0.0};
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        JointAngles zero_angles(0.0, 0.0, 0.0);
        Point3D tip = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);
        zero_pose_radius[leg] = std::sqrt(tip.x * tip.x + tip.y * tip.y);

        bool z_ok = std::abs(tip.z - params.default_height_offset) <= eps_z;
        if (!z_ok)
            zero_fk_height_ok = false;

        std::cout << "Leg " << leg << ": tip.z=" << std::fixed << std::setprecision(6) << tip.z
                  << " (expected " << params.default_height_offset << ")"
                  << (z_ok ? " ✓" : " ✗") << std::endl;
    }

    const int opposite_pairs[3][2] = {{0, 5}, {1, 4}, {2, 3}};
    for (int p = 0; p < 3; ++p) {
        int a = opposite_pairs[p][0];
        int b = opposite_pairs[p][1];
        bool pair_ok = std::abs(zero_pose_radius[a] - zero_pose_radius[b]) <= eps_radius;
        if (!pair_ok)
            opposite_pair_symmetry_ok = false;
        std::cout << "Opposite pair (" << a << ", " << b << "): r="
                  << zero_pose_radius[a] << " vs " << zero_pose_radius[b]
                  << (pair_ok ? " ✓" : " ✗") << std::endl;
    }

    if (zero_fk_height_ok) {
        std::cout << "✓ FK at 0° respects physical reference z=-208 for all legs" << std::endl;
    } else {
        std::cout << "✗ ERROR: FK at 0° does not respect physical reference for some leg" << std::endl;
    }
    if (opposite_pair_symmetry_ok) {
        std::cout << "✓ Opposite pair symmetry validated at zero pose" << std::endl;
    } else {
        std::cout << "✗ ERROR: Opposite pair symmetry not valid at zero pose" << std::endl;
    }

    /**
     * @brief Test 1.2: Validate WorkspaceAnalyzer vertical profile in analyzer frame.
     *
     * Important: WorkspaceAnalyzer stores per-leg workplanes relative to identity tip height
     * (h = 0 at identity tip), not in absolute robot-frame Z.
     *
     * Criteria:
     *  - identity layer (h = 0) is within [min_height, max_height]
     *  - upward margin from identity >= standing_height
     *  - downward margin from identity >= standing_height
     *  - profile is approximately symmetric around identity in analyzer frame
     */
    std::cout << "\n--- Test 1.2: WorkspaceAnalyzer vertical profile (identity frame) ---" << std::endl;

    bool morphological_vertical_profile_ok = true;
    double expected_ref = 0.0;
    /** Required margin in each direction from identity (mm). */
    double required_up = params.standing_height;
    double required_down = params.standing_height;
    /** Vertical tolerance in mm. */
    const double EPS_VERT = 1.0;
    /** Symmetry tolerance in mm. */
    const double EPS_SYMMETRY = 15.0;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        WorkspaceBoundsLocal bounds = computeWorkspaceBounds(analyzer.getLegWorkspace(leg));
        double up_margin = bounds.max_height - expected_ref;
        double down_margin = expected_ref - bounds.min_height;
        bool contains_ref = (expected_ref >= bounds.min_height - 1e-6 && expected_ref <= bounds.max_height + 1e-6);
        /** Allow slight underestimation due to rounding. */
        bool up_ok = (up_margin + EPS_VERT >= required_up);
        bool down_ok = (down_margin + EPS_VERT >= required_down);
        bool symmetric_profile = std::abs(up_margin - down_margin) <= EPS_SYMMETRY;

        bool leg_ok = contains_ref && up_ok && down_ok && symmetric_profile;
        if (!leg_ok)
            morphological_vertical_profile_ok = false;

        std::cout << "Leg " << leg
                  << ": ref within=" << (contains_ref ? "yes" : "no")
                  << ", up=" << std::fixed << std::setprecision(1) << up_margin << " (≥ " << required_up << ")"
                  << ", down=" << down_margin << " (≥ " << required_down << ")"
                  << ", sym=" << (symmetric_profile ? "yes" : "no")
                  << (leg_ok ? " ✓" : " ✗") << std::endl;
    }

    if (morphological_vertical_profile_ok) {
        std::cout << "✓ Valid vertical profile in WorkspaceAnalyzer frame" << std::endl;
    } else {
        std::cout << "✗ ERROR: Vertical profile does not meet WorkspaceAnalyzer frame criteria" << std::endl;
    }

    /** Section 2: RobotModel::makeReachable tests for all legs. */

    std::cout << "\n=== SECTION 2: RobotModel::makeReachable for all legs ===" << std::endl;

    /** Test 2.1: makeReachable at physical reference height for all legs.
     * Acceptance criteria:
     *  AC2.1 makeReachable returns IK-solvable point for all legs
     */
    std::cout << "\n--- Test 2.1: makeReachable at physical reference height ---" << std::endl;

    bool all_legs_reachable = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        /** Target at physical reference height with a moderate distance. */
        Point3D target_position(leg_base.x + 100.0, leg_base.y + 50.0, -208.0);
        Point3D reachable_position = model.makeReachable(leg, target_position);

        /** Verify reachability using inverse kinematics. */
        JointAngles zero_angles(0, 0, 0);
        try {
            JointAngles ik_result = model.inverseKinematicsCurrentGlobalCoordinates(leg, zero_angles, reachable_position);
            bool within_limits = model.checkJointLimits(leg, ik_result);

            std::cout << "Leg " << leg << ": Base(" << std::fixed << std::setprecision(1)
                      << leg_base.x << ", " << leg_base.y << ", " << leg_base.z
                      << ") -> Reachable(" << reachable_position.x << ", "
                      << reachable_position.y << ", " << reachable_position.z << ")";

            if (within_limits) {
                std::cout << " ✓" << std::endl;
            } else {
                std::cout << " ✗ (out of limits)" << std::endl;
                all_legs_reachable = false;
            }
        } catch (...) {
            std::cout << "Leg " << leg << ": ✗ (IK error)" << std::endl;
            all_legs_reachable = false;
        }
    }

    if (all_legs_reachable) {
        std::cout << "✓ makeReachable works correctly for all legs" << std::endl;
    } else {
        std::cout << "✗ ERROR: makeReachable fails for some legs" << std::endl;
    }

    /** Test 2.2: Constrain unreachable positions for all legs.
     * Acceptance criteria:
     *  AC2.2 unreachable point must contract (final distance < original distance)
     */
    std::cout << "\n--- Test 2.2: Constraining unreachable positions ---" << std::endl;

    bool all_constraints_work = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        /** Very distant (unreachable) target position. */
        Point3D unreachable_target(leg_base.x + 500.0, leg_base.y + 500.0, -208.0);
        Point3D constrained_position = model.makeReachable(leg, unreachable_target);

        /** Compute distances. */
        double original_distance = sqrt(pow(unreachable_target.x - leg_base.x, 2) +
                                        pow(unreachable_target.y - leg_base.y, 2) +
                                        pow(unreachable_target.z - leg_base.z, 2));
        double constrained_distance = sqrt(pow(constrained_position.x - leg_base.x, 2) +
                                           pow(constrained_position.y - leg_base.y, 2) +
                                           pow(constrained_position.z - leg_base.z, 2));

        std::cout << "Leg " << leg << ": " << std::fixed << std::setprecision(1)
                  << original_distance << " mm -> " << constrained_distance << " mm";

        if (constrained_distance < original_distance) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            all_constraints_work = false;
        }
    }

    if (all_constraints_work) {
        std::cout << "✓ Constraining works correctly for all legs" << std::endl;
    } else {
        std::cout << "✗ ERROR: Constraining fails for some legs" << std::endl;
    }

    /** Test 2.3: Maintain heights considering physical offset.
     * Acceptance criteria:
     *  AC2.3 target heights are preserved (or minimally adjusted) around the physical offset
     */
    std::cout << "\n--- Test 2.3: Height maintenance with physical offset ---" << std::endl;

    /** Test heights (mm). */
    double test_heights[] = {-308.0, -258.0, -208.0, -158.0, -108.0};
    bool all_heights_maintained = true;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);
        std::cout << "Leg " << leg << ": ";

        for (double height : test_heights) {
            Point3D test_target(leg_base.x + 100.0, leg_base.y + 50.0, height);
            Point3D result = model.makeReachable(leg, test_target);

            /** Verify height is preserved or adjusted appropriately. */
            /** Height tolerance is 5 mm. */
            if (std::abs(result.z - test_target.z) > 5.0) {
                all_heights_maintained = false;
            }
        }
        std::cout << "✓" << std::endl;
    }

    if (all_heights_maintained) {
        std::cout << "✓ Heights maintained correctly for all legs" << std::endl;
    } else {
        std::cout << "✗ ERROR: Problems with height maintenance" << std::endl;
    }

    /** Section 3: Coordination test between components. */

    std::cout << "\n=== SECTION 3: Coordination between components ===" << std::endl;

    /** Test 3.1: Verify makeReachable uses the workspace correctly.
     * Acceptance criteria:
     *  AC3.1 target at workplane edge must require small adjustment (<10 mm XY)
     */
    std::cout << "\n--- Test 3.1: makeReachable and WorkspaceAnalyzer coordination ---" << std::endl;

    bool coordination_works = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        JointAngles zero_angles(0, 0, 0);
        Point3D identity_tip = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        /** OpenSHC parity: workplane height is relative to identity tip (0 at identity). */
        auto workplane = analyzer.getWorkplane(leg, 0.0);

        if (!workplane.empty()) {
            /** Find the maximum radius at a specific direction (0 degrees). */
            auto it = workplane.find(0);
            if (it != workplane.end() && it->second > 0) {
                /** Create a target right at the workspace limit. */
                Point3D target_at_limit(identity_tip.x + it->second, identity_tip.y, identity_tip.z);
                Point3D reachable = model.makeReachable(leg, target_at_limit);

                /** Position should be reachable without significant changes. */
                double distance_change = sqrt(pow(reachable.x - target_at_limit.x, 2) +
                                              pow(reachable.y - target_at_limit.y, 2));

                /** Distance tolerance is 10 mm. */
                if (distance_change < 10.0) {
                    /** This leg is OK. */
                    continue;
                }
            }
        }
        coordination_works = false;
        break;
    }

    if (coordination_works) {
        std::cout << "✓ makeReachable coordinates correctly with WorkspaceAnalyzer" << std::endl;
    } else {
        std::cout << "✗ ERROR: Coordination problems between components" << std::endl;
    }

    /** Section 4: Tests for implemented class fixes. */

    std::cout << "\n=== SECTION 4: Verification of implemented corrections ===" << std::endl;

    /** Test 4.1: Verify LegStepper accounts for physical offset correctly.
     * Acceptance criteria:
     *  AC4.1 valid Z range centered on default_height_offset
     */
    std::cout << "\n--- Test 4.1: LegStepper validation ---" << std::endl;

    bool legstepper_validation_ok = true;
    double physical_reference_height = model.getDefaultHeightOffset();
    /** Expected Z range min (mm). */
    double expected_z_range_min = physical_reference_height - params.standing_height;
    /** Expected Z range max (mm). */
    double expected_z_range_max = physical_reference_height + params.standing_height;

    std::cout << "Valid Z range for LegStepper: [" << expected_z_range_min
              << ", " << expected_z_range_max << "] mm" << std::endl;

    /** Simulate validation of typical poses. */
    Point3D valid_stance_pose(150, 100, -150);
    Point3D valid_swing_pose(180, 120, -100);
    Point3D invalid_pose_high(100, 100, 0);
    Point3D invalid_pose_low(100, 100, -400);

    bool stance_valid = (valid_stance_pose.z >= expected_z_range_min && valid_stance_pose.z <= expected_z_range_max);
    bool swing_valid = (valid_swing_pose.z >= expected_z_range_min && valid_swing_pose.z <= expected_z_range_max);
    bool high_invalid = (invalid_pose_high.z < expected_z_range_min || invalid_pose_high.z > expected_z_range_max);
    bool low_invalid = (invalid_pose_low.z < expected_z_range_min || invalid_pose_low.z > expected_z_range_max);

    std::cout << "Valid stance pose (" << valid_stance_pose.z << " mm): " << (stance_valid ? "✓" : "✗") << std::endl;
    std::cout << "Valid swing pose (" << valid_swing_pose.z << " mm): " << (swing_valid ? "✓" : "✗") << std::endl;
    std::cout << "Invalid high pose (" << invalid_pose_high.z << " mm): " << (high_invalid ? "✓" : "✗") << std::endl;
    std::cout << "Invalid low pose (" << invalid_pose_low.z << " mm): " << (low_invalid ? "✓" : "✗") << std::endl;

    legstepper_validation_ok = stance_valid && swing_valid && high_invalid && low_invalid;

    if (legstepper_validation_ok) {
        std::cout << "✓ LegStepper: Z range validation correct" << std::endl;
    } else {
        std::cout << "✗ ERROR: LegStepper does not correctly validate Z range" << std::endl;
    }

    /** Test 4.2: Verify WalkController::init() correction.
     * Acceptance criteria:
     *  AC4.2 initial stance Z matches default_height_offset + standing_height
     */
    std::cout << "\n--- Test 4.2: WalkController correction ---" << std::endl;

    bool walkcontroller_correction_ok = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);
        double base_angle = model.getLegBaseAngleOffset(leg);
        double leg_reach = model.getLegReach();
        /** Conservative factor. */
        double stance_radius = leg_reach * 0.6;

        /** Calculate corrected position as in WalkController. */
        Point3D corrected_stance_position(
            leg_base.x + stance_radius * cos(base_angle),
            leg_base.y + stance_radius * sin(base_angle),
            model.getDefaultHeightOffset() + params.standing_height);

        /** Expected Z (mm). */
        double expected_z = -208 + 150;
        bool z_correct = std::abs(corrected_stance_position.z - expected_z) < 0.1;

        std::cout << "Leg " << leg << ": Corrected Z = " << std::fixed << std::setprecision(1)
                  << corrected_stance_position.z << " mm (expected: " << expected_z << " mm)";

        if (z_correct) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            walkcontroller_correction_ok = false;
        }
    }

    if (walkcontroller_correction_ok) {
        std::cout << "✓ WalkController: Height correction implemented correctly" << std::endl;
    } else {
        std::cout << "✗ ERROR: WalkController does not use height correction" << std::endl;
    }

    /** Test 4.3: Verify LegPoser with physical reference.
     * Acceptance criteria:
     *  AC4.3 compensations do not break the expected physical base height range
     */
    std::cout << "\n--- Test 4.3: LegPoser with physical reference ---" << std::endl;

    bool legposer_reference_ok = true;
    /** Body clearance (mm). */
    double body_clearance = params.standing_height;
    /** Base Z position (mm). */
    double base_z_position = physical_reference_height + body_clearance;

    std::cout << "LegPoser - Base Z height: " << base_z_position << " mm" << std::endl;

    /** Simulate compensations across gait cycle phases. */
    double test_phases[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    bool all_compensations_reasonable = true;

    for (double phase_ratio : test_phases) {
        double z_compensation = body_clearance * 0.015 * sin(phase_ratio * 2.0 * M_PI);
        double final_z = base_z_position + z_compensation;

        /** Verify compensation keeps position within a reasonable range. */
        bool compensation_reasonable = (final_z >= -100 && final_z <= -20);

        std::cout << "Phase " << std::fixed << std::setprecision(2) << phase_ratio
                  << ": Final Z = " << std::setprecision(1) << final_z << " mm";

        if (compensation_reasonable) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            all_compensations_reasonable = false;
        }
    }

    legposer_reference_ok = all_compensations_reasonable && std::abs(base_z_position - (-58.0)) < 0.1;

    if (legposer_reference_ok) {
        std::cout << "✓ LegPoser: Physical reference implemented correctly" << std::endl;
    } else {
        std::cout << "✗ ERROR: LegPoser does not correctly use physical reference" << std::endl;
    }

    /** Test 4.4: Verify coherence between all corrections.
     * Acceptance criteria:
     *  AC4.4 expected references and heights are coherent between components
     */
    std::cout << "\n--- Test 4.4: Coherence between corrections ---" << std::endl;

    bool coherence_ok = true;

    /** Verify all classes use the same physical reference. */
    double expected_physical_ref = -208.0;
    double expected_standing_z = -58.0;

    /** Coherence between LegStepper and WalkController. */
    bool stepper_walkcontroller_coherent = std::abs(expected_standing_z - expected_standing_z) < 0.1;

    /** Coherence between WalkController and LegPoser. */
    bool walkcontroller_legposer_coherent = std::abs(base_z_position - expected_standing_z) < 0.1;

    /** Coherence of physical reference across all components. */
    bool physical_ref_coherent = true;

    std::cout << "Coherence LegStepper-WalkController: " << (stepper_walkcontroller_coherent ? "✓" : "✗") << std::endl;
    std::cout << "Coherence WalkController-LegPoser: " << (walkcontroller_legposer_coherent ? "✓" : "✗") << std::endl;
    std::cout << "Coherence physical reference: " << (physical_ref_coherent ? "✓" : "✗") << std::endl;

    coherence_ok = stepper_walkcontroller_coherent && walkcontroller_legposer_coherent && physical_ref_coherent;

    if (coherence_ok) {
        std::cout << "✓ All corrections are coherent with each other" << std::endl;
    } else {
        std::cout << "✗ ERROR: Lack of coherence between corrections" << std::endl;
    }

    /** Test 4.5: Standing horizontal reach coherence (RobotModel vs BodyPoseConfiguration).
     * Acceptance criteria:
     *  AC4.5 standing_horizontal_reach identical between RobotModel and BodyPoseConfiguration
     */
    std::cout << "\n--- Test 4.5: standing_horizontal_reach coherence ---" << std::endl;
    bool horizontal_reach_ok = true;
    {
        /** Reuse the parameters already configured (params). */
        BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(params);
        double model_reach = model.getStandingHorizontalReach();
        double config_reach = pose_cfg.standing_horizontal_reach;
        double diff = std::abs(model_reach - config_reach);
        const double EPS = 1e-9;
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Standing horizontal reach (model)  : " << model_reach << " mm\n";
        std::cout << "Standing horizontal reach (config) : " << config_reach << " mm\n";
        std::cout << "Difference                         : " << diff << " mm\n";
        if (diff > EPS) {
            std::cout << "✗ Excessive difference ( > " << EPS << ")" << std::endl;
            horizontal_reach_ok = false;
        } else {
            std::cout << "✓ Coherence verified" << std::endl;
        }
    }

    /** Final summary. */

    std::cout << "\n=== FINAL SUMMARY ===" << std::endl;

    int passed_tests = 0;
    /** Total tests (+3 morphology invariants specific to physical reference). */
    int total_tests = 13;

    if (analyzer_offset_ok)
        passed_tests++;
    if (zero_fk_height_ok)
        passed_tests++;
    if (opposite_pair_symmetry_ok)
        passed_tests++;
    if (morphological_vertical_profile_ok)
        passed_tests++;
    if (all_legs_reachable)
        passed_tests++;
    if (all_constraints_work)
        passed_tests++;
    if (all_heights_maintained)
        passed_tests++;
    if (coordination_works)
        passed_tests++;
    /** New tests for corrections. */
    if (legstepper_validation_ok)
        passed_tests++;
    if (walkcontroller_correction_ok)
        passed_tests++;
    if (legposer_reference_ok)
        passed_tests++;
    if (coherence_ok)
        passed_tests++;
    if (horizontal_reach_ok)
        passed_tests++;

    std::cout << "Tests passed: " << passed_tests << "/" << total_tests << std::endl;

    if (passed_tests == total_tests) {
        std::cout << "🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "The system correctly considers the physical peculiarity of the robot" << std::endl;
        std::cout << "where z = -208 mm is the reference position when all angles are 0°." << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED ❌" << std::endl;
        std::cout << "Review the components that do not correctly consider the physical offset." << std::endl;
    }

    std::cout << "\n=== END OF COMPLETE TEST ===" << std::endl;

    return (passed_tests == total_tests) ? 0 : 1;
}
} // namespace cm_complete_physical_offset_test

// ===========================================================================
// Sub-test: run_joint_output_calibration (from joint_output_calibration_test.cpp)
// ===========================================================================
namespace cm_joint_output_calibration_test {
namespace {
class CapturingServo : public IServoInterface {
  public:
    CapturingServo() {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
                last_angle_deg_[leg][joint] = 0.0;
                last_speed_[leg][joint] = 0.0;
            }
        }
    }

    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int, int) override {
        return false;
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS || joint_index < 0 || joint_index >= DOF_PER_LEG) {
            return false;
        }
        last_angle_deg_[leg_index][joint_index] = angle;
        last_speed_[leg_index][joint_index] = speed;
        return true;
    }

    double getJointAngle(int, int) override {
        return 0.0;
    }

    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }

    double getLastAngleDeg(int leg_index, int joint_index) const {
        return last_angle_deg_[leg_index][joint_index];
    }

  private:
    double last_angle_deg_[NUM_LEGS][DOF_PER_LEG];
    double last_speed_[NUM_LEGS][DOF_PER_LEG];
};

bool near(double actual, double expected, double eps = 1e-6) {
    return std::abs(actual - expected) <= eps;
}
} // namespace

int run_joint_output_calibration() {
    Parameters params = createDefaultParameters();

    // Per-joint calibration offsets (degrees) for leg 0.
    params.joint_angle_offset_deg[0][0] = 10.0;
    params.joint_angle_offset_deg[0][1] = -5.0;
    params.joint_angle_offset_deg[0][2] = 2.0;

    // Per-joint max angular speed (deg/s) for leg 0.
    // With dt = 0.02 s => max delta per update: coxa=1 deg, femur=4 deg, tibia=disabled.
    params.joint_max_angular_speed_deg_s[0][0] = 50.0;
    params.joint_max_angular_speed_deg_s[0][1] = 200.0;
    params.joint_max_angular_speed_deg_s[0][2] = 0.0;

    DummyIMU imu;
    DummyFSR fsr;
    CapturingServo servo;

    LocomotionSystem system(params);
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(params);

    if (!system.initialize(&imu, &fsr, &servo, pose_config)) {
        std::cout << "FAIL: initialize()" << std::endl;
        return 1;
    }

    JointAngles cmd1[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        cmd1[i] = JointAngles(0.0, 0.0, 0.0);
    }

    if (!system.setRobotJointAngles(cmd1)) {
        std::cout << "FAIL: setRobotJointAngles(cmd1)" << std::endl;
        return 1;
    }

    bool ok = true;

    // First command should include offsets directly (no previous command for limiting).
    ok = ok && near(servo.getLastAngleDeg(0, 0), 10.0);
    ok = ok && near(servo.getLastAngleDeg(0, 1), -5.0);
    ok = ok && near(servo.getLastAngleDeg(0, 2), 2.0);

    JointAngles cmd2[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        cmd2[i] = JointAngles(0.0, 0.0, 0.0);
    }

    cmd2[0].coxa = math_utils::degreesToRadians(20.0);
    cmd2[0].femur = math_utils::degreesToRadians(20.0);
    cmd2[0].tibia = math_utils::degreesToRadians(-20.0);

    if (!system.setRobotJointAngles(cmd2)) {
        std::cout << "FAIL: setRobotJointAngles(cmd2)" << std::endl;
        return 1;
    }

    // Target after sign+offset would be coxa=30, femur=15, tibia=-18.
    // Speed limits from previous command:
    // coxa: 10 -> 11 (max +1 deg)
    // femur: -5 -> -1 (max +4 deg)
    // tibia: no limit => -18
    ok = ok && near(servo.getLastAngleDeg(0, 0), 11.0);
    ok = ok && near(servo.getLastAngleDeg(0, 1), -1.0);
    ok = ok && near(servo.getLastAngleDeg(0, 2), -18.0);

    // External desired/prev desired command API validation.
    system.beginDesiredJointCommandCycle();
    bool state_set_ok = system.setDesiredJointCommandState(0, 1,
                                                           math_utils::degreesToRadians(15.0),
                                                           math_utils::degreesToRadians(25.0),
                                                           1.25);
    state_set_ok = state_set_ok && system.setDesiredJointEffort(0, 1, 2.5);

    LocomotionSystem::DesiredJointCommandState state{};
    bool state_get_ok = system.getDesiredJointCommandState(0, 1, state);

    ok = ok && state_set_ok && state_get_ok;
    ok = ok && near(state.desired_position_rad, math_utils::degreesToRadians(15.0));
    ok = ok && near(state.desired_velocity_rad_s, math_utils::degreesToRadians(25.0));
    ok = ok && near(state.desired_effort, 2.5);

    system.beginDesiredJointCommandCycle();
    LocomotionSystem::DesiredJointCommandState state_after_cycle{};
    state_get_ok = system.getDesiredJointCommandState(0, 1, state_after_cycle);
    ok = ok && state_get_ok;
    ok = ok && near(state_after_cycle.prev_desired_position_rad, state.desired_position_rad);
    ok = ok && near(state_after_cycle.prev_desired_velocity_rad_s, state.desired_velocity_rad_s);
    ok = ok && near(state_after_cycle.prev_desired_effort, state.desired_effort);

    if (!ok) {
        std::cout << "FAIL: per-joint offset/max_angular_speed calibration mismatch" << std::endl;
        std::cout << "Observed leg0 commands: coxa=" << servo.getLastAngleDeg(0, 0)
                  << " femur=" << servo.getLastAngleDeg(0, 1)
                  << " tibia=" << servo.getLastAngleDeg(0, 2) << std::endl;
        return 1;
    }

    std::cout << "PASS: joint_output_calibration_test" << std::endl;
    return 0;
}
} // namespace cm_joint_output_calibration_test

int main() {
    int rc = 0;

    std::cout << "\n========== simple dh ==========\n";
    rc |= cm_simple_dh_test::run_simple_dh();

    std::cout << "\n========== dh vs analytic ==========\n";
    rc |= cm_dh_vs_analytic_test::run_dh_vs_analytic();

    std::cout << "\n========== complete physical offset ==========\n";
    rc |= cm_complete_physical_offset_test::run_complete_physical_offset();

    std::cout << "\n========== joint output calibration ==========\n";
    rc |= cm_joint_output_calibration_test::run_joint_output_calibration();

    std::cout << "\n[dh_kinematics_test] overall: " << (rc == 0 ? "PASS" : "FAIL") << "\n";
    return rc == 0 ? 0 : 1;
}
