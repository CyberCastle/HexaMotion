#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

/**
 * @brief Comprehensive Kinematics Validation Test for HexaMotion
 *
 * This test validates the complete kinematic system including:
 * - Forward/Inverse Kinematics in local and global coordinates
 * - Coordinate transformations between local and global frames
 * - Workspace analysis and reachability validation
 * - Joint limit validation
 * - IK/FK round-trip accuracy
 * - Pose transformations and frame conversions
 * - Jacobian calculations
 * - Multiple IK solving methods comparison
 */

struct ValidationResults {
    double max_fk_ik_error = 0.0;
    double max_local_global_transform_error = 0.0;
    double max_pose_transform_error = 0.0;
    double max_jacobian_error = 0.0;
    int total_tests = 0;
    int failed_tests = 0;
    std::vector<std::string> error_messages;

    void addError(const std::string &message) {
        error_messages.push_back(message);
        failed_tests++;
    }

    void printSummary() const {
        std::cout << "\n=== VALIDATION SUMMARY ===" << std::endl;
        std::cout << "Total tests: " << total_tests << std::endl;
        std::cout << "Failed tests: " << failed_tests << std::endl;
        std::cout << "Success rate: " << std::fixed << std::setprecision(2)
                  << (100.0 * (total_tests - failed_tests) / total_tests) << "%" << std::endl;
        std::cout << "Max FK/IK error: " << max_fk_ik_error << " mm" << std::endl;
        std::cout << "Max local/global transform error: " << max_local_global_transform_error << " mm" << std::endl;
        std::cout << "Max pose transform error: " << max_pose_transform_error << " mm" << std::endl;
        std::cout << "Max Jacobian error: " << max_jacobian_error << " mm" << std::endl;

        if (!error_messages.empty()) {
            std::cout << "\n=== ERROR DETAILS ===" << std::endl;
            for (const auto &error : error_messages) {
                std::cout << "- " << error << std::endl;
            }
        }
    }
};

class ComprehensiveKinematicsValidator {
  private:
    RobotModel model;
    ValidationResults results;

    // Test configuration
    const double step_coxa = 10.0; // Smaller steps for more thorough testing
    const double step_femur = 10.0;
    const double step_tibia = 10.0;
    const double position_tolerance = 2.0;                            // 2mm tolerance
    const double angle_tolerance = math_utils::degreesToRadians(1.0); // 1 degree tolerance

  public:
    ComprehensiveKinematicsValidator(const Parameters &params) : model(params) {
        if (!model.validate()) {
            throw std::runtime_error("Invalid robot model parameters");
        }
    }

    void runAllValidations() {
        std::cout << "Starting comprehensive kinematics validation..." << std::endl;

        validateBasicIKFK();
        validateCoordinateTransformations();
        validatePoseTransformations();
        validateJacobianCalculations();
        validateWorkspaceAnalysis();
        validateMultipleIKMethods();
        validateJointLimits();
        validateEdgeCases();

        results.printSummary();
    }

  private:
    void validateBasicIKFK() {
        std::cout << "\n--- Basic IK/FK Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing leg " << leg << "..." << std::endl;

            for (double coxa = model.getParams().coxa_angle_limits[0];
                 coxa <= model.getParams().coxa_angle_limits[1];
                 coxa += step_coxa) {
                for (double femur = model.getParams().femur_angle_limits[0];
                     femur <= model.getParams().femur_angle_limits[1];
                     femur += step_femur) {
                    for (double tibia = model.getParams().tibia_angle_limits[0];
                         tibia <= model.getParams().tibia_angle_limits[1];
                         tibia += step_tibia) {

                        JointAngles original_angles(math_utils::degreesToRadians(coxa), math_utils::degreesToRadians(femur), math_utils::degreesToRadians(tibia));

                        if (!model.checkJointLimits(leg, original_angles)) {
                            continue;
                        }

                        results.total_tests++;

                        // Test 1: Global coordinates FK -> IK round-trip
                        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, original_angles);
                        JointAngles ik_angles = model.inverseKinematicsGlobalCoordinates(leg, global_pos);
                        Point3D fk_verify = model.forwardKinematicsGlobalCoordinates(leg, ik_angles);

                        double error = math_utils::distance(fk_verify, global_pos);
                        results.max_fk_ik_error = std::max(results.max_fk_ik_error, error);

                        if (error > position_tolerance) {
                            results.addError("Leg " + std::to_string(leg) +
                                             " FK/IK error: " + std::to_string(error) + "mm");
                        }

                        // Test 2: Local coordinates FK -> IK round-trip
                        Point3D local_pos = model.transformGlobalToLocalCoordinates(leg, global_pos, JointAngles(0, 0, 0));
                        JointAngles local_ik = model.solveIKLocalCoordinates(leg, global_pos, original_angles);
                        Point3D local_fk = model.forwardKinematics(leg, local_ik);

                        error = math_utils::distance(local_fk, local_pos);
                        if (error > position_tolerance) {
                            results.addError("Leg " + std::to_string(leg) +
                                             " Local FK/IK error: " + std::to_string(error) + "mm");
                        }
                    }
                }
            }
        }
    }

    void validateCoordinateTransformations() {
        std::cout << "\n--- Coordinate Transformation Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing coordinate transformations for leg " << leg << "..." << std::endl;

            // Test with various joint configurations (convert degrees to radians)
            std::vector<JointAngles> test_configs = {
                JointAngles(0, 0, 0),
                JointAngles(math_utils::degreesToRadians(30), math_utils::degreesToRadians(-30), math_utils::degreesToRadians(20)),
                JointAngles(math_utils::degreesToRadians(-30), math_utils::degreesToRadians(30), math_utils::degreesToRadians(-20)),
                JointAngles(math_utils::degreesToRadians(45), math_utils::degreesToRadians(-45), math_utils::degreesToRadians(30)),
                JointAngles(math_utils::degreesToRadians(-45), math_utils::degreesToRadians(45), math_utils::degreesToRadians(-30))};

            for (const auto &angles : test_configs) {
                if (!model.checkJointLimits(leg, angles)) {
                    continue;
                }

                results.total_tests++;

                // Test global -> local -> global transformation
                Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, angles);
                Point3D local_pos = model.transformGlobalToLocalCoordinates(leg, global_pos, angles);
                Point3D global_back = model.transformLocalToGlobalCoordinates(leg, local_pos, angles);

                double error = math_utils::distance(global_pos, global_back);
                results.max_local_global_transform_error = std::max(results.max_local_global_transform_error, error);

                if (error > position_tolerance) {
                    results.addError("Leg " + std::to_string(leg) +
                                     " Global->Local->Global error: " + std::to_string(error) + "mm");
                }

                // Test with different reference angles (convert degrees to radians)
                JointAngles ref_angles(math_utils::degreesToRadians(15), math_utils::degreesToRadians(-15), math_utils::degreesToRadians(10));
                Point3D local_ref = model.transformGlobalToLocalCoordinates(leg, global_pos, ref_angles);
                Point3D global_ref = model.transformLocalToGlobalCoordinates(leg, local_ref, ref_angles);

                error = math_utils::distance(global_pos, global_ref);
                if (error > position_tolerance) {
                    results.addError("Leg " + std::to_string(leg) +
                                     " Reference frame transform error: " + std::to_string(error) + "mm");
                }
            }
        }
    }

    void validatePoseTransformations() {
        std::cout << "\n--- Pose Transformation Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing pose transformations for leg " << leg << "..." << std::endl;

            JointAngles test_angles(math_utils::degreesToRadians(20), math_utils::degreesToRadians(-20), math_utils::degreesToRadians(15));
            if (!model.checkJointLimits(leg, test_angles)) {
                continue;
            }

            results.total_tests++;

            // Test robot frame to leg frame transformations
            Pose robot_pose(Point3D(10, 20, -30), Eigen::Vector3d(5, 10, 15));
            Pose leg_pose = model.getPoseLegFrame(leg, test_angles, robot_pose);
            Pose robot_back = model.getPoseRobotFrame(leg, test_angles, leg_pose);

            double pos_error = math_utils::distance(robot_pose.position, robot_back.position);
            double rot_error = (robot_pose.rotation.inverse() * robot_back.rotation).vec().norm();

            results.max_pose_transform_error = std::max(results.max_pose_transform_error, pos_error);

            if (pos_error > position_tolerance) {
                results.addError("Leg " + std::to_string(leg) +
                                 " Pose position error: " + std::to_string(pos_error) + "mm");
            }

            if (rot_error > 0.1) { // 0.1 rad tolerance for rotation
                results.addError("Leg " + std::to_string(leg) +
                                 " Pose rotation error: " + std::to_string(rot_error) + "rad");
            }

            // Test tip pose transformations
                    Pose tip_pose = model.getPoseRobotFrame(leg, test_angles, Pose::Identity());
        Pose tip_leg = model.getPoseLegFrame(leg, test_angles, Pose::Identity());

            // Verify tip pose is consistent with FK
            Point3D fk_pos = model.forwardKinematicsGlobalCoordinates(leg, test_angles);
            double tip_error = math_utils::distance(tip_pose.position, fk_pos);

            if (tip_error > position_tolerance) {
                results.addError("Leg " + std::to_string(leg) +
                                 " Tip pose consistency error: " + std::to_string(tip_error) + "mm");
            }
        }
    }

    void validateJacobianCalculations() {
        std::cout << "\n--- Jacobian Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing Jacobian for leg " << leg << "..." << std::endl;

            JointAngles test_angles(math_utils::degreesToRadians(15), math_utils::degreesToRadians(-15), math_utils::degreesToRadians(10));
            if (!model.checkJointLimits(leg, test_angles)) {
                continue;
            }

            results.total_tests++;

            // Test numerical Jacobian consistency
            Point3D current_pos = model.forwardKinematicsGlobalCoordinates(leg, test_angles);
            Eigen::Matrix3d numeric_jac = model.calculateJacobian(leg, test_angles, current_pos);

            // Test Jacobian consistency with small joint changes
            const double delta = 0.001; // 1mm joint change
            JointAngles angles_plus = test_angles;
            angles_plus.coxa += delta;

            Point3D pos_plus = model.forwardKinematicsGlobalCoordinates(leg, angles_plus);
            Point3D pos_diff = pos_plus - current_pos;
            Point3D jac_prediction = Point3D(
                numeric_jac(0, 0) * delta,
                numeric_jac(1, 0) * delta,
                numeric_jac(2, 0) * delta);

            double consistency_error = math_utils::distance(pos_diff, jac_prediction);
            results.max_jacobian_error = std::max(results.max_jacobian_error, consistency_error);

            if (consistency_error > 0.1) { // 0.1mm tolerance for Jacobian consistency
                results.addError("Leg " + std::to_string(leg) +
                                 " Jacobian consistency error: " + std::to_string(consistency_error) + "mm");
            }

            // Test Jacobian determinant (should be non-zero for valid configurations)
            double det = numeric_jac.determinant();
            if (std::abs(det) < 1e-6) {
                results.addError("Leg " + std::to_string(leg) +
                                 " Jacobian near singular: det = " + std::to_string(det));
            }
        }
    }

    void validateWorkspaceAnalysis() {
        std::cout << "\n--- Workspace Analysis Validation ---" << std::endl;

        // Test height range calculation
        auto height_range = model.calculateHeightRange();
        if (height_range.first < 0 || height_range.second < 0) {
            results.addError("Invalid height range calculated");
        } else {
            std::cout << "Height range: [" << height_range.first << ", " << height_range.second << "] mm" << std::endl;
        }

        // Test workspace boundaries
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Analyzing workspace for leg " << leg << "..." << std::endl;

            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();
            double min_z = std::numeric_limits<double>::max();
            double max_z = std::numeric_limits<double>::lowest();

            int valid_configs = 0;

            for (double coxa = model.getParams().coxa_angle_limits[0];
                 coxa <= model.getParams().coxa_angle_limits[1];
                 coxa += step_coxa) {
                for (double femur = model.getParams().femur_angle_limits[0];
                     femur <= model.getParams().femur_angle_limits[1];
                     femur += step_femur) {
                    for (double tibia = model.getParams().tibia_angle_limits[0];
                         tibia <= model.getParams().tibia_angle_limits[1];
                         tibia += step_tibia) {

                        JointAngles angles(math_utils::degreesToRadians(coxa), math_utils::degreesToRadians(femur), math_utils::degreesToRadians(tibia));
                        if (!model.checkJointLimits(leg, angles)) {
                            continue;
                        }

                        valid_configs++;
                        Point3D global_pos = model.forwardKinematicsGlobalCoordinates(leg, angles);
                        Point3D local_pos = model.transformGlobalToLocalCoordinates(leg, global_pos, angles);

                        min_x = std::min(min_x, local_pos.x);
                        max_x = std::max(max_x, local_pos.x);
                        min_y = std::min(min_y, local_pos.y);
                        max_y = std::max(max_y, local_pos.y);
                        min_z = std::min(min_z, local_pos.z);
                        max_z = std::max(max_z, local_pos.z);
                    }
                }
            }

            std::cout << "  Valid configurations: " << valid_configs << std::endl;
            std::cout << "  Local workspace X: [" << min_x << ", " << max_x << "] mm" << std::endl;
            std::cout << "  Local workspace Y: [" << min_y << ", " << max_y << "] mm" << std::endl;
            std::cout << "  Local workspace Z: [" << min_z << ", " << max_z << "] mm" << std::endl;

            // Validate workspace is reasonable
            if (max_x - min_x < 50 || max_y - min_y < 50 || max_z - min_z < 50) {
                results.addError("Leg " + std::to_string(leg) + " workspace too small");
            }
        }
    }

    void validateMultipleIKMethods() {
        std::cout << "\n--- Multiple IK Methods Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing multiple IK methods for leg " << leg << "..." << std::endl;

            // Test different target positions
            std::vector<Point3D> test_targets = {
                Point3D(100, 0, -80),  // Forward reach
                Point3D(-100, 0, -80), // Backward reach
                Point3D(0, 100, -80),  // Side reach
                Point3D(0, -100, -80), // Opposite side reach
                Point3D(80, 80, -208), // Diagonal reach
                Point3D(0, 0, -60)     // High position
            };

            JointAngles current_angles(0, -30, 45);

            for (const auto &target : test_targets) {
                results.total_tests++;

                // Test different IK methods
                JointAngles ik1 = model.inverseKinematicsGlobalCoordinates(leg, target);
                JointAngles ik2 = model.inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, target);
                JointAngles ik3 = model.inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, target);
                JointAngles ik4 = model.solveIKLocalCoordinates(leg, target, current_angles);

                // Verify all methods produce similar results
                std::vector<JointAngles> solutions = {ik1, ik2, ik3, ik4};
                std::vector<Point3D> positions;

                for (const auto &sol : solutions) {
                    if (model.checkJointLimits(leg, sol)) {
                        positions.push_back(model.forwardKinematicsGlobalCoordinates(leg, sol));
                    }
                }

                if (positions.size() > 1) {
                    // Check consistency between solutions
                    for (size_t i = 1; i < positions.size(); ++i) {
                        double error = math_utils::distance(positions[0], positions[i]);
                        if (error > position_tolerance * 2) { // Allow more tolerance for different methods
                            results.addError("Leg " + std::to_string(leg) +
                                             " IK method inconsistency: " + std::to_string(error) + "mm");
                        }
                    }
                }
            }
        }
    }

    void validateJointLimits() {
        std::cout << "\n--- Joint Limits Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing joint limits for leg " << leg << "..." << std::endl;

            results.total_tests++;

            // Test valid angles
            JointAngles valid_angles(math_utils::degreesToRadians(30), math_utils::degreesToRadians(-30), math_utils::degreesToRadians(20));
            if (!model.checkJointLimits(leg, valid_angles)) {
                results.addError("Leg " + std::to_string(leg) + " incorrectly rejected valid angles");
            }

            // Test invalid angles
            JointAngles invalid_coxa(math_utils::degreesToRadians(100), 0, 0);  // Beyond coxa limits
            JointAngles invalid_femur(0, math_utils::degreesToRadians(100), 0); // Beyond femur limits
            JointAngles invalid_tibia(0, 0, math_utils::degreesToRadians(100)); // Beyond tibia limits

            if (model.checkJointLimits(leg, invalid_coxa)) {
                results.addError("Leg " + std::to_string(leg) + " incorrectly accepted invalid coxa angle");
            }
            if (model.checkJointLimits(leg, invalid_femur)) {
                results.addError("Leg " + std::to_string(leg) + " incorrectly accepted invalid femur angle");
            }
            if (model.checkJointLimits(leg, invalid_tibia)) {
                results.addError("Leg " + std::to_string(leg) + " incorrectly accepted invalid tibia angle");
            }

            // Test angle normalization
            double normalized_rad = model.normalizeAngle(math_utils::degreesToRadians(370.0)); // Should normalize to 10 degrees
            double normalized_deg = math_utils::radiansToDegrees(normalized_rad);
            if (std::abs(normalized_deg - 10.0) > angle_tolerance) {
                results.addError("Angle normalization error: expected 10°, got " + std::to_string(normalized_deg) + "°");
            }

            // Test angle constraining
            double constrained_rad = model.constrainAngle(math_utils::degreesToRadians(100.0),
                                                          math_utils::degreesToRadians(-45.0),
                                                          math_utils::degreesToRadians(45.0)); // Should constrain to 45 degrees
            double constrained_deg = math_utils::radiansToDegrees(constrained_rad);
            if (std::abs(constrained_deg - 45.0) > angle_tolerance) {
                results.addError("Angle constraining error: expected 45°, got " + std::to_string(constrained_deg) + "°");
            }
        }
    }

    void validateEdgeCases() {
        std::cout << "\n--- Edge Cases Validation ---" << std::endl;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::cout << "Testing edge cases for leg " << leg << "..." << std::endl;

            results.total_tests++;

            // Test zero angles
            JointAngles zero_angles(0, 0, 0);
            Point3D zero_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);
            JointAngles zero_ik = model.inverseKinematicsGlobalCoordinates(leg, zero_pos);

            double error = math_utils::distance(
                model.forwardKinematicsGlobalCoordinates(leg, zero_ik),
                zero_pos);
            if (error > position_tolerance) {
                results.addError("Leg " + std::to_string(leg) +
                                 " zero angles FK/IK error: " + std::to_string(error) + "mm");
            }

            // Test limit angles
            const auto &params = model.getParams();
            JointAngles limit_angles(
                params.coxa_angle_limits[1],
                params.femur_angle_limits[1],
                params.tibia_angle_limits[1]);

            if (model.checkJointLimits(leg, limit_angles)) {
                Point3D limit_pos = model.forwardKinematicsGlobalCoordinates(leg, limit_angles);
                JointAngles limit_ik = model.inverseKinematicsGlobalCoordinates(leg, limit_pos);

                error = math_utils::distance(
                    model.forwardKinematicsGlobalCoordinates(leg, limit_ik),
                    limit_pos);
                if (error > position_tolerance) {
                    results.addError("Leg " + std::to_string(leg) +
                                     " limit angles FK/IK error: " + std::to_string(error) + "mm");
                }
            }

            // Test very small movements
            JointAngles small_angles(math_utils::degreesToRadians(0.1), math_utils::degreesToRadians(-0.1), math_utils::degreesToRadians(0.1));
            if (model.checkJointLimits(leg, small_angles)) {
                Point3D small_pos = model.forwardKinematicsGlobalCoordinates(leg, small_angles);
                JointAngles small_ik = model.inverseKinematicsGlobalCoordinates(leg, small_pos);

                error = math_utils::distance(
                    model.forwardKinematicsGlobalCoordinates(leg, small_ik),
                    small_pos);
                if (error > position_tolerance) {
                    results.addError("Leg " + std::to_string(leg) +
                                     " small angles FK/IK error: " + std::to_string(error) + "mm");
                }
            }
        }
    }
};

int main() {
    try {
        // Configure robot parameters
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

        // Enable advanced features
        p.use_custom_dh_parameters = false;
        p.ik.max_iterations = 50;
        p.ik.pos_threshold_mm = 0.5;

        std::cout << "=== HexaMotion Comprehensive Kinematics Validation Test ===" << std::endl;
        std::cout << "Robot parameters:" << std::endl;
        std::cout << "  Hexagon radius: " << p.hexagon_radius << " mm" << std::endl;
        std::cout << "  Coxa length: " << p.coxa_length << " mm" << std::endl;
        std::cout << "  Femur length: " << p.femur_length << " mm" << std::endl;
        std::cout << "  Tibia length: " << p.tibia_length << " mm" << std::endl;
        std::cout << "  Robot height: " << p.robot_height << " mm" << std::endl;

        ComprehensiveKinematicsValidator validator(p);
        validator.runAllValidations();

        std::cout << "\n=== TEST COMPLETED ===" << std::endl;
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}