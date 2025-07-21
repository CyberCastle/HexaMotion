#include "leg.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations for test helper functions
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos);
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos);

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

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Advanced IK Validation Test ===" << std::endl;
    std::cout << "Testing applyAdvancedIK and solveDeltaIK methods" << std::endl;

    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {-30.0f, -90.0f, -150.0f, 150.0f, 90.0f, 30.0f};
    bool ok = true;

    // Test 1: Advanced IK vs Standard IK Comparison
    std::cout << "\n--- Test 1: Advanced IK vs Standard IK Comparison ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles zero_angles(0, 0, 0);
        Point3D current_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        // Target position: slightly displaced from current
        Point3D target_pos = current_pos + Point3D(10.0, 5.0, -10.0);

        // Standard IK
        JointAngles standard_ik = model.inverseKinematicsCurrentGlobalCoordinates(leg, zero_angles, target_pos);
        Point3D standard_fk = model.forwardKinematicsGlobalCoordinates(leg, standard_ik);
        double standard_error = sqrt(pow(target_pos.x - standard_fk.x, 2) +
                                     pow(target_pos.y - standard_fk.y, 2) +
                                     pow(target_pos.z - standard_fk.z, 2));

        // Advanced IK
        JointAngles advanced_ik = model.applyAdvancedIK(leg, current_pos, target_pos, zero_angles);
        Point3D advanced_fk = model.forwardKinematicsGlobalCoordinates(leg, advanced_ik);
        double advanced_error = sqrt(pow(target_pos.x - advanced_fk.x, 2) +
                                     pow(target_pos.y - advanced_fk.y, 2) +
                                     pow(target_pos.z - advanced_fk.z, 2));

        std::cout << "Leg " << leg << ":" << std::endl;
        std::cout << "  Target: (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;
        std::cout << "  Standard IK: error=" << standard_error << " mm" << std::endl;
        std::cout << "  Advanced IK: error=" << advanced_error << " mm" << std::endl;

        if (standard_error > 2.0f || advanced_error > 2.0f) {
            std::cout << "  *** FAIL: IK error too high ***" << std::endl;
            ok = false;
        }
    }

    // Test 2: Delta IK Core Function Validation
    std::cout << "\n--- Test 2: Delta IK Core Function Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles current_angles(math_utils::degreesToRadians(10.0),
                                   math_utils::degreesToRadians(-20.0),
                                   math_utils::degreesToRadians(15.0));

        // Create a small position delta
        Eigen::MatrixXd delta = Eigen::Matrix<double, 6, 1>::Zero();
        delta(0) = 5.0;  // 5mm in x
        delta(1) = 3.0;  // 3mm in y
        delta(2) = -2.0; // -2mm in z

        // Test solveDeltaIK
        Eigen::Vector3d joint_delta = model.solveDeltaIK(leg, delta, current_angles);

        // Apply the delta to get new angles
        JointAngles new_angles = current_angles;
        new_angles.coxa += joint_delta(0);
        new_angles.femur += joint_delta(1);
        new_angles.tibia += joint_delta(2);

        // Verify the position change
        Point3D old_pos = model.forwardKinematicsGlobalCoordinates(leg, current_angles);
        Point3D new_pos = model.forwardKinematicsGlobalCoordinates(leg, new_angles);
        Point3D actual_delta = new_pos - old_pos;

        // Transform expected delta to global coordinates for comparison
        Point3D old_pos_local = model.transformGlobalToLocalCoordinates(leg, old_pos, current_angles);
        Point3D expected_new_local = old_pos_local + Point3D(delta(0), delta(1), delta(2));
        Point3D expected_new_global = model.transformLocalToGlobalCoordinates(leg, expected_new_local, current_angles);
        Point3D expected_delta = expected_new_global - old_pos;

        double delta_error = sqrt(pow(actual_delta.x - expected_delta.x, 2) +
                                  pow(actual_delta.y - expected_delta.y, 2) +
                                  pow(actual_delta.z - expected_delta.z, 2));

        std::cout << "Leg " << leg << ":" << std::endl;
        std::cout << "  Input delta: (" << delta(0) << ", " << delta(1) << ", " << delta(2) << ")" << std::endl;
        std::cout << "  Joint delta: (" << joint_delta(0) << ", " << joint_delta(1) << ", " << joint_delta(2) << ")" << std::endl;
        std::cout << "  Expected pos delta: (" << expected_delta.x << ", " << expected_delta.y << ", " << expected_delta.z << ")" << std::endl;
        std::cout << "  Actual pos delta: (" << actual_delta.x << ", " << actual_delta.y << ", " << actual_delta.z << ")" << std::endl;
        std::cout << "  Delta error: " << delta_error << " mm" << std::endl;

        if (delta_error > 1.0f) {
            std::cout << "  *** FAIL: Delta error too high ***" << std::endl;
            ok = false;
        }
    }

    // Test 3: Leg Class Advanced IK Integration Test
    std::cout << "\n--- Test 3: Leg Class Advanced IK Integration Test ---" << std::endl;
    for (int leg_id = 0; leg_id < NUM_LEGS; ++leg_id) {
        Leg leg(leg_id, model);

        // Initialize leg with default stance position
        JointAngles initial_angles(0.0, math_utils::degreesToRadians(-20.0), math_utils::degreesToRadians(20.0));
        leg.setJointAngles(initial_angles);
        leg.updateTipPosition();

        Point3D initial_pos = leg.getCurrentTipPositionGlobal();

        // Set a target position
        Point3D target_pos = initial_pos + Point3D(15.0, 10.0, -5.0);

        // Use advanced IK through Leg class
        bool ik_success = leg.applyAdvancedIK(target_pos);

        if (ik_success) {
            Point3D final_pos = leg.getCurrentTipPositionGlobal();
            double position_error = sqrt(pow(target_pos.x - final_pos.x, 2) +
                                         pow(target_pos.y - final_pos.y, 2) +
                                         pow(target_pos.z - final_pos.z, 2));

            JointAngles final_angles = leg.getJointAngles();

            std::cout << "Leg " << leg_id << ":" << std::endl;
            std::cout << "  Initial pos: (" << initial_pos.x << ", " << initial_pos.y << ", " << initial_pos.z << ")" << std::endl;
            std::cout << "  Target pos: (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;
            std::cout << "  Final pos: (" << final_pos.x << ", " << final_pos.y << ", " << final_pos.z << ")" << std::endl;
            std::cout << "  Position error: " << position_error << " mm" << std::endl;
            std::cout << "  Final angles (deg): coxa=" << math_utils::radiansToDegrees(final_angles.coxa)
                      << ", femur=" << math_utils::radiansToDegrees(final_angles.femur)
                      << ", tibia=" << math_utils::radiansToDegrees(final_angles.tibia) << std::endl;

            if (position_error > 2.0f) {
                std::cout << "  *** FAIL: Position error too high ***" << std::endl;
                ok = false;
            }
        } else {
            std::cout << "Leg " << leg_id << ": *** FAIL: Advanced IK failed ***" << std::endl;
            ok = false;
        }
    }

    // Test 4: Joint Limit Cost Function Validation
    std::cout << "\n--- Test 4: Joint Limit Cost Function Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Test near joint limits
        JointAngles near_limits(math_utils::degreesToRadians(60.0),  // Near coxa limit (65°)
                                math_utils::degreesToRadians(-70.0), // Near femur limit (-75°)
                                math_utils::degreesToRadians(40.0)); // Near tibia limit (45°)

        // Small velocities
        Eigen::Vector3d joint_velocities;
        joint_velocities << 0.1, -0.2, 0.15;

        // Calculate cost gradient
        Eigen::Vector3d cost_gradient = model.calculateJointLimitCostGradient(near_limits, joint_velocities, leg);

        std::cout << "Leg " << leg << ":" << std::endl;
        std::cout << "  Near-limit angles (deg): coxa=" << math_utils::radiansToDegrees(near_limits.coxa)
                  << ", femur=" << math_utils::radiansToDegrees(near_limits.femur)
                  << ", tibia=" << math_utils::radiansToDegrees(near_limits.tibia) << std::endl;
        std::cout << "  Cost gradient: (" << cost_gradient(0) << ", " << cost_gradient(1) << ", " << cost_gradient(2) << ")" << std::endl;

        // The gradient should be non-zero for joints near limits
        double gradient_magnitude = cost_gradient.norm();
        if (gradient_magnitude < 1e-6) {
            std::cout << "  *** WARNING: Cost gradient is very small near limits ***" << std::endl;
        }
    }

    // Test 5: Advanced IK Convergence with Multiple Steps
    std::cout << "\n--- Test 5: Advanced IK Convergence with Multiple Steps ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles start_angles(0.0, math_utils::degreesToRadians(-30.0), math_utils::degreesToRadians(30.0));
        Point3D start_pos = model.forwardKinematicsGlobalCoordinates(leg, start_angles);

        // Large displacement target
        Point3D target_pos = start_pos + Point3D(50.0, 30.0, -20.0);

        // Apply multiple advanced IK steps to reach target
        JointAngles current_angles = start_angles;
        Point3D current_pos = start_pos;
        int steps = 0;
        const int max_steps = 10;
        const double convergence_threshold = 1.0; // 1mm

        while (steps < max_steps) {
            JointAngles new_angles = model.applyAdvancedIK(leg, current_pos, target_pos, current_angles, 0.02);
            Point3D new_pos = model.forwardKinematicsGlobalCoordinates(leg, new_angles);

            double error = sqrt(pow(target_pos.x - new_pos.x, 2) +
                                pow(target_pos.y - new_pos.y, 2) +
                                pow(target_pos.z - new_pos.z, 2));

            current_angles = new_angles;
            current_pos = new_pos;
            steps++;

            if (error < convergence_threshold) {
                break;
            }
        }

        double final_error = sqrt(pow(target_pos.x - current_pos.x, 2) +
                                  pow(target_pos.y - current_pos.y, 2) +
                                  pow(target_pos.z - current_pos.z, 2));

        std::cout << "Leg " << leg << ":" << std::endl;
        std::cout << "  Start pos: (" << start_pos.x << ", " << start_pos.y << ", " << start_pos.z << ")" << std::endl;
        std::cout << "  Target pos: (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;
        std::cout << "  Final pos: (" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ")" << std::endl;
        std::cout << "  Steps: " << steps << ", Final error: " << final_error << " mm" << std::endl;

        if (final_error > 5.0f) { // More lenient for large displacements
            std::cout << "  *** FAIL: Failed to converge ***" << std::endl;
            ok = false;
        }
    }

    // Final Results
    std::cout << "\n=== Test Results ===" << std::endl;
    if (ok) {
        std::cout << "✅ ALL TESTS PASSED - Advanced IK implementation is working correctly" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME TESTS FAILED - Advanced IK implementation needs review" << std::endl;
        return 1;
    }
}

// ===== HELPER FUNCTION IMPLEMENTATIONS =====

/**
 * @brief Transform a point from global robot coordinates to local leg coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param global_pos Position in global robot coordinates
 * @return Position in local leg coordinates
 */
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos) {
    // Use RobotModel's transformation functions with zero angles for base transform
    JointAngles zero_angles(0, 0, 0);
    return model.transformGlobalToLocalCoordinates(leg, global_pos, zero_angles);
}

/**
 * @brief Transform a point from local leg coordinates to global robot coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param local_pos Position in local leg coordinates
 * @return Position in global robot coordinates
 */
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos) {
    // Use RobotModel's transformation functions with zero angles for base transform
    JointAngles zero_angles(0, 0, 0);
    return model.transformLocalToGlobalCoordinates(leg, local_pos, zero_angles);
}
