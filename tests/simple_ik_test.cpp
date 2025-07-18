#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations for coordinate transformation functions
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos);
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos);
JointAngles solveIKLocal(const RobotModel &model, int leg, const Point3D &local_target);

JointAngles solveIKLocalCurrent(const RobotModel &model, int leg, const JointAngles &start_angles, const Point3D &local_target);

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
    std::cout << "=== Inverse Kinematics Validation Test ===" << std::endl;

    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {-30.0f, -90.0f, -150.0f, 150.0f, 90.0f, 30.0f};
    bool ok = true;

    // Test 1: Simple Horizontal Test (original functionality)
    std::cout << "\n--- Test 1: Simple Horizontal Test ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles zero_angles(0, 0, 0);
        Point3D target = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

        JointAngles ik = model.inverseKinematicsCurrentGlobalCoordinates(leg, zero_angles, target);
        Point3D fk = model.forwardKinematicsGlobalCoordinates(leg, ik);
        double err = sqrt(pow(target.x - fk.x, 2) +
                          pow(target.y - fk.y, 2) +
                          pow(target.z - fk.z, 2));
        std::cout << "Leg " << leg << ": target(" << target.x << ", " << target.y
                  << ", " << target.z << ") -> IK(" << ik.coxa << ", "
                  << ik.femur << ", " << ik.tibia << ") FK(" << fk.x << ", "
                  << fk.y << ", " << fk.z << ") error=" << err << std::endl;
        if (std::abs(ik.coxa) > 1e-3f || std::abs(ik.femur) > 1e-3f || std::abs(ik.tibia) > 1e-3f || err > 1e-2f) {
            ok = false;
        }
    }

    // Test 2: Local IK validation
    std::cout << "\n--- Test 2: Local Coordinate IK Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Test a local target position
        Point3D local_target(208.0, 30.0, -180.0); // Reachable position in local coordinates

        JointAngles ik_local = solveIKLocal(model, leg, local_target);

        // Verify by transforming to global and using global IK
        Point3D global_target = transformLocalToGlobal(model, leg, local_target);
        JointAngles ik_global = model.inverseKinematicsGlobalCoordinates(leg, global_target);

        // Check consistency between local and global IK
        double angle_diff = std::sqrt(std::pow(ik_local.coxa - ik_global.coxa, 2) +
                                      std::pow(ik_local.femur - ik_global.femur, 2) +
                                      std::pow(ik_local.tibia - ik_global.tibia, 2));

        std::cout << "Leg " << leg << " local IK: target(" << local_target.x << ", " << local_target.y << ", " << local_target.z
                  << ") -> local_IK(" << ik_local.coxa << ", " << ik_local.femur << ", " << ik_local.tibia
                  << ") global_IK(" << ik_global.coxa << ", " << ik_global.femur << ", " << ik_global.tibia
                  << ") angle_diff=" << angle_diff << std::endl;

        if (angle_diff > 1.0f) { // 1 degree tolerance
            ok = false;
        }
    }

    // Test 3: IK-FK coherence validation (enhanced)
    std::cout << "\n--- Test 3: Enhanced IK-FK Coherence Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Use the same relative joint angles for every leg (converted from degrees to radians)
        JointAngles test_angles(0, 20 * M_PI / 180.0, 20 * M_PI / 180.0); // 0°, 20°, 20° -> 0, 0.349, 0.349 rad
        Point3D target = model.forwardKinematicsGlobalCoordinates(leg, test_angles);

        // Usa test_angles como estimación inicial
        JointAngles ik = model.inverseKinematicsCurrentGlobalCoordinates(leg, test_angles, target);
        Point3D fk = model.forwardKinematicsGlobalCoordinates(leg, ik);

        double position_error = std::sqrt(std::pow(fk.x - target.x, 2) +
                                          std::pow(fk.y - target.y, 2) +
                                          std::pow(fk.z - target.z, 2));

        std::cout << "Leg " << leg << ": target(" << target.x << ", " << target.y << ", " << target.z
                  << ") -> IK(" << ik.coxa << ", " << ik.femur << ", " << ik.tibia
                  << ") FK(" << fk.x << ", " << fk.y << ", " << fk.z
                  << ") error=" << position_error << std::endl;

        if (position_error > 2.0f) {
            ok = false;
        }
    }

    // Test 4: Local-Global-Local Round-Trip Validation
    std::cout << "\n--- Test 4: Local-Global-Local Round-Trip Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Elige un objetivo local arbitrario pero alcanzable
        Point3D local_target(100.0, 0.0, -180.0);
        JointAngles zero_angles(0, 0, 0);
        // Transforma a global usando los ángulos cero
        Point3D global_target = model.transformLocalToGlobalCoordinates(leg, local_target, zero_angles);
        // Transforma de vuelta a local usando los mismos ángulos
        Point3D local_roundtrip = model.transformGlobalToLocalCoordinates(leg, global_target, zero_angles);
        // Calcula el error de round-trip
        double roundtrip_error = std::sqrt(
            std::pow(local_roundtrip.x - local_target.x, 2) +
            std::pow(local_roundtrip.y - local_target.y, 2) +
            std::pow(local_roundtrip.z - local_target.z, 2));
        std::cout << "Leg " << leg << ": local_target(" << local_target.x << ", " << local_target.y << ", " << local_target.z
                  << ") -> global(" << global_target.x << ", " << global_target.y << ", " << global_target.z
                  << ") -> local_roundtrip(" << local_roundtrip.x << ", " << local_roundtrip.y << ", " << local_roundtrip.z
                  << ") error=" << roundtrip_error << std::endl;
        if (roundtrip_error > 1.0f) {
            std::cout << "  *** FAIL: Local-Global-Local round-trip error too high ***" << std::endl;
            ok = false;
        }
    }

    // Test 5: Local vs Global IK Round-Trip Validation
    std::cout << "\n--- Test 5: Local vs Global IK Round-Trip Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Test multiple positions
        Point3D test_positions[3] = {
            Point3D(130.0, 0.0, -150.0),  // Forward position
            Point3D(100.0, 50.0, -200.0), // Side position
            Point3D(80.0, -30.0, -160.0)  // Diagonal position
        };

        for (int i = 0; i < 3; ++i) {
            Point3D local_target = test_positions[i];

            // Solve in local coordinates
            JointAngles ik_local = solveIKLocal(model, leg, local_target);

            // Convert to global and solve in global coordinates
            Point3D global_target = transformLocalToGlobal(model, leg, local_target);
            JointAngles ik_global = model.inverseKinematicsGlobalCoordinates(leg, global_target);

            // Verify both solutions reach the same position
            Point3D fk_local = model.forwardKinematicsGlobalCoordinates(leg, ik_local);
            Point3D fk_global = model.forwardKinematicsGlobalCoordinates(leg, ik_global);

            double consistency_error = std::sqrt(std::pow(fk_local.x - fk_global.x, 2) +
                                                 std::pow(fk_local.y - fk_global.y, 2) +
                                                 std::pow(fk_local.z - fk_global.z, 2));

            std::cout << "Leg " << leg << " pos " << i << ": local_target(" << local_target.x << ", " << local_target.y << ", " << local_target.z
                      << ") consistency_error=" << consistency_error << std::endl;

            if (consistency_error > 1.0f) {
                ok = false;
            }
        }
    }

    if (ok) {
        std::cout << "\n✓ All inverse kinematics tests passed!" << std::endl;
        return 0;
    } else {
        std::cerr << "\n✗ Some inverse kinematics tests failed." << std::endl;
        return 1;
    }
}

// ===== COORDINATE TRANSFORMATION AND LOCAL IK FUNCTION IMPLEMENTATIONS =====

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

/**
 * @brief Solve inverse kinematics using local leg coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param local_target Target position in local leg coordinates
 * @return Joint angles to reach the target position
 */
JointAngles solveIKLocal(const RobotModel &model, int leg, const Point3D &local_target) {
    // Transform local target to global coordinates
    Point3D global_target = transformLocalToGlobal(model, leg, local_target);

    // Use the robot model's global IK solver
    JointAngles result = model.inverseKinematicsGlobalCoordinates(leg, global_target);

    return result;
}

// Helper: IK with starting angles in local frame
JointAngles solveIKLocalCurrent(const RobotModel &model, int leg, const JointAngles &start_angles, const Point3D &local_target) {
    Point3D global_target = transformLocalToGlobal(model, leg, local_target);
    return model.inverseKinematicsCurrentGlobalCoordinates(leg, start_angles, global_target);
}
