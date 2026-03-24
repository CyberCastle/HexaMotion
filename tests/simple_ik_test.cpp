#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

/** Forward declarations for coordinate transformation functions. */
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
    std::cout << "=== Inverse Kinematics Validation Test ===" << std::endl;

    bool ok = true;

    /** Test 1: Simple horizontal test (original functionality). */
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

    /** Test 2: Local IK validation. */
    std::cout << "\n--- Test 2: Local Coordinate IK Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        /** Test a local target position (reachable in local coordinates). */
        Point3D local_target(208.0, 30.0, -180.0);

        JointAngles ik_local = solveIKLocal(model, leg, local_target);

        /** Verify by transforming to global and using global IK. */
        Point3D global_target = transformLocalToGlobal(model, leg, local_target);
        JointAngles ik_global = model.inverseKinematicsGlobalCoordinates(leg, global_target);

        /** Check consistency between local and global IK. */
        double angle_diff = std::sqrt(std::pow(ik_local.coxa - ik_global.coxa, 2) +
                                      std::pow(ik_local.femur - ik_global.femur, 2) +
                                      std::pow(ik_local.tibia - ik_global.tibia, 2));

        std::cout << "Leg " << leg << " local IK: target(" << local_target.x << ", " << local_target.y << ", " << local_target.z
                  << ") -> local_IK(" << ik_local.coxa << ", " << ik_local.femur << ", " << ik_local.tibia
                  << ") global_IK(" << ik_global.coxa << ", " << ik_global.femur << ", " << ik_global.tibia
                  << ") angle_diff=" << angle_diff << std::endl;

        /** 1 degree tolerance. */
        if (angle_diff > 1.0f) {
            ok = false;
        }
    }

    /** Test 3: IK-FK coherence validation (enhanced). */
    std::cout << "\n--- Test 3: Enhanced IK-FK Coherence Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        /** Use the same relative joint angles for every leg (converted from degrees to radians). */
        /** 0, 20, 20 degrees -> 0, 0.349, 0.349 rad. */
        JointAngles test_angles(0, math_utils::degreesToRadians(20.0), math_utils::degreesToRadians(20.0));
        Point3D target = model.forwardKinematicsGlobalCoordinates(leg, test_angles);

        /** Use test_angles as the initial estimate. */
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

    /** Test 4: Local-Global-Local round-trip validation. */
    std::cout << "\n--- Test 4: Local-Global-Local Round-Trip Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        /** Choose an arbitrary but reachable local target. */
        Point3D local_target(100.0, 0.0, -180.0);
        JointAngles zero_angles(0, 0, 0);
        /** Transform to global using zero angles. */
        Point3D global_target = model.transformLocalToGlobalCoordinates(leg, local_target, zero_angles);
        /** Transform back to local using the same angles. */
        Point3D local_roundtrip = model.transformGlobalToLocalCoordinates(leg, global_target, zero_angles);
        /** Calculate round-trip error. */
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

    /** Test 5: Local vs global IK round-trip validation. */
    std::cout << "\n--- Test 5: Local vs Global IK Round-Trip Validation ---" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        /** Test multiple positions. */
        Point3D test_positions[3] = {
            /** Forward position. */
            Point3D(130.0, 0.0, -150.0),
            /** Side position. */
            Point3D(100.0, 50.0, -200.0),
            /** Diagonal position. */
            Point3D(80.0, -30.0, -160.0)};

        for (int i = 0; i < 3; ++i) {
            Point3D local_target = test_positions[i];

            /** Solve in local coordinates. */
            JointAngles ik_local = solveIKLocal(model, leg, local_target);

            /** Convert to global and solve in global coordinates. */
            Point3D global_target = transformLocalToGlobal(model, leg, local_target);
            JointAngles ik_global = model.inverseKinematicsGlobalCoordinates(leg, global_target);

            /** Verify both solutions reach the same position. */
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

    /** Test 6: Canonical geometric oracle — independent FK position check.
     *
     *  At zero joint angles the tip position is derived from pure geometry,
     *  without any DH matrix:
     *    reach = hexagon_radius + coxa + femur = 200 + 50 + 101 = 351 mm
     *    global_x = reach * cos(base_angle)
     *    global_y = reach * sin(base_angle)
     *    global_z = -tibia = -208 mm
     */
    std::cout << "\n--- Test 6: Canonical Geometric Oracle (Independent FK Position) ---" << std::endl;
    {
        const double base_angles_deg[NUM_LEGS] = {-30.0, -90.0, -150.0, 150.0, 90.0, 30.0};
        const double reach = p.hexagon_radius + p.coxa_length + p.femur_length; // 351 mm
        const double z_expected = -p.tibia_length;                              // -208 mm

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            double theta = math_utils::degreesToRadians(base_angles_deg[leg]);
            double x_expected = reach * std::cos(theta);
            double y_expected = reach * std::sin(theta);

            JointAngles zero_angles(0, 0, 0);
            Point3D fk_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);

            double err = std::sqrt(std::pow(fk_pos.x - x_expected, 2) +
                                   std::pow(fk_pos.y - y_expected, 2) +
                                   std::pow(fk_pos.z - z_expected, 2));

            std::cout << "Leg " << leg << ": expected(" << x_expected << ", " << y_expected
                      << ", " << z_expected << ") FK(" << fk_pos.x << ", " << fk_pos.y
                      << ", " << fk_pos.z << ") error=" << err << std::endl;

            if (err > 0.5) {
                std::cout << "  *** FAIL: FK at zero angles deviates from independent geometry ***" << std::endl;
                ok = false;
            }
        }
    }

    /** Test 7: Unreachable target detection.
     *
     *  A target far outside the workspace should yield a large FK residual
     *  after IK, confirming the solver does not silently succeed.
     */
    std::cout << "\n--- Test 7: Unreachable Target Detection ---" << std::endl;
    {
        const double max_reach = p.hexagon_radius + p.coxa_length + p.femur_length + p.tibia_length; // 559 mm
        Point3D far_target(max_reach * 3.0, 0.0, 0.0);                                               // well beyond workspace

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            JointAngles ik = model.inverseKinematicsGlobalCoordinates(leg, far_target);
            Point3D fk = model.forwardKinematicsGlobalCoordinates(leg, ik);
            double residual = std::sqrt(std::pow(fk.x - far_target.x, 2) +
                                        std::pow(fk.y - far_target.y, 2) +
                                        std::pow(fk.z - far_target.z, 2));

            std::cout << "Leg " << leg << ": far_target(" << far_target.x << ", " << far_target.y
                      << ", " << far_target.z << ") IK residual=" << residual << std::endl;

            /** Residual must be large — the solver cannot reach this point. */
            if (residual < 100.0) {
                std::cout << "  *** FAIL: IK claims to reach an unreachable target ***" << std::endl;
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

/** Coordinate transformation and local IK function implementations. */

/**
 * @brief Transform a point from global robot coordinates to local leg coordinates
 * @param model Robot model containing DH parameters
 * @param leg Leg index (0-5)
 * @param global_pos Position in global robot coordinates
 * @return Position in local leg coordinates
 */
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos) {
    /** Use RobotModel transformations with zero angles for base transform. */
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
    /** Use RobotModel transformations with zero angles for base transform. */
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
    /** Transform local target to global coordinates. */
    Point3D global_target = transformLocalToGlobal(model, leg, local_target);

    /** Use the robot model's global IK solver. */
    JointAngles result = model.inverseKinematicsGlobalCoordinates(leg, global_target);

    return result;
}

/** Helper: IK with starting angles in local frame. */
JointAngles solveIKLocalCurrent(const RobotModel &model, int leg, const JointAngles &start_angles, const Point3D &local_target) {
    Point3D global_target = transformLocalToGlobal(model, leg, local_target);
    return model.inverseKinematicsCurrentGlobalCoordinates(leg, start_angles, global_target);
}
