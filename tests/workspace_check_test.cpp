#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 120;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Workspace Check Test ===" << std::endl;

    // Test the problematic target
    Point3D local_target(80.0, -30.0, -160.0);

    std::cout << "\n--- Checking if target ("
              << local_target.x << ", " << local_target.y << ", " << local_target.z
              << ") is reachable ---" << std::endl;

    // Calculate max and min reach
    double max_reach = p.coxa_length + p.femur_length + p.tibia_length;
    double min_reach = std::abs(p.femur_length - p.tibia_length);
    double target_distance = std::sqrt(local_target.x * local_target.x +
                                       local_target.y * local_target.y +
                                       local_target.z * local_target.z);

    std::cout << "Max reach: " << max_reach << " mm" << std::endl;
    std::cout << "Min reach: " << min_reach << " mm" << std::endl;
    std::cout << "Target distance: " << target_distance << " mm" << std::endl;
    std::cout << "Target reachable: " << (target_distance <= max_reach && target_distance >= min_reach ? "YES" : "NO") << std::endl;

    // Test with different targets that should be reachable
    std::vector<Point3D> test_targets = {
        Point3D(120.0, 0.0, -150.0),   // Forward position
        Point3D(100.0, 0.0, -180.0),   // Forward, lower
        Point3D(80.0, 0.0, -160.0),    // Similar to problematic but centered
        Point3D(60.0, 0.0, -140.0),    // Closer, higher
        Point3D(140.0, 0.0, -160.0),   // Further forward
    };

    std::cout << "\n--- Testing different targets ---" << std::endl;

    for (int i = 0; i < test_targets.size(); ++i) {
        Point3D target = test_targets[i];
        double dist = std::sqrt(target.x * target.x + target.y * target.y + target.z * target.z);

        std::cout << "\nTarget " << i << ": (" << target.x << ", " << target.y << ", " << target.z
                  << ") distance=" << dist << " reachable="
                  << (dist <= max_reach && dist >= min_reach ? "YES" : "NO") << std::endl;

        // Test IK for leg 0
        JointAngles zero_angles(0, 0, 0);
        Point3D global_target = model.transformLocalToGlobalCoordinates(0, target, zero_angles);
        JointAngles ik = model.inverseKinematics(0, global_target);
        Point3D fk = model.forwardKinematics(0, ik);
        Point3D fk_local = model.transformGlobalToLocalCoordinates(0, fk, zero_angles);

        double error = std::sqrt(
            std::pow(fk_local.x - target.x, 2) +
            std::pow(fk_local.y - target.y, 2) +
            std::pow(fk_local.z - target.z, 2));

        std::cout << "  IK angles: (" << ik.coxa << ", " << ik.femur << ", " << ik.tibia << ")" << std::endl;
        std::cout << "  FK result: (" << fk_local.x << ", " << fk_local.y << ", " << fk_local.z << ")" << std::endl;
        std::cout << "  Error: " << error << std::endl;
        std::cout << "  Within limits: " << (model.checkJointLimits(0, ik) ? "YES" : "NO") << std::endl;
    }

    return 0;
}