#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

// Analytical IK implementation for comparison
JointAngles analyticalIK(const Point3D &target, const Parameters &params, int leg_index) {
    // Transform target to leg base coordinate system
    float base_angle = leg_index * 60.0f * M_PI / 180.0f;
    float base_x = params.hexagon_radius * cos(base_angle);
    float base_y = params.hexagon_radius * sin(base_angle);

    // Target relative to leg base
    float rel_x = target.x - base_x;
    float rel_y = target.y - base_y;
    float rel_z = target.z;

    // Coxa angle (rotation around Z axis)
    float coxa_angle = atan2(rel_y, rel_x) * 180.0f / M_PI;

    // Distance in XY plane from coxa joint
    float xy_dist = sqrt(rel_x * rel_x + rel_y * rel_y) - params.coxa_length;

    // Distance from femur joint to target
    float r = sqrt(xy_dist * xy_dist + rel_z * rel_z);

    // Check if target is reachable
    float max_reach = params.femur_length + params.tibia_length;
    float min_reach = abs(params.femur_length - params.tibia_length);

    if (r > max_reach || r < min_reach) {
        std::cout << "Target unreachable: r=" << r << ", range=[" << min_reach << ", " << max_reach << "]" << std::endl;
        return JointAngles(coxa_angle, 0, 0); // Return safe fallback
    }

    // Tibia angle using law of cosines
    float cos_tibia = (params.femur_length * params.femur_length + params.tibia_length * params.tibia_length - r * r) /
                      (2.0f * params.femur_length * params.tibia_length);
    cos_tibia = std::max(-1.0f, std::min(1.0f, cos_tibia)); // Clamp to valid range

    float tibia_angle = acos(cos_tibia) * 180.0f / M_PI;
    tibia_angle = 180.0f - tibia_angle; // Convert to our convention (positive is bent)

    // Femur angle
    float alpha = atan2(-rel_z, xy_dist) * 180.0f / M_PI; // Angle to target
    float beta = acos((params.femur_length * params.femur_length + r * r - params.tibia_length * params.tibia_length) /
                      (2.0f * params.femur_length * r)) *
                 180.0f / M_PI;
    float femur_angle = alpha - beta;

    return JointAngles(coxa_angle, femur_angle, tibia_angle);
}

int main() {
    std::cout << "=== Analytical IK Test ===" << std::endl;

    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 100;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;

    // Initialize IK parameters
    p.ik.max_iterations = 30;
    p.ik.pos_threshold_mm = 0.5f;
    p.ik.use_damping = true;
    p.ik.damping_lambda = 0.01f;
    p.ik.clamp_joints = true;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    sys.initialize(&imu, &fsr, &servos);

    // Test reasonable targets for a walking robot
    std::vector<Point3D> test_targets = {
        Point3D(450, 0, -90),    // Default standing position
        Point3D(420, 0, -120),   // Lower position
        Point3D(480, 0, -60),    // Higher position
        Point3D(440, 20, -90),   // Slightly to the side
        Point3D(430, -20, -100), // Other side, lower
    };

    for (auto &target : test_targets) {
        std::cout << "\n--- Target: (" << target.x << ", " << target.y << ", " << target.z << ") ---" << std::endl;

        // Test current iterative IK
        JointAngles q_iterative = sys.calculateInverseKinematics(0, target);
        Point3D pos_iterative = sys.calculateForwardKinematics(0, q_iterative);
        float error_iterative = sqrt(pow(target.x - pos_iterative.x, 2) +
                                     pow(target.y - pos_iterative.y, 2) +
                                     pow(target.z - pos_iterative.z, 2));

        std::cout << "Iterative IK: (" << q_iterative.coxa << "°, " << q_iterative.femur << "°, " << q_iterative.tibia << "°)" << std::endl;
        std::cout << "  -> (" << pos_iterative.x << ", " << pos_iterative.y << ", " << pos_iterative.z << "), error: " << error_iterative << "mm" << std::endl;

        // Test analytical IK
        JointAngles q_analytical = analyticalIK(target, p, 0);
        Point3D pos_analytical = sys.calculateForwardKinematics(0, q_analytical);
        float error_analytical = sqrt(pow(target.x - pos_analytical.x, 2) +
                                      pow(target.y - pos_analytical.y, 2) +
                                      pow(target.z - pos_analytical.z, 2));

        std::cout << "Analytical IK: (" << q_analytical.coxa << "°, " << q_analytical.femur << "°, " << q_analytical.tibia << "°)" << std::endl;
        std::cout << "  -> (" << pos_analytical.x << ", " << pos_analytical.y << ", " << pos_analytical.z << "), error: " << error_analytical << "mm" << std::endl;
    }

    return 0;
}
