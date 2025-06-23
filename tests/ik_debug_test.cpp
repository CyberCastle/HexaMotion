#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    std::cout << "=== IK Debug Test ===" << std::endl;

    // Set up basic parameters
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 100;
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;

    // Initialize IK parameters for proper convergence
    p.ik.max_iterations = 30;
    p.ik.pos_threshold_mm = 0.5f;
    p.ik.use_damping = true;
    p.ik.damping_lambda = 0.01f; // Lower damping for better convergence
    p.ik.clamp_joints = true;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    sys.initialize(&imu, &fsr, &servos);

    std::cout << "Robot parameters:" << std::endl;
    std::cout << "  Hexagon radius: " << p.hexagon_radius << "mm" << std::endl;
    std::cout << "  Coxa length: " << p.coxa_length << "mm" << std::endl;
    std::cout << "  Femur length: " << p.femur_length << "mm" << std::endl;
    std::cout << "  Tibia length: " << p.tibia_length << "mm" << std::endl;
    std::cout << "  Robot height: " << p.robot_height << "mm" << std::endl;

    // Test forward kinematics with reasonable angles
    std::cout << "\n=== Forward Kinematics Test ===" << std::endl;

    std::vector<JointAngles> test_angles = {
        JointAngles(0, 0, 0),     // All straight
        JointAngles(0, -30, 60),  // Reasonable pose 1
        JointAngles(0, -45, 90),  // Reasonable pose 2
        JointAngles(0, -60, 120), // More extreme pose
    };

    for (int i = 0; i < test_angles.size(); i++) {
        JointAngles q = test_angles[i];
        Point3D pos = sys.calculateForwardKinematics(0, q); // Test leg 0

        std::cout << "Angles (" << q.coxa << "°, " << q.femur << "°, " << q.tibia << "°) -> ";
        std::cout << "Position (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;

        // Now test if IK can get back to the same angles
        JointAngles q_ik = sys.calculateInverseKinematics(0, pos);
        Point3D pos_verify = sys.calculateForwardKinematics(0, q_ik);

        std::cout << "  IK result: (" << q_ik.coxa << "°, " << q_ik.femur << "°, " << q_ik.tibia << "°)";
        std::cout << " -> (" << pos_verify.x << ", " << pos_verify.y << ", " << pos_verify.z << ")" << std::endl;

        float error = sqrt(pow(pos.x - pos_verify.x, 2) + pow(pos.y - pos_verify.y, 2) + pow(pos.z - pos_verify.z, 2));
        std::cout << "  Error: " << error << "mm" << std::endl
                  << std::endl;
    }

    // Test the specific target that's failing
    std::cout << "=== Test Specific Failing Target ===" << std::endl;
    Point3D target(425, 0, -100);
    std::cout << "Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

    JointAngles q_result = sys.calculateInverseKinematics(0, target);
    Point3D pos_result = sys.calculateForwardKinematics(0, q_result);

    std::cout << "IK result: (" << q_result.coxa << "°, " << q_result.femur << "°, " << q_result.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << pos_result.x << ", " << pos_result.y << ", " << pos_result.z << ")" << std::endl;

    float error = sqrt(pow(target.x - pos_result.x, 2) + pow(target.y - pos_result.y, 2) + pow(target.z - pos_result.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    // Workspace analysis
    std::cout << "\n=== Workspace Analysis ===" << std::endl;
    float leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
    std::cout << "Maximum leg reach: " << leg_reach << "mm" << std::endl;

    float target_distance = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    std::cout << "Target distance from base: " << target_distance << "mm" << std::endl;

    // Test a reachable target closer to the robot
    std::cout << "\n=== Test Reachable Target ===" << std::endl;
    Point3D reachable_target(350, 0, -50); // Much closer and higher
    std::cout << "Reachable target: (" << reachable_target.x << ", " << reachable_target.y << ", " << reachable_target.z << ")" << std::endl;

    JointAngles q_reachable = sys.calculateInverseKinematics(0, reachable_target);
    Point3D pos_reachable = sys.calculateForwardKinematics(0, q_reachable);

    std::cout << "IK result: (" << q_reachable.coxa << "°, " << q_reachable.femur << "°, " << q_reachable.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << pos_reachable.x << ", " << pos_reachable.y << ", " << pos_reachable.z << ")" << std::endl;

    float error_reachable = sqrt(pow(reachable_target.x - pos_reachable.x, 2) + pow(reachable_target.y - pos_reachable.y, 2) + pow(reachable_target.z - pos_reachable.z, 2));
    std::cout << "Error: " << error_reachable << "mm" << std::endl;

    // Test default standing position for leg 0
    std::cout << "\n=== Test Default Standing Position ===" << std::endl;
    float angle = 0 * 60.0f; // Leg 0 angle
    Point3D default_pos;
    default_pos.x = p.hexagon_radius * cos(0) + p.coxa_length; // 400 + 50 = 450
    default_pos.y = p.hexagon_radius * sin(0);                 // 0
    default_pos.z = -p.robot_height;                           // -90

    std::cout << "Default position: (" << default_pos.x << ", " << default_pos.y << ", " << default_pos.z << ")" << std::endl;

    JointAngles q_default = sys.calculateInverseKinematics(0, default_pos);
    Point3D pos_default = sys.calculateForwardKinematics(0, q_default);

    std::cout << "IK result: (" << q_default.coxa << "°, " << q_default.femur << "°, " << q_default.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << pos_default.x << ", " << pos_default.y << ", " << pos_default.z << ")" << std::endl;

    float error_default = sqrt(pow(default_pos.x - pos_default.x, 2) + pow(default_pos.y - pos_default.y, 2) + pow(default_pos.z - pos_default.z, 2));
    std::cout << "Error: " << error_default << "mm" << std::endl;

    // Test with a better initial guess
    std::cout << "\n=== Test with Better Initial Guess ===" << std::endl;
    Point3D simple_target(450, 0, -90); // Same as default position
    std::cout << "Target: (" << simple_target.x << ", " << simple_target.y << ", " << simple_target.z << ")" << std::endl;

    // Check what happens step by step
    std::cout << "IK iteration analysis:" << std::endl;

    // Manual IK debugging - let's trace through what should happen
    // For leg 0: base at (400, 0, 0), target at (450, 0, -90)
    // Relative to base: (50, 0, -90)

    // Analytical solution attempt for simple 2D case
    float dx = 50.0f;                  // relative x
    float dz = -90.0f;                 // relative z (negative = below)
    float r = sqrt(dx * dx + dz * dz); // distance = sqrt(50^2 + 90^2) = 103.08mm

    std::cout << "  Distance to target (relative to leg base): " << r << "mm" << std::endl;
    std::cout << "  Femur + Tibia length: " << (p.femur_length + p.tibia_length) << "mm" << std::endl;

    if (r <= (p.femur_length + p.tibia_length)) {
        std::cout << "  Target is reachable!" << std::endl;

        // Simple analytical IK for 2-link arm in 2D
        float L1 = p.femur_length; // 101mm
        float L2 = p.tibia_length; // 208mm

        // Law of cosines for elbow angle
        float cos_tibia = (L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2);
        if (cos_tibia >= -1.0f && cos_tibia <= 1.0f) {
            float tibia_angle = acos(cos_tibia) * 180.0f / M_PI;
            tibia_angle = 180.0f - tibia_angle; // Convention: positive bend

            // Shoulder angle
            float alpha = atan2(abs(dz), dx) * 180.0f / M_PI;
            float beta = acos((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r)) * 180.0f / M_PI;
            float femur_angle = -(alpha + beta); // Negative for downward

            std::cout << "  Analytical solution: coxa=0°, femur=" << femur_angle << "°, tibia=" << tibia_angle << "°" << std::endl;

            // Test this solution
            JointAngles analytical(0, femur_angle, tibia_angle);
            Point3D analytical_pos = sys.calculateForwardKinematics(0, analytical);
            std::cout << "  Analytical FK result: (" << analytical_pos.x << ", " << analytical_pos.y << ", " << analytical_pos.z << ")" << std::endl;

            float analytical_error = sqrt(pow(simple_target.x - analytical_pos.x, 2) + pow(simple_target.y - analytical_pos.y, 2) + pow(simple_target.z - analytical_pos.z, 2));
            std::cout << "  Analytical error: " << analytical_error << "mm" << std::endl;
        } else {
            std::cout << "  Law of cosines failed: cos_tibia = " << cos_tibia << std::endl;
        }
    } else {
        std::cout << "  Target is NOT reachable!" << std::endl;
    }

    return 0;
}
