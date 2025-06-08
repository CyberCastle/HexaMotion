#include "../include/locomotion_system.h"
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
    p.robot_height = 90;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;

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

    return 0;
}
