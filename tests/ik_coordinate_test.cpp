// Test to understand the coordinate system and fix IK
#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

int main() {
    std::cout << "=== IK Coordinate System Analysis ===" << std::endl;

    Parameters p{};
    p.hexagon_radius = 200;
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

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    sys.initialize(&imu, &fsr, &servos);

    std::cout << "\n=== Leg 0 Coordinate Analysis ===" << std::endl;
    std::cout << "Leg 0 base should be at: (200, 0, 0)" << std::endl;

    // Test a simple case: leg straight out horizontally
    JointAngles straight(0, 0, 0);
    Point3D pos_straight = sys.calculateForwardKinematics(0, straight);
    std::cout << "Straight leg (0°, 0°, 0°): (" << pos_straight.x << ", " << pos_straight.y << ", " << pos_straight.z << ")" << std::endl;

    // Expected: 200 + 50 + 101 + 208 = 559 in X direction
    float expected_x = p.hexagon_radius + p.coxa_length + p.femur_length + p.tibia_length;
    std::cout << "Expected X: " << expected_x << std::endl;

    // Test femur only rotated down
    JointAngles femur_down(0, -45, 0);
    Point3D pos_femur_down = sys.calculateForwardKinematics(0, femur_down);
    std::cout << "Femur -45° (0°, -45°, 0°): (" << pos_femur_down.x << ", " << pos_femur_down.y << ", " << pos_femur_down.z << ")" << std::endl;

    // Test tibia only rotated
    JointAngles tibia_bent(0, 0, 45);
    Point3D pos_tibia_bent = sys.calculateForwardKinematics(0, tibia_bent);
    std::cout << "Tibia 45° (0°, 0°, 45°): (" << pos_tibia_bent.x << ", " << pos_tibia_bent.y << ", " << pos_tibia_bent.z << ")" << std::endl;

    // Test coxa only rotated
    JointAngles coxa_rot(30, 0, 0);
    Point3D pos_coxa_rot = sys.calculateForwardKinematics(0, coxa_rot);
    std::cout << "Coxa 30° (30°, 0°, 0°): (" << pos_coxa_rot.x << ", " << pos_coxa_rot.y << ", " << pos_coxa_rot.z << ")" << std::endl;

    std::cout << "\n=== Simple IK Test on Simple Target ===" << std::endl;

    // Test IK on the straight leg result
    JointAngles ik_straight = sys.calculateInverseKinematics(0, pos_straight);
    Point3D verify_straight = sys.calculateForwardKinematics(0, ik_straight);

    std::cout << "Target: (" << pos_straight.x << ", " << pos_straight.y << ", " << pos_straight.z << ")" << std::endl;
    std::cout << "IK result: (" << ik_straight.coxa << "°, " << ik_straight.femur << "°, " << ik_straight.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << verify_straight.x << ", " << verify_straight.y << ", " << verify_straight.z << ")" << std::endl;

    float error = sqrt(pow(pos_straight.x - verify_straight.x, 2) +
                       pow(pos_straight.y - verify_straight.y, 2) +
                       pow(pos_straight.z - verify_straight.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    return 0;
}
