// Test to understand the coordinate system and fix IK
#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

int main() {
    std::cout << "=== IK Coordinate System Analysis ===" << std::endl;

    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 90;
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
    std::cout << "Leg 0 base should be at: (400, 0, 0)" << std::endl;

    // Test a simple case: leg straight out horizontally
    JointAngles straight(0, 0, 0);
    Point3D pos_straight = sys.calculateForwardKinematics(0, straight);
    std::cout << "Straight leg (0°, 0°, 0°): (" << pos_straight.x << ", " << pos_straight.y << ", " << pos_straight.z << ")" << std::endl;

    // Expected: 400 + 50 + 101 + 208 = 759 in X direction
    float expected_x = p.hexagon_radius + p.coxa_length + p.femur_length + p.tibia_length;
    std::cout << "Expected X: " << expected_x << std::endl;

    // Test simple IK on the straight leg result
    JointAngles ik_straight = sys.calculateInverseKinematics(0, pos_straight);
    Point3D verify_straight = sys.calculateForwardKinematics(0, ik_straight);

    std::cout << "\nTarget: (" << pos_straight.x << ", " << pos_straight.y << ", " << pos_straight.z << ")" << std::endl;
    std::cout << "IK result: (" << ik_straight.coxa << "°, " << ik_straight.femur << "°, " << ik_straight.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << verify_straight.x << ", " << verify_straight.y << ", " << verify_straight.z << ")" << std::endl;

    float error = sqrt(pow(pos_straight.x - verify_straight.x, 2) +
                       pow(pos_straight.y - verify_straight.y, 2) +
                       pow(pos_straight.z - verify_straight.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    // Test the specific failing case from geometry test
    std::cout << "\n=== Test Failing Case from Geometry Test ===" << std::endl;
    JointAngles test_bent(0, -45, 90);
    Point3D pos_bent = sys.calculateForwardKinematics(0, test_bent);
    std::cout << "Bent leg (0°, -45°, 90°): (" << pos_bent.x << ", " << pos_bent.y << ", " << pos_bent.z << ")" << std::endl;

    JointAngles ik_bent = sys.calculateInverseKinematics(0, pos_bent);
    Point3D verify_bent = sys.calculateForwardKinematics(0, ik_bent);

    std::cout << "Target: (" << pos_bent.x << ", " << pos_bent.y << ", " << pos_bent.z << ")" << std::endl;
    std::cout << "IK result: (" << ik_bent.coxa << "°, " << ik_bent.femur << "°, " << ik_bent.tibia << "°)" << std::endl;
    std::cout << "FK verify: (" << verify_bent.x << ", " << verify_bent.y << ", " << verify_bent.z << ")" << std::endl;

    float error_bent = sqrt(pow(pos_bent.x - verify_bent.x, 2) +
                            pow(pos_bent.y - verify_bent.y, 2) +
                            pow(pos_bent.z - verify_bent.z, 2));
    std::cout << "Error: " << error_bent << "mm" << std::endl;

    // Debug the IK calculation step by step
    std::cout << "\n=== IK Step-by-Step Debug ===" << std::endl;
    float base_x = 400; // hexagon_radius * cos(0)
    float base_y = 0;   // hexagon_radius * sin(0)
    float rel_x = pos_bent.x - base_x;
    float rel_y = pos_bent.y - base_y;
    float rel_z = pos_bent.z;

    std::cout << "Target relative to leg base: (" << rel_x << ", " << rel_y << ", " << rel_z << ")" << std::endl;

    float coxa_angle = atan2(rel_y, rel_x) * 180.0f / M_PI;
    std::cout << "Calculated coxa angle: " << coxa_angle << "°" << std::endl;

    float xy_dist = sqrt(rel_x * rel_x + rel_y * rel_y);
    float horizontal_dist = xy_dist - p.coxa_length;
    float r = sqrt(horizontal_dist * horizontal_dist + rel_z * rel_z);

    std::cout << "XY distance: " << xy_dist << "mm" << std::endl;
    std::cout << "Horizontal distance (after coxa): " << horizontal_dist << "mm" << std::endl;
    std::cout << "Total distance r: " << r << "mm" << std::endl;
    std::cout << "Max reach: " << (p.femur_length + p.tibia_length) << "mm" << std::endl;
    std::cout << "Min reach: " << abs(p.femur_length - p.tibia_length) << "mm" << std::endl;

    return 0;
}
