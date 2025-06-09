// Test to validate the geometry and coordinate systems
#include "../include/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>

int main() {
    std::cout << "=== Geometry Validation Test ===" << std::endl;

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

    // Test basic leg 0 geometry
    std::cout << "\n=== Leg 0 FK Analysis ===" << std::endl;
    std::cout << "Leg 0 base position: (400, 0, 0)" << std::endl;

    // Test some simple angles to understand the FK
    std::cout << "\nSimple test cases:" << std::endl;
    JointAngles straight(0, 0, 0);
    Point3D pos_straight = sys.calculateForwardKinematics(0, straight);
    std::cout << "Straight leg (0°, 0°, 0°): " << pos_straight.x << ", " << pos_straight.y << ", " << pos_straight.z << std::endl;

    JointAngles down(0, -90, 0);
    Point3D pos_down = sys.calculateForwardKinematics(0, down);
    std::cout << "Femur down (0°, -90°, 0°): " << pos_down.x << ", " << pos_down.y << ", " << pos_down.z << std::endl;

    JointAngles bent(0, -45, 90);
    Point3D pos_bent = sys.calculateForwardKinematics(0, bent);
    std::cout << "Bent leg (0°, -45°, 90°): " << pos_bent.x << ", " << pos_bent.y << ", " << pos_bent.z << std::endl;

    // Try the analytical IK on a simple known case
    std::cout << "\n=== IK Test on FK Results ===" << std::endl;

    // Test IK on the bent leg position
    JointAngles ik_bent = sys.calculateInverseKinematics(0, pos_bent);
    Point3D verify_bent = sys.calculateForwardKinematics(0, ik_bent);
    std::cout << "Target: " << pos_bent.x << ", " << pos_bent.y << ", " << pos_bent.z << std::endl;
    std::cout << "IK result: (" << ik_bent.coxa << "°, " << ik_bent.femur << "°, " << ik_bent.tibia << "°)" << std::endl;
    std::cout << "FK verify: " << verify_bent.x << ", " << verify_bent.y << ", " << verify_bent.z << std::endl;

    float error = sqrt(pow(pos_bent.x - verify_bent.x, 2) +
                       pow(pos_bent.y - verify_bent.y, 2) +
                       pow(pos_bent.z - verify_bent.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    return 0;
}
