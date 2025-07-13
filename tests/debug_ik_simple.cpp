#include "include/locomotion_system.h"
#include "tests/test_stubs.h"
#include <iomanip>
#include <iostream>

int main() {
    // Simple parameters
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 150;
    p.height_offset = 0;
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

    // BodyPoseConfiguration pose_config; // Elimina o reemplaza por una construcción válida con parámetros
    sys.initialize(&imu, &fsr, &servos, pose_config);

    std::cout << "=== IK Test Debug ===" << std::endl;

    // Test with known angles first (Forward then Inverse)
    JointAngles test_angles(30, 45, -60);
    Point3D fk_pos = sys.calculateForwardKinematics(0, test_angles);

    std::cout << "Input angles: Coxa=" << test_angles.coxa
              << "° Femur=" << test_angles.femur
              << "° Tibia=" << test_angles.tibia << "°" << std::endl;
    std::cout << "FK result: x=" << fk_pos.x << ", y=" << fk_pos.y << ", z=" << fk_pos.z << std::endl;

    // Now test IK on the same position
    JointAngles ik_result = sys.calculateInverseKinematics(0, fk_pos);
    std::cout << "IK result: Coxa=" << ik_result.coxa
              << "° Femur=" << ik_result.femur
              << "° Tibia=" << ik_result.tibia << "°" << std::endl;

    // Verify with FK again
    Point3D verify_pos = sys.calculateForwardKinematics(0, ik_result);
    std::cout << "FK verify: x=" << verify_pos.x << ", y=" << verify_pos.y << ", z=" << verify_pos.z << std::endl;

    double error = sqrt(pow(fk_pos.x - verify_pos.x, 2) +
                        pow(fk_pos.y - verify_pos.y, 2) +
                        pow(fk_pos.z - verify_pos.z, 2));
    std::cout << "Total error: " << error << "mm" << std::endl;

    // Test simple position
    std::cout << "\n=== Simple Position Test ===" << std::endl;
    Point3D simple_pos(450, 0, -150); // Simple forward position
    std::cout << "Target: x=" << simple_pos.x << ", y=" << simple_pos.y << ", z=" << simple_pos.z << std::endl;

    JointAngles simple_ik = sys.calculateInverseKinematics(0, simple_pos);
    std::cout << "IK result: Coxa=" << simple_ik.coxa
              << "° Femur=" << simple_ik.femur
              << "° Tibia=" << simple_ik.tibia << "°" << std::endl;

    Point3D simple_verify = sys.calculateForwardKinematics(0, simple_ik);
    std::cout << "FK verify: x=" << simple_verify.x << ", y=" << simple_verify.y << ", z=" << simple_verify.z << std::endl;

    double simple_error = sqrt(pow(simple_pos.x - simple_verify.x, 2) +
                               pow(simple_pos.y - simple_verify.y, 2) +
                               pow(simple_pos.z - simple_verify.z, 2));
    std::cout << "Error: " << simple_error << "mm" << std::endl;

    return 0;
}
