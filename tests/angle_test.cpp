// Test to understand angle conventions
#include "../src/locomotion_system.h"
#include "test_stubs.h"
#include <cmath>
#include <iostream>
#include "../src/body_pose_config.h"

int main() {
    std::cout << "=== Angle Convention Analysis ===" << std::endl;

    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
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
    // Create a default pose configuration
    BodyPoseConfiguration pose_config(p);
    pose_config.body_clearance = 208.0;
    sys.initialize(&imu, &fsr, &servos, pose_config);

    std::cout << "\n=== Test Individual Joint Effects ===" << std::endl;

    // Base case: all joints at 0
    JointAngles base(0, 0, 0);
    Point3D pos_base = sys.calculateForwardKinematics(0, base);
    std::cout << "Base (0°, 0°, 0°): (" << pos_base.x << ", " << pos_base.y << ", " << pos_base.z << ")" << std::endl;

    // Test coxa rotation (convert degrees to radians)
    JointAngles coxa_test(math_utils::degreesToRadians(30), 0, 0);
    Point3D pos_coxa = sys.calculateForwardKinematics(0, coxa_test);
    std::cout << "Coxa 30° (30°, 0°, 0°): (" << pos_coxa.x << ", " << pos_coxa.y << ", " << pos_coxa.z << ")" << std::endl;

    // Test femur rotation (convert degrees to radians)
    JointAngles femur_test(0, math_utils::degreesToRadians(-30), 0);
    Point3D pos_femur = sys.calculateForwardKinematics(0, femur_test);
    std::cout << "Femur -30° (0°, -30°, 0°): (" << pos_femur.x << ", " << pos_femur.y << ", " << pos_femur.z << ")" << std::endl;

    // Test tibia rotation (convert degrees to radians)
    JointAngles tibia_test(0, 0, math_utils::degreesToRadians(30));
    Point3D pos_tibia = sys.calculateForwardKinematics(0, tibia_test);
    std::cout << "Tibia 30° (0°, 0°, 30°): (" << pos_tibia.x << ", " << pos_tibia.y << ", " << pos_tibia.z << ")" << std::endl;

    // Test tibia 90° (convert degrees to radians)
    JointAngles tibia_90(0, 0, math_utils::degreesToRadians(90));
    Point3D pos_t90 = sys.calculateForwardKinematics(0, tibia_90);
    std::cout << "Tibia 90° (0°, 0°, 90°): (" << pos_t90.x << ", " << pos_t90.y << ", " << pos_t90.z << ")" << std::endl;

    // Test combined femur + tibia that should fold the leg (convert degrees to radians)
    JointAngles folded(0, math_utils::degreesToRadians(-90), math_utils::degreesToRadians(90));
    Point3D pos_folded = sys.calculateForwardKinematics(0, folded);
    std::cout << "Folded (0°, -90°, 90°): (" << pos_folded.x << ", " << pos_folded.y << ", " << pos_folded.z << ")" << std::endl;

    return 0;
}
