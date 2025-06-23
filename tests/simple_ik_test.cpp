#include "HexaModel.h"
#include <iomanip>
#include <iostream>

int main() {
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

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(3);

    // Test simple horizontal target (fully extended leg)
    std::cout << "=== Simple Horizontal Test ===" << std::endl;
    Point3D horizontal_target(759, 0, 0); // Same as neutral position
    JointAngles ik_horizontal = model.inverseKinematics(0, horizontal_target);
    std::cout << "Horizontal target: (" << horizontal_target.x << ", " << horizontal_target.y << ", " << horizontal_target.z << ")" << std::endl;
    std::cout << "IK result: (" << ik_horizontal.coxa << "°, " << ik_horizontal.femur << "°, " << ik_horizontal.tibia << "°)" << std::endl;
    std::cout << "Expected: (0°, 0°, 0°)" << std::endl;

    Point3D fk_verify = model.forwardKinematics(0, ik_horizontal);
    std::cout << "FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;

    float error = sqrt(pow(horizontal_target.x - fk_verify.x, 2) + pow(horizontal_target.y - fk_verify.y, 2) + pow(horizontal_target.z - fk_verify.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    return 0;
}
