#include "HexaModel.h"
#include <iostream>
#include <iomanip>

int main() {
    Parameters params;
    params.hexagon_radius = 200.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 101.0f;
    params.tibia_length = 208.0f;
    params.robot_height = 120.0f;
    params.use_custom_dh_parameters = false;

    RobotModel model(params);

    std::cout << "=== Simple DH Test ===" << std::endl;
    std::cout << "Testing with coxa=0, femur=0, tibia=0" << std::endl;

    JointAngles angles(0.0f, 0.0f, 0.0f);
    Point3D result = model.forwardKinematics(0, angles);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Result: (" << result.x << ", " << result.y << ", " << result.z << ")" << std::endl;
    std::cout << "Expected: (351, -208, 0) for DH parameters with coxa=0, femur=0, tibia=0" << std::endl;

    // Test with different angles to verify DH chain
    std::cout << "\n=== Additional DH Tests ===" << std::endl;

    // Test with femur = 90 degrees (pointing down)
    JointAngles angles2(0.0f, 90.0f, 0.0f);
    Point3D result2 = model.forwardKinematics(0, angles2);
    std::cout << "coxa=0, femur=90, tibia=0: (" << result2.x << ", " << result2.y << ", " << result2.z << ")" << std::endl;

    // Test with femur = -90 degrees (pointing up)
    JointAngles angles3(0.0f, -90.0f, 0.0f);
    Point3D result3 = model.forwardKinematics(0, angles3);
    std::cout << "coxa=0, femur=-90, tibia=0: (" << result3.x << ", " << result3.y << ", " << result3.z << ")" << std::endl;

    // Test with coxa = 90 degrees (pointing to the right)
    JointAngles angles4(90.0f, 0.0f, 0.0f);
    Point3D result4 = model.forwardKinematics(0, angles4);
    std::cout << "coxa=90, femur=0, tibia=0: (" << result4.x << ", " << result4.y << ", " << result4.z << ")" << std::endl;

    return 0;
}