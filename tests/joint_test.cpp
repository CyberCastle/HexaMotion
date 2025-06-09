#include "model.h"
#include <iomanip>
#include <iostream>

void printJointTest(RobotModel &model, int leg, const char *name, const JointAngles &angles) {
    Point3D pos = model.forwardKinematics(leg, angles);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << name << " (" << angles.coxa << "°, " << angles.femur << "°, " << angles.tibia << "°): ";
    std::cout << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

int main() {
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

    RobotModel model(p);

    std::cout << "=== Individual Joint Tests for Leg 0 ===" << std::endl;

    // Test neutral position
    printJointTest(model, 0, "Neutral    ", JointAngles(0, 0, 0));

    // Test coxa only
    printJointTest(model, 0, "Coxa +10   ", JointAngles(10, 0, 0));
    printJointTest(model, 0, "Coxa -10   ", JointAngles(-10, 0, 0));

    // Test femur only
    printJointTest(model, 0, "Femur +10  ", JointAngles(0, 10, 0));
    printJointTest(model, 0, "Femur -10  ", JointAngles(0, -10, 0));
    printJointTest(model, 0, "Femur +45  ", JointAngles(0, 45, 0));
    printJointTest(model, 0, "Femur -45  ", JointAngles(0, -45, 0));

    // Test tibia only
    printJointTest(model, 0, "Tibia +10  ", JointAngles(0, 0, 10));
    printJointTest(model, 0, "Tibia -10  ", JointAngles(0, 0, -10));
    printJointTest(model, 0, "Tibia +90  ", JointAngles(0, 0, 90));
    printJointTest(model, 0, "Tibia -90  ", JointAngles(0, 0, -90));

    std::cout << "\n=== Combined Test ===" << std::endl;
    printJointTest(model, 0, "Bend Down  ", JointAngles(0, -45, 90));

    std::cout << "\n=== IK Test on Combined ===" << std::endl;
    Point3D target(653.851, -8.00533e-06, -183.141);
    JointAngles ik_result = model.inverseKinematics(0, target);
    std::cout << "Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
    std::cout << "IK result: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;

    Point3D fk_verify = model.forwardKinematics(0, ik_result);
    std::cout << "FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;

    float error = sqrt(pow(target.x - fk_verify.x, 2) + pow(target.y - fk_verify.y, 2) + pow(target.z - fk_verify.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    return 0;
}
