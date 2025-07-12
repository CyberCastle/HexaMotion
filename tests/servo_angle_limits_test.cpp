#include "robot_model.h"
#include "body_pose_config_factory.h"
#include <cassert>
#include <iostream>
#include <vector>

// Simplified validation using IK, based on validateLegConfigurationWithIK
bool validateWithIK(const RobotModel &model, int leg, const JointAngles &angles,
                    double tol_mm = 1.0) {
    Point3D target = model.forwardKinematicsGlobalCoordinates(leg, angles);
    JointAngles ik =
        model.inverseKinematicsCurrentGlobalCoordinates(leg, angles, target);
    Point3D fk = model.forwardKinematicsGlobalCoordinates(leg, ik);

    double err = std::sqrt(std::pow(target.x - fk.x, 2) +
                            std::pow(target.y - fk.y, 2) +
                            std::pow(target.z - fk.z, 2));
    return err <= tol_mm;
}

Parameters setupParameters() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    return p;
}

int main() {
    Parameters params = setupParameters();
    RobotModel model(params);

    std::vector<double> heights = {140.0, params.robot_height};

    for (double h : heights) {
        CalculatedServoAngles calc = calculateServoAnglesForHeight(h, params);
        std::cout << "Height " << h << " -> femur " << calc.femur
                  << " tibia " << calc.tibia << std::endl;
        assert(calc.valid);
        assert(calc.femur >= params.femur_angle_limits[0] &&
               calc.femur <= params.femur_angle_limits[1]);
        assert(calc.tibia >= params.tibia_angle_limits[0] &&
               calc.tibia <= params.tibia_angle_limits[1]);

        JointAngles angles(0.0, math_utils::degreesToRadians(calc.femur),
                           math_utils::degreesToRadians(calc.tibia));
        assert(validateWithIK(model, 0, angles));
    }

    std::cout << "servo_angle_limits_test passed" << std::endl;
    return 0;
}
