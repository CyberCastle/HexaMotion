#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 120;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    if (!model.validate()) {
        std::cerr << "Invalid model parameters" << std::endl;
        return 1;
    }

    const double step_coxa = 15.0;
    const double step_femur = 15.0;
    const double step_tibia = 15.0;

    double max_error = 0.0;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();

        for (double coxa = p.coxa_angle_limits[0]; coxa <= p.coxa_angle_limits[1]; coxa += step_coxa) {
            for (double femur = p.femur_angle_limits[0]; femur <= p.femur_angle_limits[1]; femur += step_femur) {
                for (double tibia = p.tibia_angle_limits[0]; tibia <= p.tibia_angle_limits[1]; tibia += step_tibia) {
                    JointAngles angles(math_utils::degreesToRadians(coxa), math_utils::degreesToRadians(femur), math_utils::degreesToRadians(tibia));
                    if (!model.checkJointLimits(leg, angles)) {
                        continue;
                    }

                    // FK in global coordinates
                    Point3D global_pos = model.forwardKinematics(leg, angles);
                    // Transform back to local coordinates using zero pose as reference
                    Point3D local_pos = model.transformGlobalToLocalCoordinates(leg, global_pos, JointAngles(0, 0, 0));
                    // Check IK round-trip
                    JointAngles ik_angles = model.inverseKinematics(leg, global_pos);
                    Point3D fk_verify = model.forwardKinematics(leg, ik_angles);
                    double err = std::sqrt(std::pow(fk_verify.x - global_pos.x, 2) +
                                           std::pow(fk_verify.y - global_pos.y, 2) +
                                           std::pow(fk_verify.z - global_pos.z, 2));
                    max_error = std::max(max_error, err);

                    min_x = std::min(min_x, local_pos.x);
                    max_x = std::max(max_x, local_pos.x);
                    min_y = std::min(min_y, local_pos.y);
                    max_y = std::max(max_y, local_pos.y);
                    min_z = std::min(min_z, local_pos.z);
                    max_z = std::max(max_z, local_pos.z);
                }
            }
        }

        std::cout << "Leg " << leg << " workspace (local)" << std::endl;
        std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
        std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
        std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
    }

    std::cout << "Max IK/FK error encountered: " << max_error << " mm" << std::endl;
    return 0;
}

