#include "HexaModel.h"
#include <cmath>
#include <iostream>

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
    (void)model; // unused, ensures build uses class

    JointAngles q(0, 0, 0);
    double orientation = q.femur - q.tibia; // tibia relative to ground

    bool vertical = std::fabs(orientation) < 1e-6;
    if (vertical)
        std::cout << "\xE2\x9C\x85 Tibia is vertical at zero angles." << std::endl;
    else
        std::cout << "\xE2\x9D\x8C Tibia orientation error: " << orientation << " deg" << std::endl;

    return vertical ? 0 : 1;
}
