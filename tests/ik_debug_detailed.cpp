#include "HexaModel.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
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

    std::cout << std::fixed << std::setprecision(6);

    // Target from failing case
    float rel_x = 453.851 - 200; // 253.851
    float rel_y = -8.00533e-06;
    float rel_z = -183.141;

    float horizontal_dist = 203.851;                                   // sqrt(rel_x^2 + rel_y^2) - coxa_length
    float r = sqrt(horizontal_dist * horizontal_dist + rel_z * rel_z); // 274.036

    std::cout << "=== IK Step by Step Debug ===" << std::endl;
    std::cout << "rel_z: " << rel_z << std::endl;
    std::cout << "horizontal_dist: " << horizontal_dist << std::endl;
    std::cout << "r: " << r << std::endl;

    // Alpha calculation (angle to target)
    float alpha = atan2(-rel_z, horizontal_dist) * 180.0f / M_PI;
    std::cout << "alpha (angle to target): " << alpha << "°" << std::endl;

    // Beta calculation (internal triangle angle)
    float cos_beta = (p.femur_length * p.femur_length + r * r - p.tibia_length * p.tibia_length) /
                     (2.0f * p.femur_length * r);
    float beta = acos(cos_beta) * 180.0f / M_PI;
    std::cout << "beta (internal angle): " << beta << "°" << std::endl;

    float femur_angle = alpha - beta;
    std::cout << "femur_angle (alpha - beta): " << femur_angle << "°" << std::endl;

    // Tibia calculation
    float cos_tibia = (p.femur_length * p.femur_length + p.tibia_length * p.tibia_length - r * r) /
                      (2.0f * p.femur_length * p.tibia_length);
    float tibia_raw = acos(cos_tibia) * 180.0f / M_PI;
    float tibia_angle = 180.0f - tibia_raw;
    std::cout << "tibia_raw (acos): " << tibia_raw << "°" << std::endl;
    std::cout << "tibia_angle (180 - raw): " << tibia_angle << "°" << std::endl;

    std::cout << "\n=== Expected from FK ===" << std::endl;
    std::cout << "Should be approximately: (0°, -45°, 90°)" << std::endl;

    return 0;
}
