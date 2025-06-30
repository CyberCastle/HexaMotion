#include "HexaModel.h"
#include <iostream>
#include <iomanip>
#include <cmath>

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

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== DH Parameter Validation ===" << std::endl;

    static const double base_theta_offsets[NUM_LEGS] = {0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};

    JointAngles q(0, 0, 0);

    bool ok = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D pos = model.forwardKinematics(leg, q);
        double theta_rad = base_theta_offsets[leg] * M_PI / 180.0f;
        double reach = p.hexagon_radius + p.coxa_length + p.femur_length;
        double expected_x = reach * cos(theta_rad);
        double expected_y = reach * sin(theta_rad);
        double expected_z = -p.tibia_length;
        double err = std::sqrt(std::pow(pos.x - expected_x, 2) +
                              std::pow(pos.y - expected_y, 2) +
                              std::pow(pos.z - expected_z, 2));
        std::cout << "Leg " << leg << ": (" << pos.x << ", " << pos.y << ", " << pos.z
                  << ") expected (" << expected_x << ", " << expected_y << ", " << expected_z
                  << ") error=" << err << "\n";
        if (err > 1e-3f) {
            ok = false;
        }
    }

    if (ok) {
        std::cout << "DH parameters appear to be consistent." << std::endl;
        return 0;
    } else {
        std::cerr << "DH parameter validation failed." << std::endl;
        return 1;
    }
}
