#include "HexaModel.h"
#include <iomanip>
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

    std::cout << std::fixed << std::setprecision(3);

    static const float base_theta_offsets[NUM_LEGS] = {0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};

    std::cout << "=== Simple Horizontal Test ===" << std::endl;

    bool ok = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        float theta_rad = base_theta_offsets[leg] * M_PI / 180.0f;
        Point3D target;
        float reach = p.hexagon_radius + p.coxa_length + p.femur_length;
        float default_x = reach * cos(theta_rad) + p.tibia_length * sin(theta_rad);
        float default_y = reach * sin(theta_rad) - p.tibia_length * cos(theta_rad);
        target.x = 0.8f * default_x;
        target.y = 0.8f * default_y;
        target.z = 0.0f;

        JointAngles ik = model.inverseKinematics(leg, target);
        Point3D fk = model.forwardKinematics(leg, ik);
        float err = sqrt(pow(target.x - fk.x, 2) +
                         pow(target.y - fk.y, 2) +
                         pow(target.z - fk.z, 2));
        std::cout << "Leg " << leg << ": target(" << target.x << ", " << target.y
                  << ", " << target.z << ") -> IK(" << ik.coxa << ", "
                  << ik.femur << ", " << ik.tibia << ") FK(" << fk.x << ", "
                  << fk.y << ", " << fk.z << ") error=" << err << std::endl;
        if (err > 1e-2f) {
            ok = false;
        }
    }

    if (ok) {
        std::cout << "IK results within tolerance." << std::endl;
        return 0;
    } else {
        std::cerr << "IK validation failed." << std::endl;
        return 1;
    }
}
