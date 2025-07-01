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

    static const double base_theta_offsets[NUM_LEGS] = {0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};

    std::cout << "=== Simple Horizontal Test ===" << std::endl;

    bool ok = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        // Target: posición de reposo física real de la punta de la pierna
        JointAngles zero_angles(0, 0, 0);
        Point3D target = model.forwardKinematics(leg, zero_angles);

        JointAngles ik = model.inverseKinematics(leg, target);
        Point3D fk = model.forwardKinematics(leg, ik);
        double err = sqrt(pow(target.x - fk.x, 2) +
                         pow(target.y - fk.y, 2) +
                         pow(target.z - fk.z, 2));
        std::cout << "Leg " << leg << ": target(" << target.x << ", " << target.y
                  << ", " << target.z << ") -> IK(" << ik.coxa << ", "
                  << ik.femur << ", " << ik.tibia << ") FK(" << fk.x << ", "
                  << fk.y << ", " << fk.z << ") error=" << err << std::endl;
        if (std::abs(ik.coxa) > 1e-3f || std::abs(ik.femur) > 1e-3f || std::abs(ik.tibia) > 1e-3f || err > 1e-2f) {
            ok = false;
        }
    }

    if (ok) {
        std::cout << "IK results within tolerance." << std::endl;
    } else {
        std::cerr << "IK validation failed." << std::endl;
        return 1;
    }

    // Validación coherencia de IK con FK
    std::cout << "\n=== IK Height 120mm Test ===" << std::endl;
    bool height_ok = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {

        // Start angles are the same as the base_theta_offsets
        JointAngles test_angles(base_theta_offsets[leg], 20, 20);
        Point3D target = model.forwardKinematics(leg, test_angles);

        JointAngles start_angles(base_theta_offsets[leg], 0, 0); // Test symmetric configuration
        JointAngles ik = model.inverseKinematicsCurrent(leg, start_angles, target);
        Point3D fk = model.forwardKinematics(leg, ik);
        printf("target: %f, %f, %f\n", target.x, target.y, target.z);
        printf("fk    : %f, %f, %f\n", fk.x, fk.y, fk.z);
        double z_err = std::abs(fk.z - target.z);
        std::cout << "Leg " << leg << ": target altura -"<< target.z <<" -> IK(" << ik.coxa << ", "
                  << ik.femur << ", " << ik.tibia << ") FK altura=" << fk.z << " error_z=" << z_err << std::endl;
        if (z_err > 2.0f) {
            height_ok = false;
        }
        printf("\n");
    }
    if (height_ok) {
        std::cout << "Hay coherencia entre IK y FK" << std::endl;
    } else {
        std::cerr << "No hay coherencia entre IK y FK" << std::endl;
    }

    return 0;
}
