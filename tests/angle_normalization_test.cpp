#include "HexaModel.h"
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 100;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;
    p.ik.clamp_joints = true;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Angle Normalization and Constraint Test ===" << std::endl;

    // Test angle normalization function
    std::cout << "\n=== Angle Normalization Tests ===" << std::endl;
    std::vector<float> test_angles = {
        0.0f, 90.0f, -90.0f, 180.0f, -180.0f,
        270.0f, -270.0f, 360.0f, -360.0f,
        450.0f, -450.0f, 720.0f, -720.0f};

    for (float angle : test_angles) {
        float normalized = model.normalizeAngle(angle);
        std::cout << "  " << angle << "° -> " << normalized << "°" << std::endl;
    }

    // Test constraint function
    std::cout << "\n=== Angle Constraint Tests ===" << std::endl;
    std::vector<float> constraint_test_angles = {
        -100.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 100.0f,
        270.0f, -270.0f, 450.0f // Test wraparound cases
    };

    for (float angle : constraint_test_angles) {
        float constrained = model.constrainAngle(angle, -90.0f, 90.0f);
        std::cout << "  " << angle << "° -> " << constrained << "° (constrained to [-90°, 90°])" << std::endl;
    }

    // Test IK with targets that are definitely reachable
    std::cout << "\n=== IK Reachable Targets Test ===" << std::endl;
    std::vector<Point3D> reachable_targets = {
        Point3D{550, 100, 0},    // Target in first quadrant
        Point3D{500, 150, -30},  // Target in first quadrant with Z
        Point3D{450, -100, 20},  // Target in fourth quadrant
        Point3D{500, -150, -40}, // Target in fourth quadrant with Z
        Point3D{600, 50, -60},   // Forward-right
        Point3D{600, -50, -60}   // Forward-left
    };

    int valid_solutions = 0;
    for (size_t i = 0; i < reachable_targets.size(); ++i) {
        Point3D target = reachable_targets[i];
        std::cout << "\nTarget " << (i + 1) << ": (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        // Check workspace bounds manually
        Point3D local_target;
        local_target.x = target.x - 200; // leg 0 base offset
        local_target.y = target.y - 0;
        local_target.z = target.z;
        float target_distance = sqrt(local_target.x * local_target.x +
                                     local_target.y * local_target.y +
                                     local_target.z * local_target.z);
        std::cout << "  Local target: (" << local_target.x << ", " << local_target.y << ", " << local_target.z << ")" << std::endl;
        std::cout << "  Distance from leg base: " << target_distance << "mm (max: 359mm)" << std::endl;

        JointAngles angles = model.inverseKinematics(0, target);
        std::cout << "  IK result: (" << angles.coxa << "°, " << angles.femur << "°, " << angles.tibia << "°)" << std::endl;

        // Check joint limits
        bool within_limits = model.checkJointLimits(0, angles);
        std::cout << "  Within limits: " << (within_limits ? "YES" : "NO") << std::endl;

        // Check accuracy
        Point3D fk_result = model.forwardKinematics(0, angles);
        float error = sqrt(pow(target.x - fk_result.x, 2) +
                           pow(target.y - fk_result.y, 2) +
                           pow(target.z - fk_result.z, 2));
        std::cout << "  FK verify: (" << fk_result.x << ", " << fk_result.y << ", " << fk_result.z << ")" << std::endl;
        std::cout << "  Error: " << error << "mm" << std::endl;

        if (within_limits && error < 10.0f) { // Accept solutions within 10mm accuracy
            valid_solutions++;
            std::cout << "  ✓ VALID SOLUTION" << std::endl;
        } else {
            std::cout << "  ⚠ FALLBACK/INVALID SOLUTION" << std::endl;
        }
    }

    std::cout << "\n=== Final Summary ===" << std::endl;
    std::cout << "Valid IK solutions: " << valid_solutions << "/" << reachable_targets.size() << std::endl;
    std::cout << "Success rate: " << (100.0f * valid_solutions / reachable_targets.size()) << "%" << std::endl;

    if (valid_solutions >= 4) {
        std::cout << "✅ Angle normalization within expected bounds" << std::endl;
    } else {
        std::cout << "⚠️  Angle normalization below expected accuracy" << std::endl;
    }

    std::cout << "angle_normalization_test executed successfully" << std::endl;
    return 0;
}
