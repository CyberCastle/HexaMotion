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
    p.robot_height = 120;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;
    p.ik.clamp_joints = true;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Edge Case Analysis ===" << std::endl;

    // Test the specific problematic case from geometry_test
    std::cout << "\n=== Problematic Case Analysis ===" << std::endl;
    Point3D problem_target(653.851, -8.00533e-06, -183.141);
    std::cout << "Target: (" << problem_target.x << ", " << problem_target.y << ", " << problem_target.z << ")" << std::endl;

    // Check if this target is reachable using centralized function
    float max_reach = p.coxa_length + p.femur_length + p.tibia_length;
    float min_reach = std::abs(p.femur_length - p.tibia_length);

    float target_distance = math_utils::magnitude(problem_target);
    bool reachable = math_utils::isPointReachable(problem_target, min_reach, max_reach);

    std::cout << "Target distance: " << target_distance << "mm" << std::endl;
    std::cout << "Max reach: " << max_reach << "mm" << std::endl;
    std::cout << "Min reach: " << min_reach << "mm" << std::endl;
    std::cout << "Target reachable: " << (reachable ? "YES" : "NO") << std::endl;

    JointAngles ik_result = model.inverseKinematics(0, problem_target);
    std::cout << "IK result: (" << ik_result.coxa << "°, "
              << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;

    Point3D fk_verify = model.forwardKinematics(0, ik_result);
    std::cout << "FK verify: (" << fk_verify.x << ", "
              << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;

    float error = sqrt(pow(problem_target.x - fk_verify.x, 2) +
                       pow(problem_target.y - fk_verify.y, 2) +
                       pow(problem_target.z - fk_verify.z, 2));
    std::cout << "Error: " << error << "mm" << std::endl;

    // Test what the original FK should have been
    std::cout << "\n=== Original FK Configuration ===" << std::endl;
    JointAngles original_config(0.0f, -45.0f, 90.0f);
    std::cout << "Original config: (" << original_config.coxa << "°, "
              << original_config.femur << "°, " << original_config.tibia << "°)" << std::endl;

    bool within_limits = model.checkJointLimits(0, original_config);
    std::cout << "Within joint limits: " << (within_limits ? "YES" : "NO") << std::endl;

    Point3D original_fk = model.forwardKinematics(0, original_config);
    std::cout << "Original FK: (" << original_fk.x << ", "
              << original_fk.y << ", " << original_fk.z << ")" << std::endl;

    // Test some valid reachable targets
    std::cout << "\n=== Valid Reachable Targets ===" << std::endl;
    std::vector<Point3D> valid_targets = {
        Point3D{759, 0, 0},     // Straight
        Point3D{600, 0, -50},   // Forward down
        Point3D{500, 200, 0},   // Forward right
        Point3D{450, -150, 100} // Forward left up
    };

    for (size_t i = 0; i < valid_targets.size(); ++i) {
        Point3D target = valid_targets[i];
        std::cout << "\nTarget " << (i + 1) << ": (" << target.x << ", "
                  << target.y << ", " << target.z << ")" << std::endl;

        JointAngles angles = model.inverseKinematics(0, target);
        std::cout << "Result: (" << angles.coxa << "°, "
                  << angles.femur << "°, " << angles.tibia << "°)" << std::endl;

        Point3D fk_check = model.forwardKinematics(0, angles);
        float err = sqrt(pow(target.x - fk_check.x, 2) +
                         pow(target.y - fk_check.y, 2) +
                         pow(target.z - fk_check.z, 2));
        std::cout << "Error: " << err << "mm" << std::endl;
    }

    return 0;
}
