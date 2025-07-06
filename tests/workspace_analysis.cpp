#include "robot_model.h"
#include "math_utils.h"
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    // Configure test parameters
    Parameters params;
    params.hexagon_radius = 200.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 100.0f;
    params.tibia_length = 159.0f;

    // Angle limits for joints
    params.coxa_angle_limits[0] = -180.0f;
    params.coxa_angle_limits[1] = 180.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -180.0f;
    params.tibia_angle_limits[1] = 180.0f;

    // Use default DH parameters (RobotModel will initialize them automatically)
    params.use_custom_dh_parameters = false;

    RobotModel model(params);

    std::cout << "=== Workspace Analysis ===" << std::endl;
    std::cout << "Robot configuration:" << std::endl;
    std::cout << "  Hexagon radius: " << params.hexagon_radius << "mm" << std::endl;
    std::cout << "  Coxa length: " << params.coxa_length << "mm" << std::endl;
    std::cout << "  Femur length: " << params.femur_length << "mm" << std::endl;
    std::cout << "  Tibia length: " << params.tibia_length << "mm" << std::endl;

    double max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double min_reach = abs(params.femur_length - params.tibia_length);
    std::cout << "  Theoretical max reach: " << max_reach << "mm" << std::endl;
    std::cout << "  Theoretical min reach: " << min_reach << "mm" << std::endl;

    std::cout << "\n=== Test Target Analysis ===" << std::endl;

    // Test the targets from the failing cases
    JointAngles test_configs[] = {
        JointAngles{0.0f, -45.0f, 90.0f},   // Test 1 (working)
        JointAngles{-45.0f, 45.0f, -90.0f}, // Test 3 (failing)
        JointAngles{90.0f, 0.0f, 0.0f},     // Test 5 (failing)
        JointAngles{0.0f, 0.0f, 0.0f}       // Test 6 (failing)
    };

    const char *test_names[] = {
        "Test 1 (working)",
        "Test 3 (failing)",
        "Test 5 (failing)",
        "Test 6 (failing)"};

    for (int i = 0; i < 4; i++) {
        JointAngles config = test_configs[i];
        std::cout << "\n"
                  << test_names[i] << ": ("
                  << config.coxa << "°, " << config.femur << "°, " << config.tibia << "°)" << std::endl;

        // Forward kinematics to get target position
        Point3D target = model.forwardKinematics(0, config);
        std::cout << "  Target position: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        // Check reachability
        double distance_from_origin = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);

        // For leg 0, base is at (200, 0, 0)
        double base_x = 200.0f;
        double base_y = 0.0f;
        double distance_from_base = sqrt((target.x - base_x) * (target.x - base_x) +
                                        (target.y - base_y) * (target.y - base_y) +
                                        target.z * target.z);

        std::cout << "  Distance from origin: " << distance_from_origin << "mm" << std::endl;
        std::cout << "  Distance from leg base: " << distance_from_base << "mm" << std::endl;
        std::cout << "  Within reach limits: " << (distance_from_base <= max_reach && distance_from_base >= min_reach ? "YES" : "NO") << std::endl;

        if (distance_from_base > max_reach) {
            std::cout << "  ⚠️  Target is " << (distance_from_base - max_reach) << "mm beyond max reach" << std::endl;
        }
        if (distance_from_base < min_reach) {
            std::cout << "  ⚠️  Target is " << (min_reach - distance_from_base) << "mm inside min reach" << std::endl;
        }
    }

    std::cout << "\n=== Workspace Boundary Test ===" << std::endl;
    std::cout << "Testing positions at workspace boundaries..." << std::endl;

    // Test positions at different distances
    double test_distances[] = {100.0f, 200.0f, 300.0f, 309.0f, 350.0f};
    for (double dist : test_distances) {
        Point3D test_pos(200.0f + dist, 0.0f, -100.0f); // Relative to leg 0 base

        std::cout << "\nTesting position at distance " << dist << "mm from leg base:" << std::endl;
        std::cout << "  Position: (" << test_pos.x << ", " << test_pos.y << ", " << test_pos.z << ")" << std::endl;

        JointAngles ik_result = model.inverseKinematics(0, test_pos);
        Point3D fk_verify = model.forwardKinematics(0, ik_result);

        double error = sqrt(pow(test_pos.x - fk_verify.x, 2) +
                           pow(test_pos.y - fk_verify.y, 2) +
                           pow(test_pos.z - fk_verify.z, 2));

        std::cout << "  IK result: (" << ik_result.coxa << "°, " << ik_result.femur << "°, " << ik_result.tibia << "°)" << std::endl;
        std::cout << "  FK verify: (" << fk_verify.x << ", " << fk_verify.y << ", " << fk_verify.z << ")" << std::endl;
        std::cout << "  Error: " << error << "mm" << std::endl;
        std::cout << "  Status: " << (error < 10.0f ? "✓ Good" : (dist > max_reach ? "⚠️ Unreachable (expected)" : "✗ Poor")) << std::endl;
    }

    return 0;
}
