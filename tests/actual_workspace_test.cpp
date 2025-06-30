#include "HexaModel.h"
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

    // Initialize DH parameters
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG + 1; ++j) {
            for (int k = 0; k < 4; ++k) {
                params.dh_parameters[l][j][k] = 0.0f;
            }
        }
    }

    RobotModel model(params);

    std::cout << "=== Actual Workspace Generation ===" << std::endl;
    std::cout << "Generating reachable workspace by sampling joint space..." << std::endl;

    // Sample the joint space to find actual reachable positions
    int samples = 0;
    int valid_positions = 0;
    std::vector<Point3D> reachable_positions;

    // Sample joint angles within limits
    for (double coxa = params.coxa_angle_limits[0]; coxa <= params.coxa_angle_limits[1]; coxa += 30.0f) {
        for (double femur = params.femur_angle_limits[0]; femur <= params.femur_angle_limits[1]; femur += 15.0f) {
            for (double tibia = params.tibia_angle_limits[0]; tibia <= params.tibia_angle_limits[1]; tibia += 30.0f) {
                JointAngles angles(coxa, femur, tibia);
                samples++;

                if (model.checkJointLimits(0, angles)) {
                    Point3D pos = model.forwardKinematics(0, angles);
                    reachable_positions.push_back(pos);
                    valid_positions++;
                }
            }
        }
    }

    std::cout << "Sampled " << samples << " joint configurations" << std::endl;
    std::cout << "Found " << valid_positions << " valid positions" << std::endl;

    // Find workspace bounds
    double min_x = 1000, max_x = -1000;
    double min_y = 1000, max_y = -1000;
    double min_z = 1000, max_z = -1000;
    double min_dist = 1000, max_dist = 0;

    // Leg 0 base position
    double base_x = 200.0f;
    double base_y = 0.0f;

    for (const Point3D &pos : reachable_positions) {
        min_x = std::min(min_x, pos.x);
        max_x = std::max(max_x, pos.x);
        min_y = std::min(min_y, pos.y);
        max_y = std::max(max_y, pos.y);
        min_z = std::min(min_z, pos.z);
        max_z = std::max(max_z, pos.z);

        double dist = sqrt((pos.x - base_x) * (pos.x - base_x) +
                          (pos.y - base_y) * (pos.y - base_y) +
                          pos.z * pos.z);
        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);
    }

    std::cout << "\n=== Actual Workspace Bounds ===" << std::endl;
    std::cout << "X range: [" << min_x << ", " << max_x << "] (span: " << (max_x - min_x) << "mm)" << std::endl;
    std::cout << "Y range: [" << min_y << ", " << max_y << "] (span: " << (max_y - min_y) << "mm)" << std::endl;
    std::cout << "Z range: [" << min_z << ", " << max_z << "] (span: " << (max_z - min_z) << "mm)" << std::endl;
    std::cout << "Distance from base: [" << min_dist << ", " << max_dist << "]mm" << std::endl;

    std::cout << "\n=== Test Target Workspace Check ===" << std::endl;

    // Test our problematic targets
    struct TestCase {
        JointAngles config;
        const char *name;
    };

    TestCase test_cases[] = {
        {JointAngles{0.0f, -45.0f, 90.0f}, "Test 1 (working)"},
        {JointAngles{-45.0f, 45.0f, -90.0f}, "Test 3 (failing)"},
        {JointAngles{90.0f, 0.0f, 0.0f}, "Test 5 (failing)"},
        {JointAngles{0.0f, 0.0f, 0.0f}, "Test 6 (failing)"}};

    for (const TestCase &test : test_cases) {
        Point3D target = model.forwardKinematics(0, test.config);

        std::cout << "\n"
                  << test.name << ":" << std::endl;
        std::cout << "  Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;

        // Check if target is within actual workspace bounds
        bool within_x = (target.x >= min_x && target.x <= max_x);
        bool within_y = (target.y >= min_y && target.y <= max_y);
        bool within_z = (target.z >= min_z && target.z <= max_z);

        double target_dist = sqrt((target.x - base_x) * (target.x - base_x) +
                                 (target.y - base_y) * (target.y - base_y) +
                                 target.z * target.z);
        bool within_dist = (target_dist >= min_dist && target_dist <= max_dist);

        std::cout << "  Within X bounds: " << (within_x ? "YES" : "NO") << std::endl;
        std::cout << "  Within Y bounds: " << (within_y ? "YES" : "NO") << std::endl;
        std::cout << "  Within Z bounds: " << (within_z ? "YES" : "NO") << std::endl;
        std::cout << "  Within distance bounds: " << (within_dist ? "YES" : "NO") << std::endl;

        bool reachable = within_x && within_y && within_z && within_dist;
        std::cout << "  Overall reachable: " << (reachable ? "YES" : "NO") << std::endl;

        if (!reachable) {
            if (!within_x)
                std::cout << "    X out of bounds by: " << std::min(abs(target.x - min_x), abs(target.x - max_x)) << "mm" << std::endl;
            if (!within_y)
                std::cout << "    Y out of bounds by: " << std::min(abs(target.y - min_y), abs(target.y - max_y)) << "mm" << std::endl;
            if (!within_z)
                std::cout << "    Z out of bounds by: " << std::min(abs(target.z - min_z), abs(target.z - max_z)) << "mm" << std::endl;
            if (!within_dist)
                std::cout << "    Distance out of bounds by: " << std::min(abs(target_dist - min_dist), abs(target_dist - max_dist)) << "mm" << std::endl;
        }
    }

    return 0;
}
