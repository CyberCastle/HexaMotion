#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Forward declarations for coordinate transformation functions from test
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos);
Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos);

int main() {
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Transform Debug Test ===" << std::endl;

    // Test the problematic target from test 4
    Point3D local_target(80.0, -30.0, -160.0);

    std::cout << "\n--- Testing Transformations for Local Target: ("
              << local_target.x << ", " << local_target.y << ", " << local_target.z << ") ---" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "\nLeg " << leg << ":" << std::endl;

        // Test 1: Test functions from simple_ik_test.cpp
        Point3D global_from_test = transformLocalToGlobal(model, leg, local_target);
        Point3D local_back_from_test = transformGlobalToLocal(model, leg, global_from_test);

        std::cout << "  Test functions: local -> global -> local" << std::endl;
        std::cout << "    Global: (" << global_from_test.x << ", " << global_from_test.y << ", " << global_from_test.z << ")" << std::endl;
        std::cout << "    Local back: (" << local_back_from_test.x << ", " << local_back_from_test.y << ", " << local_back_from_test.z << ")" << std::endl;

        double test_error = std::sqrt(
            std::pow(local_back_from_test.x - local_target.x, 2) +
            std::pow(local_back_from_test.y - local_target.y, 2) +
            std::pow(local_back_from_test.z - local_target.z, 2));
        std::cout << "    Round-trip error: " << test_error << std::endl;

        // Test 2: Test RobotModel functions with zero angles
        JointAngles zero_angles(0, 0, 0);
        Point3D global_from_model = model.transformLocalToGlobalCoordinates(leg, local_target, zero_angles);
        Point3D local_back_from_model = model.transformGlobalToLocalCoordinates(leg, global_from_model, zero_angles);

        std::cout << "  Model functions (zero angles): local -> global -> local" << std::endl;
        std::cout << "    Global: (" << global_from_model.x << ", " << global_from_model.y << ", " << global_from_model.z << ")" << std::endl;
        std::cout << "    Local back: (" << local_back_from_model.x << ", " << local_back_from_model.y << ", " << local_back_from_model.z << ")" << std::endl;

        double model_error = std::sqrt(
            std::pow(local_back_from_model.x - local_target.x, 2) +
            std::pow(local_back_from_model.y - local_target.y, 2) +
            std::pow(local_back_from_model.z - local_target.z, 2));
        std::cout << "    Round-trip error: " << model_error << std::endl;

        // Test 3: Compare the global coordinates
        double global_diff = std::sqrt(
            std::pow(global_from_test.x - global_from_model.x, 2) +
            std::pow(global_from_test.y - global_from_model.y, 2) +
            std::pow(global_from_test.z - global_from_model.z, 2));
        std::cout << "    Global coordinate difference: " << global_diff << std::endl;
    }

    return 0;
}

// Copy the transformation functions from simple_ik_test.cpp
Point3D transformGlobalToLocal(const RobotModel &model, int leg, const Point3D &global_pos) {
    // Get the leg base position and orientation from DH parameters
    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {-30.0f, -90.0f, -150.0f, 150.0f, 90.0f, 30.0f};
    const Parameters &params = model.getParams();

    // Calculate leg base position
    double base_angle_deg = BASE_THETA_OFFSETS[leg];
    double base_angle_rad = math_utils::degreesToRadians(base_angle_deg);
    double base_x = params.hexagon_radius * cos(base_angle_rad);
    double base_y = params.hexagon_radius * sin(base_angle_rad);

    // Translate to leg base coordinate system
    double dx = global_pos.x - base_x;
    double dy = global_pos.y - base_y;
    double dz = global_pos.z;

    // Rotate by negative base angle to align with leg's local coordinate system
    double neg_angle_rad = -base_angle_rad;
    double local_x = cos(neg_angle_rad) * dx - sin(neg_angle_rad) * dy;
    double local_y = sin(neg_angle_rad) * dx + cos(neg_angle_rad) * dy;
    double local_z = dz;

    return Point3D{local_x, local_y, local_z};
}

Point3D transformLocalToGlobal(const RobotModel &model, int leg, const Point3D &local_pos) {
    // Get the leg base position and orientation from DH parameters
    static const double BASE_THETA_OFFSETS[NUM_LEGS] = {-30.0f, -90.0f, -150.0f, 150.0f, 90.0f, 30.0f};
    const Parameters &params = model.getParams();

    // Calculate leg base position
    double base_angle_deg = BASE_THETA_OFFSETS[leg];
    double base_angle_rad = math_utils::degreesToRadians(base_angle_deg);
    double base_x = params.hexagon_radius * cos(base_angle_rad);
    double base_y = params.hexagon_radius * sin(base_angle_rad);

    // Rotate from leg's local coordinate system by base angle
    double rotated_x = cos(base_angle_rad) * local_pos.x - sin(base_angle_rad) * local_pos.y;
    double rotated_y = sin(base_angle_rad) * local_pos.x + cos(base_angle_rad) * local_pos.y;
    double rotated_z = local_pos.z;

    // Translate to global coordinate system
    double global_x = rotated_x + base_x;
    double global_y = rotated_y + base_y;
    double global_z = rotated_z;

    return Point3D{global_x, global_y, global_z};
}