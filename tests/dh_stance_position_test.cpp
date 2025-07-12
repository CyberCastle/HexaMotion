#include <iostream>
#include <iomanip>
#include "../src/body_pose_config_factory.h"
#include "../src/robot_model.h"

int main() {
    std::cout << "=== DH-Based Stance Position Test ===" << std::endl;

    // Use the test parameters from AGENTS.md
    Parameters params{};
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    params.robot_height = 208;
    params.control_frequency = 50;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    std::cout << "Robot Parameters:" << std::endl;
    std::cout << "  Hexagon radius: " << params.hexagon_radius << " mm" << std::endl;
    std::cout << "  Coxa length: " << params.coxa_length << " mm" << std::endl;
    std::cout << "  Femur length: " << params.femur_length << " mm" << std::endl;
    std::cout << "  Tibia length: " << params.tibia_length << " mm" << std::endl;
    std::cout << "  Robot height: " << params.robot_height << " mm" << std::endl;
    std::cout << std::endl;

    // Test the DH-based stance position calculation
    std::cout << "Calculating stance positions using DH parameters..." << std::endl;
    auto stance_positions = calculateHexagonalStancePositions(params);

    std::cout << std::endl;
    std::cout << "Stance Positions (DH-based calculation):" << std::endl;
    std::cout << "Leg | X (mm) | Y (mm) | Radius (mm)" << std::endl;
    std::cout << "----|--------|--------|------------" << std::endl;

    double expected_radius = params.hexagon_radius + params.coxa_length; // 200 + 50 = 250mm

    for (int i = 0; i < NUM_LEGS; i++) {
        double actual_radius = sqrt(stance_positions[i].x * stance_positions[i].x +
                                   stance_positions[i].y * stance_positions[i].y);

        std::cout << " L" << (i+1) << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << stance_positions[i].x << " | "
                  << std::setw(6) << stance_positions[i].y << " | "
                  << std::setw(10) << actual_radius << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Expected radius: " << expected_radius << " mm" << std::endl;

    // Additional verification: test with RobotModel directly
    std::cout << std::endl;
    std::cout << "=== Direct RobotModel Verification ===" << std::endl;
    RobotModel robot_model(params);
    JointAngles neutral_angles(0.0, 0.0, 0.0);

    std::cout << "Verifying with direct forward kinematics..." << std::endl;
    std::cout << "Leg | Base Pos (x,y) | FK Pos (x,y,z) | Radius" << std::endl;
    std::cout << "----|----------------|----------------|-------" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D base_pos = robot_model.getLegBasePosition(i);
        Point3D fk_position = robot_model.forwardKinematicsGlobalCoordinates(i, neutral_angles);
        double fk_radius = sqrt(fk_position.x * fk_position.x + fk_position.y * fk_position.y);

        std::cout << " L" << (i+1) << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << base_pos.x << ","
                  << std::setw(6) << base_pos.y << " | "
                  << std::setw(6) << fk_position.x << ","
                  << std::setw(6) << fk_position.y << ","
                  << std::setw(6) << fk_position.z << " | "
                  << std::setw(5) << fk_radius << std::endl;

        double x_diff = std::abs(fk_position.x - stance_positions[i].x);
        double y_diff = std::abs(fk_position.y - stance_positions[i].y);

        if (x_diff > 0.1 || y_diff > 0.1) {
            std::cout << "❌ ERROR: Leg " << (i+1) << " position mismatch!" << std::endl;
            std::cout << "  DH calculation: (" << stance_positions[i].x << ", " << stance_positions[i].y << ")" << std::endl;
            std::cout << "  Direct FK: (" << fk_position.x << ", " << fk_position.y << ")" << std::endl;
            return 1;
        }
    }

    // Calculate what the analytical result should be
    std::cout << std::endl;
    std::cout << "=== Analytical Calculation (for comparison) ===" << std::endl;
    std::cout << "Leg | Analytical (x,y) | DH (x,y) | Difference" << std::endl;
    std::cout << "----|------------------|----------|-----------" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D base_pos = robot_model.getLegBasePosition(i);
        double analytical_x = base_pos.x + params.coxa_length * cos(atan2(base_pos.y, base_pos.x));
        double analytical_y = base_pos.y + params.coxa_length * sin(atan2(base_pos.y, base_pos.x));

        double x_diff = std::abs(analytical_x - stance_positions[i].x);
        double y_diff = std::abs(analytical_y - stance_positions[i].y);

        std::cout << " L" << (i+1) << " | "
                  << std::fixed << std::setprecision(1) << std::setw(6) << analytical_x << ","
                  << std::setw(6) << analytical_y << " | "
                  << std::setw(6) << stance_positions[i].x << ","
                  << std::setw(6) << stance_positions[i].y << " | "
                  << std::setw(9) << sqrt(x_diff*x_diff + y_diff*y_diff) << std::endl;
    }

    std::cout << std::endl;
    std::cout << "✅ SUCCESS: DH-based calculation matches direct forward kinematics!" << std::endl;
    std::cout << "Note: The DH calculation includes the full leg chain (coxa + femur + tibia)" << std::endl;
    std::cout << "while the analytical calculation only includes coxa length." << std::endl;

    return 0;
}