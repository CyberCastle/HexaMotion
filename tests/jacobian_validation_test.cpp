#include "HexaModel.h"
#include <iomanip>
#include <iostream>
#include <cmath>

// Numerical differentiation to validate Jacobian
Eigen::Matrix3f numericalJacobian(const RobotModel& model, int leg, const JointAngles& angles, float delta = 0.001f) {
    Eigen::Matrix3f jacobian;

    // Get base position
    Point3D base_pos = model.forwardKinematics(leg, angles);

    // Test each joint
    for (int joint = 0; joint < 3; ++joint) {
        JointAngles perturbed = angles;

        // Perturb the joint angle
        switch (joint) {
            case 0: perturbed.coxa += delta; break;
            case 1: perturbed.femur += delta; break;
            case 2: perturbed.tibia += delta; break;
        }

        Point3D perturbed_pos = model.forwardKinematics(leg, perturbed);

        // Calculate partial derivative
        jacobian(0, joint) = (perturbed_pos.x - base_pos.x) / delta;
        jacobian(1, joint) = (perturbed_pos.y - base_pos.y) / delta;
        jacobian(2, joint) = (perturbed_pos.z - base_pos.z) / delta;
    }

    return jacobian;
}

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

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "=== Jacobian Validation Test ===" << std::endl;

    // Test with zero angles first
    JointAngles zero_angles(0, 0, 0);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "\n--- Leg " << leg << " (Zero Angles) ---" << std::endl;

        // Get analytical Jacobian
        Eigen::Matrix3f analytical_jacobian = model.calculateJacobian(leg, zero_angles, Point3D(0,0,0));

        // Get numerical Jacobian
        Eigen::Matrix3f numerical_jacobian = numericalJacobian(model, leg, zero_angles);

        std::cout << "Analytical Jacobian:" << std::endl;
        std::cout << analytical_jacobian << std::endl;

        std::cout << "Numerical Jacobian:" << std::endl;
        std::cout << numerical_jacobian << std::endl;

        // Calculate error
        Eigen::Matrix3f error = analytical_jacobian - numerical_jacobian;
        float max_error = error.cwiseAbs().maxCoeff();
        float avg_error = error.cwiseAbs().mean();

        std::cout << "Max Error: " << max_error << std::endl;
        std::cout << "Avg Error: " << avg_error << std::endl;

        // Detailed error analysis
        std::cout << "Error Matrix:" << std::endl;
        std::cout << error << std::endl;

        if (max_error > 1e-3f) {
            std::cout << "WARNING: Large Jacobian error detected!" << std::endl;
        }
    }

    // Test with a very simple case - just one joint
    std::cout << "\n=== Testing Single Joint Perturbation ===" << std::endl;

    for (int leg = 0; leg < 1; ++leg) { // Just test leg 0
        std::cout << "\n--- Leg " << leg << " Single Joint Test ---" << std::endl;

        JointAngles test_angles(0, 0, 0);
        Point3D base_pos = model.forwardKinematics(leg, test_angles);
        std::cout << "Base position: (" << base_pos.x << ", " << base_pos.y << ", " << base_pos.z << ")" << std::endl;

        // Test coxa joint only
        JointAngles perturbed = test_angles;
        perturbed.coxa += 0.001f; // 0.001 radians
        Point3D perturbed_pos = model.forwardKinematics(leg, perturbed);
        std::cout << "Perturbed position: (" << perturbed_pos.x << ", " << perturbed_pos.y << ", " << perturbed_pos.z << ")" << std::endl;

        float dx = (perturbed_pos.x - base_pos.x) / 0.001f;
        float dy = (perturbed_pos.y - base_pos.y) / 0.001f;
        float dz = (perturbed_pos.z - base_pos.z) / 0.001f;

        std::cout << "Numerical derivatives: dx=" << dx << ", dy=" << dy << ", dz=" << dz << std::endl;

        Eigen::Matrix3f analytical_jacobian = model.calculateJacobian(leg, test_angles, Point3D(0,0,0));
        std::cout << "Analytical first column: (" << analytical_jacobian(0,0) << ", "
                  << analytical_jacobian(1,0) << ", " << analytical_jacobian(2,0) << ")" << std::endl;
    }

    return 0;
}