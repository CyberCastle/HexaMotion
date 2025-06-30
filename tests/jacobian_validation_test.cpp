#include "HexaModel.h"
#include "math_utils.h"
#include <iomanip>
#include <iostream>
#include <cmath>

// Numerical differentiation to validate Jacobian
Eigen::Matrix3f numericalJacobian(const RobotModel& model, int leg,
                                  const JointAngles& angles,
                                  float delta = 0.001f) {
    Eigen::Matrix3f jacobian;

    // Get base position
    Point3D base_pos = model.forwardKinematics(leg, angles);

    // Test each joint using central differences for better accuracy
    for (int joint = 0; joint < 3; ++joint) {
        float delta_deg = math_utils::radiansToDegrees(delta);

        JointAngles plus = angles;
        JointAngles minus = angles;

        switch (joint) {
            case 0:
                plus.coxa += delta_deg * 0.5f;
                minus.coxa -= delta_deg * 0.5f;
                break;
            case 1:
                plus.femur += delta_deg * 0.5f;
                minus.femur -= delta_deg * 0.5f;
                break;
            case 2:
                plus.tibia += delta_deg * 0.5f;
                minus.tibia -= delta_deg * 0.5f;
                break;
        }

        Point3D pos_plus = model.forwardKinematics(leg, plus);
        Point3D pos_minus = model.forwardKinematics(leg, minus);

        // Calculate partial derivative
        jacobian(0, joint) = (pos_plus.x - pos_minus.x) / delta;
        jacobian(1, joint) = (pos_plus.y - pos_minus.y) / delta;
        jacobian(2, joint) = (pos_plus.z - pos_minus.z) / delta;
    }

    return jacobian;
}

void printMatrix(const Eigen::Matrix3f& matrix, const std::string& name) {
    std::cout << name << ":" << std::endl;
    std::cout << "┌─────────────────────────────────────┐" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << "│ ";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(6) << matrix(i, j);
        }
        std::cout << " │" << std::endl;
    }
    std::cout << "└─────────────────────────────────────┘" << std::endl;
}

void printErrorAnalysis(const Eigen::Matrix3f& analytical, const Eigen::Matrix3f& numerical) {
    Eigen::Matrix3f error = analytical - numerical;
    float max_error = error.cwiseAbs().maxCoeff();
    float avg_error = error.cwiseAbs().mean();
    float max_relative_error = (numerical.cwiseAbs().array() > 1e-6f).select(
        (error.cwiseAbs().array() / numerical.cwiseAbs().array()), 0.0f).maxCoeff();

    std::cout << "📊 ERROR ANALYSIS:" << std::endl;
    std::cout << "├─ Max Absolute Error: " << std::setw(12) << std::fixed << std::setprecision(6) << max_error << std::endl;
    std::cout << "├─ Avg Absolute Error: " << std::setw(12) << std::fixed << std::setprecision(6) << avg_error << std::endl;
    std::cout << "└─ Max Relative Error: " << std::setw(12) << std::fixed << std::setprecision(6) << max_relative_error << std::endl;

    if (max_error > 1e-3f) {
        std::cout << "⚠️  WARNING: Large Jacobian error detected!" << std::endl;
    } else if (max_error > 1e-4f) {
        std::cout << "⚠️  WARNING: Moderate Jacobian error detected!" << std::endl;
    } else {
        std::cout << "✅ Jacobian validation PASSED" << std::endl;
    }

    std::cout << std::endl;
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

    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    JACOBIAN VALIDATION TEST                  ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  This test validates the analytical Jacobian calculation     ║" << std::endl;
    std::cout << "║  by comparing it with numerical differentiation results.    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Robot Parameters:                                          ║" << std::endl;
    std::cout << "║    • Hexagon radius: " << std::setw(4) << p.hexagon_radius << " mm" << std::setw(32) << "║" << std::endl;
    std::cout << "║    • Coxa length: " << std::setw(6) << p.coxa_length << " mm" << std::setw(34) << "║" << std::endl;
    std::cout << "║    • Femur length: " << std::setw(5) << p.femur_length << " mm" << std::setw(34) << "║" << std::endl;
    std::cout << "║    • Tibia length: " << std::setw(5) << p.tibia_length << " mm" << std::setw(34) << "║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;

    // Test with zero angles first
    JointAngles zero_angles(0, 0, 0);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                        LEG " << leg << " VALIDATION                          ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
        std::cout << std::endl;

        std::cout << "🔧 Test Configuration:" << std::endl;
        std::cout << "   • Joint angles: coxa=" << std::setw(6) << zero_angles.coxa
                  << "°, femur=" << std::setw(6) << zero_angles.femur
                  << "°, tibia=" << std::setw(6) << zero_angles.tibia << "°" << std::endl;

        Point3D base_pos = model.forwardKinematics(leg, zero_angles);
        std::cout << "   • End-effector position: (" << std::setw(8) << base_pos.x
                  << ", " << std::setw(8) << base_pos.y
                  << ", " << std::setw(8) << base_pos.z << ") mm" << std::endl;
        std::cout << std::endl;

        std::cout << "📐 Calculating Analytical Jacobian..." << std::endl;
        Eigen::Matrix3f analytical_jacobian = model.calculateJacobian(leg, zero_angles, Point3D(0,0,0));
        printMatrix(analytical_jacobian, "Analytical Jacobian (∂x/∂θ)");

        std::cout << "🔢 Calculating Numerical Jacobian (finite differences)..." << std::endl;
        Eigen::Matrix3f numerical_jacobian = numericalJacobian(model, leg, zero_angles);
        printMatrix(numerical_jacobian, "Numerical Jacobian (finite differences)");

        printErrorAnalysis(analytical_jacobian, numerical_jacobian);
    }

    // Test with a very simple case - just one joint
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                 SINGLE JOINT PERTURBATION TEST              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;

    for (int leg = 0; leg < 1; ++leg) { // Just test leg 0
        std::cout << "🔍 Detailed Analysis for Leg " << leg << ":" << std::endl;
        std::cout << std::endl;

        JointAngles test_angles(0, 0, 0);
        Point3D base_pos = model.forwardKinematics(leg, test_angles);
        std::cout << "📍 Base position: (" << std::setw(8) << base_pos.x
                  << ", " << std::setw(8) << base_pos.y
                  << ", " << std::setw(8) << base_pos.z << ") mm" << std::endl;

        // Test coxa joint only
        float perturbation = 0.001f; // 0.001 radians ≈ 0.057 degrees
        float perturbation_deg = math_utils::radiansToDegrees(perturbation);

        JointAngles plus = test_angles;
        JointAngles minus = test_angles;
        plus.coxa += perturbation_deg * 0.5f;
        minus.coxa -= perturbation_deg * 0.5f;
        Point3D pos_plus = model.forwardKinematics(leg, plus);
        Point3D pos_minus = model.forwardKinematics(leg, minus);
        std::cout << "📍 Perturbed position (coxa +" << perturbation << " rad): ("
                  << std::setw(8) << pos_plus.x
                  << ", " << std::setw(8) << pos_plus.y
                  << ", " << std::setw(8) << pos_plus.z << ") mm" << std::endl;

        float dx = (pos_plus.x - pos_minus.x) / perturbation;
        float dy = (pos_plus.y - pos_minus.y) / perturbation;
        float dz = (pos_plus.z - pos_minus.z) / perturbation;

        std::cout << "📊 Numerical derivatives (∂x/∂θ_coxa):" << std::endl;
        std::cout << "   • ∂x/∂θ_coxa = " << std::setw(12) << dx << " mm/rad" << std::endl;
        std::cout << "   • ∂y/∂θ_coxa = " << std::setw(12) << dy << " mm/rad" << std::endl;
        std::cout << "   • ∂z/∂θ_coxa = " << std::setw(12) << dz << " mm/rad" << std::endl;

        Eigen::Matrix3f analytical_jacobian = model.calculateJacobian(leg, test_angles, Point3D(0,0,0));
        std::cout << "📐 Analytical derivatives (∂x/∂θ_coxa):" << std::endl;
        std::cout << "   • ∂x/∂θ_coxa = " << std::setw(12) << analytical_jacobian(0,0) << " mm/rad" << std::endl;
        std::cout << "   • ∂y/∂θ_coxa = " << std::setw(12) << analytical_jacobian(1,0) << " mm/rad" << std::endl;
        std::cout << "   • ∂z/∂θ_coxa = " << std::setw(12) << analytical_jacobian(2,0) << " mm/rad" << std::endl;

        std::cout << std::endl;

        // Calculate individual errors
        float error_x = std::abs(dx - analytical_jacobian(0,0));
        float error_y = std::abs(dy - analytical_jacobian(1,0));
        float error_z = std::abs(dz - analytical_jacobian(2,0));

        std::cout << "📊 Individual Errors:" << std::endl;
        std::cout << "   • Error in ∂x/∂θ_coxa: " << std::setw(12) << error_x << " mm/rad" << std::endl;
        std::cout << "   • Error in ∂y/∂θ_coxa: " << std::setw(12) << error_y << " mm/rad" << std::endl;
        std::cout << "   • Error in ∂z/∂θ_coxa: " << std::setw(12) << error_z << " mm/rad" << std::endl;

        float max_error = std::max({error_x, error_y, error_z});
        if (max_error < 1e-4f) {
            std::cout << "✅ Single joint test PASSED (max error < 1e-4)" << std::endl;
        } else {
            std::cout << "⚠️  Single joint test FAILED (max error = " << max_error << ")" << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                        TEST COMPLETED                       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;

    return 0;
}