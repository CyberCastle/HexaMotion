#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>

/** Numerical differentiation to validate Jacobian. */
Eigen::Matrix3d numericalJacobian(const RobotModel &model, int leg,
                                  const JointAngles &angles,
                                  double delta = JACOBIAN_DELTA) {
    Eigen::Matrix3d jacobian;

    /** Get base position. */
    Point3D base_pos = model.forwardKinematicsGlobalCoordinates(leg, angles);

    /** Test each joint using central differences for better accuracy. */
    for (int joint = 0; joint < 3; ++joint) {
        JointAngles plus = angles;
        JointAngles minus = angles;

        switch (joint) {
        case 0:
            plus.coxa += delta * 0.5f;
            minus.coxa -= delta * 0.5f;
            break;
        case 1:
            plus.femur += delta * 0.5f;
            minus.femur -= delta * 0.5f;
            break;
        case 2:
            plus.tibia += delta * 0.5f;
            minus.tibia -= delta * 0.5f;
            break;
        }

        Point3D pos_plus = model.forwardKinematicsGlobalCoordinates(leg, plus);
        Point3D pos_minus = model.forwardKinematicsGlobalCoordinates(leg, minus);

        /** Calculate partial derivative. */
        jacobian(0, joint) = (pos_plus.x - pos_minus.x) / delta;
        jacobian(1, joint) = (pos_plus.y - pos_minus.y) / delta;
        jacobian(2, joint) = (pos_plus.z - pos_minus.z) / delta;
    }

    return jacobian;
}

void printMatrix(const Eigen::Matrix3d &matrix, const std::string &name) {
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

void printErrorAnalysis(const Eigen::Matrix3d &analytical, const Eigen::Matrix3d &numerical) {
    Eigen::Matrix3d error = analytical - numerical;
    double max_error = error.cwiseAbs().maxCoeff();
    double avg_error = error.cwiseAbs().mean();
    double max_relative_error = (numerical.cwiseAbs().array() > 1e-6f).select((error.cwiseAbs().array() / numerical.cwiseAbs().array()), 0.0f).maxCoeff();

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
    /** Set to -tibia_length for explicit configuration. */
    p.default_height_offset = -208.0;
    p.robot_height = 208;
    p.time_delta = 1.0 / 50.0;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    /** Initialize WorkspaceAnalyzer. */
    model.workspaceAnalyzerInitializer();

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

    /** Test with zero angles first. */
    JointAngles zero_angles(0, 0, 0);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                        LEG " << leg << " VALIDATION                          ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
        std::cout << std::endl;

        std::cout << "🔧 Test Configuration:" << std::endl;
        std::cout << "   • Joint angles: coxa=" << std::setw(6) << math_utils::radiansToDegrees(zero_angles.coxa)
                  << "°, femur=" << std::setw(6) << math_utils::radiansToDegrees(zero_angles.femur)
                  << "°, tibia=" << std::setw(6) << math_utils::radiansToDegrees(zero_angles.tibia) << "°" << std::endl;

        Point3D base_pos = model.forwardKinematicsGlobalCoordinates(leg, zero_angles);
        std::cout << "   • End-effector position: (" << std::setw(8) << base_pos.x
                  << ", " << std::setw(8) << base_pos.y
                  << ", " << std::setw(8) << base_pos.z << ") mm" << std::endl;
        std::cout << std::endl;

        std::cout << "📐 Calculating Analytical Jacobian..." << std::endl;
        Eigen::Matrix3d analytical_jacobian = model.calculateJacobian(leg, zero_angles, Point3D(0, 0, 0));
        printMatrix(analytical_jacobian, "Analytical Jacobian (∂x/∂θ)");

        std::cout << "🔢 Calculating Numerical Jacobian (finite differences)..." << std::endl;
        Eigen::Matrix3d numerical_jacobian = numericalJacobian(model, leg, zero_angles);
        printMatrix(numerical_jacobian, "Numerical Jacobian (finite differences)");

        printErrorAnalysis(analytical_jacobian, numerical_jacobian);
    }

    /** Test with a very simple case: just one joint. */
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                 SINGLE JOINT PERTURBATION TEST              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;

    /** Just test leg 0. */
    for (int leg = 0; leg < 1; ++leg) {
        std::cout << "🔍 Detailed Analysis for Leg " << leg << ":" << std::endl;
        std::cout << std::endl;

        JointAngles test_angles(0, 0, 0);
        Point3D base_pos = model.forwardKinematicsGlobalCoordinates(leg, test_angles);
        std::cout << "📍 Base position: (" << std::setw(8) << base_pos.x
                  << ", " << std::setw(8) << base_pos.y
                  << ", " << std::setw(8) << base_pos.z << ") mm" << std::endl;

        /** Test coxa joint only. */
        /** 0.001 radians ~= 0.057 degree. */
        double perturbation = JACOBIAN_DELTA;

        JointAngles plus = test_angles;
        JointAngles minus = test_angles;
        plus.coxa += perturbation * 0.5f;
        minus.coxa -= perturbation * 0.5f;
        Point3D pos_plus = model.forwardKinematicsGlobalCoordinates(leg, plus);
        Point3D pos_minus = model.forwardKinematicsGlobalCoordinates(leg, minus);
        std::cout << "📍 Perturbed position (coxa +" << perturbation << " rad): ("
                  << std::setw(8) << pos_plus.x
                  << ", " << std::setw(8) << pos_plus.y
                  << ", " << std::setw(8) << pos_plus.z << ") mm" << std::endl;

        double dx = (pos_plus.x - pos_minus.x) / perturbation;
        double dy = (pos_plus.y - pos_minus.y) / perturbation;
        double dz = (pos_plus.z - pos_minus.z) / perturbation;

        std::cout << "📊 Numerical derivatives (∂x/∂θ_coxa):" << std::endl;
        std::cout << "   • ∂x/∂θ_coxa = " << std::setw(12) << dx << " mm/rad" << std::endl;
        std::cout << "   • ∂y/∂θ_coxa = " << std::setw(12) << dy << " mm/rad" << std::endl;
        std::cout << "   • ∂z/∂θ_coxa = " << std::setw(12) << dz << " mm/rad" << std::endl;

        Eigen::Matrix3d analytical_jacobian = model.calculateJacobian(leg, test_angles, Point3D(0, 0, 0));
        std::cout << "📐 Analytical derivatives (∂x/∂θ_coxa):" << std::endl;
        std::cout << "   • ∂x/∂θ_coxa = " << std::setw(12) << analytical_jacobian(0, 0) << " mm/rad" << std::endl;
        std::cout << "   • ∂y/∂θ_coxa = " << std::setw(12) << analytical_jacobian(1, 0) << " mm/rad" << std::endl;
        std::cout << "   • ∂z/∂θ_coxa = " << std::setw(12) << analytical_jacobian(2, 0) << " mm/rad" << std::endl;

        std::cout << std::endl;

        /** Calculate individual errors. */
        double error_x = std::abs(dx - analytical_jacobian(0, 0));
        double error_y = std::abs(dy - analytical_jacobian(1, 0));
        double error_z = std::abs(dz - analytical_jacobian(2, 0));

        std::cout << "📊 Individual Errors:" << std::endl;
        std::cout << "   • Error in ∂x/∂θ_coxa: " << std::setw(12) << error_x << " mm/rad" << std::endl;
        std::cout << "   • Error in ∂y/∂θ_coxa: " << std::setw(12) << error_y << " mm/rad" << std::endl;
        std::cout << "   • Error in ∂z/∂θ_coxa: " << std::setw(12) << error_z << " mm/rad" << std::endl;

        double max_error = std::max({error_x, error_y, error_z});
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