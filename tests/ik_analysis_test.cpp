#include "../src/robot_model.h"
#include "../src/math_utils.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Initialize robot model
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
    p.ik.max_iterations = 50;
    p.ik.clamp_joints = true;

    RobotModel model(p);

    std::cout << "=== IK Analysis Test ===" << std::endl;
    std::cout << "IK_TOLERANCE: " << IK_TOLERANCE << " mm" << std::endl;
    std::cout << "IK_DLS_COEFFICIENT: " << IK_DLS_COEFFICIENT << std::endl;
    std::cout << "IK_MAX_ANGLE_STEP: " << IK_MAX_ANGLE_STEP << " degrees" << std::endl;
    std::cout << "Max iterations: " << p.ik.max_iterations << std::endl;

    // Test specific cases from walk_controller_test
    std::cout << "\n--- Analyzing Leg 0 Case ---" << std::endl;

    // Leg 0 from the test output
    int leg = 0;
    JointAngles original_angles(-0.000781926, -0.0962164, 0.0647442); // From test output
    Point3D target_position(282.56, -170.799, -221.524); // From test output

    std::cout << "Original angles (rad): coxa=" << original_angles.coxa
              << ", femur=" << original_angles.femur
              << ", tibia=" << original_angles.tibia << std::endl;

    std::cout << "Original angles (deg): coxa=" << (original_angles.coxa * 180.0 / M_PI)
              << ", femur=" << (original_angles.femur * 180.0 / M_PI)
              << ", tibia=" << (original_angles.tibia * 180.0 / M_PI) << std::endl;

    std::cout << "Target position: (" << target_position.x << ", "
              << target_position.y << ", " << target_position.z << ")" << std::endl;

    // Verify FK works correctly
    Point3D fk_position = model.forwardKinematicsGlobalCoordinates(leg, original_angles);
    std::cout << "FK position: (" << fk_position.x << ", "
              << fk_position.y << ", " << fk_position.z << ")" << std::endl;

    double fk_error = std::sqrt(std::pow(fk_position.x - target_position.x, 2) +
                                std::pow(fk_position.y - target_position.y, 2) +
                                std::pow(fk_position.z - target_position.z, 2));
    std::cout << "FK error: " << fk_error << " mm" << std::endl;

    // Test IK with different methods
    std::cout << "\n--- Testing Different IK Methods ---" << std::endl;

    // Method 1: Standard IK
    JointAngles ik1 = model.inverseKinematicsGlobalCoordinates(leg, target_position);
    Point3D fk1 = model.forwardKinematicsGlobalCoordinates(leg, ik1);
    double error1 = std::sqrt(std::pow(fk1.x - target_position.x, 2) +
                              std::pow(fk1.y - target_position.y, 2) +
                              std::pow(fk1.z - target_position.z, 2));

    std::cout << "Method 1 (Standard IK):" << std::endl;
    std::cout << "  IK angles (rad): coxa=" << ik1.coxa << ", femur=" << ik1.femur << ", tibia=" << ik1.tibia << std::endl;
    std::cout << "  IK angles (deg): coxa=" << (ik1.coxa * 180.0 / M_PI) << ", femur=" << (ik1.femur * 180.0 / M_PI) << ", tibia=" << (ik1.tibia * 180.0 / M_PI) << std::endl;
    std::cout << "  FK result: (" << fk1.x << ", " << fk1.y << ", " << fk1.z << ")" << std::endl;
    std::cout << "  Error: " << error1 << " mm" << std::endl;

    // Method 2: IK with current angles as starting point
    JointAngles ik2 = model.inverseKinematicsCurrentGlobalCoordinates(leg, original_angles, target_position);
    Point3D fk2 = model.forwardKinematicsGlobalCoordinates(leg, ik2);
    double error2 = std::sqrt(std::pow(fk2.x - target_position.x, 2) +
                              std::pow(fk2.y - target_position.y, 2) +
                              std::pow(fk2.z - target_position.z, 2));

    std::cout << "\nMethod 2 (IK with current angles):" << std::endl;
    std::cout << "  IK angles (rad): coxa=" << ik2.coxa << ", femur=" << ik2.femur << ", tibia=" << ik2.tibia << std::endl;
    std::cout << "  IK angles (deg): coxa=" << (ik2.coxa * 180.0 / M_PI) << ", femur=" << (ik2.femur * 180.0 / M_PI) << ", tibia=" << (ik2.tibia * 180.0 / M_PI) << std::endl;
    std::cout << "  FK result: (" << fk2.x << ", " << fk2.y << ", " << fk2.z << ")" << std::endl;
    std::cout << "  Error: " << error2 << " mm" << std::endl;

    // Method 3: Local coordinates IK
    JointAngles ik3 = model.solveIKLocalCoordinates(leg, target_position, original_angles);
    Point3D fk3 = model.forwardKinematicsGlobalCoordinates(leg, ik3);
    double error3 = std::sqrt(std::pow(fk3.x - target_position.x, 2) +
                              std::pow(fk3.y - target_position.y, 2) +
                              std::pow(fk3.z - target_position.z, 2));

    std::cout << "\nMethod 3 (Local coordinates IK):" << std::endl;
    std::cout << "  IK angles (rad): coxa=" << ik3.coxa << ", femur=" << ik3.femur << ", tibia=" << ik3.tibia << std::endl;
    std::cout << "  IK angles (deg): coxa=" << (ik3.coxa * 180.0 / M_PI) << ", femur=" << (ik3.femur * 180.0 / M_PI) << ", tibia=" << (ik3.tibia * 180.0 / M_PI) << std::endl;
    std::cout << "  FK result: (" << fk3.x << ", " << fk3.y << ", " << fk3.z << ")" << std::endl;
    std::cout << "  Error: " << error3 << " mm" << std::endl;

    // Analyze the differences
    std::cout << "\n--- Analysis ---" << std::endl;
    std::cout << "Original vs Method 1 angle differences (deg):" << std::endl;
    std::cout << "  coxa: " << ((ik1.coxa - original_angles.coxa) * 180.0 / M_PI) << std::endl;
    std::cout << "  femur: " << ((ik1.femur - original_angles.femur) * 180.0 / M_PI) << std::endl;
    std::cout << "  tibia: " << ((ik1.tibia - original_angles.tibia) * 180.0 / M_PI) << std::endl;

    std::cout << "\nOriginal vs Method 2 angle differences (deg):" << std::endl;
    std::cout << "  coxa: " << ((ik2.coxa - original_angles.coxa) * 180.0 / M_PI) << std::endl;
    std::cout << "  femur: " << ((ik2.femur - original_angles.femur) * 180.0 / M_PI) << std::endl;
    std::cout << "  tibia: " << ((ik2.tibia - original_angles.tibia) * 180.0 / M_PI) << std::endl;

    std::cout << "\nOriginal vs Method 3 angle differences (deg):" << std::endl;
    std::cout << "  coxa: " << ((ik3.coxa - original_angles.coxa) * 180.0 / M_PI) << std::endl;
    std::cout << "  femur: " << ((ik3.femur - original_angles.femur) * 180.0 / M_PI) << std::endl;
    std::cout << "  tibia: " << ((ik3.tibia - original_angles.tibia) * 180.0 / M_PI) << std::endl;

    // Test with different tolerances
    std::cout << "\n--- Testing with Different Tolerances ---" << std::endl;

    // Temporarily modify tolerance for testing
    const double original_tolerance = IK_TOLERANCE;
    std::cout << "Testing with tolerance: " << original_tolerance << " mm" << std::endl;

    // Test with tighter tolerance
    std::cout << "Note: Current tolerance is " << original_tolerance << " mm" << std::endl;
    std::cout << "This means IK will stop when error < " << original_tolerance << " mm" << std::endl;

    if (error1 > original_tolerance) {
        std::cout << "WARNING: Method 1 error (" << error1 << " mm) > tolerance (" << original_tolerance << " mm)" << std::endl;
    }
    if (error2 > original_tolerance) {
        std::cout << "WARNING: Method 2 error (" << error2 << " mm) > tolerance (" << original_tolerance << " mm)" << std::endl;
    }
    if (error3 > original_tolerance) {
        std::cout << "WARNING: Method 3 error (" << error3 << " mm) > tolerance (" << original_tolerance << " mm)" << std::endl;
    }

    // Test multiple solutions for the same target
    std::cout << "\n--- Testing Multiple Solutions ---" << std::endl;

    // Test with different starting angles
    std::vector<JointAngles> starting_angles = {
        JointAngles(0, 0, 0),
        JointAngles(0.1, 0.1, 0.1),
        JointAngles(-0.1, -0.1, -0.1),
        original_angles
    };

    for (size_t i = 0; i < starting_angles.size(); ++i) {
        JointAngles start = starting_angles[i];
        JointAngles ik_result = model.inverseKinematicsCurrentGlobalCoordinates(leg, start, target_position);
        Point3D fk_result = model.forwardKinematicsGlobalCoordinates(leg, ik_result);
        double error = std::sqrt(std::pow(fk_result.x - target_position.x, 2) +
                                std::pow(fk_result.y - target_position.y, 2) +
                                std::pow(fk_result.z - target_position.z, 2));

        std::cout << "Starting angles " << i << " (deg): coxa=" << (start.coxa * 180.0 / M_PI)
                  << ", femur=" << (start.femur * 180.0 / M_PI)
                  << ", tibia=" << (start.tibia * 180.0 / M_PI) << std::endl;
        std::cout << "  Result angles (deg): coxa=" << (ik_result.coxa * 180.0 / M_PI)
                  << ", femur=" << (ik_result.femur * 180.0 / M_PI)
                  << ", tibia=" << (ik_result.tibia * 180.0 / M_PI) << std::endl;
        std::cout << "  Error: " << error << " mm" << std::endl;
    }

    // Test the new estimateInitialAngles function
    std::cout << "\n--- Testing New estimateInitialAngles Function ---" << std::endl;

    // Test with the same target position
    JointAngles estimated_angles = model.estimateInitialAngles(leg, target_position);
    std::cout << "Estimated angles (rad): coxa=" << estimated_angles.coxa
              << ", femur=" << estimated_angles.femur
              << ", tibia=" << estimated_angles.tibia << std::endl;

    std::cout << "Estimated angles (deg): coxa=" << (estimated_angles.coxa * 180.0 / M_PI)
              << ", femur=" << (estimated_angles.femur * 180.0 / M_PI)
              << ", tibia=" << (estimated_angles.tibia * 180.0 / M_PI) << std::endl;

    // Test IK with estimated angles as starting point
    JointAngles ik_with_estimated = model.inverseKinematicsCurrentGlobalCoordinates(leg, estimated_angles, target_position);
    Point3D fk_with_estimated = model.forwardKinematicsGlobalCoordinates(leg, ik_with_estimated);
    double error_with_estimated = std::sqrt(std::pow(fk_with_estimated.x - target_position.x, 2) +
                                           std::pow(fk_with_estimated.y - target_position.y, 2) +
                                           std::pow(fk_with_estimated.z - target_position.z, 2));

    std::cout << "IK with estimated angles:" << std::endl;
    std::cout << "  Final angles (deg): coxa=" << (ik_with_estimated.coxa * 180.0 / M_PI)
              << ", femur=" << (ik_with_estimated.femur * 180.0 / M_PI)
              << ", tibia=" << (ik_with_estimated.tibia * 180.0 / M_PI) << std::endl;
    std::cout << "  FK result: (" << fk_with_estimated.x << ", " << fk_with_estimated.y << ", " << fk_with_estimated.z << ")" << std::endl;
    std::cout << "  Error: " << error_with_estimated << " mm" << std::endl;

    // Compare with original angles
    std::cout << "\nComparison with original angles:" << std::endl;
    std::cout << "Original vs Estimated angle differences (deg):" << std::endl;
    std::cout << "  coxa: " << ((estimated_angles.coxa - original_angles.coxa) * 180.0 / M_PI) << std::endl;
    std::cout << "  femur: " << ((estimated_angles.femur - original_angles.femur) * 180.0 / M_PI) << std::endl;
    std::cout << "  tibia: " << ((estimated_angles.tibia - original_angles.tibia) * 180.0 / M_PI) << std::endl;

    // Test with different target positions
    std::cout << "\n--- Testing estimateInitialAngles with Different Targets ---" << std::endl;

    std::vector<Point3D> test_targets = {
        Point3D(300.0, 0.0, -200.0),    // Forward position
        Point3D(200.0, 100.0, -150.0),  // Diagonal position
        Point3D(150.0, -50.0, -180.0),  // Side position
        Point3D(250.0, 0.0, -208.0)     // Higher position
    };

    for (size_t i = 0; i < test_targets.size(); ++i) {
        Point3D test_target = test_targets[i];
        JointAngles test_estimated = model.estimateInitialAngles(leg, test_target);
        JointAngles test_ik = model.inverseKinematicsCurrentGlobalCoordinates(leg, test_estimated, test_target);
        Point3D test_fk = model.forwardKinematicsGlobalCoordinates(leg, test_ik);
        double test_error = std::sqrt(std::pow(test_fk.x - test_target.x, 2) +
                                     std::pow(test_fk.y - test_target.y, 2) +
                                     std::pow(test_fk.z - test_target.z, 2));

        std::cout << "Target " << (i + 1) << " (" << test_target.x << ", " << test_target.y << ", " << test_target.z << "):" << std::endl;
        std::cout << "  Estimated angles (deg): coxa=" << (test_estimated.coxa * 180.0 / M_PI)
                  << ", femur=" << (test_estimated.femur * 180.0 / M_PI)
                  << ", tibia=" << (test_estimated.tibia * 180.0 / M_PI) << std::endl;
        std::cout << "  Final IK angles (deg): coxa=" << (test_ik.coxa * 180.0 / M_PI)
                  << ", femur=" << (test_ik.femur * 180.0 / M_PI)
                  << ", tibia=" << (test_ik.tibia * 180.0 / M_PI) << std::endl;
        std::cout << "  Error: " << test_error << " mm" << std::endl;
    }

    std::cout << "\n=== Analysis Complete ===" << std::endl;
    return 0;
}