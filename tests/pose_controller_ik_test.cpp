#include "../src/body_pose_controller.h"
#include "../src/body_pose_config_factory.h"
#include "../src/robot_model.h"
#include "../src/leg.h"
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

    RobotModel model(p);

    // Create body pose configuration
    BodyPoseConfiguration config = getDefaultBodyPoseConfig(p);

    // Create BodyPoseController
    BodyPoseController pose_controller(model, config);

    // Create leg objects
    Leg legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)
    };

    // Initialize legs with default stance
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].initialize(model, Pose::Identity());
        legs[i].updateForwardKinematics(model);
    }

    std::cout << "=== BodyPoseController IK Test ===" << std::endl;

    // Test 1: Check initial leg positions
    std::cout << "\n--- Test 1: Initial Leg Positions ---" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles angles = legs[i].getJointAngles();
        Point3D tip_pos = legs[i].getCurrentTipPositionGlobal();

        std::cout << "Leg " << i << ":" << std::endl;
        std::cout << "  Angles (deg): coxa=" << (angles.coxa * 180.0 / M_PI)
                  << ", femur=" << (angles.femur * 180.0 / M_PI)
                  << ", tibia=" << (angles.tibia * 180.0 / M_PI) << std::endl;
        std::cout << "  Tip position: (" << tip_pos.x << ", " << tip_pos.y << ", " << tip_pos.z << ")" << std::endl;
    }

    // Test 2: Set individual leg position and verify IK consistency
    std::cout << "\n--- Test 2: Individual Leg Position Setting ---" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Get current position
        Point3D current_pos = legs[i].getCurrentTipPositionGlobal();
        JointAngles current_angles = legs[i].getJointAngles();

        // Create a small offset target
        Point3D target_pos = current_pos;
        target_pos.x += 10.0; // Move 10mm in X direction
        target_pos.z -= 5.0;  // Move 5mm down

        std::cout << "Leg " << i << " - Moving from (" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z
                  << ") to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;

        // Set new position using BodyPoseController
        bool success = pose_controller.setLegPosition(i, target_pos, legs);

        if (success) {
            // Get new angles and position
            JointAngles new_angles = legs[i].getJointAngles();
            Point3D new_pos = legs[i].getCurrentTipPositionGlobal();

            // Calculate IK error
            double ik_error = (new_pos - target_pos).norm();

            std::cout << "  Success: " << (success ? "YES" : "NO") << std::endl;
            std::cout << "  Angle change (deg): coxa=" << ((new_angles.coxa - current_angles.coxa) * 180.0 / M_PI)
                      << ", femur=" << ((new_angles.femur - current_angles.femur) * 180.0 / M_PI)
                      << ", tibia=" << ((new_angles.tibia - current_angles.tibia) * 180.0 / M_PI) << std::endl;
            std::cout << "  IK error: " << ik_error << " mm" << std::endl;

            // Verify that angles changed (should use current angles as starting point)
            double angle_change = std::abs(new_angles.coxa - current_angles.coxa) +
                                std::abs(new_angles.femur - current_angles.femur) +
                                std::abs(new_angles.tibia - current_angles.tibia);

            if (angle_change < 0.001) {
                std::cout << "  ⚠️  WARNING: No angle change detected - IK may not be working properly" << std::endl;
            } else {
                std::cout << "  ✅ Angle change detected - IK working properly" << std::endl;
            }

            if (ik_error > 1.0) {
                std::cout << "  ⚠️  WARNING: High IK error (" << ik_error << " mm)" << std::endl;
            } else {
                std::cout << "  ✅ Low IK error (" << ik_error << " mm)" << std::endl;
            }
        } else {
            std::cout << "  ❌ Failed to set leg position" << std::endl;
        }
    }

    // Test 3: Set body pose and verify all legs use current angles
    std::cout << "\n--- Test 3: Body Pose Setting ---" << std::endl;

    // Store current angles for comparison
    JointAngles original_angles[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        original_angles[i] = legs[i].getJointAngles();
    }

    // Set a new body pose
    Eigen::Vector3d new_position(0, 0, -130); // 10mm lower
    Eigen::Vector3d new_orientation(0, 0, 0); // No rotation

    bool pose_success = pose_controller.setBodyPose(new_position, new_orientation, legs);
    std::cout << "Body pose setting: " << (pose_success ? "SUCCESS" : "FAILED") << std::endl;

    // Check if legs were updated and angles changed appropriately
    for (int i = 0; i < NUM_LEGS; ++i) {
        JointAngles new_angles = legs[i].getJointAngles();
        Point3D new_pos = legs[i].getCurrentTipPositionGlobal();

        double angle_change = std::abs(new_angles.coxa - original_angles[i].coxa) +
                            std::abs(new_angles.femur - original_angles[i].femur) +
                            std::abs(new_angles.tibia - original_angles[i].tibia);

        std::cout << "Leg " << i << ":" << std::endl;
        std::cout << "  New position: (" << new_pos.x << ", " << new_pos.y << ", " << new_pos.z << ")" << std::endl;
        std::cout << "  Angle change magnitude: " << (angle_change * 180.0 / M_PI) << " degrees" << std::endl;

        if (angle_change < 0.001) {
            std::cout << "  ⚠️  WARNING: No angle change - body pose may not be working" << std::endl;
        } else {
            std::cout << "  ✅ Angle change detected - body pose working" << std::endl;
        }
    }

    std::cout << "\n=== BodyPoseController IK Test Complete ===" << std::endl;
    return 0;
}