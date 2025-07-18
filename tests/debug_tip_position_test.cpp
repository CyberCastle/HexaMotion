#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config.h"
#include "../src/gait_config_factory.h"
#include "../src/leg_stepper.h"
#include "../src/walk_controller.h"
#include "../src/walkspace_analyzer.h"
#include "../src/workspace_validator.h"
#include "test_stubs.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

void debugTipPositionGeneration(LegStepper &stepper, Leg &leg, const RobotModel &model) {
    std::cout << "\n=== DEBUG: Tip Position Generation ===" << std::endl;

    // Get the actual initial position from the leg (after IK has been applied)
    Point3D initial_position(288.115, -166.343, -150);
    std::cout << "Initial tip position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;

    // Debug stepper state before update
    std::cout << "Identity tip pose: (" << stepper.getIdentityTipPose().x << ", " << stepper.getIdentityTipPose().y << ", " << stepper.getIdentityTipPose().z << ")" << std::endl;
    std::cout << "Default tip pose: (" << stepper.getDefaultTipPose().x << ", " << stepper.getDefaultTipPose().y << ", " << stepper.getDefaultTipPose().z << ")" << std::endl;
    std::cout << "Target tip pose: (" << stepper.getTargetTipPose().x << ", " << stepper.getTargetTipPose().y << ", " << stepper.getTargetTipPose().z << ")" << std::endl;
    std::cout << "Stride vector: (" << stepper.getStrideVector().x << ", " << stepper.getStrideVector().y << ", " << stepper.getStrideVector().z << ")" << std::endl;
    std::cout << "Swing clearance: (" << stepper.getSwingClearance().x << ", " << stepper.getSwingClearance().y << ", " << stepper.getSwingClearance().z << ")" << std::endl;

    // Test with desired velocity like in the original test
    stepper.setDesiredVelocity(Point3D(0.0, 0.0, 0.0), 0.0);
    stepper.updateStride();
    std::cout << "\nAfter setting zero velocity:" << std::endl;
    std::cout << "Stride vector: (" << stepper.getStrideVector().x << ", " << stepper.getStrideVector().y << ", " << stepper.getStrideVector().z << ")" << std::endl;

    // Test with velocity like in the original test
    Point3D initial_velocity = Point3D(10.0, 0, 0);
    stepper.setSwingOriginTipVelocity(initial_velocity);
    std::cout << "\nAfter setting swing origin velocity to (10, 0, 0):" << std::endl;

    // Set stepper to swing state
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(5);
    stepper.setStepProgress(0.5);

    // CRITICAL: Initialize timing parameters by calling updateTipPosition first
    // This will internally call updateDynamicTiming() and generate control nodes
    stepper.updateTipPosition(20.0, 0.02, false, false);

    // Now display the control nodes that were generated
    std::cout << "\nSwing control nodes (primary):" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing1ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    std::cout << "\nSwing control nodes (secondary):" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing2ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    // Test position update
    std::cout << "\nBefore updateTipPosition:" << std::endl;
    std::cout << "step_progress_: " << stepper.getStepProgress() << std::endl;

    // Generate complete swing trajectory (0% to 100%)
    std::cout << "\n=== COMPLETE SWING TRAJECTORY ===" << std::endl;
    std::cout << "Step | Progress | Position (x, y, z) | Curve Used" << std::endl;
    std::cout << "-----+----------+-------------------+-----------" << std::endl;

    for (int step = 0; step <= 20; step++) {
        double progress = step / 20.0; // 0.0 to 1.0
        stepper.setStepProgress(progress);

        // Update tip position for this progress
        stepper.updateTipPosition(20.0, 0.02, false, false);
        Point3D pos = leg.getCurrentTipPositionGlobal();

        std::string curve_used = (progress < 0.5) ? "Primary" : "Secondary";

        printf("%4d | %8.3f | (%8.3f, %8.3f, %8.3f) | %s\n",
               step, progress, pos.x, pos.y, pos.z, curve_used.c_str());
    }

    // Reset to original state
    stepper.setStepProgress(0.5);
    stepper.updateTipPosition(20.0, 0.02, false, false);

    Point3D final_position = leg.getCurrentTipPositionGlobal();
    std::cout << "\nFinal tip position (progress=0.5): (" << final_position.x << ", " << final_position.y << ", " << final_position.z << ")" << std::endl;

    double position_change = (final_position - initial_position).norm();
    std::cout << "Position change magnitude: " << position_change << " mm" << std::endl;

    // Verify joint limits for extreme positions
    std::cout << "\n=== JOINT LIMITS VERIFICATION ===" << std::endl;
    bool all_valid = true;
    for (int step = 0; step <= 20; step++) {
        double progress = step / 20.0;
        stepper.setStepProgress(progress);
        stepper.updateTipPosition(20.0, 0.02, false, false);

        JointAngles angles = leg.getJointAngles();
        bool valid = model.checkJointLimits(stepper.getLegIndex(), angles);

        if (!valid) {
            Point3D pos = leg.getCurrentTipPositionGlobal();
            printf("❌ Invalid joint limits at progress %.3f: position (%.3f, %.3f, %.3f)\n",
                   progress, pos.x, pos.y, pos.z);
            printf("   Joint angles: coxa=%.1f°, femur=%.1f°, tibia=%.1f°\n",
                   angles.coxa, angles.femur, angles.tibia);
            all_valid = false;
        }
    }

    if (all_valid) {
        std::cout << "✅ All positions have valid joint limits" << std::endl;
    } else {
        std::cout << "❌ Some positions have invalid joint limits" << std::endl;
    }
}

int main() {
    std::cout << "=== Debug Tip Position Test ===" << std::endl;

    // Initialize parameters
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.standing_height = 150;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);

    // Create leg object for testing (using leg 0)
    Leg test_leg(0, model);
    test_leg.initialize(Pose::Identity());
    test_leg.updateTipPosition();

    // Configure standing pose using BodyPoseController
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_config);
    pose_controller.setWalkPlanePoseEnabled(true);

    // Create array of legs for pose controller initialization
    Leg test_legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)};

    for (int i = 0; i < NUM_LEGS; ++i) {
        test_legs[i].initialize(Pose::Identity());
        test_legs[i].updateTipPosition();
    }

    pose_controller.initializeLegPosers(test_legs);
    assert(pose_controller.setStandingPose(test_legs));

    // Get leg's identity pose from BodyPoseConfiguration (OpenSHC equivalent)
    // Use leg stance position to calculate identity tip pose like in OpenSHC
    const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[0];
    Point3D identity_tip_pose = Point3D(
        leg_stance_position.x,
        leg_stance_position.y,
        leg_stance_position.z); // Use standing height

    // Create required objects for LegStepper
    WalkspaceAnalyzer walkspace_analyzer(model);
    WorkspaceValidator workspace_validator(model);

    // Create LegStepper for leg 0 using the correct identity tip pose
    LegStepper stepper(0, identity_tip_pose, test_leg, model, &walkspace_analyzer, &workspace_validator);
    stepper.setDefaultTipPose(identity_tip_pose);

    debugTipPositionGeneration(stepper, test_leg, model);

    return 0;
}
