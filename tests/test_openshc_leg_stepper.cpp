#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config.h"
#include "../src/gait_config_factory.h"
#include "../src/walk_controller.h"
#include "../src/walkspace_analyzer.h"
#include "../src/workspace_validator.h"
#include "leg_stepper_openshc.h"
#include "test_stubs.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

void testOpenSHCImplementation(LegStepperOpenSHC &stepper, Leg &leg, const RobotModel &model) {
    std::cout << "\n=== TESTING OPENSHC IMPLEMENTATION ===" << std::endl;

    // Configure stepper
    Point3D linear_velocity(20.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_velocity, 0.0);
    stepper.setSwingClearance(Point3D(0.0, 0.0, 30.0));
    stepper.setSwingOriginTipVelocity(linear_velocity);

    // Set initial state
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(5);

    Point3D initial_position = leg.getCurrentTipPositionGlobal();
    stepper.setCurrentTipPose(initial_position);

    std::cout << "Initial position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;

    // Initialize timing
    double time_delta = 0.02; // 50Hz control frequency
    stepper.calculateSwingTiming(time_delta);
    stepper.printDebugInfo();

    // Simulate complete swing cycle (OpenSHC iterative approach)
    std::cout << "\n=== OPENSHC SWING CYCLE SIMULATION ===" << std::endl;
    std::cout << "Iter | Position (x, y, z) | Delta (x, y, z) | Z Elevation | Notes" << std::endl;
    std::cout << "-----+-------------------+-----------------+-------------+------" << std::endl;

    Point3D previous_position = initial_position;
    bool max_elevation_reached = false;
    double max_z = initial_position.z;

    // Run 20 iterations (typical swing cycle)
    for (int iteration = 1; iteration <= 20; iteration++) {
        // Store position before update
        Point3D pos_before = stepper.getCurrentTipPose();

        // Update tip position using OpenSHC iterative method
        stepper.updateTipPositionIterative(iteration, time_delta, false, false);

        // Get updated position FROM THE STEPPER (not the leg)
        Point3D pos_after = stepper.getCurrentTipPose();
        Point3D delta_pos = pos_after - previous_position;
        double z_elevation = pos_after.z - initial_position.z;

        // Track maximum elevation
        if (pos_after.z > max_z) {
            max_z = pos_after.z;
        }

        // Determine phase
        std::string notes = "";
        if (iteration == 1)
            notes = "Swing start";
        else if (iteration == 10)
            notes = "Mid-swing";
        else if (iteration == 20)
            notes = "Swing end";
        else if (iteration <= 10)
            notes = "Primary curve";
        else
            notes = "Secondary curve";

        printf("%4d | (%7.3f, %7.3f, %7.3f) | (%6.3f, %6.3f, %6.3f) | %10.3f | %s\n",
               iteration, pos_after.x, pos_after.y, pos_after.z,
               delta_pos.x, delta_pos.y, delta_pos.z, z_elevation, notes.c_str());

        previous_position = pos_after;
    }

    Point3D final_position = stepper.getCurrentTipPose();

    std::cout << "\n=== OPENSHC IMPLEMENTATION RESULTS ===" << std::endl;
    std::cout << "Initial position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;
    std::cout << "Final position: (" << final_position.x << ", " << final_position.y << ", " << final_position.z << ")" << std::endl;
    std::cout << "Maximum elevation: " << (max_z - initial_position.z) << " mm" << std::endl;

    // Validate results
    double total_movement = (final_position - initial_position).norm();
    double z_difference = std::abs(final_position.z - initial_position.z);
    double elevation_achieved = max_z - initial_position.z;

    std::cout << "Total movement: " << total_movement << " mm" << std::endl;
    std::cout << "Z difference (start vs end): " << z_difference << " mm" << std::endl;
    std::cout << "Elevation achieved: " << elevation_achieved << " mm" << std::endl;

    // Validation checks
    std::cout << "\n=== VALIDATION RESULTS ===" << std::endl;

    if (total_movement > 0.5) {
        std::cout << "✅ Leg moved during swing (total movement > 0.5mm)" << std::endl;
    } else {
        std::cout << "❌ Leg did not move significantly" << std::endl;
    }

    if (z_difference < 5.0) {
        std::cout << "✅ Z positions approximately match (difference < 5mm)" << std::endl;
    } else {
        std::cout << "❌ Z positions don't match properly" << std::endl;
    }

    if (elevation_achieved > 5.0) {
        std::cout << "✅ Swing elevation achieved (> 5mm)" << std::endl;
    } else {
        std::cout << "❌ No significant swing elevation" << std::endl;
    }

    // Check control nodes
    std::cout << "\n=== CONTROL NODES ANALYSIS ===" << std::endl;
    std::cout << "Primary swing control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing1ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    std::cout << "\nSecondary swing control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing2ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    // Check if control nodes have reasonable values
    Point3D node_0 = stepper.getSwing1ControlNode(0);
    Point3D node_4 = stepper.getSwing1ControlNode(4);
    double node_separation = (node_4 - node_0).norm();

    std::cout << "\nControl nodes validation:" << std::endl;
    std::cout << "Distance between node 0 and node 4: " << node_separation << " mm" << std::endl;

    if (node_separation > 5.0 && node_separation < 100.0) {
        std::cout << "✅ Control nodes have reasonable separation" << std::endl;
    } else if (node_separation < 5.0) {
        std::cout << "❌ Control nodes too close together" << std::endl;
    } else {
        std::cout << "❌ Control nodes too far apart (workspace issue)" << std::endl;
    }
}

int main() {
    std::cout << "=== OpenSHC LegStepper Implementation Test ===" << std::endl;

    // Standard configuration
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
    Leg test_leg(0, model);
    test_leg.initialize(Pose::Identity());
    test_leg.updateTipPosition();

    // Configure standing pose
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_config);
    pose_controller.setWalkPlanePoseEnabled(true);

    Leg test_legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)};

    for (int i = 0; i < NUM_LEGS; ++i) {
        test_legs[i].initialize(Pose::Identity());
        test_legs[i].updateTipPosition();
    }

    pose_controller.initializeLegPosers(test_legs);
    assert(pose_controller.setStandingPose(test_legs));

    const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[0];
    Point3D identity_tip_pose = Point3D(
        leg_stance_position.x,
        leg_stance_position.y,
        leg_stance_position.z);

    WalkspaceAnalyzer walkspace_analyzer(model);
    WorkspaceValidator workspace_validator(model);

    // Create OpenSHC-style LegStepper
    LegStepperOpenSHC stepper(0, identity_tip_pose, test_leg, model, &walkspace_analyzer, &workspace_validator);
    stepper.setDefaultTipPose(identity_tip_pose);

    testOpenSHCImplementation(stepper, test_leg, model);

    return 0;
}
