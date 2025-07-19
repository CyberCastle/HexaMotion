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

void debugTipPositionGeneration(LegStepper &stepper, Leg &leg, const RobotModel &model, const GaitConfiguration &gait_config) {
    std::cout << "\n=== DEBUG: Tip Position Generation (Gait: " << gait_config.gait_name << ") ===" << std::endl;

    // Use the actual current position from the leg that was set up via StandingPose
    // This follows the complete BodyPoseConfiguration -> StandingPose -> leg pose flow
    Point3D initial_position = leg.getCurrentTipPositionGlobal();
    std::cout << "Initial tip position (from StandingPose setup): (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;

    // Calculate and display initial joint angles in degrees
    bool initial_ik_success = leg.applyIK(initial_position);
    JointAngles initial_angles = leg.getJointAngles();
    Point3D initial_actual_pos = leg.getCurrentTipPositionGlobal();
    double initial_ik_error = (initial_actual_pos - initial_position).norm();

    std::cout << "Initial IK success: " << (initial_ik_success ? "YES" : "NO") << std::endl;
    std::cout << "Initial IK error: " << initial_ik_error << " mm" << std::endl;
    std::cout << "Initial actual position: (" << initial_actual_pos.x << ", " << initial_actual_pos.y << ", " << initial_actual_pos.z << ")" << std::endl;
    std::cout << "Initial joint angles (deg): coxa=" << (initial_angles.coxa * 180.0 / M_PI)
              << "°, femur=" << (initial_angles.femur * 180.0 / M_PI)
              << "°, tibia=" << (initial_angles.tibia * 180.0 / M_PI) << "°" << std::endl;

    // Check if initial position is reachable
    bool is_reachable = leg.isTargetReachable(initial_position);
    std::cout << "Initial position reachable: " << (is_reachable ? "YES" : "NO") << std::endl;

    // Get leg reach information
    double max_reach = leg.getLegReach();
    Point3D base_pos = leg.getBasePosition();
    double distance_from_base = (initial_position - base_pos).norm();
    std::cout << "Leg base position: (" << base_pos.x << ", " << base_pos.y << ", " << base_pos.z << ")" << std::endl;
    std::cout << "Max leg reach: " << max_reach << " mm" << std::endl;
    std::cout << "Distance from base: " << distance_from_base << " mm" << std::endl;

    // Calculate workspace limits around current position for debug
    Point3D workspace_center = base_pos + Point3D(150, 0, -150); // Conservative reachable position
    double workspace_center_distance = (workspace_center - base_pos).norm();
    bool workspace_center_reachable = leg.isTargetReachable(workspace_center);
    std::cout << "Workspace center test: (" << workspace_center.x << ", " << workspace_center.y << ", " << workspace_center.z << ")" << std::endl;
    std::cout << "Workspace center distance: " << workspace_center_distance << " mm, reachable: " << (workspace_center_reachable ? "YES" : "NO") << std::endl;

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

    // Test with velocity using gait configuration parameters
    Point3D initial_velocity = Point3D(gait_config.max_velocity * 0.1, 0, 0); // 10% of max velocity
    stepper.setSwingOriginTipVelocity(initial_velocity);
    std::cout << "\nAfter setting swing origin velocity to (" << initial_velocity.x << ", 0, 0) from gait config:" << std::endl;

    // Set stepper to swing state with gait configuration
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(gait_config.phase_config.swing_phase);
    stepper.setStepProgress(0.5);

    // Configure iteration count using OpenSHC-compatible calculation
    // OpenSHC formula: swing_iterations = int((swing_period/period) / (frequency * time_delta))
    double time_delta = 0.02;                                 // OpenSHC standard: 50Hz control loop (0.02s per iteration)
    double period = 1.0;                                      // Complete gait cycle period in seconds
    double swing_period = period * gait_config.swing_ratio;   // Time spent in swing phase
    double stance_period = period * gait_config.stance_ratio; // Time spent in stance phase

    // Calculate iterations using OpenSHC method
    int swing_iterations = (int)((swing_period / period) / (gait_config.step_frequency * time_delta));
    int stance_iterations = (int)((stance_period / period) / (gait_config.step_frequency * time_delta));
    int total_iterations = swing_iterations + stance_iterations;

    // Use OpenSHC-compatible iteration time
    double iteration_time = time_delta;

    std::cout << "OpenSHC-compatible gait timing:" << std::endl;
    std::cout << "  time_delta=" << time_delta << "s, period=" << period << "s" << std::endl;
    std::cout << "  swing_period=" << swing_period << "s, stance_period=" << stance_period << "s" << std::endl;
    std::cout << "  swing_iterations=" << swing_iterations << ", stance_iterations=" << stance_iterations << std::endl;
    std::cout << "  total_iterations=" << total_iterations << ", iteration_time=" << iteration_time << "s" << std::endl;

    // CRITICAL: Initialize timing parameters by calling updateTipPositionIterative first
    // This will internally call calculateSwingTiming() and generate control nodes
    // BUT first we need to set the stepper to SWING state and reset to iteration 1
    stepper.setStepState(STEP_SWING);
    stepper.setCurrentTipPose(initial_position); // Reset to initial position
    stepper.updateTipPositionIterative(1, iteration_time, false, false);

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

    // Generate complete swing trajectory using gait configuration
    // IMPORTANT: Only generate swing_iterations, not total_iterations
    std::cout << "\n=== COMPLETE SWING TRAJECTORY WITH JOINT ANGLES (Gait: " << gait_config.gait_name << ") ===" << std::endl;
    std::cout << "Step | Iteration | Position (x, y, z) | Delta (x, y, z) | Joint Angles (deg) | IK Debug Info" << std::endl;
    std::cout << "-----+-----------+-------------------+----------------+-------------------+--------------" << std::endl;

    // Reset to initial position for trajectory generation
    stepper.setCurrentTipPose(initial_position);
    Point3D previous_pos = initial_position;

    // Calculate initial joint angles for reference in trajectory
    leg.applyIK(initial_position);
    JointAngles trajectory_initial_angles = leg.getJointAngles();

    // GENERATE ONLY SWING TRAJECTORY (not stance)
    for (int iteration = 1; iteration <= swing_iterations; iteration++) {
        // Get current joint angles BEFORE update to calculate delta
        JointAngles angles_before = leg.getJointAngles();
        Point3D pos_before = leg.getCurrentTipPositionGlobal();

        // Update tip position using iteration-based approach (OpenSHC style)
        // This internally applies IK to the calculated Bezier position
        stepper.updateTipPositionIterative(iteration, iteration_time, false, false);

        // Get the new position calculated by the stepper (Bezier result)
        Point3D pos_bezier = stepper.getCurrentTipPose();
        Point3D delta = pos_bezier - previous_pos;

        // CRITICAL DEBUG: Check if Bezier position is reachable BEFORE applying IK
        bool bezier_reachable = leg.isTargetReachable(pos_bezier);
        Point3D leg_base = leg.getBasePosition();
        double distance_from_base = (pos_bezier - leg_base).norm();
        double max_reach = leg.getLegReach();

        // Try to apply IK manually to the Bezier position using current angles as starting point
        JointAngles current_angles = leg.getJointAngles();
        bool manual_ik_success = leg.applyIK(pos_bezier);

        // Get joint angles AFTER IK attempt
        JointAngles angles_after = leg.getJointAngles();

        // Get actual tip position from forward kinematics of the resulting joint angles
        Point3D pos_actual = leg.getCurrentTipPositionGlobal();

        // Calculate position delta in local coordinates (OpenSHC approach) for info
        Point3D position_delta = model.calculatePositionDeltaLocalCoordinates(
            0, pos_bezier, pos_before, angles_before);

        // Convert joint angles from radians to degrees
        double coxa_deg = angles_after.coxa * 180.0 / M_PI;
        double femur_deg = angles_after.femur * 180.0 / M_PI;
        double tibia_deg = angles_after.tibia * 180.0 / M_PI;

        // Check if joint limits are being violated
        bool valid_joints = model.checkJointLimits(0, angles_after);
        std::string joint_status = valid_joints ? "✓" : "❌";

        // Calculate IK error (difference between desired Bezier position and actual achieved position)
        Point3D ik_error = pos_actual - pos_bezier;
        double ik_error_magnitude = ik_error.norm();

        // Show delta magnitude for debugging
        double delta_magnitude = position_delta.norm();

        std::string curve_used = (iteration <= swing_iterations / 2) ? "Primary" : "Secondary";
        std::string reach_status = bezier_reachable ? "R" : "X";
        std::string ik_status = manual_ik_success ? "IK✓" : "IK❌";

        printf("%4d | %9d | (%8.3f, %8.3f, %8.3f) | (%6.3f, %6.3f, %6.3f) | (%6.1f, %6.1f, %6.1f) | %s Err:%4.1f D:%3.0f %s %s %s\n",
               iteration, iteration, pos_bezier.x, pos_bezier.y, pos_bezier.z, delta.x, delta.y, delta.z,
               coxa_deg, femur_deg, tibia_deg, ik_status.c_str(), ik_error_magnitude, distance_from_base,
               reach_status.c_str(), curve_used.c_str(), joint_status.c_str());

        previous_pos = pos_bezier;
    }

    // Reset to original state and test final position (complete swing)
    stepper.setCurrentTipPose(initial_position);

    // Execute complete swing trajectory using calculated iterations
    for (int iter = 1; iter <= swing_iterations; iter++) {
        stepper.updateTipPositionIterative(iter, iteration_time, false, false);
    }

    Point3D final_position = stepper.getCurrentTipPose();
    std::cout << "\nFinal tip position (iteration=" << swing_iterations << ", complete swing): (" << final_position.x << ", " << final_position.y << ", " << final_position.z << ")" << std::endl;

    double position_change = (final_position - initial_position).norm();
    std::cout << "Position change magnitude: " << position_change << " mm" << std::endl;

    // Verify joint limits for extreme positions
    std::cout << "\n=== JOINT LIMITS VERIFICATION ===" << std::endl;
    bool all_valid = true;
    stepper.setCurrentTipPose(initial_position); // Reset position

    for (int iteration = 1; iteration <= total_iterations; iteration++) {
        stepper.updateTipPositionIterative(iteration, iteration_time, false, false);
        Point3D pos = stepper.getCurrentTipPose();

        // Apply IK to get joint angles
        leg.applyIK(pos);
        JointAngles angles = leg.getJointAngles();
        bool valid = model.checkJointLimits(stepper.getLegIndex(), angles);

        if (!valid) {
            printf("❌ Invalid joint limits at iteration %d: position (%.3f, %.3f, %.3f)\n",
                   iteration, pos.x, pos.y, pos.z);
            printf("   Joint angles: coxa=%.1f°, femur=%.1f°, tibia=%.1f°\n",
                   angles.coxa * 180.0 / M_PI, angles.femur * 180.0 / M_PI, angles.tibia * 180.0 / M_PI);
            all_valid = false;
        }
    }
}

int main() {
    std::cout << "=== Debug Tip Position Test (Tripod Gait Configuration) ===" << std::endl;

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

    // Configure gait factors for tripod gait (add these to parameters)
    p.gait_factors.tripod_length_factor = 0.4;  // 40% of leg reach for step length
    p.gait_factors.tripod_height_factor = 0.15; // 15% of standing height for swing

    // Create tripod gait configuration
    GaitConfiguration tripod_config = createTripodGaitConfig(p);
    std::cout << "Tripod Gait Configuration Loaded:" << std::endl;
    std::cout << "  Step length: " << tripod_config.step_length << " mm" << std::endl;
    std::cout << "  Swing height: " << tripod_config.swing_height << " mm" << std::endl;
    std::cout << "  Step frequency: " << tripod_config.step_frequency << " Hz" << std::endl;
    std::cout << "  Stance ratio: " << tripod_config.stance_ratio << std::endl;
    std::cout << "  Swing ratio: " << tripod_config.swing_ratio << std::endl;

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

    // Debug: Show the standing pose configuration
    std::cout << "\n=== STANDING POSE CONFIGURATION ===" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D pos = test_legs[i].getCurrentTipPositionGlobal();
        JointAngles angles = test_legs[i].getJointAngles();
        std::cout << "Leg " << i << " standing position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
        std::cout << "  Joint angles (deg): coxa=" << (angles.coxa * 180.0 / M_PI)
                  << "°, femur=" << (angles.femur * 180.0 / M_PI)
                  << "°, tibia=" << (angles.tibia * 180.0 / M_PI) << "°" << std::endl;
    }

    // Get leg's identity pose from BodyPoseConfiguration (OpenSHC equivalent)
    // Use leg stance position to calculate identity tip pose like in OpenSHC
    const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[0];
    Point3D identity_tip_pose = Point3D(
        leg_stance_position.x,
        leg_stance_position.y,
        leg_stance_position.z); // Use standing height

    std::cout << "\nLeg stance position from config: (" << leg_stance_position.x << ", " << leg_stance_position.y << ", " << leg_stance_position.z << ")" << std::endl;
    std::cout << "Identity tip pose: (" << identity_tip_pose.x << ", " << identity_tip_pose.y << ", " << identity_tip_pose.z << ")" << std::endl;

    // Create required objects for LegStepper
    WalkspaceAnalyzer walkspace_analyzer(model);
    WorkspaceValidator workspace_validator(model);

    // Create LegStepper for leg 0 using the correct identity tip pose
    LegStepper stepper(0, identity_tip_pose, test_legs[0], model, &walkspace_analyzer, &workspace_validator);
    stepper.setDefaultTipPose(identity_tip_pose);

    // *** CONFIGURAR USANDO PARÁMETROS DEL TRIPOD GAIT ***
    // Configure swing clearance using tripod gait swing height
    Point3D swing_clearance(0, 0, tripod_config.swing_height);
    stepper.setSwingClearance(swing_clearance);
    std::cout << "Swing clearance configurado desde tripod gait: (0, 0, " << tripod_config.swing_height << ")" << std::endl;

    // Configure velocity to generate proper stride using tripod gait step length
    // Calculate velocity from step_length and step_frequency
    double desired_velocity = tripod_config.step_length * tripod_config.step_frequency * 0.1; // 10% of max
    stepper.setDesiredVelocity(Point3D(desired_velocity, 0, 0), 0.0);
    stepper.updateStride(); // This will calculate stride_vector_ automatically
    Point3D calculated_stride = stepper.getStrideVector();
    std::cout << "Velocidad configurada: " << desired_velocity << " mm/s" << std::endl;
    std::cout << "Stride vector calculado: (" << calculated_stride.x << ", " << calculated_stride.y << ", " << calculated_stride.z << ")" << std::endl;

    // Configure timing parameters from tripod gait
    // Use stance and swing ratios for phase calculations
    int total_iterations = 20; // Test iterations
    int swing_iterations = (int)(total_iterations * tripod_config.swing_ratio);
    int stance_iterations = total_iterations - swing_iterations;
    std::cout << "Timing configurado - Total: " << total_iterations << ", Swing: " << swing_iterations << ", Stance: " << stance_iterations << std::endl;

    // Show gait configuration being used
    std::cout << "\nUsando configuración " << tripod_config.gait_name << ":" << std::endl;
    std::cout << "  Stance phase: " << tripod_config.phase_config.stance_phase << std::endl;
    std::cout << "  Swing phase: " << tripod_config.phase_config.swing_phase << std::endl;
    std::cout << "  Phase offset: " << tripod_config.phase_config.phase_offset << std::endl;

    debugTipPositionGeneration(stepper, test_legs[0], model, tripod_config);

    return 0;
}
