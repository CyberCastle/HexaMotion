#include "../src/body_pose_config_factory.h"
#include "../src/body_pose_controller.h"
#include "../src/gait_config.h"
#include "../src/gait_config_factory.h"
#include "../src/leg_stepper.h"
#include "../src/walk_controller.h"
#include "../src/workspace_analyzer.h"
#include "test_stubs.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

// Helper function to check if a position is reachable using OpenSHC-style WorkspaceAnalyzer
bool isPositionReachable(const RobotModel &model, int leg_id, const Point3D &position) {
    WorkspaceAnalyzer temp_analyzer(const_cast<RobotModel &>(model));
    return temp_analyzer.isReachable(leg_id, position);
}

// Structure to hold analysis results for a single leg
struct LegAnalysisResult {
    int leg_id;
    bool traditional_ik_success;
    bool delta_ik_success;
    double traditional_ik_error;
    double delta_ik_error;
    Point3D initial_position;
    Point3D final_swing_position;
    Point3D final_stance_position;
    Point3D target_position;
    double swing_precision;
    double stance_precision;
    double target_error;
    bool workspace_reachable;
    bool all_joint_limits_valid;
    double reach_utilization;
    Point3D calculated_stride;

    // Stance movement analysis
    double total_xy_displacement;
    double coxa_movement;
    double femur_movement;
    double tibia_movement;
    bool correct_stance_pattern;
};

// Analyze a single leg trajectory
LegAnalysisResult analyzeLegTrajectory(int leg_id, LegStepper &stepper, Leg &leg, const RobotModel &model,
                                       const GaitConfiguration &gait_config, bool verbose = false) {
    LegAnalysisResult result;
    result.leg_id = leg_id;
    result.traditional_ik_success = false;
    result.delta_ik_success = false;
    result.traditional_ik_error = 0.0;
    result.delta_ik_error = 0.0;
    result.initial_position = Point3D(0, 0, 0);
    result.final_swing_position = Point3D(0, 0, 0);
    result.final_stance_position = Point3D(0, 0, 0);
    result.target_position = Point3D(0, 0, 0);
    result.swing_precision = 0.0;
    result.stance_precision = 0.0;
    result.target_error = 0.0;
    result.workspace_reachable = false;
    result.all_joint_limits_valid = false;
    result.reach_utilization = 0.0;
    result.calculated_stride = Point3D(0, 0, 0);
    result.total_xy_displacement = 0.0;
    result.coxa_movement = 0.0;
    result.femur_movement = 0.0;
    result.tibia_movement = 0.0;
    result.correct_stance_pattern = false;

    if (verbose) {
        std::cout << "\n=== ANALYZING LEG " << leg_id << " (Gait: " << gait_config.gait_name << ") ===" << std::endl;
    }

    // Use the actual current position from the leg that was set up via StandingPose
    Point3D initial_position = leg.getCurrentTipPositionGlobal();
    result.initial_position = initial_position;

    if (verbose) {
        std::cout << "Initial tip position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;
    }

    // Test both IK methods with initial position
    bool traditional_ik_success = leg.applyIK(initial_position);
    JointAngles traditional_angles = leg.getJointAngles();
    Point3D traditional_actual_pos = leg.getCurrentTipPositionGlobal();
    double traditional_ik_error = (traditional_actual_pos - initial_position).norm();

    // Reset leg and test advanced delta-based IK
    leg.setJointAngles(traditional_angles);
    JointAngles current_angles = leg.getJointAngles();
    Point3D current_pos = leg.getCurrentTipPositionGlobal();
    JointAngles new_angles = model.applyAdvancedIK(leg.getLegId(), current_pos, initial_position, current_angles, 0.02);
    leg.setJointAngles(new_angles);
    Point3D delta_actual_pos = leg.getCurrentTipPositionGlobal();
    double delta_ik_error = (delta_actual_pos - initial_position).norm();
    bool delta_ik_success = (delta_ik_error < 1.0);

    result.traditional_ik_success = traditional_ik_success;
    result.delta_ik_success = delta_ik_success;
    result.traditional_ik_error = traditional_ik_error;
    result.delta_ik_error = delta_ik_error;

    // Check workspace reachability
    result.workspace_reachable = isPositionReachable(model, leg.getLegId(), initial_position);

    // Calculate reach utilization
    Point3D base_pos = leg.getBasePosition();
    double distance_from_base = (initial_position - base_pos).norm();
    double max_reach = model.getLegReach();
    result.reach_utilization = (distance_from_base / max_reach) * 100.0;

    // Get stride information
    result.calculated_stride = stepper.getStrideVector();

    // Calculate timing parameters
    StepCycle step_cycle = stepper.getStepCycle();
    double time_delta = 0.02;
    double period = step_cycle.period_;
    double swing_period = step_cycle.swing_period_;
    double stance_period = step_cycle.stance_period_;
    double frequency = step_cycle.frequency_;

    int swing_iterations = (int)((double(swing_period) / period) / (frequency * time_delta));
    int stance_iterations = (int)((double(stance_period) / period) / (frequency * time_delta));
    int total_iterations = swing_iterations + stance_iterations;

    if (verbose) {
        std::cout << "Timing: swing=" << swing_iterations << ", stance=" << stance_iterations << ", total=" << total_iterations << std::endl;
    }

    // Test swing phase trajectory
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(gait_config.phase_config.swing_phase);
    stepper.setStepProgress(0.5);
    stepper.setCurrentTipPose(initial_position);

    // Initialize timing and generate control nodes
    stepper.updateTipPositionIterative(1, time_delta, false, false);

    // Execute complete swing trajectory
    stepper.setCurrentTipPose(initial_position);
    for (int iter = 1; iter <= swing_iterations; iter++) {
        stepper.updateTipPositionIterative(iter, time_delta, false, false);
    }

    Point3D final_swing_position = stepper.getCurrentTipPose();
    Point3D target_position = stepper.getTargetTipPose();
    Point3D expected_stride = stepper.getStrideVector();

    result.final_swing_position = final_swing_position;
    result.target_position = target_position;
    result.target_error = (final_swing_position - target_position).norm();

    double position_change = (final_swing_position - initial_position).norm();
    double expected_swing_displacement = expected_stride.norm() * 0.5;
    result.swing_precision = (position_change / expected_swing_displacement) * 100.0;

    // Test stance phase
    stepper.setCurrentTipPose(initial_position);
    stepper.setStepState(STEP_STANCE);
    stepper.setPhase(gait_config.phase_config.stance_phase);
    stepper.setStepProgress(0.0);

    // Store initial stance joint angles
    JointAngles temp_angles = leg.getJointAngles();
    Point3D temp_pos = leg.getCurrentTipPositionGlobal();
    JointAngles updated_angles = model.applyAdvancedIK(leg.getLegId(), temp_pos, initial_position, temp_angles, 0.02);
    leg.setJointAngles(updated_angles);
    JointAngles initial_stance_angles = leg.getJointAngles();

    // Execute stance trajectory
    for (int iter = swing_iterations + 1; iter <= total_iterations; iter++) {
        stepper.updateTipPositionIterative(iter, time_delta, false, false);
    }

    Point3D final_stance_position = stepper.getCurrentTipPose();
    result.final_stance_position = final_stance_position;

    // Apply IK for final stance analysis
    JointAngles temp_angles2 = leg.getJointAngles();
    Point3D temp_pos2 = leg.getCurrentTipPositionGlobal();
    JointAngles final_updated_angles = model.applyAdvancedIK(leg.getLegId(), temp_pos2, final_stance_position, temp_angles2, 0.02);
    leg.setJointAngles(final_updated_angles);
    JointAngles final_stance_angles = leg.getJointAngles();

    // Analyze stance movement
    Point3D total_xy_movement = final_stance_position - initial_position;
    total_xy_movement.z = 0;
    result.total_xy_displacement = total_xy_movement.norm();

    result.coxa_movement = std::abs((final_stance_angles.coxa - initial_stance_angles.coxa) * 180.0 / M_PI);
    result.femur_movement = std::abs((final_stance_angles.femur - initial_stance_angles.femur) * 180.0 / M_PI);
    result.tibia_movement = std::abs((final_stance_angles.tibia - initial_stance_angles.tibia) * 180.0 / M_PI);

    result.correct_stance_pattern = (result.coxa_movement > result.femur_movement) &&
                                    (result.coxa_movement > result.tibia_movement);

    double stance_position_change = (final_stance_position - initial_position).norm();
    double expected_stance_displacement = expected_stride.norm() * 0.5;
    result.stance_precision = (stance_position_change / expected_stance_displacement) * 100.0;

    // Verify joint limits throughout trajectory
    result.all_joint_limits_valid = true;
    stepper.setCurrentTipPose(initial_position);

    for (int iteration = 1; iteration <= total_iterations; iteration++) {
        stepper.updateTipPositionIterative(iteration, time_delta, false, false);
        Point3D pos = stepper.getCurrentTipPose();

        JointAngles current_angles = leg.getJointAngles();
        Point3D current_pos = leg.getCurrentTipPositionGlobal();
        JointAngles new_angles = model.applyAdvancedIK(leg.getLegId(), current_pos, pos, current_angles, 0.02);
        leg.setJointAngles(new_angles);
        JointAngles angles = leg.getJointAngles();

        bool valid = model.checkJointLimits(stepper.getLegIndex(), angles);
        if (!valid) {
            result.all_joint_limits_valid = false;
            if (verbose) {
                printf("❌ Invalid joint limits at iteration %d: position (%.3f, %.3f, %.3f)\n",
                       iteration, pos.x, pos.y, pos.z);
            }
        }
    }

    if (verbose) {
        std::cout << "Results - Swing precision: " << result.swing_precision << "%, Stance precision: " << result.stance_precision
                  << "%, Target error: " << result.target_error << "mm" << std::endl;
    }

    return result;
}

// Print detailed results for all legs
void printHexapodAnalysisSummary(const std::vector<LegAnalysisResult> &results, const GaitConfiguration &gait_config) {
    std::cout << "\n"
              << std::string(100, '=') << std::endl;
    std::cout << "HEXAPOD TRAJECTORY ANALYSIS SUMMARY (Gait: " << gait_config.gait_name << ")" << std::endl;
    std::cout << std::string(100, '=') << std::endl;

    // Header
    std::cout << "\nLeg | IK Success | IK Error | Target Err | Swing Prec | Stance Prec | Reach% | Workspace | Joint Limits" << std::endl;
    std::cout << "----+------------+----------+------------+------------+-------------+--------+-----------+-------------" << std::endl;

    // Individual leg results
    for (const auto &result : results) {
        std::string ik_status = (result.traditional_ik_success && result.delta_ik_success) ? "T+D✓" : result.traditional_ik_success ? "T✓D❌"
                                                                                                  : result.delta_ik_success         ? "T❌D✓"
                                                                                                                                    : "T❌D❌";

        std::string workspace_status = result.workspace_reachable ? "✓" : "❌";
        std::string joint_status = result.all_joint_limits_valid ? "✓" : "❌";

        printf(" %2d | %-10s | %8.3f | %10.3f | %10.1f | %11.1f | %6.1f | %9s | %11s\n",
               result.leg_id, ik_status.c_str(), result.delta_ik_error, result.target_error,
               result.swing_precision, result.stance_precision, result.reach_utilization,
               workspace_status.c_str(), joint_status.c_str());
    }

    // Statistical analysis
    std::cout << "\n"
              << std::string(60, '-') << std::endl;
    std::cout << "STATISTICAL ANALYSIS" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    double avg_target_error = 0, max_target_error = 0, min_target_error = 1000;
    double avg_swing_precision = 0, avg_stance_precision = 0;
    double avg_reach_utilization = 0;
    int successful_ik_count = 0, workspace_reachable_count = 0, valid_joints_count = 0;
    int correct_stance_pattern_count = 0;

    for (const auto &result : results) {
        avg_target_error += result.target_error;
        max_target_error = std::max(max_target_error, result.target_error);
        min_target_error = std::min(min_target_error, result.target_error);
        avg_swing_precision += result.swing_precision;
        avg_stance_precision += result.stance_precision;
        avg_reach_utilization += result.reach_utilization;

        if (result.traditional_ik_success && result.delta_ik_success)
            successful_ik_count++;
        if (result.workspace_reachable)
            workspace_reachable_count++;
        if (result.all_joint_limits_valid)
            valid_joints_count++;
        if (result.correct_stance_pattern)
            correct_stance_pattern_count++;
    }

    int num_legs = results.size();
    avg_target_error /= num_legs;
    avg_swing_precision /= num_legs;
    avg_stance_precision /= num_legs;
    avg_reach_utilization /= num_legs;

    std::cout << "Target Error Statistics:" << std::endl;
    std::cout << "  Average: " << avg_target_error << " mm" << std::endl;
    std::cout << "  Min: " << min_target_error << " mm" << std::endl;
    std::cout << "  Max: " << max_target_error << " mm" << std::endl;

    std::cout << "\nPrecision Statistics:" << std::endl;
    std::cout << "  Average Swing Precision: " << avg_swing_precision << "%" << std::endl;
    std::cout << "  Average Stance Precision: " << avg_stance_precision << "%" << std::endl;
    std::cout << "  Average Reach Utilization: " << avg_reach_utilization << "%" << std::endl;

    std::cout << "\nSystem Validation:" << std::endl;
    std::cout << "  IK Success Rate: " << successful_ik_count << "/" << num_legs << " (" << (successful_ik_count * 100.0 / num_legs) << "%)" << std::endl;
    std::cout << "  Workspace Reachable: " << workspace_reachable_count << "/" << num_legs << " (" << (workspace_reachable_count * 100.0 / num_legs) << "%)" << std::endl;
    std::cout << "  Joint Limits Valid: " << valid_joints_count << "/" << num_legs << " (" << (valid_joints_count * 100.0 / num_legs) << "%)" << std::endl;
    std::cout << "  Correct Stance Pattern: " << correct_stance_pattern_count << "/" << num_legs << " (" << (correct_stance_pattern_count * 100.0 / num_legs) << "%)" << std::endl;

    // Gait-specific analysis
    std::cout << "\n"
              << std::string(60, '-') << std::endl;
    std::cout << "GAIT-SPECIFIC ANALYSIS" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    std::cout << "Gait Configuration: " << gait_config.gait_name << std::endl;
    std::cout << "  Step Length: " << gait_config.step_length << " mm" << std::endl;
    std::cout << "  Swing Height: " << gait_config.swing_height << " mm" << std::endl;
    std::cout << "  Step Frequency: " << gait_config.getStepFrequency() << " Hz" << std::endl;
    std::cout << "  Stance Ratio: " << gait_config.getStanceRatio() << std::endl;
    std::cout << "  Swing Ratio: " << gait_config.getSwingRatio() << std::endl;

    // Movement pattern analysis
    std::cout << "\nMovement Pattern Analysis:" << std::endl;
    for (const auto &result : results) {
        std::cout << "  Leg " << result.leg_id << " stance movement: ";
        std::cout << "Coxa=" << result.coxa_movement << "°, ";
        std::cout << "Femur=" << result.femur_movement << "°, ";
        std::cout << "Tibia=" << result.tibia_movement << "°";
        std::cout << " [" << (result.correct_stance_pattern ? "✓ Correct" : "❌ Incorrect") << "]" << std::endl;
    }

    // Overall assessment
    std::cout << "\n"
              << std::string(60, '-') << std::endl;
    std::cout << "OVERALL ASSESSMENT" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    bool overall_success = (successful_ik_count == num_legs) &&
                           (workspace_reachable_count == num_legs) &&
                           (valid_joints_count == num_legs) &&
                           (avg_target_error < 2.0);

    if (overall_success) {
        std::cout << "✅ HEXAPOD TRAJECTORY ANALYSIS: PASSED" << std::endl;
        std::cout << "All legs demonstrate successful trajectory generation with acceptable precision." << std::endl;
    } else {
        std::cout << "❌ HEXAPOD TRAJECTORY ANALYSIS: ISSUES DETECTED" << std::endl;
        if (successful_ik_count < num_legs) {
            std::cout << "- IK failures detected in " << (num_legs - successful_ik_count) << " legs" << std::endl;
        }
        if (workspace_reachable_count < num_legs) {
            std::cout << "- Workspace violations in " << (num_legs - workspace_reachable_count) << " legs" << std::endl;
        }
        if (valid_joints_count < num_legs) {
            std::cout << "- Joint limit violations in " << (num_legs - valid_joints_count) << " legs" << std::endl;
        }
        if (avg_target_error >= 2.0) {
            std::cout << "- Average target error (" << avg_target_error << "mm) exceeds acceptable threshold (2.0mm)" << std::endl;
        }
    }

    // Recommendations
    if (avg_reach_utilization > 90.0) {
        std::cout << "\n⚠ RECOMMENDATION: High reach utilization (" << avg_reach_utilization << "%) may cause precision issues." << std::endl;
        std::cout << "Consider reducing velocity or step length for better stability." << std::endl;
    }

    if (correct_stance_pattern_count < num_legs) {
        std::cout << "\n⚠ RECOMMENDATION: " << (num_legs - correct_stance_pattern_count) << " legs show incorrect stance patterns." << std::endl;
        std::cout << "Verify that coxa joints are primarily responsible for XY displacement during stance." << std::endl;
    }

    if (avg_target_error > 1.0) {
        std::cout << "\n💡 SUGGESTION: Target error (" << avg_target_error << "mm) could benefit from OpenSHC precision correction." << std::endl;
        std::cout << "Consider implementing forceNormalTouchdown for improved trajectory accuracy." << std::endl;
    }
}

// Print detailed trajectory for a specific leg
void printDetailedLegTrajectory(int leg_id, LegStepper &stepper, Leg &leg, const RobotModel &model,
                                const GaitConfiguration &gait_config) {
    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "DETAILED TRAJECTORY ANALYSIS - LEG " << leg_id << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    Point3D initial_position = leg.getCurrentTipPositionGlobal();

    // Calculate timing parameters
    StepCycle step_cycle = stepper.getStepCycle();
    double time_delta = 0.02;
    double period = step_cycle.period_;
    double swing_period = step_cycle.swing_period_;
    double stance_period = step_cycle.stance_period_;
    double frequency = step_cycle.frequency_;

    int swing_iterations = (int)((double(swing_period) / period) / (frequency * time_delta));
    int stance_iterations = (int)((double(stance_period) / period) / (frequency * time_delta));

    // Configure for swing phase
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(gait_config.phase_config.swing_phase);
    stepper.setCurrentTipPose(initial_position);
    stepper.updateTipPositionIterative(1, time_delta, false, false);

    std::cout << "SWING TRAJECTORY:" << std::endl;
    std::cout << "Step | Position (x, y, z) | Coxa (deg) | Femur (deg) | Tibia (deg) | Radio | Delta R" << std::endl;
    std::cout << "-----+--------------------+------------+-------------+-------------+-------+--------" << std::endl;

    stepper.setCurrentTipPose(initial_position);
    double initial_radio = std::sqrt(initial_position.x * initial_position.x + initial_position.y * initial_position.y);

    for (int iteration = 1; iteration <= swing_iterations; iteration++) {
        JointAngles angles_before = leg.getJointAngles();
        Point3D pos_before = leg.getCurrentTipPositionGlobal();

        stepper.updateTipPositionIterative(iteration, time_delta, false, false);
        Point3D pos_bezier = stepper.getCurrentTipPose();

        leg.setJointAngles(angles_before);
        JointAngles new_angles = model.applyAdvancedIK(leg.getLegId(), pos_before, pos_bezier, angles_before, time_delta);
        leg.setJointAngles(new_angles);
        JointAngles angles_after = leg.getJointAngles();

        double coxa_deg = angles_after.coxa * 180.0 / M_PI;
        double femur_deg = angles_after.femur * 180.0 / M_PI;
        double tibia_deg = angles_after.tibia * 180.0 / M_PI;
        double radio = std::sqrt(pos_bezier.x * pos_bezier.x + pos_bezier.y * pos_bezier.y);
        double delta_radio = radio - initial_radio;

        printf("%4d | (%8.3f, %8.3f, %8.3f) | %6.1f | %6.1f | %6.1f | %5.1f | %6.2f\n",
               iteration, pos_bezier.x, pos_bezier.y, pos_bezier.z,
               coxa_deg, femur_deg, tibia_deg, radio, delta_radio);
    }

    // Configure for stance phase
    stepper.setCurrentTipPose(initial_position);
    stepper.setStepState(STEP_STANCE);
    stepper.setPhase(gait_config.phase_config.stance_phase);

    std::cout << "\nSTANCE TRAJECTORY:" << std::endl;
    std::cout << "Step | Position (x, y, z) | Coxa (deg) | Femur (deg) | Tibia (deg) | Radio | Delta R" << std::endl;
    std::cout << "-----+--------------------+------------+-------------+-------------+-------+--------" << std::endl;

    for (int iteration = swing_iterations + 1; iteration <= swing_iterations + stance_iterations; iteration++) {
        JointAngles angles_before = leg.getJointAngles();
        Point3D pos_before = leg.getCurrentTipPositionGlobal();

        stepper.updateTipPositionIterative(iteration, time_delta, false, false);
        Point3D pos_stance = stepper.getCurrentTipPose();

        leg.setJointAngles(angles_before);
        JointAngles new_angles = model.applyAdvancedIK(leg.getLegId(), pos_before, pos_stance, angles_before, time_delta);
        leg.setJointAngles(new_angles);
        JointAngles angles_after = leg.getJointAngles();

        double coxa_deg = angles_after.coxa * 180.0 / M_PI;
        double femur_deg = angles_after.femur * 180.0 / M_PI;
        double tibia_deg = angles_after.tibia * 180.0 / M_PI;
        double radio = std::sqrt(pos_stance.x * pos_stance.x + pos_stance.y * pos_stance.y);
        double delta_radio = radio - initial_radio;

        printf("%4d | (%8.3f, %8.3f, %8.3f) | %6.1f | %6.1f | %6.1f | %5.1f | %6.2f\n",
               iteration - swing_iterations, pos_stance.x, pos_stance.y, pos_stance.z,
               coxa_deg, femur_deg, tibia_deg, radio, delta_radio);
    }
}

// New function to analyze geometric stance issues
void analyzeStanceGeometry(const std::vector<LegAnalysisResult> &results, const Point3D &velocity) {
    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "STANCE GEOMETRY ANALYSIS" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    std::cout << "Movement velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ") mm/s" << std::endl;
    std::cout << "Movement direction angle: " << (std::atan2(velocity.y, velocity.x) * 180.0 / M_PI) << "°" << std::endl;

    std::cout << "\nLeg | Position (x, y) | Leg Angle | Rel to Movement | Coxa Move | Tibia Move | Pattern" << std::endl;
    std::cout << "----+-----------------+-----------+-----------------+-----------+------------+--------" << std::endl;

    for (const auto &result : results) {
        // Calculate leg angle from origin
        double leg_angle = std::atan2(result.initial_position.y, result.initial_position.x) * 180.0 / M_PI;

        // Calculate relative angle to movement direction
        double movement_angle = std::atan2(velocity.y, velocity.x) * 180.0 / M_PI;
        double relative_angle = leg_angle - movement_angle;

        // Normalize to [-180, 180]
        while (relative_angle > 180)
            relative_angle -= 360;
        while (relative_angle < -180)
            relative_angle += 360;

        std::string pattern_type = result.correct_stance_pattern ? "CORRECT" : "PROBLEM";

        printf(" %2d | (%7.1f, %7.1f) | %9.1f | %15.1f | %9.2f | %10.2f | %s\n",
               result.leg_id,
               result.initial_position.x, result.initial_position.y,
               leg_angle, relative_angle,
               result.coxa_movement, result.tibia_movement,
               pattern_type.c_str());
    }

    std::cout << "\nGEOMETRIC ANALYSIS:" << std::endl;
    std::cout << "- Legs with relative angles close to ±90° may have geometric constraints" << std::endl;
    std::cout << "- Tibia-dominant movement suggests workspace boundary issues" << std::endl;
    std::cout << "- Problem legs may need velocity vector adjustment or IK tuning" << std::endl;

    // Suggest optimal movement directions
    std::cout << "\nOPTIMAL MOVEMENT SUGGESTIONS:" << std::endl;
    for (const auto &result : results) {
        if (!result.correct_stance_pattern) {
            double leg_angle = std::atan2(result.initial_position.y, result.initial_position.x) * 180.0 / M_PI;
            double optimal_angle = leg_angle + 45.0; // 45° offset for better coxa utilization

            double optimal_vel_x = 50.0 * std::cos(optimal_angle * M_PI / 180.0);
            double optimal_vel_y = 50.0 * std::sin(optimal_angle * M_PI / 180.0);

            printf("  Leg %d: Try velocity (%.1f, %.1f) for better coxa dominance\n",
                   result.leg_id, optimal_vel_x, optimal_vel_y);
        }
    }
}

int main() {
    std::cout << "=== HEXAPOD TRAJECTORY ANALYSIS TEST (All 6 Legs) ===" << std::endl;
    std::cout << "Analyzing trajectory generation for complete hexapod configuration\n"
              << std::endl;

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

    // Create tripod gait configuration
    GaitConfiguration tripod_config = createTripodGaitConfig(p);
    std::cout << "Tripod Gait Configuration:" << std::endl;
    std::cout << "  Step length: " << tripod_config.step_length << " mm" << std::endl;
    std::cout << "  Swing height: " << tripod_config.swing_height << " mm" << std::endl;
    std::cout << "  Step frequency: " << tripod_config.getStepFrequency() << " Hz" << std::endl;
    std::cout << "  Stance ratio: " << tripod_config.getStanceRatio() << std::endl;
    std::cout << "  Swing ratio: " << tripod_config.getSwingRatio() << std::endl;

    RobotModel model(p);

    // Create array of legs for all 6 legs
    Leg hexapod_legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)};

    // Initialize all legs
    for (int i = 0; i < NUM_LEGS; ++i) {
        hexapod_legs[i].initialize(Pose::Identity());
        hexapod_legs[i].updateTipPosition();
    }

    // Configure standing pose using BodyPoseController
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_config);
    pose_controller.setWalkPlanePoseEnabled(true);
    pose_controller.initializeLegPosers(hexapod_legs);

    bool standing_pose_success = pose_controller.setStandingPose(hexapod_legs);
    if (!standing_pose_success) {
        std::cerr << "❌ FAILED to set standing pose for hexapod legs" << std::endl;
        return 1;
    }

    std::cout << "\n✅ Standing pose configured for all 6 legs" << std::endl;

    // Display standing pose configuration
    std::cout << "\nSTANDING POSE CONFIGURATION:" << std::endl;
    std::cout << "Leg | Position (x, y, z) | Coxa (deg) | Femur (deg) | Tibia (deg)" << std::endl;
    std::cout << "----+--------------------+------------+-------------+-------------" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D pos = hexapod_legs[i].getCurrentTipPositionGlobal();
        JointAngles angles = hexapod_legs[i].getJointAngles();
        printf(" %2d | (%8.3f, %8.3f, %8.3f) | %6.1f | %6.1f | %6.1f\n",
               i, pos.x, pos.y, pos.z,
               angles.coxa * 180.0 / M_PI, angles.femur * 180.0 / M_PI, angles.tibia * 180.0 / M_PI);
    }

    // Create required objects for LegSteppers
    WorkspaceAnalyzer workspace_analyzer(const_cast<RobotModel &>(model));

    // Create LegSteppers for all 6 legs
    std::vector<std::unique_ptr<LegStepper>> steppers;

    for (int i = 0; i < NUM_LEGS; i++) {
        const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[i];
        Point3D identity_tip_pose = Point3D(leg_stance_position.x, leg_stance_position.y, leg_stance_position.z);

        auto stepper = std::make_unique<LegStepper>(i, identity_tip_pose, hexapod_legs[i],
                                                    const_cast<RobotModel &>(model), &workspace_analyzer);
        stepper->setDefaultTipPose(identity_tip_pose);

        // Configure StepCycle from tripod gait configuration
        double openshc_default_frequency = 1.0; // OpenSHC uses 1.0 Hz as default
        StepCycle step_cycle = tripod_config.generateStepCycle(openshc_default_frequency);
        stepper->setStepCycle(step_cycle);

        steppers.push_back(std::move(stepper));
    }

    std::cout << "\n✅ LegSteppers created and configured for all 6 legs" << std::endl;

    // Configure velocity for proper stride generation
    double desired_velocity_x = 50.0; // mm/s in X direction
    double desired_velocity_y = 50.0; // mm/s in Y direction

    std::cout << "\nConfiguring velocity: (" << desired_velocity_x << ", " << desired_velocity_y << ", 0) mm/s" << std::endl;

    // Configure velocity and update stride for all steppers
    for (auto &stepper : steppers) {
        stepper->setDesiredVelocity(Point3D(desired_velocity_x, desired_velocity_y, 0), 0.0);
        stepper->updateStride();

        Point3D calculated_stride = stepper->getStrideVector();
        std::cout << "Leg " << stepper->getLegIndex() << " stride: (" << calculated_stride.x
                  << ", " << calculated_stride.y << ", " << calculated_stride.z << ")" << std::endl;
    }

    // Perform trajectory analysis for all legs
    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "PERFORMING TRAJECTORY ANALYSIS FOR ALL 6 LEGS" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    std::vector<LegAnalysisResult> analysis_results;

    for (int i = 0; i < NUM_LEGS; i++) {
        std::cout << "\nAnalyzing Leg " << i << "..." << std::endl;

        LegAnalysisResult result = analyzeLegTrajectory(i, *steppers[i], hexapod_legs[i],
                                                        model, tripod_config, true);
        analysis_results.push_back(result);
    }

    // Print comprehensive summary
    printHexapodAnalysisSummary(analysis_results, tripod_config);

    // Analyze stance geometry issues
    analyzeStanceGeometry(analysis_results, Point3D(desired_velocity_x, desired_velocity_y, 0));

    // Print detailed trajectory for all legs
    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "GENERATING DETAILED TRAJECTORY ANALYSIS FOR ALL 6 LEGS" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        std::cout << "\nGenerating detailed trajectory analysis for Leg " << i << ":" << std::endl;
        printDetailedLegTrajectory(i, *steppers[i], hexapod_legs[i], model, tripod_config);
    }

    // Test different velocities to analyze workspace limits
    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "WORKSPACE LIMIT ANALYSIS" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    std::vector<double> test_velocities = {30.0, 60.0, 90.0, 120.0};

    for (double vel : test_velocities) {
        std::cout << "\nTesting velocity: " << vel << " mm/s" << std::endl;

        int reachable_count = 0;
        int valid_joints_count = 0;
        double avg_reach_utilization = 0;

        for (int i = 0; i < NUM_LEGS; i++) {
            steppers[i]->setDesiredVelocity(Point3D(vel, vel, 0), 0.0);
            steppers[i]->updateStride();

            Point3D stride = steppers[i]->getStrideVector();
            Point3D initial_pos = hexapod_legs[i].getCurrentTipPositionGlobal();
            Point3D target = initial_pos + stride * 0.5;

            bool reachable = isPositionReachable(model, i, target);
            if (reachable)
                reachable_count++;

            Point3D base_pos = hexapod_legs[i].getBasePosition();
            double distance = (target - base_pos).norm();
            double reach_util = (distance / model.getLegReach()) * 100.0;
            avg_reach_utilization += reach_util;

            // Quick joint limit check
            if (hexapod_legs[i].applyIK(target)) {
                JointAngles angles = hexapod_legs[i].getJointAngles();
                if (model.checkJointLimits(i, angles)) {
                    valid_joints_count++;
                }
            }
        }

        avg_reach_utilization /= NUM_LEGS;

        std::cout << "  Reachable targets: " << reachable_count << "/6 (" << (reachable_count * 100.0 / 6) << "%)" << std::endl;
        std::cout << "  Valid joint limits: " << valid_joints_count << "/6 (" << (valid_joints_count * 100.0 / 6) << "%)" << std::endl;
        std::cout << "  Average reach utilization: " << avg_reach_utilization << "%" << std::endl;

        if (reachable_count == 6 && valid_joints_count == 6) {
            std::cout << "  ✅ All legs can handle this velocity" << std::endl;
        } else {
            std::cout << "  ⚠ Some legs have issues at this velocity" << std::endl;
        }
    }

    std::cout << "\n"
              << std::string(80, '=') << std::endl;
    std::cout << "HEXAPOD TRAJECTORY ANALYSIS COMPLETED" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    return 0;
}
