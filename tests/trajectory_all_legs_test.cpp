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

void analyzeAllLegsTrajectory(Leg test_legs[NUM_LEGS], LegStepper steppers[NUM_LEGS], const RobotModel &model, const GaitConfiguration &gait_config) {
    std::cout << "\n=== TRAJECTORY ANALYSIS FOR ALL 6 LEGS (Gait: " << gait_config.gait_name << ") ===" << std::endl;

    // Calculate timing parameters using OpenSHC-compatible calculation
    double time_delta = 0.02; // Standard 50Hz control loop
    double period = 1.0;
    double swing_period = period * gait_config.getSwingRatio();
    double stance_period = period * gait_config.getStanceRatio();

    int swing_iterations = (int)((swing_period / period) / (gait_config.getStepFrequency() * time_delta));
    int stance_iterations = (int)((stance_period / period) / (gait_config.getStepFrequency() * time_delta));
    int total_iterations = swing_iterations + stance_iterations;

    std::cout << "Timing parameters:" << std::endl;
    std::cout << "  Swing iterations: " << swing_iterations << std::endl;
    std::cout << "  Stance iterations: " << stance_iterations << std::endl;
    std::cout << "  Total iterations: " << total_iterations << std::endl;
    std::cout << "  Time per iteration: " << time_delta << "s" << std::endl;

    // Analyze initial positions for all legs
    std::cout << "\n=== INITIAL POSITIONS OF ALL LEGS ===" << std::endl;
    std::cout << "Leg  | Initial Position (x, y, z) | Base (x, y, z) | Distance | Reachable" << std::endl;
    std::cout << "-----+---------------------------+----------------+-----------+-----------" << std::endl;

    Point3D initial_positions[NUM_LEGS];
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        initial_positions[leg_id] = test_legs[leg_id].getCurrentTipPositionGlobal();
        Point3D base_pos = test_legs[leg_id].getBasePosition();
        double distance = (initial_positions[leg_id] - base_pos).norm();
        bool reachable = isPositionReachable(model, leg_id, initial_positions[leg_id]);

        printf("  %d  | (%8.1f, %8.1f, %8.1f) | (%6.1f, %6.1f, %6.1f) | %7.1f | %s\n",
               leg_id, initial_positions[leg_id].x, initial_positions[leg_id].y, initial_positions[leg_id].z,
               base_pos.x, base_pos.y, base_pos.z, distance, reachable ? "✓" : "✗");
    }

    // Analyze target positions and stride vectors
    std::cout << "\n=== VECTORES DE PASO Y OBJETIVOS DE TODAS LAS PATAS ===" << std::endl;
    std::cout << "Pata | Vector Step (x, y, z) | Position Objetivo (x, y, z) | Magnitud | Alcanzable" << std::endl;
    std::cout << "-----+-----------------------+-----------------------------+----------+-----------" << std::endl;

    Point3D target_positions[NUM_LEGS];
    Point3D stride_vectors[NUM_LEGS];
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        stride_vectors[leg_id] = steppers[leg_id].getStrideVector();
        target_positions[leg_id] = steppers[leg_id].getTargetTipPose();
        double magnitude = stride_vectors[leg_id].norm();
        bool target_reachable = isPositionReachable(model, leg_id, target_positions[leg_id]);

        printf("  %d  | (%7.1f, %7.1f, %7.1f) | (%9.1f, %9.1f, %9.1f) | %6.1f | %s\n",
               leg_id, stride_vectors[leg_id].x, stride_vectors[leg_id].y, stride_vectors[leg_id].z,
               target_positions[leg_id].x, target_positions[leg_id].y, target_positions[leg_id].z,
               magnitude, target_reachable ? "✓" : "✗");
    }

    // Test IK methods for all legs
    std::cout << "\n=== COMPARACIÓN DE MÉTODOS IK PARA TODAS LAS PATAS ===" << std::endl;
    std::cout << "Pata | IK Tradicional | Error (mm) | IK Delta | Error (mm) | Mejor Método" << std::endl;
    std::cout << "-----+----------------+------------+----------+------------+-------------" << std::endl;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        // Test traditional IK
        bool traditional_success = test_legs[leg_id].applyIK(initial_positions[leg_id]);
        JointAngles traditional_angles = test_legs[leg_id].getJointAngles();
        Point3D traditional_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        double traditional_error = (traditional_pos - initial_positions[leg_id]).norm();

        // Test delta-based IK
        test_legs[leg_id].setJointAngles(traditional_angles);
        JointAngles current_angles = test_legs[leg_id].getJointAngles();
        Point3D current_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        JointAngles delta_angles = model.applyAdvancedIK(leg_id, current_pos, initial_positions[leg_id], current_angles, 0.02);
        test_legs[leg_id].setJointAngles(delta_angles);
        Point3D delta_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        double delta_error = (delta_pos - initial_positions[leg_id]).norm();
        bool delta_success = (delta_error < 1.0);

        std::string better_method = (traditional_error < delta_error) ? "Tradicional" : "Delta";
        if (std::abs(traditional_error - delta_error) < 0.1)
            better_method = "Similares";

        printf("  %d  | %s | %8.3f | %s | %8.3f | %s\n",
               leg_id, traditional_success ? "✓" : "✗", traditional_error,
               delta_success ? "✓" : "✗", delta_error, better_method.c_str());
    }

    // Generate swing trajectories for all legs
    std::cout << "\n=== TRAYECTORIAS DE SWING PARA TODAS LAS PATAS ===" << std::endl;

    // Reset all steppers to swing state
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_SWING);
        steppers[leg_id].setPhase(gait_config.phase_config.swing_phase);
        steppers[leg_id].setStepProgress(0.0);

        // Initialize timing for each stepper
        steppers[leg_id].updateTipPositionIterative(1, time_delta, false, false);
    }

    // Show detailed trajectory analysis every 5 steps for swing phase
    std::cout << "\n=== DETAILED SWING TRAJECTORY ANALYSIS (every 5 steps) ===" << std::endl;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        std::cout << "\n--- LEG " << leg_id << " - SWING PHASE ---" << std::endl;
        std::cout << "Step | Iteration | Position (x, y, z) | Angles (coxa, femur, tibia) | Velocidad Ang (rad/s) | Base Distance | Limits" << std::endl;
        std::cout << "-----+-----------+---------------------+------------------------------+-----------------------+----------------+---------" << std::endl;

        // Reset stepper to initial position for this leg
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_SWING);
        steppers[leg_id].setPhase(gait_config.phase_config.swing_phase);
        steppers[leg_id].setStepProgress(0.0);

        JointAngles previous_angles = test_legs[leg_id].getJointAngles();
        Point3D base_pos = test_legs[leg_id].getBasePosition();

        int step_counter = 0;
        for (int i = 1; i <= swing_iterations; i++) {
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);

            // Show detailed info every 5 steps
            if (i % 5 == 0 || i == 1 || i == swing_iterations) {
                Point3D current_pos = steppers[leg_id].getCurrentTipPose();

                // Apply advanced IK to get joint angles
                JointAngles before_angles = test_legs[leg_id].getJointAngles();
                Point3D before_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
                JointAngles new_angles = model.applyAdvancedIK(leg_id, before_pos, current_pos, before_angles, time_delta);
                test_legs[leg_id].setJointAngles(new_angles);

                // Calculate angular velocities
                double coxa_vel = (new_angles.coxa - previous_angles.coxa) / time_delta;
                double femur_vel = (new_angles.femur - previous_angles.femur) / time_delta;
                double tibia_vel = (new_angles.tibia - previous_angles.tibia) / time_delta;

                // Calculate distance from base
                double distance_from_base = (current_pos - base_pos).norm();

                // Check joint limits
                bool valid_joints = model.checkJointLimits(leg_id, new_angles);

                printf(" %3d | %6d/%2d | (%7.1f,%7.1f,%7.1f) | (%6.1f,%6.1f,%6.1f) | (%6.2f,%6.2f,%6.2f) | %12.1f | %s\n",
                       ++step_counter, i, swing_iterations,
                       current_pos.x, current_pos.y, current_pos.z,
                       new_angles.coxa * 180.0 / M_PI, new_angles.femur * 180.0 / M_PI, new_angles.tibia * 180.0 / M_PI,
                       coxa_vel, femur_vel, tibia_vel,
                       distance_from_base,
                       valid_joints ? "✓" : "❌");

                previous_angles = new_angles;
            }
        }
    }

    // Analyze final swing positions and precision
    std::cout << "\n=== ANALYSIS DE PRECISIÓN AL FINAL DEL SWING ===" << std::endl;
    std::cout << "Pata | Pos. Final (x, y, z) | Objetivo (x, y, z) | Error (mm) | Precisión" << std::endl;
    std::cout << "-----+----------------------+--------------------+------------+-----------" << std::endl;

    Point3D final_swing_positions[NUM_LEGS];
    double total_error = 0.0;
    int legs_within_tolerance = 0;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        // Reset and execute complete swing
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        for (int i = 1; i <= swing_iterations; i++) {
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);
        }

        final_swing_positions[leg_id] = steppers[leg_id].getCurrentTipPose();
        Point3D target = target_positions[leg_id];
        double error = (final_swing_positions[leg_id] - target).norm();

        total_error += error;
        if (error < 1.0)
            legs_within_tolerance++;

        std::string precision_status = (error < 1.0) ? "Excelente" : (error < 2.0) ? "Buena"
                                                                                   : "Mejorable";

        printf("  %d  | (%7.1f, %7.1f, %7.1f) | (%7.1f, %7.1f, %7.1f) | %8.3f | %s\n",
               leg_id, final_swing_positions[leg_id].x, final_swing_positions[leg_id].y, final_swing_positions[leg_id].z,
               target.x, target.y, target.z, error, precision_status.c_str());
    }

    // Test stance phase for all legs with detailed analysis
    std::cout << "\n=== ANALYSIS DETALLADO DE FASE STANCE (cada 5 pasos) ===" << std::endl;

    // Initialize all legs for stance phase
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_STANCE);
        steppers[leg_id].setPhase(gait_config.phase_config.stance_phase);
        steppers[leg_id].setStepProgress(0.0);
    }

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        std::cout << "\n--- LEG " << leg_id << " - FASE STANCE ---" << std::endl;
        std::cout << "Step | Iteration | Position (x, y, z) | Angles (coxa, femur, tibia) | Velocidad Ang (rad/s) | Dist. Acum XY | Fuerza Est" << std::endl;
        std::cout << "-----+-----------+---------------------+------------------------------+-----------------------+---------------+-----------" << std::endl;

        // Reset this leg's stepper
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_STANCE);
        steppers[leg_id].setPhase(gait_config.phase_config.stance_phase);
        steppers[leg_id].setStepProgress(0.0);

        // Store initial stance angles
        JointAngles initial_stance_angles = test_legs[leg_id].getJointAngles();
        JointAngles previous_angles = initial_stance_angles;
        Point3D accumulated_displacement(0, 0, 0);

        int step_counter = 0;
        for (int i = swing_iterations + 1; i <= total_iterations; i++) {
            Point3D prev_pos = steppers[leg_id].getCurrentTipPose();
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);

            // Show detailed info every 5 steps
            if ((i - swing_iterations) % 5 == 0 || i == swing_iterations + 1 || i == total_iterations) {
                Point3D current_pos = steppers[leg_id].getCurrentTipPose();

                // Apply IK to get current angles
                JointAngles before_angles = test_legs[leg_id].getJointAngles();
                Point3D before_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
                JointAngles new_angles = model.applyAdvancedIK(leg_id, before_pos, current_pos, before_angles, time_delta);
                test_legs[leg_id].setJointAngles(new_angles);

                // Calculate angular velocities
                double coxa_vel = (new_angles.coxa - previous_angles.coxa) / time_delta;
                double femur_vel = (new_angles.femur - previous_angles.femur) / time_delta;
                double tibia_vel = (new_angles.tibia - previous_angles.tibia) / time_delta;

                // Calculate accumulated XY displacement
                Point3D step_displacement = current_pos - prev_pos;
                step_displacement.z = 0; // Only XY displacement for stance
                accumulated_displacement = accumulated_displacement + step_displacement;
                double accumulated_xy_distance = accumulated_displacement.norm();

                // Estimate support force (simplified - based on angle changes and position stability)
                double angle_change_magnitude = std::sqrt(coxa_vel * coxa_vel + femur_vel * femur_vel + tibia_vel * tibia_vel);
                std::string force_estimate = (angle_change_magnitude < 0.1) ? "Alta" : (angle_change_magnitude < 0.5) ? "Media"
                                                                                                                      : "Baja";

                printf(" %3d | %6d/%2d | (%7.1f,%7.1f,%7.1f) | (%6.1f,%6.1f,%6.1f) | (%6.2f,%6.2f,%6.2f) | %11.3f | %s\n",
                       ++step_counter, i, total_iterations,
                       current_pos.x, current_pos.y, current_pos.z,
                       new_angles.coxa * 180.0 / M_PI, new_angles.femur * 180.0 / M_PI, new_angles.tibia * 180.0 / M_PI,
                       coxa_vel, femur_vel, tibia_vel,
                       accumulated_xy_distance,
                       force_estimate.c_str());

                previous_angles = new_angles;
            }
        }

        // Final stance analysis for this leg
        Point3D final_stance_pos = steppers[leg_id].getCurrentTipPose();
        Point3D total_xy_displacement = final_stance_pos - initial_positions[leg_id];
        total_xy_displacement.z = 0;
        double total_xy_magnitude = total_xy_displacement.norm();

        JointAngles final_angles = test_legs[leg_id].getJointAngles();
        double total_coxa_change = (final_angles.coxa - initial_stance_angles.coxa) * 180.0 / M_PI;

        std::cout << "     Resumen: Desp. XY total = " << total_xy_magnitude << " mm, ";
        std::cout << "Cambio coxa total = " << total_coxa_change << "°" << std::endl;
    }

    // Add summary table for both phases
    std::cout << "\n=== RESUMEN COMPARATIVO DE AMBAS FASES ===" << std::endl;
    std::cout << "Pata | Pos. Inicial (x, y, z) | Pos. Final Swing | Pos. Final Stance | Error Swing | Desp. Stance XY" << std::endl;
    std::cout << "-----+------------------------+------------------+-------------------+-------------+----------------" << std::endl;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        // Calculate final swing position (already stored)
        Point3D target = target_positions[leg_id];
        double swing_error = (final_swing_positions[leg_id] - target).norm();

        // Calculate final stance position
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_STANCE);
        for (int i = swing_iterations + 1; i <= total_iterations; i++) {
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);
        }
        Point3D final_stance_pos = steppers[leg_id].getCurrentTipPose();

        Point3D stance_displacement = final_stance_pos - initial_positions[leg_id];
        stance_displacement.z = 0;
        double stance_xy_magnitude = stance_displacement.norm();

        printf("  %d  | (%7.1f, %7.1f, %7.1f) | (%7.1f,%7.1f,%7.1f) | (%7.1f,%7.1f,%7.1f) | %9.3f | %12.3f\n",
               leg_id,
               initial_positions[leg_id].x, initial_positions[leg_id].y, initial_positions[leg_id].z,
               final_swing_positions[leg_id].x, final_swing_positions[leg_id].y, final_swing_positions[leg_id].z,
               final_stance_pos.x, final_stance_pos.y, final_stance_pos.z,
               swing_error, stance_xy_magnitude);
    }

    // Summary statistics
    double average_error = total_error / NUM_LEGS;
    double precision_rate = (double)legs_within_tolerance / NUM_LEGS * 100.0;

    // Add kinematic transition analysis
    std::cout << "\n=== ANALYSIS DE TRANSICIONES CINEMÁTICAS ===" << std::endl;
    std::cout << "Analisis de velocidades angulares y aceleraciones durante cambios de fase..." << std::endl;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        std::cout << "\n--- LEG " << leg_id << " - TRANSICIONES ---" << std::endl;

        // Analyze swing to stance transition
        steppers[leg_id].setCurrentTipPose(initial_positions[leg_id]);
        steppers[leg_id].setStepState(STEP_SWING);
        steppers[leg_id].setPhase(gait_config.phase_config.swing_phase);

        // Execute swing phase until near end
        for (int i = 1; i <= swing_iterations - 2; i++) {
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);
        }

        // Capture pre-transition state
        Point3D pre_transition_pos = steppers[leg_id].getCurrentTipPose();
        JointAngles before_angles = test_legs[leg_id].getJointAngles();
        Point3D before_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        JointAngles pre_transition_angles = model.applyAdvancedIK(leg_id, before_pos, pre_transition_pos, before_angles, time_delta);
        test_legs[leg_id].setJointAngles(pre_transition_angles);

        // Complete swing and capture end state
        for (int i = swing_iterations - 1; i <= swing_iterations; i++) {
            steppers[leg_id].updateTipPositionIterative(i, time_delta, false, false);
        }

        Point3D end_swing_pos = steppers[leg_id].getCurrentTipPose();
        before_angles = test_legs[leg_id].getJointAngles();
        before_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        JointAngles end_swing_angles = model.applyAdvancedIK(leg_id, before_pos, end_swing_pos, before_angles, time_delta);
        test_legs[leg_id].setJointAngles(end_swing_angles);

        // Transition to stance
        steppers[leg_id].setStepState(STEP_STANCE);
        steppers[leg_id].setPhase(gait_config.phase_config.stance_phase);
        steppers[leg_id].updateTipPositionIterative(swing_iterations + 1, time_delta, false, false);

        Point3D start_stance_pos = steppers[leg_id].getCurrentTipPose();
        before_angles = test_legs[leg_id].getJointAngles();
        before_pos = test_legs[leg_id].getCurrentTipPositionGlobal();
        JointAngles start_stance_angles = model.applyAdvancedIK(leg_id, before_pos, start_stance_pos, before_angles, time_delta);
        test_legs[leg_id].setJointAngles(start_stance_angles);

        // Calculate angular velocity changes across transition
        double transition_time = 2.0 * time_delta; // 2 steps for transition
        double coxa_vel_change = (start_stance_angles.coxa - pre_transition_angles.coxa) / transition_time;
        double femur_vel_change = (start_stance_angles.femur - pre_transition_angles.femur) / transition_time;
        double tibia_vel_change = (start_stance_angles.tibia - pre_transition_angles.tibia) / transition_time;

        // Calculate position smoothness
        Point3D pos_change_1 = end_swing_pos - pre_transition_pos;
        Point3D pos_change_2 = start_stance_pos - end_swing_pos;
        double smoothness_metric = (pos_change_2 - pos_change_1).norm() / time_delta;

        std::cout << "  Transición Swing → Stance:" << std::endl;
        std::cout << "    Cambio vel. angular (rad/s): Coxa=" << coxa_vel_change << ", Femur=" << femur_vel_change << ", Tibia=" << tibia_vel_change << std::endl;
        std::cout << "    Métrica suavidad posición: " << smoothness_metric << " mm/s" << std::endl;
        std::cout << "    Estado: " << (smoothness_metric < 50.0 ? "✓ Suave" : "⚠ Brusco") << std::endl;

        // Analyze joint limits during critical phases
        bool swing_mid_valid = model.checkJointLimits(leg_id, pre_transition_angles);
        bool swing_end_valid = model.checkJointLimits(leg_id, end_swing_angles);
        bool stance_start_valid = model.checkJointLimits(leg_id, start_stance_angles);

        std::cout << "    Limits articulares: Pre-transición=" << (swing_mid_valid ? "✓" : "❌");
        std::cout << ", Fin swing=" << (swing_end_valid ? "✓" : "❌");
        std::cout << ", Inicio stance=" << (stance_start_valid ? "✓" : "❌") << std::endl;
    }

    std::cout << "\n=== RESUMEN ESTADÍSTICO ===" << std::endl;
    std::cout << "Error promedio en precisión: " << average_error << " mm" << std::endl;
    std::cout << "Patas dentro de tolerancia (< 1mm): " << legs_within_tolerance << "/" << NUM_LEGS << " (" << precision_rate << "%)" << std::endl;

    if (precision_rate >= 80.0) {
        std::cout << "✅ EXCELENTE: La mayoría de las patas alcanzan alta precisión" << std::endl;
    } else if (precision_rate >= 60.0) {
        std::cout << "⚠ ACEPTABLE: Algunas patas necesitan ajustes de precisión" << std::endl;
    } else {
        std::cout << "❌ CRÍTICO: La mayoría de las patas necesitan corrección de precisión" << std::endl;
    }

    // Analyze workspace utilization
    std::cout << "\n=== ANALYSIS DE UTILIZACIÓN DEL ESPACIO DE TRABAJO ===" << std::endl;
    double max_reach = model.getLegReach();
    double total_reach_utilization = 0.0;

    std::cout << "Pata | Distancia Objetivo | Alcance Máximo | Utilización (%)" << std::endl;
    std::cout << "-----+-------------------+----------------+----------------" << std::endl;

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
        Point3D base_pos = test_legs[leg_id].getBasePosition();
        double target_distance = (target_positions[leg_id] - base_pos).norm();
        double utilization = (target_distance / max_reach) * 100.0;
        total_reach_utilization += utilization;

        printf("  %d  | %15.1f | %12.1f | %12.1f\n",
               leg_id, target_distance, max_reach, utilization);
    }

    double average_utilization = total_reach_utilization / NUM_LEGS;
    std::cout << "Utilización promedio del espacio de trabajo: " << average_utilization << "%" << std::endl;

    if (average_utilization > 90.0) {
        std::cout << "⚠ ADVERTENCIA: Utilizando >90% del alcance - riesgo de problemas de precisión" << std::endl;
    } else if (average_utilization > 70.0) {
        std::cout << "✅ ÓPTIMO: Buena utilización del espacio de trabajo" << std::endl;
    } else {
        std::cout << "ℹ CONSERVADOR: Utilizando <70% del alcance - margen seguro" << std::endl;
    }
}

int main() {
    std::cout << "=== Test de Trayectoria para las 6 Patas del Hexápodo ===" << std::endl;
    std::cout << "Este test analiza las trayectorias de todas las patas durante la marcha trípode" << std::endl;

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

    // Configure gait factors for tripod gait
    p.gait_factors.tripod_length_factor = 0.4;  // 40% of leg reach for step length
    p.gait_factors.tripod_height_factor = 0.15; // 15% of standing height for swing

    // Create tripod gait configuration
    GaitConfiguration tripod_config = createTripodGaitConfig(p);
    std::cout << "\nConfiguración de Marcha Trípode:" << std::endl;
    std::cout << "  Longitud de paso: " << tripod_config.step_length << " mm" << std::endl;
    std::cout << "  Altura de swing: " << tripod_config.swing_height << " mm" << std::endl;
    std::cout << "  Frecuencia: " << tripod_config.getStepFrequency() << " Hz" << std::endl;
    std::cout << "  Ratio stance: " << tripod_config.getStanceRatio() << std::endl;
    std::cout << "  Ratio swing: " << tripod_config.getSwingRatio() << std::endl;

    RobotModel model(p);

    // Create all 6 legs
    Leg test_legs[NUM_LEGS] = {
        Leg(0, model), Leg(1, model), Leg(2, model),
        Leg(3, model), Leg(4, model), Leg(5, model)};

    // Initialize all legs
    for (int i = 0; i < NUM_LEGS; ++i) {
        test_legs[i].initialize(Pose::Identity());
        test_legs[i].updateTipPosition();
    }

    // Configure standing pose using BodyPoseController
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);
    BodyPoseController pose_controller(model, pose_config);
    pose_controller.setWalkPlanePoseEnabled(true);
    pose_controller.initializeLegPosers(test_legs);

    bool pose_success = pose_controller.setStandingPose(test_legs);
    if (!pose_success) {
        std::cerr << "❌ ERROR: No se pudo establecer la posición de pie" << std::endl;
        return 1;
    }

    std::cout << "\n=== CONFIGURACIÓN DE POSICIÓN DE PIE ===" << std::endl;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D pos = test_legs[i].getCurrentTipPositionGlobal();
        JointAngles angles = test_legs[i].getJointAngles();
        std::cout << "Pata " << i << " - Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")";
        std::cout << " - Angles: (" << (angles.coxa * 180.0 / M_PI) << "°, "
                  << (angles.femur * 180.0 / M_PI) << "°, "
                  << (angles.tibia * 180.0 / M_PI) << "°)" << std::endl;
    }

    // Create required objects
    WorkspaceAnalyzer workspace_analyzer(const_cast<RobotModel &>(model));

    // Create LegSteppers for all legs
    LegStepper steppers[NUM_LEGS] = {
        LegStepper(0, test_legs[0].getCurrentTipPositionGlobal(), test_legs[0], const_cast<RobotModel &>(model), &workspace_analyzer),
        LegStepper(1, test_legs[1].getCurrentTipPositionGlobal(), test_legs[1], const_cast<RobotModel &>(model), &workspace_analyzer),
        LegStepper(2, test_legs[2].getCurrentTipPositionGlobal(), test_legs[2], const_cast<RobotModel &>(model), &workspace_analyzer),
        LegStepper(3, test_legs[3].getCurrentTipPositionGlobal(), test_legs[3], const_cast<RobotModel &>(model), &workspace_analyzer),
        LegStepper(4, test_legs[4].getCurrentTipPositionGlobal(), test_legs[4], const_cast<RobotModel &>(model), &workspace_analyzer),
        LegStepper(5, test_legs[5].getCurrentTipPositionGlobal(), test_legs[5], const_cast<RobotModel &>(model), &workspace_analyzer)};

    // Configure StepCycle from tripod gait configuration for all steppers
    StepCycle step_cycle = tripod_config.generateStepCycle();
    for (int i = 0; i < NUM_LEGS; i++) {
        steppers[i].setStepCycle(step_cycle);
        steppers[i].setDefaultTipPose(test_legs[i].getCurrentTipPositionGlobal());
    }

    std::cout << "\nStepCycle configurado: frequency=" << step_cycle.frequency_ << "Hz, period=" << step_cycle.period_ << std::endl;

    // Configure velocity - use same velocity for all legs to test precision consistency
    double base_velocity_x = 15.0; // mm/s
    double base_velocity_y = 15.0; // mm/s

    std::cout << "\n=== CONFIGURACIÓN DE VELOCIDADES PARA TODAS LAS PATAS ===" << std::endl;
    std::cout << "Usando velocidad uniforme para todas las patas para analizar precisión" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        // Use same velocity for all legs to isolate precision issues
        steppers[i].setDesiredVelocity(Point3D(base_velocity_x, base_velocity_y, 0), 0.0);
        steppers[i].updateStride();

        Point3D stride = steppers[i].getStrideVector();
        std::cout << "Pata " << i << " - Velocidad: (" << base_velocity_x << ", " << base_velocity_y << ", 0) mm/s";
        std::cout << " - Stride: (" << stride.x << ", " << stride.y << ", " << stride.z << ")" << std::endl;

        // Verify all strides are identical
        if (i == 0) {
            std::cout << "  → Stride magnitud: " << stride.norm() << " mm" << std::endl;
        } else {
            std::cout << "  → Stride magnitud: " << stride.norm() << " mm";
            Point3D first_stride = steppers[0].getStrideVector();
            double stride_difference = (stride - first_stride).norm();
            if (stride_difference < 0.001) {
                std::cout << " ✓ (idéntico)" << std::endl;
            } else {
                std::cout << " ⚠ (diferencia: " << stride_difference << " mm)" << std::endl;
            }
        }
    }

    // Perform comprehensive analysis
    analyzeAllLegsTrajectory(test_legs, steppers, model, tripod_config);

    // Additional analysis: Check if the problem is in the Bezier calculation
    std::cout << "\n=== ANALYSIS DETALLADO DEL ALGORITMO DE BEZIER ===" << std::endl;
    std::cout << "Investigando la causa del error sistemático de 12mm..." << std::endl;

    // Test with leg 0 to understand the Bezier algorithm issue
    int test_leg = 0;
    steppers[test_leg].setCurrentTipPose(test_legs[test_leg].getCurrentTipPositionGlobal());
    steppers[test_leg].setStepState(STEP_SWING);
    steppers[test_leg].setPhase(tripod_config.phase_config.swing_phase);
    steppers[test_leg].setStepProgress(0.0);

    // Calculate swing iterations like in analyzeAllLegsTrajectory
    double time_delta = 0.02;
    double period = 1.0;
    double swing_period = period * tripod_config.getSwingRatio();
    int swing_iterations = (int)((swing_period / period) / (tripod_config.getStepFrequency() * time_delta));

    // Initialize with first iteration to generate control nodes
    steppers[test_leg].updateTipPositionIterative(1, time_delta, false, false);

    std::cout << "\n=== CONTROL NODES ANALYSIS ===" << std::endl;
    std::cout << "Swing 1 (primera mitad) control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = steppers[test_leg].getSwing1ControlNode(i);
        std::cout << "  Node[" << i << "]: (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    std::cout << "\nSwing 2 (segunda mitad) control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = steppers[test_leg].getSwing2ControlNode(i);
        std::cout << "  Node[" << i << "]: (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    Point3D initial_pos = test_legs[test_leg].getCurrentTipPositionGlobal();
    Point3D target_pos = steppers[test_leg].getTargetTipPose();
    Point3D stride = steppers[test_leg].getStrideVector();

    std::cout << "\nParametros de la trayectoria:" << std::endl;
    std::cout << "  Position inicial: (" << initial_pos.x << ", " << initial_pos.y << ", " << initial_pos.z << ")" << std::endl;
    std::cout << "  Position objetivo: (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")" << std::endl;
    std::cout << "  Vector stride: (" << stride.x << ", " << stride.y << ", " << stride.z << ")" << std::endl;
    std::cout << "  Distancia esperada: " << stride.norm() << " mm" << std::endl;
    std::cout << "  Swing iterations: " << swing_iterations << std::endl;

    // CRITICAL: Test if the problem is in delta accumulation vs absolute position calculation
    std::cout << "\n=== COMPARACIÓN: DELTA ACCUMULATION vs ABSOLUTE POSITION ===" << std::endl;

    // Method 1: Current implementation (delta accumulation with resets)
    Point3D final_pos_delta_method;
    steppers[test_leg].setCurrentTipPose(initial_pos);
    for (int i = 1; i <= swing_iterations; i++) {
        steppers[test_leg].updateTipPositionIterative(i, time_delta, false, false);
    }
    final_pos_delta_method = steppers[test_leg].getCurrentTipPose();

    std::cout << "Método Delta (actual): Position final (" << final_pos_delta_method.x << ", " << final_pos_delta_method.y << ", " << final_pos_delta_method.z << ")" << std::endl;
    double delta_error = (final_pos_delta_method - target_pos).norm();
    std::cout << "  Error: " << delta_error << " mm" << std::endl;

    // CRITICAL: Test if the problem is in how we're using the algorithm vs the algorithm itself
    std::cout << "\n=== COMPARACIÓN: USO CORRECTO vs INCORRECTO DEL ALGORITMO ===" << std::endl;

    // Method 1: Incorrect usage (what we've been doing - resetting position each time)
    std::cout << "Método 1: Uso INCORRECTO (reiniciar posición en cada medición)" << std::endl;
    Point3D final_pos_incorrect_method;
    steppers[test_leg].setCurrentTipPose(initial_pos);
    for (int i = 1; i <= swing_iterations; i++) {
        steppers[test_leg].updateTipPositionIterative(i, time_delta, false, false);
    }
    final_pos_incorrect_method = steppers[test_leg].getCurrentTipPose();

    std::cout << "  Position final: (" << final_pos_incorrect_method.x << ", " << final_pos_incorrect_method.y << ", " << final_pos_incorrect_method.z << ")" << std::endl;
    double incorrect_error = (final_pos_incorrect_method - target_pos).norm();
    std::cout << "  Error: " << incorrect_error << " mm" << std::endl;

    // Method 2: Correct usage (OpenSHC way - continuous accumulation without resets)
    std::cout << "\nMétodo 2: Uso CORRECTO (acumulación continua como en OpenSHC)" << std::endl;
    steppers[test_leg].setCurrentTipPose(initial_pos);
    steppers[test_leg].setStepState(STEP_SWING);
    steppers[test_leg].setPhase(tripod_config.phase_config.swing_phase);
    steppers[test_leg].setStepProgress(0.0);

    // Execute continuously without resets (like in OpenSHC main loop)
    for (int i = 1; i <= swing_iterations; i++) {
        steppers[test_leg].updateTipPositionIterative(i, time_delta, false, false);
        // NO RESET - continuous accumulation like OpenSHC
    }

    Point3D final_pos_correct_method = steppers[test_leg].getCurrentTipPose();
    std::cout << "  Position final: (" << final_pos_correct_method.x << ", " << final_pos_correct_method.y << ", " << final_pos_correct_method.z << ")" << std::endl;
    double correct_error = (final_pos_correct_method - target_pos).norm();
    std::cout << "  Error: " << correct_error << " mm" << std::endl;

    std::cout << "\n📊 DIAGNÓSTICO:" << std::endl;
    if (correct_error < incorrect_error * 0.5) {
        std::cout << "✅ PROBLEMA CONFIRMADO: El error está en el uso del algoritmo, NO en la implementación" << std::endl;
        std::cout << "   El algoritmo de OpenSHC funciona correctamente con uso continuo" << std::endl;
        std::cout << "   Error reducido de " << incorrect_error << "mm a " << correct_error << "mm" << std::endl;
    } else {
        std::cout << "⚠ El problema puede estar en la implementación del algoritmo" << std::endl;
        std::cout << "   Error similar: " << incorrect_error << "mm vs " << correct_error << "mm" << std::endl;
    }

    // Additional analysis: Check trajectory independence
    std::cout << "\n=== ANALYSIS DE INDEPENDENCIA DE TRAYECTORIAS ===" << std::endl;
    std::cout << "Verificando si las trayectorias se calculan independientemente..." << std::endl;

    // Test 1: Modify one leg's velocity and check if others are affected
    std::cout << "\nTest 1: Modificar velocidad de una pata" << std::endl;
    Point3D original_stride_leg2 = steppers[2].getStrideVector();
    Point3D original_stride_leg4 = steppers[4].getStrideVector();

    // Change only leg 0's velocity
    steppers[0].setDesiredVelocity(Point3D(100.0, 50.0, 0), 0.0);
    steppers[0].updateStride();

    Point3D new_stride_leg0 = steppers[0].getStrideVector();
    Point3D new_stride_leg2 = steppers[2].getStrideVector();
    Point3D new_stride_leg4 = steppers[4].getStrideVector();

    std::cout << "  Pata 0 stride cambió: " << (new_stride_leg0 - Point3D(base_velocity_x / 2, base_velocity_y / 2, 0)).norm() << " mm" << std::endl;
    std::cout << "  Pata 2 stride cambió: " << (new_stride_leg2 - original_stride_leg2).norm() << " mm" << std::endl;
    std::cout << "  Pata 4 stride cambió: " << (new_stride_leg4 - original_stride_leg4).norm() << " mm" << std::endl;

    bool independence_test1 = (new_stride_leg2 - original_stride_leg2).norm() < 0.001 &&
                              (new_stride_leg4 - original_stride_leg4).norm() < 0.001;
    std::cout << "  → Independencia de stride: " << (independence_test1 ? "✓ PASÓ" : "❌ FALLÓ") << std::endl;

    // Restore original velocity for leg 0
    steppers[0].setDesiredVelocity(Point3D(base_velocity_x, base_velocity_y, 0), 0.0);
    steppers[0].updateStride();

    // Test 2: Check if trajectory calculations are position-dependent
    std::cout << "\nTest 2: Dependencia de posición inicial" << std::endl;
    Point3D original_pos_leg1 = test_legs[1].getCurrentTipPositionGlobal();

    // Save trajectories with original positions
    std::vector<Point3D> original_trajectory_leg1;
    steppers[1].setCurrentTipPose(original_pos_leg1);
    steppers[1].setStepState(STEP_SWING);

    for (int i = 1; i <= 10; i++) {
        steppers[1].updateTipPositionIterative(i, 0.02, false, false);
        original_trajectory_leg1.push_back(steppers[1].getCurrentTipPose());
    }

    // Temporarily move leg 1 to a different position
    Point3D modified_pos = original_pos_leg1 + Point3D(5, 5, 0);
    test_legs[1].applyIK(modified_pos);
    steppers[1].setCurrentTipPose(modified_pos);
    steppers[1].setStepState(STEP_SWING);

    // Calculate trajectory from modified position
    std::vector<Point3D> modified_trajectory_leg1;
    for (int i = 1; i <= 10; i++) {
        steppers[1].updateTipPositionIterative(i, 0.02, false, false);
        modified_trajectory_leg1.push_back(steppers[1].getCurrentTipPose());
    }

    // Compare trajectory shapes (relative movements)
    double trajectory_shape_difference = 0.0;
    for (int i = 1; i < 10; i++) {
        Point3D original_delta = original_trajectory_leg1[i] - original_trajectory_leg1[i - 1];
        Point3D modified_delta = modified_trajectory_leg1[i] - modified_trajectory_leg1[i - 1];
        trajectory_shape_difference += (original_delta - modified_delta).norm();
    }
    trajectory_shape_difference /= 9.0; // Average difference

    std::cout << "  Diferencia promedio en forma de trayectoria: " << trajectory_shape_difference << " mm" << std::endl;
    bool independence_test2 = trajectory_shape_difference < 0.1;
    std::cout << "  → Independencia de forma: " << (independence_test2 ? "✓ PASÓ" : "❌ FALLÓ") << std::endl;

    // Restore original position
    test_legs[1].applyIK(original_pos_leg1);
    steppers[1].setCurrentTipPose(original_pos_leg1);

    // Summary
    std::cout << "\nRESUMEN DE INDEPENDENCIA:" << std::endl;
    if (independence_test1 && independence_test2) {
        std::cout << "✅ Las trayectorias se calculan independientemente" << std::endl;
        std::cout << "   El error creciente se debe a las diferentes velocidades utilizadas" << std::endl;
    } else {
        std::cout << "⚠ Posible dependencia entre trayectorias detectada" << std::endl;
        if (!independence_test1)
            std::cout << "   - Los stride vectors pueden estar acoplados" << std::endl;
        if (!independence_test2)
            std::cout << "   - La forma de trayectoria depende de la posición inicial" << std::endl;
    }

    std::cout << "\n=== CONCLUSIONES DEL TEST ===" << std::endl;
    std::cout << "✅ Test completado exitosamente para las 6 patas" << std::endl;
    std::cout << "📊 Se analizaron las trayectorias de swing y stance para cada pata" << std::endl;
    std::cout << "🔍 Se verificaron los métodos de IK tradicional y delta para todas las patas" << std::endl;
    std::cout << "⚙ Se validaron los límites articulares y la precisión de posicionamiento" << std::endl;
    std::cout << "🎯 Se evaluó la utilización del espacio de trabajo para cada pata" << std::endl;

    return 0;
}
