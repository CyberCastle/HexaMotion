/***
 * ANÁLISIS DEL PROBLEMA EN LA GENERACIÓN DE TRAYECTORIAS BÉZIER
 *
 * PROBLEMA IDENTIFICADO:
 * 1. Los nodos de control Bézier se generan con posiciones extremas debido a swing_delta_t_ = 0
 * 2. Se usa enfoque de posición absoluta en lugar de incremental como OpenSHC
 * 3. La fórmula de separación de nodos difiere de OpenSHC
 *
 * SOLUCIÓN IMPLEMENTADA:
 * - Usar enfoque incremental como OpenSHC (quarticBezierDot + acumulación)
 * - Corregir cálculo de timing parameters
 * - Usar fórmulas idénticas a OpenSHC para separación de nodos
 ***/

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

void testOriginalVsFixedApproach(LegStepper &stepper, Leg &leg, const RobotModel &model) {
    std::cout << "\n=== COMPARISON: ORIGINAL vs FIXED APPROACH ===" << std::endl;

    // Configurar una velocidad real para generar un stride vector
    Point3D linear_velocity(20.0, 0.0, 0.0); // 20 mm/s hacia adelante
    double angular_velocity = 0.0;
    stepper.setDesiredVelocity(linear_velocity, angular_velocity);

    // Configurar swing clearance para elevar la trayectoria
    Point3D swing_clearance(0.0, 0.0, 30.0); // 30mm de altura
    stepper.setSwingClearance(swing_clearance);

    // Configurar state y phase
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(5);
    stepper.setStepProgress(0.0);

    // Configurar posición inicial de swing
    Point3D initial_position = leg.getCurrentTipPositionGlobal();
    stepper.setSwingOriginTipVelocity(linear_velocity);

    std::cout << "Initial position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;

    // Llamar updateStride para generar el stride vector
    stepper.updateStride();
    Point3D stride_vector = stepper.getStrideVector();
    std::cout << "Stride vector: (" << stride_vector.x << ", " << stride_vector.y << ", " << stride_vector.z << ")" << std::endl;

    // ORIGINAL APPROACH - Con los problemas existentes
    std::cout << "\n--- ORIGINAL APPROACH (Current HexaMotion) ---" << std::endl;
    stepper.updateTipPosition(20.0, 0.02, false, false);

    std::cout << "Primary swing control nodes (Original):" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing1ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    // FIXED APPROACH - Implementar lógica similar a OpenSHC
    std::cout << "\n--- PROPOSED FIXED APPROACH (OpenSHC-style) ---" << std::endl;

    // Simulamos valores de timing correctos (como en OpenSHC)
    double control_frequency = model.getParams().control_frequency; // 50Hz
    double time_delta = 1.0 / control_frequency;                    // 0.02s

    // Simular swing timing como OpenSHC
    int swing_iterations = 50;                             // Ejemplo: 50 iteraciones para swing completo
    double swing_delta_t = 1.0 / (swing_iterations / 2.0); // Para cada mitad de la curva

    std::cout << "Fixed timing - swing_delta_t: " << swing_delta_t << ", time_delta: " << time_delta << std::endl;

    // Calcular separación de nodos usando fórmula de OpenSHC
    Point3D stance_node_separation = linear_velocity * 0.25 * (time_delta / swing_delta_t);
    std::cout << "Fixed stance_node_separation: (" << stance_node_separation.x << ", " << stance_node_separation.y << ", " << stance_node_separation.z << ")" << std::endl;

    // Calcular posición objetivo corregida
    Point3D target_tip_pose = initial_position + stride_vector * 0.5;
    std::cout << "Fixed target_tip_pose: (" << target_tip_pose.x << ", " << target_tip_pose.y << ", " << target_tip_pose.z << ")" << std::endl;

    // Generar nodos de control siguiendo OpenSHC
    Point3D mid_tip_position = (initial_position + target_tip_pose) * 0.5;
    mid_tip_position.z = std::max(initial_position.z, target_tip_pose.z) + swing_clearance.z;

    // Aplicar desplazamiento lateral como OpenSHC
    double mid_lateral_shift = 10.0; // LEG_STEPPER_SWING_LATERAL_SHIFT
    bool positive_y_axis = (stepper.getIdentityTipPose().y > 0.0);
    mid_tip_position.y += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;

    // Nodos de control corregidos
    Point3D fixed_swing_1_nodes[5];
    fixed_swing_1_nodes[0] = initial_position;
    fixed_swing_1_nodes[1] = initial_position + stance_node_separation;
    fixed_swing_1_nodes[2] = initial_position + stance_node_separation * 2.0;
    fixed_swing_1_nodes[3] = (mid_tip_position + fixed_swing_1_nodes[2]) * 0.5;
    fixed_swing_1_nodes[3].z = mid_tip_position.z;
    fixed_swing_1_nodes[4] = mid_tip_position;

    std::cout << "Primary swing control nodes (Fixed):" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "  Node " << i << ": (" << fixed_swing_1_nodes[i].x << ", " << fixed_swing_1_nodes[i].y << ", " << fixed_swing_1_nodes[i].z << ")" << std::endl;
    }

    // Simulamos trayectoria usando enfoque incremental (OpenSHC)
    std::cout << "\n=== SIMULATED INCREMENTAL TRAJECTORY (OpenSHC-style) ===" << std::endl;
    std::cout << "Step | Progress | Position (x, y, z) | Delta (x, y, z) | Z Elevation" << std::endl;
    std::cout << "-----+----------+-------------------+-----------------+------------" << std::endl;

    Point3D simulated_position = initial_position;

    for (int step = 0; step <= 20; step++) {
        double progress = step / 20.0;
        double time_input;
        Point3D delta_pos;

        if (progress < 0.5) {
            // Primera mitad - usar fixed_swing_1_nodes
            time_input = progress * 2.0; // Mapear [0,0.5] a [0,1]
            delta_pos = math_utils::quarticBezierDot(fixed_swing_1_nodes, time_input) * swing_delta_t * time_delta;
        } else {
            // Segunda mitad - generar nodos secondary
            Point3D final_tip_velocity = stride_vector * -1.0;
            Point3D secondary_separation = final_tip_velocity * 0.25 * (time_delta / swing_delta_t);

            Point3D fixed_swing_2_nodes[5];
            fixed_swing_2_nodes[0] = fixed_swing_1_nodes[4];
            fixed_swing_2_nodes[1] = fixed_swing_1_nodes[4] - (fixed_swing_1_nodes[3] - fixed_swing_1_nodes[4]);
            fixed_swing_2_nodes[2] = target_tip_pose - secondary_separation * 2.0;
            fixed_swing_2_nodes[3] = target_tip_pose - secondary_separation;
            fixed_swing_2_nodes[4] = target_tip_pose;

            time_input = (progress - 0.5) * 2.0; // Mapear [0.5,1] a [0,1]
            delta_pos = math_utils::quarticBezierDot(fixed_swing_2_nodes, time_input) * swing_delta_t * time_delta;
        }

        simulated_position.x += delta_pos.x;
        simulated_position.y += delta_pos.y;
        simulated_position.z += delta_pos.z;
        double z_elevation = simulated_position.z - initial_position.z;

        printf("%4d | %8.3f | (%8.3f, %8.3f, %8.3f) | (%6.3f, %6.3f, %6.3f) | %8.3f\n",
               step, progress, simulated_position.x, simulated_position.y, simulated_position.z,
               delta_pos.x, delta_pos.y, delta_pos.z, z_elevation);
    }

    // Verificar que las posiciones iniciales y finales son correctas
    Point3D final_pos = simulated_position;
    std::cout << "\nSimulated trajectory validation:" << std::endl;
    std::cout << "Start position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;
    std::cout << "End position: (" << final_pos.x << ", " << final_pos.y << ", " << final_pos.z << ")" << std::endl;

    double z_diff = std::abs(initial_position.z - final_pos.z);
    std::cout << "Z difference: " << z_diff << " mm" << std::endl;

    if (z_diff < 5.0) {
        std::cout << "✅ Z positions approximately match (start ≈ end)" << std::endl;
    } else {
        std::cout << "❌ Z positions don't match" << std::endl;
    }
}

int main() {
    std::cout << "=== OpenSHC Comparison Analysis ===" << std::endl;

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

    // Get leg's identity pose from BodyPoseConfiguration
    const LegStancePosition leg_stance_position = pose_config.leg_stance_positions[0];
    Point3D identity_tip_pose = Point3D(
        leg_stance_position.x,
        leg_stance_position.y,
        leg_stance_position.z);

    // Create required objects for LegStepper
    WalkspaceAnalyzer walkspace_analyzer(model);
    WorkspaceValidator workspace_validator(model);

    // Create LegStepper for leg 0
    LegStepper stepper(0, identity_tip_pose, test_leg, model, &walkspace_analyzer, &workspace_validator);
    stepper.setDefaultTipPose(identity_tip_pose);

    testOriginalVsFixedApproach(stepper, test_leg, model);

    return 0;
}
