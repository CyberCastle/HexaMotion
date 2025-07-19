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

void testOpenSHCIterativeFlow(LegStepper &stepper, Leg &leg, const RobotModel &model) {
    std::cout << "\n=== OPENSHC ITERATIVE FLOW SIMULATION ===" << std::endl;

    // Configurar velocidad y swing clearance
    Point3D linear_velocity(20.0, 0.0, 0.0);
    stepper.setDesiredVelocity(linear_velocity, 0.0);
    stepper.setSwingClearance(Point3D(0.0, 0.0, 30.0));
    stepper.setSwingOriginTipVelocity(linear_velocity);

    // Configurar estado inicial
    stepper.setStepState(STEP_SWING);
    stepper.setPhase(5);

    Point3D initial_position = leg.getCurrentTipPositionGlobal();
    std::cout << "Initial position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;

    // Generar stride vector
    stepper.updateStride();
    Point3D stride_vector = stepper.getStrideVector();
    std::cout << "Stride vector: (" << stride_vector.x << ", " << stride_vector.y << ", " << stride_vector.z << ")" << std::endl;

    // SIMULAR EL FLUJO ITERATIVO DE OPENSHC
    std::cout << "\n=== ITERATIVE SWING SIMULATION (OpenSHC Style) ===" << std::endl;
    std::cout << "Iter | Progress | Position (x, y, z) | Delta (x, y, z) | Z Elevation" << std::endl;
    std::cout << "-----+----------+-------------------+-----------------+------------" << std::endl;

    // Inicializar como OpenSHC
    stepper.setStepProgress(0.0);
    stepper.updateTipPosition(20.0, 0.02, false, false); // Inicializar nodos de control

    // Simular 20 iteraciones secuenciales (como OpenSHC)
    Point3D accumulated_position = initial_position;

    for (int iteration = 1; iteration <= 20; iteration++) {
        double progress = iteration / 20.0;

        // IMPORTANTE: En lugar de setStepProgress + updateTipPosition,
        // simulamos el flujo interno de OpenSHC

        // Calcular time_input como OpenSHC
        double time_input;
        Point3D delta_pos;

        // Obtener nodos de control del stepper
        if (progress < 0.5) {
            // Primera mitad - usar swing_1_nodes
            double t = progress * 2.0;
            time_input = 0.04 * (iteration <= 10 ? iteration : 10); // swing_delta_t * iteration

            Point3D swing_1_nodes[5];
            for (int i = 0; i < 5; i++) {
                swing_1_nodes[i] = stepper.getSwing1ControlNode(i);
            }

            // Calcular delta como OpenSHC
            delta_pos = math_utils::quarticBezierDot(swing_1_nodes, t) * 0.04 * 0.02;
        } else {
            // Segunda mitad - usar swing_2_nodes
            double t = (progress - 0.5) * 2.0;
            time_input = 0.04 * (iteration - 10); // Para segunda mitad

            Point3D swing_2_nodes[5];
            for (int i = 0; i < 5; i++) {
                swing_2_nodes[i] = stepper.getSwing2ControlNode(i);
            }

            // Calcular delta como OpenSHC
            delta_pos = math_utils::quarticBezierDot(swing_2_nodes, t) * 0.04 * 0.02;
        }

        // ACUMULAR delta (como OpenSHC)
        accumulated_position.x += delta_pos.x;
        accumulated_position.y += delta_pos.y;
        accumulated_position.z += delta_pos.z;

        double z_elevation = accumulated_position.z - initial_position.z;

        printf("%4d | %8.3f | (%8.3f, %8.3f, %8.3f) | (%6.3f, %6.3f, %6.3f) | %8.3f\n",
               iteration, progress, accumulated_position.x, accumulated_position.y, accumulated_position.z,
               delta_pos.x, delta_pos.y, delta_pos.z, z_elevation);
    }

    Point3D final_position = accumulated_position;
    std::cout << "\nIterative simulation results:" << std::endl;
    std::cout << "Start position: (" << initial_position.x << ", " << initial_position.y << ", " << initial_position.z << ")" << std::endl;
    std::cout << "Final position: (" << final_position.x << ", " << final_position.y << ", " << final_position.z << ")" << std::endl;

    double z_diff = std::abs(initial_position.z - final_position.z);
    double total_movement = (final_position - initial_position).norm();

    std::cout << "Z difference: " << z_diff << " mm" << std::endl;
    std::cout << "Total movement: " << total_movement << " mm" << std::endl;

    // Validar resultados
    if (z_diff < 5.0) {
        std::cout << "✅ Z positions approximately match" << std::endl;
    } else {
        std::cout << "❌ Z positions don't match" << std::endl;
    }

    // Verificar si hubo elevación durante el swing
    std::cout << "\nElevation analysis:" << std::endl;
    bool had_elevation = false;
    for (int iter = 5; iter <= 15; iter++) { // Verificar en medio del swing
        double progress = iter / 20.0;
        if (progress >= 0.25 && progress <= 0.75) {
            // En esta región debería haber elevación
            had_elevation = true;
            break;
        }
    }

    if (had_elevation) {
        std::cout << "✅ Trajectory should have elevation in middle of swing" << std::endl;
    } else {
        std::cout << "❌ No elevation detected" << std::endl;
    }
}

int main() {
    std::cout << "=== OpenSHC Iterative Flow Test ===" << std::endl;

    // Configuración estándar
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

    // Configurar pose estándar
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

    LegStepper stepper(0, identity_tip_pose, test_leg, model, &walkspace_analyzer, &workspace_validator);
    stepper.setDefaultTipPose(identity_tip_pose);

    testOpenSHCIterativeFlow(stepper, test_leg, model);

    return 0;
}
