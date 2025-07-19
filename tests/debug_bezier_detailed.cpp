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

void debugBezierGeneration(LegStepper &stepper, Leg &leg, const RobotModel &model) {
    std::cout << "\n=== BEZIER GENERATION DEBUG ===" << std::endl;

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
    std::cout << "Linear velocity: (" << linear_velocity.x << ", " << linear_velocity.y << ", " << linear_velocity.z << ")" << std::endl;
    std::cout << "Swing clearance: (" << swing_clearance.x << ", " << swing_clearance.y << ", " << swing_clearance.z << ")" << std::endl;

    // Llamar updateStride para generar el stride vector
    stepper.updateStride();
    Point3D stride_vector = stepper.getStrideVector();
    std::cout << "Stride vector: (" << stride_vector.x << ", " << stride_vector.y << ", " << stride_vector.z << ")" << std::endl;

    // Ahora generar control nodes
    stepper.updateTipPosition(20.0, 0.02, false, false);

    // Mostrar información detallada
    std::cout << "\nAfter updateTipPosition:" << std::endl;
    std::cout << "Identity tip pose: (" << stepper.getIdentityTipPose().x << ", " << stepper.getIdentityTipPose().y << ", " << stepper.getIdentityTipPose().z << ")" << std::endl;
    std::cout << "Default tip pose: (" << stepper.getDefaultTipPose().x << ", " << stepper.getDefaultTipPose().y << ", " << stepper.getDefaultTipPose().z << ")" << std::endl;
    std::cout << "Target tip pose: (" << stepper.getTargetTipPose().x << ", " << stepper.getTargetTipPose().y << ", " << stepper.getTargetTipPose().z << ")" << std::endl;

    // Mostrar nodos de control generados
    std::cout << "\nPrimary swing control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing1ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    std::cout << "\nSecondary swing control nodes:" << std::endl;
    for (int i = 0; i < 5; i++) {
        Point3D node = stepper.getSwing2ControlNode(i);
        std::cout << "  Node " << i << ": (" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
    }

    // Generar trayectoria completa
    std::cout << "\n=== COMPLETE SWING TRAJECTORY ===" << std::endl;
    std::cout << "Step | Progress | Position (x, y, z) | Curve Used | Z Change from Initial" << std::endl;
    std::cout << "-----+----------+-------------------+------------+---------------------" << std::endl;

    for (int step = 0; step <= 20; step++) {
        double progress = step / 20.0;
        stepper.setStepProgress(progress);
        stepper.updateTipPosition(20.0, 0.02, false, false);

        Point3D pos = leg.getCurrentTipPositionGlobal();
        std::string curve_used = (progress < 0.5) ? "Primary" : "Secondary";
        double z_change = pos.z - initial_position.z;

        printf("%4d | %8.3f | (%8.3f, %8.3f, %8.3f) | %s | %8.3f\n",
               step, progress, pos.x, pos.y, pos.z, curve_used.c_str(), z_change);
    }

    // Verificar que las posiciones iniciales y finales son correctas
    stepper.setStepProgress(0.0);
    stepper.updateTipPosition(20.0, 0.02, false, false);
    Point3D start_pos = leg.getCurrentTipPositionGlobal();

    stepper.setStepProgress(1.0);
    stepper.updateTipPosition(20.0, 0.02, false, false);
    Point3D end_pos = leg.getCurrentTipPositionGlobal();

    std::cout << "\nTrajectory validation:" << std::endl;
    std::cout << "Start position: (" << start_pos.x << ", " << start_pos.y << ", " << start_pos.z << ")" << std::endl;
    std::cout << "End position: (" << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << ")" << std::endl;

    double z_diff = std::abs(start_pos.z - end_pos.z);
    std::cout << "Z difference: " << z_diff << " mm" << std::endl;

    if (z_diff < 1.0) {
        std::cout << "✅ Z positions match (start == end)" << std::endl;
    } else {
        std::cout << "❌ Z positions don't match" << std::endl;
    }

    // Verificar que la trayectoria tiene elevación
    bool has_elevation = false;
    for (int step = 1; step <= 19; step++) {
        double progress = step / 20.0;
        stepper.setStepProgress(progress);
        stepper.updateTipPosition(20.0, 0.02, false, false);

        Point3D pos = leg.getCurrentTipPositionGlobal();
        if (pos.z > initial_position.z + 5.0) { // Al menos 5mm de elevación
            has_elevation = true;
            break;
        }
    }

    if (has_elevation) {
        std::cout << "✅ Trajectory has elevation during swing" << std::endl;
    } else {
        std::cout << "❌ Trajectory is flat (no elevation)" << std::endl;
    }
}

int main() {
    std::cout << "=== Detailed Bezier Debug Test ===" << std::endl;

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

    debugBezierGeneration(stepper, test_leg, model);

    return 0;
}
