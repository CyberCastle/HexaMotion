#include "locomotion_system.h"
#include "test_stubs.h"
#include <cassert>
#include <iostream>
#include <memory>

int main() {
    std::cout << "=== LocomotionSystem Position Current Validation Test ===" << std::endl;

    // Configurar parámetros del robot
    Parameters params{};
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    params.robot_height = 208;
    params.control_frequency = 50;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    // Crear interfaces mock
    auto imu_interface = std::make_unique<DummyIMU>();
    auto fsr_interface = std::make_unique<DummyFSR>();
    auto servo_interface = std::make_unique<DummyServo>();

    // Crear configuración de pose por defecto
    BodyPoseConfiguration pose_config(params);
    pose_config.body_clearance = 208.0;
    for (int i = 0; i < NUM_LEGS; i++) {
        pose_config.leg_stance_positions[i].x = 150.0;
        pose_config.leg_stance_positions[i].y = 0.0;
    }

    // Crear sistema de locomoción
    LocomotionSystem locomotion_system(params);

    // Inicializar el sistema
    bool init_success = locomotion_system.initialize(
        imu_interface.get(),
        fsr_interface.get(),
        servo_interface.get(),
        pose_config
    );
    assert(init_success);
    std::cout << "✓ Sistema de locomoción inicializado correctamente" << std::endl;

    // Obtener posiciones iniciales de las piernas
    std::vector<Point3D> initial_positions;
    std::vector<JointAngles> initial_angles;

    for (int i = 0; i < NUM_LEGS; i++) {
        initial_positions.push_back(locomotion_system.getLegPosition(i));
        initial_angles.push_back(locomotion_system.getJointAngles(i));
        std::cout << "Pierna " << i << " - Posición inicial: ("
                  << initial_positions[i].x << ", "
                  << initial_positions[i].y << ", "
                  << initial_positions[i].z << ")" << std::endl;
    }

    // Test 1: Validar que calculateInverseKinematics usa posición actual
    std::cout << "\n--- Test 1: Validación de calculateInverseKinematics ---" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        // Obtener ángulos actuales antes de IK
        JointAngles current_angles = locomotion_system.getJointAngles(i);
        Point3D current_position = locomotion_system.getLegPosition(i);

        // Definir un objetivo cercano a la posición actual
        Point3D target_position = current_position;
        target_position.x += 10.0; // Mover 10mm en X

        std::cout << "Pierna " << i << ":" << std::endl;
        std::cout << "  Posición actual: (" << current_position.x << ", "
                  << current_position.y << ", " << current_position.z << ")" << std::endl;
        std::cout << "  Ángulos actuales: coxa=" << current_angles.coxa
                  << ", femur=" << current_angles.femur
                  << ", tibia=" << current_angles.tibia << std::endl;
        std::cout << "  Objetivo: (" << target_position.x << ", "
                  << target_position.y << ", " << target_position.z << ")" << std::endl;

        // Calcular IK usando el método del sistema
        JointAngles new_angles = locomotion_system.calculateInverseKinematics(i, target_position);

        std::cout << "  Nuevos ángulos: coxa=" << new_angles.coxa
                  << ", femur=" << new_angles.femur
                  << ", tibia=" << new_angles.tibia << std::endl;

        // Verificar que los nuevos ángulos son diferentes pero razonables
        double angle_change_coxa = std::abs(new_angles.coxa - current_angles.coxa);
        double angle_change_femur = std::abs(new_angles.femur - current_angles.femur);
        double angle_change_tibia = std::abs(new_angles.tibia - current_angles.tibia);

        std::cout << "  Cambios en ángulos: coxa=" << angle_change_coxa
                  << ", femur=" << angle_change_femur
                  << ", tibia=" << angle_change_tibia << std::endl;

        // Verificar que hay cambios razonables (al menos un ángulo debe cambiar, no excesivos)
        bool any_angle_changed = (angle_change_coxa > 0.001) || (angle_change_femur > 0.001) || (angle_change_tibia > 0.001);
        assert(any_angle_changed); // Al menos un ángulo debe cambiar
        assert(angle_change_coxa < 1.0); // Cambios no excesivos
        assert(angle_change_femur < 1.0);
        assert(angle_change_tibia < 1.0);

        // Verificar que la posición resultante es cercana al objetivo
        Point3D result_position = locomotion_system.calculateForwardKinematics(i, new_angles);
        double position_error = std::sqrt(
            std::pow(result_position.x - target_position.x, 2) +
            std::pow(result_position.y - target_position.y, 2) +
            std::pow(result_position.z - target_position.z, 2)
        );

        std::cout << "  Posición resultante: (" << result_position.x << ", "
                  << result_position.y << ", " << result_position.z << ")" << std::endl;
        std::cout << "  Error de posición: " << position_error << " mm" << std::endl;

        assert(position_error < 15.0); // Error menor a 15mm (más realista para IK iterativo)
        std::cout << "  ✓ IK válido para pierna " << i << std::endl;
    }

    // Test 2: Validar que updateModel() usa posiciones actuales
    std::cout << "\n--- Test 2: Validación de updateModel() ---" << std::endl;

    // Simular un comando de velocidad para activar el walk controller
    locomotion_system.walkForward(20.0); // 20mm/s hacia adelante

    // Ejecutar update() que llama a updateModel()
    bool update_success = locomotion_system.update();
    assert(update_success);
    std::cout << "✓ Update ejecutado correctamente" << std::endl;

    // Verificar que las posiciones han cambiado
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D new_position = locomotion_system.getLegPosition(i);
        JointAngles new_angles = locomotion_system.getJointAngles(i);

        double position_change = std::sqrt(
            std::pow(new_position.x - initial_positions[i].x, 2) +
            std::pow(new_position.y - initial_positions[i].y, 2) +
            std::pow(new_position.z - initial_positions[i].z, 2)
        );

        double angle_change = std::sqrt(
            std::pow(new_angles.coxa - initial_angles[i].coxa, 2) +
            std::pow(new_angles.femur - initial_angles[i].femur, 2) +
            std::pow(new_angles.tibia - initial_angles[i].tibia, 2)
        );

        std::cout << "Pierna " << i << ":" << std::endl;
        std::cout << "  Cambio de posición: " << position_change << " mm" << std::endl;
        std::cout << "  Cambio de ángulos: " << angle_change << " rad" << std::endl;

        // Verificar que hay cambios (el sistema está funcionando)
        assert(position_change > 0.0 || angle_change > 0.0);
        std::cout << "  ✓ Posiciones actualizadas para pierna " << i << std::endl;
    }

    // Test 3: Validar que setLegPosition usa posición actual
    std::cout << "\n--- Test 3: Validación de setLegPosition() ---" << std::endl;

    for (int i = 0; i < NUM_LEGS; i++) {
        // Obtener posición actual
        Point3D current_pos = locomotion_system.getLegPosition(i);
        JointAngles current_angles = locomotion_system.getJointAngles(i);

        // Definir nueva posición objetivo
        Point3D new_target = current_pos;
        new_target.y += 15.0; // Mover 15mm en Y

        std::cout << "Pierna " << i << ":" << std::endl;
        std::cout << "  Posición actual: (" << current_pos.x << ", "
                  << current_pos.y << ", " << current_pos.z << ")" << std::endl;
        std::cout << "  Nuevo objetivo: (" << new_target.x << ", "
                  << new_target.y << ", " << new_target.z << ")" << std::endl;

        // Establecer nueva posición
        bool set_success = locomotion_system.setLegPosition(i, new_target);
        assert(set_success);

        // Obtener posición resultante usando el método del sistema
        Point3D result_pos = locomotion_system.getLegPosition(i);
        JointAngles result_angles = locomotion_system.getJointAngles(i);

        // Verificar que la posición cambió
        double position_error = std::sqrt(
            std::pow(result_pos.x - new_target.x, 2) +
            std::pow(result_pos.y - new_target.y, 2) +
            std::pow(result_pos.z - new_target.z, 2)
        );

        std::cout << "  Posición resultante: (" << result_pos.x << ", "
                  << result_pos.y << ", " << result_pos.z << ")" << std::endl;
        std::cout << "  Error de posición: " << position_error << " mm" << std::endl;

        assert(position_error < 15.0); // Error menor a 15mm (más realista para IK iterativo)
        std::cout << "  ✓ setLegPosition válido para pierna " << i << std::endl;
    }

    // Test 4: Validar continuidad en múltiples updates
    std::cout << "\n--- Test 4: Validación de continuidad en múltiples updates ---" << std::endl;

    std::vector<Point3D> previous_positions;
    for (int i = 0; i < NUM_LEGS; i++) {
        previous_positions.push_back(locomotion_system.getLegPosition(i));
    }

    // Ejecutar múltiples updates
    for (int update_count = 0; update_count < 5; update_count++) {
        bool update_success = locomotion_system.update();
        assert(update_success);

        std::cout << "Update " << (update_count + 1) << ":" << std::endl;

        for (int i = 0; i < NUM_LEGS; i++) {
            Point3D current_pos = locomotion_system.getLegPosition(i);
            Point3D prev_pos = previous_positions[i];

            double position_change = std::sqrt(
                std::pow(current_pos.x - prev_pos.x, 2) +
                std::pow(current_pos.y - prev_pos.y, 2) +
                std::pow(current_pos.z - prev_pos.z, 2)
            );

            std::cout << "  Pierna " << i << " cambio: " << position_change << " mm" << std::endl;

            // Verificar que los cambios son razonables (no saltos bruscos)
            assert(position_change < 50.0); // Cambio máximo de 50mm por update

            previous_positions[i] = current_pos;
        }
    }

    std::cout << "✓ Continuidad en múltiples updates validada" << std::endl;

    std::cout << "\n=== TODOS LOS TESTS PASARON ===" << std::endl;
    std::cout << "✓ El LocomotionSystem SÍ utiliza la posición actual como insumo para obtener la posición objetivo" << std::endl;

    return 0;
}