#include "../src/robot_model.h"
#include "../src/velocity_limits.h"
#include "../src/workspace_analyzer.h"
#include <iomanip>
#include <iostream>

/**
 * @brief Test completo para verificar que WorkspaceAnalyzer, VelocityLimits y RobotModel::makeReachable
 *        consideran correctamente la peculiaridad física del robot donde z = -208 mm
 */
int main() {
    std::cout << "=== TEST COMPLETO: Offset físico z = -208 mm para TODAS las patas ===" << std::endl;

    // Configurar parámetros del robot según las especificaciones físicas
    Parameters params;
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    params.robot_height = 208;
    params.standing_height = 150;
    params.control_frequency = 50;
    params.coxa_angle_limits[0] = -65;
    params.coxa_angle_limits[1] = 65;
    params.femur_angle_limits[0] = -75;
    params.femur_angle_limits[1] = 75;
    params.tibia_angle_limits[0] = -45;
    params.tibia_angle_limits[1] = 45;

    std::cout << "Parámetros del robot:" << std::endl;
    std::cout << "  - Longitud tibia: " << params.tibia_length << " mm" << std::endl;
    std::cout << "  - Posición física de referencia: z = -" << params.tibia_length << " mm" << std::endl;
    std::cout << "  - Número de patas: " << NUM_LEGS << std::endl;

    // Crear modelo del robot
    RobotModel model(params);

    // ========================================================================
    // SECCIÓN 1: Tests de WorkspaceAnalyzer y VelocityLimits
    // ========================================================================

    std::cout << "\n=== SECCIÓN 1: WorkspaceAnalyzer y VelocityLimits ===" << std::endl;

    ValidationConfig validation_config;
    WorkspaceAnalyzer analyzer(model, ComputeConfig::medium(), validation_config);
    analyzer.initialize();

    VelocityLimits velocity_limits(model);

    // Test 1.1: Verificar offset de altura física
    double analyzer_reference_height = analyzer.getPhysicalReferenceHeight();
    double velocity_reference_height = velocity_limits.getPhysicalReferenceHeight();

    std::cout << "\n--- Test 1.1: Offset de altura física ---" << std::endl;
    std::cout << "WorkspaceAnalyzer - Altura de referencia: " << analyzer_reference_height << " mm" << std::endl;
    std::cout << "VelocityLimits - Altura de referencia: " << velocity_reference_height << " mm" << std::endl;

    bool analyzer_offset_ok = std::abs(analyzer_reference_height - (-params.tibia_length)) < 0.001;
    bool velocity_offset_ok = std::abs(velocity_reference_height - (-params.tibia_length)) < 0.001;

    if (analyzer_offset_ok && velocity_offset_ok) {
        std::cout << "✓ Ambos componentes consideran correctamente el offset físico" << std::endl;
    } else {
        std::cout << "✗ ERROR: Offset físico incorrecto" << std::endl;
    }

    // Test 1.2: Verificar workspaces centrados correctamente para todas las patas
    std::cout << "\n--- Test 1.2: Workspaces centrados para todas las patas ---" << std::endl;

    bool all_workspaces_centered = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        WorkspaceBounds bounds = analyzer.getWorkspaceBounds(leg);
        double actual_center = (bounds.min_height + bounds.max_height) / 2.0;
        double expected_center = -params.tibia_length;

        std::cout << "Pata " << leg << ": Centro = " << std::fixed << std::setprecision(1)
                  << actual_center << " mm (esperado: " << expected_center << " mm)";

        if (std::abs(actual_center - expected_center) < 1.0) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            all_workspaces_centered = false;
        }
    }

    if (all_workspaces_centered) {
        std::cout << "✓ Todos los workspaces están correctamente centrados" << std::endl;
    } else {
        std::cout << "✗ ERROR: Algunos workspaces no están centrados correctamente" << std::endl;
    }

    // ========================================================================
    // SECCIÓN 2: Tests de RobotModel::makeReachable para todas las patas
    // ========================================================================

    std::cout << "\n=== SECCIÓN 2: RobotModel::makeReachable para todas las patas ===" << std::endl;

    // Test 2.1: makeReachable en altura de referencia física para todas las patas
    std::cout << "\n--- Test 2.1: makeReachable en altura de referencia física ---" << std::endl;

    bool all_legs_reachable = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        // Posición objetivo en la altura de referencia física, a una distancia moderada
        Point3D target_position(leg_base.x + 100.0, leg_base.y + 50.0, -208.0);
        Point3D reachable_position = model.makeReachable(leg, target_position);

        // Verificar que la posición es alcanzable usando cinemática inversa
        JointAngles zero_angles(0, 0, 0);
        try {
            JointAngles ik_result = model.inverseKinematicsCurrentGlobalCoordinates(leg, zero_angles, reachable_position);
            bool within_limits = model.checkJointLimits(leg, ik_result);

            std::cout << "Pata " << leg << ": Base(" << std::fixed << std::setprecision(1)
                      << leg_base.x << ", " << leg_base.y << ", " << leg_base.z
                      << ") -> Alcanzable(" << reachable_position.x << ", "
                      << reachable_position.y << ", " << reachable_position.z << ")";

            if (within_limits) {
                std::cout << " ✓" << std::endl;
            } else {
                std::cout << " ✗ (fuera de límites)" << std::endl;
                all_legs_reachable = false;
            }
        } catch (...) {
            std::cout << "Pata " << leg << ": ✗ (error en IK)" << std::endl;
            all_legs_reachable = false;
        }
    }

    if (all_legs_reachable) {
        std::cout << "✓ makeReachable funciona correctamente para todas las patas" << std::endl;
    } else {
        std::cout << "✗ ERROR: makeReachable falla en algunas patas" << std::endl;
    }

    // Test 2.2: Constrañimiento de posiciones inalcanzables para todas las patas
    std::cout << "\n--- Test 2.2: Constrañimiento de posiciones inalcanzables ---" << std::endl;

    bool all_constraints_work = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        // Posición muy lejana (inalcanzable)
        Point3D unreachable_target(leg_base.x + 500.0, leg_base.y + 500.0, -208.0);
        Point3D constrained_position = model.makeReachable(leg, unreachable_target);

        // Calcular distancias
        double original_distance = sqrt(pow(unreachable_target.x - leg_base.x, 2) +
                                        pow(unreachable_target.y - leg_base.y, 2) +
                                        pow(unreachable_target.z - leg_base.z, 2));
        double constrained_distance = sqrt(pow(constrained_position.x - leg_base.x, 2) +
                                           pow(constrained_position.y - leg_base.y, 2) +
                                           pow(constrained_position.z - leg_base.z, 2));

        std::cout << "Pata " << leg << ": " << std::fixed << std::setprecision(1)
                  << original_distance << " mm -> " << constrained_distance << " mm";

        if (constrained_distance < original_distance) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            all_constraints_work = false;
        }
    }

    if (all_constraints_work) {
        std::cout << "✓ Constrañimiento funciona correctamente para todas las patas" << std::endl;
    } else {
        std::cout << "✗ ERROR: Constrañimiento falla en algunas patas" << std::endl;
    }

    // Test 2.3: Mantenimiento de alturas considerando offset físico
    std::cout << "\n--- Test 2.3: Mantenimiento de alturas con offset físico ---" << std::endl;

    double test_heights[] = {-308.0, -258.0, -208.0, -158.0, -108.0}; // Diferentes alturas
    bool all_heights_maintained = true;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);
        std::cout << "Pata " << leg << ": ";

        for (double height : test_heights) {
            Point3D test_target(leg_base.x + 100.0, leg_base.y + 50.0, height);
            Point3D result = model.makeReachable(leg, test_target);

            // Verificar que la altura se mantiene o se ajusta apropiadamente
            if (std::abs(result.z - test_target.z) > 5.0) { // Tolerancia de 5mm
                all_heights_maintained = false;
            }
        }
        std::cout << "✓" << std::endl;
    }

    if (all_heights_maintained) {
        std::cout << "✓ Alturas mantenidas correctamente para todas las patas" << std::endl;
    } else {
        std::cout << "✗ ERROR: Problemas con mantenimiento de alturas" << std::endl;
    }

    // ========================================================================
    // SECCIÓN 3: Test de coordinación entre componentes
    // ========================================================================

    std::cout << "\n=== SECCIÓN 3: Coordinación entre componentes ===" << std::endl;

    // Test 3.1: Verificar que makeReachable usa el workspace correctamente
    std::cout << "\n--- Test 3.1: Coordinación makeReachable y WorkspaceAnalyzer ---" << std::endl;

    bool coordination_works = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        // Obtener el workplane para la altura de referencia
        auto workplane = analyzer.getWorkplane(leg, -208.0);

        if (!workplane.empty()) {
            // Buscar el radio máximo en una dirección específica (0°)
            auto it = workplane.find(0);
            if (it != workplane.end() && it->second > 0) {
                // Crear target justo en el límite del workspace
                Point3D target_at_limit(leg_base.x + it->second, leg_base.y, -208.0);
                Point3D reachable = model.makeReachable(leg, target_at_limit);

                // La posición debería ser alcanzable sin cambios significativos
                double distance_change = sqrt(pow(reachable.x - target_at_limit.x, 2) +
                                              pow(reachable.y - target_at_limit.y, 2));

                if (distance_change < 10.0) { // Tolerancia de 10mm
                    continue;                 // Esta pata está bien
                }
            }
        }
        coordination_works = false;
        break;
    }

    if (coordination_works) {
        std::cout << "✓ makeReachable coordina correctamente con WorkspaceAnalyzer" << std::endl;
    } else {
        std::cout << "✗ ERROR: Problemas de coordinación entre componentes" << std::endl;
    }

    // ========================================================================
    // RESUMEN FINAL
    // ========================================================================

    std::cout << "\n=== RESUMEN FINAL ===" << std::endl;

    int passed_tests = 0;
    int total_tests = 6;

    if (analyzer_offset_ok && velocity_offset_ok)
        passed_tests++;
    if (all_workspaces_centered)
        passed_tests++;
    if (all_legs_reachable)
        passed_tests++;
    if (all_constraints_work)
        passed_tests++;
    if (all_heights_maintained)
        passed_tests++;
    if (coordination_works)
        passed_tests++;

    std::cout << "Tests pasados: " << passed_tests << "/" << total_tests << std::endl;

    if (passed_tests == total_tests) {
        std::cout << "🎉 ¡TODOS LOS TESTS PASARON! 🎉" << std::endl;
        std::cout << "El sistema considera correctamente la peculiaridad física del robot" << std::endl;
        std::cout << "donde z = -208 mm es la posición de referencia cuando todos los ángulos son 0°." << std::endl;
    } else {
        std::cout << "❌ ALGUNOS TESTS FALLARON ❌" << std::endl;
        std::cout << "Revisar los componentes que no consideran correctamente el offset físico." << std::endl;
    }

    std::cout << "\n=== FIN DEL TEST COMPLETO ===" << std::endl;

    return (passed_tests == total_tests) ? 0 : 1;
}
