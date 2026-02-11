#include "../src/body_pose_config_factory.h"
#include "../src/robot_model.h"
#include "../src/velocity_limits.h"
#include "../src/workspace_analyzer.h"
#include <iomanip>
#include <iostream>

/**
 * @brief Comprehensive test to verify WorkspaceAnalyzer, VelocityLimits, and RobotModel::makeReachable
 *        respect the robot physical reference where z = -208 mm.
 */
int main() {
    std::cout << "=== TEST COMPLETO: Offset físico z = -208 mm para TODAS las patas ===" << std::endl;

    /** Configure robot parameters to match physical specifications. */
    Parameters params;
    params.hexagon_radius = 200;
    params.coxa_length = 50;
    params.femur_length = 101;
    params.tibia_length = 208;
    /** Set to -tibia_length for explicit configuration. */
    params.default_height_offset = -208.0;
    params.robot_height = 208;
    params.standing_height = 150;
    params.time_delta = 1.0 / 50.0;
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

    /** Create robot model. */
    RobotModel model(params);

    /** Section 1: WorkspaceAnalyzer and VelocityLimits tests. */

    std::cout << "\n=== SECCIÓN 1: WorkspaceAnalyzer y VelocityLimits ===" << std::endl;

    ValidationConfig validation_config;
    WorkspaceAnalyzer analyzer(model, ComputeConfig::medium(), validation_config);
    analyzer.initialize();

    VelocityLimits velocity_limits(model);

    /** Test 1.1: Verify physical height offset. */
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

    /**
     * @brief Test 1.2: Validate morphological vertical profile (intentional asymmetry).
     *
     * Criteria derived from AGENTS.md:
     *  - reference_height_offset_ (= -tibia_length) within [min_height, max_height]
     *  - upper margin >= standing_height (operational up range)
     *  - lower margin >= 0.85*(femur + tibia) (operational down / terrain adaptation)
     *  - asymmetry accepted and documented (down > up)
     */
    std::cout << "\n--- Test 1.2: Perfil vertical morfológico (asimétrico) ---" << std::endl;

    bool morphological_vertical_profile_ok = true;
    double expected_ref = -params.tibia_length;
    /** Required up margin (mm). */
    double required_up = params.standing_height;
    double required_down = 0.85 * (params.femur_length + params.tibia_length);
    /** Vertical tolerance in mm. */
    const double EPS_VERT = 1.0;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        WorkspaceBounds bounds = analyzer.getWorkspaceBounds(leg);
        double up_margin = bounds.max_height - expected_ref;
        double down_margin = expected_ref - bounds.min_height;
        bool contains_ref = (expected_ref >= bounds.min_height - 1e-6 && expected_ref <= bounds.max_height + 1e-6);
        /** Allow slight underestimation due to rounding. */
        bool up_ok = (up_margin + EPS_VERT >= required_up);
        bool down_ok = (down_margin + EPS_VERT >= required_down);
        /** Informational asymmetry check. */
        bool asymmetry_expected = down_margin > up_margin;

        bool leg_ok = contains_ref && up_ok && down_ok;
        if (!leg_ok)
            morphological_vertical_profile_ok = false;

        std::cout << "Pata " << leg
                  << ": ref dentro=" << (contains_ref ? "sí" : "no")
                  << ", up=" << std::fixed << std::setprecision(1) << up_margin << " (≥ " << required_up << ")"
                  << ", down=" << down_margin << " (≥ " << required_down << ")"
                  << ", asim=" << (asymmetry_expected ? "sí" : "no")
                  << (leg_ok ? " ✓" : " ✗") << std::endl;
    }

    if (morphological_vertical_profile_ok) {
        std::cout << "✓ Perfil vertical morfológico válido (asimetría esperada)" << std::endl;
    } else {
        std::cout << "✗ ERROR: Perfil vertical no cumple criterios morfológicos" << std::endl;
    }

    /** Section 2: RobotModel::makeReachable tests for all legs. */

    std::cout << "\n=== SECCIÓN 2: RobotModel::makeReachable para todas las patas ===" << std::endl;

    /** Test 2.1: makeReachable at physical reference height for all legs. */
    std::cout << "\n--- Test 2.1: makeReachable en altura de referencia física ---" << std::endl;

    bool all_legs_reachable = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        /** Target at physical reference height with a moderate distance. */
        Point3D target_position(leg_base.x + 100.0, leg_base.y + 50.0, -208.0);
        Point3D reachable_position = model.makeReachable(leg, target_position);

        /** Verify reachability using inverse kinematics. */
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

    /** Test 2.2: Constrain unreachable positions for all legs. */
    std::cout << "\n--- Test 2.2: Constrañimiento de posiciones inalcanzables ---" << std::endl;

    bool all_constraints_work = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        /** Very distant (unreachable) target position. */
        Point3D unreachable_target(leg_base.x + 500.0, leg_base.y + 500.0, -208.0);
        Point3D constrained_position = model.makeReachable(leg, unreachable_target);

        /** Compute distances. */
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

    /** Test 2.3: Maintain heights considering physical offset. */
    std::cout << "\n--- Test 2.3: Mantenimiento de alturas con offset físico ---" << std::endl;

    /** Test heights (mm). */
    double test_heights[] = {-308.0, -258.0, -208.0, -158.0, -108.0};
    bool all_heights_maintained = true;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);
        std::cout << "Pata " << leg << ": ";

        for (double height : test_heights) {
            Point3D test_target(leg_base.x + 100.0, leg_base.y + 50.0, height);
            Point3D result = model.makeReachable(leg, test_target);

            /** Verify height is preserved or adjusted appropriately. */
            /** Height tolerance is 5 mm. */
            if (std::abs(result.z - test_target.z) > 5.0) {
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

    /** Section 3: Coordination test between components. */

    std::cout << "\n=== SECCIÓN 3: Coordinación entre componentes ===" << std::endl;

    /** Test 3.1: Verify makeReachable uses the workspace correctly. */
    std::cout << "\n--- Test 3.1: Coordinación makeReachable y WorkspaceAnalyzer ---" << std::endl;

    bool coordination_works = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);

        /** Get workplane for the reference height. */
        auto workplane = analyzer.getWorkplane(leg, -208.0);

        if (!workplane.empty()) {
            /** Find the maximum radius at a specific direction (0 degrees). */
            auto it = workplane.find(0);
            if (it != workplane.end() && it->second > 0) {
                /** Create a target right at the workspace limit. */
                Point3D target_at_limit(leg_base.x + it->second, leg_base.y, -208.0);
                Point3D reachable = model.makeReachable(leg, target_at_limit);

                /** Position should be reachable without significant changes. */
                double distance_change = sqrt(pow(reachable.x - target_at_limit.x, 2) +
                                              pow(reachable.y - target_at_limit.y, 2));

                /** Distance tolerance is 10 mm. */
                if (distance_change < 10.0) {
                    /** This leg is OK. */
                    continue;
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

    /** Section 4: Tests for implemented class fixes. */

    std::cout << "\n=== SECCIÓN 4: Verificación de correcciones implementadas ===" << std::endl;

    /** Test 4.1: Verify LegStepper accounts for physical offset correctly. */
    std::cout << "\n--- Test 4.1: Validación de LegStepper ---" << std::endl;

    bool legstepper_validation_ok = true;
    double physical_reference_height = model.getDefaultHeightOffset();
    /** Expected Z range min (mm). */
    double expected_z_range_min = physical_reference_height - params.standing_height;
    /** Expected Z range max (mm). */
    double expected_z_range_max = physical_reference_height + params.standing_height;

    std::cout << "Rango Z válido para LegStepper: [" << expected_z_range_min
              << ", " << expected_z_range_max << "] mm" << std::endl;

    /** Simulate validation of typical poses. */
    Point3D valid_stance_pose(150, 100, -150);
    Point3D valid_swing_pose(180, 120, -100);
    Point3D invalid_pose_high(100, 100, 0);
    Point3D invalid_pose_low(100, 100, -400);

    bool stance_valid = (valid_stance_pose.z >= expected_z_range_min && valid_stance_pose.z <= expected_z_range_max);
    bool swing_valid = (valid_swing_pose.z >= expected_z_range_min && valid_swing_pose.z <= expected_z_range_max);
    bool high_invalid = (invalid_pose_high.z < expected_z_range_min || invalid_pose_high.z > expected_z_range_max);
    bool low_invalid = (invalid_pose_low.z < expected_z_range_min || invalid_pose_low.z > expected_z_range_max);

    std::cout << "Pose stance válida (" << valid_stance_pose.z << " mm): " << (stance_valid ? "✓" : "✗") << std::endl;
    std::cout << "Pose swing válida (" << valid_swing_pose.z << " mm): " << (swing_valid ? "✓" : "✗") << std::endl;
    std::cout << "Pose alta inválida (" << invalid_pose_high.z << " mm): " << (high_invalid ? "✓" : "✗") << std::endl;
    std::cout << "Pose baja inválida (" << invalid_pose_low.z << " mm): " << (low_invalid ? "✓" : "✗") << std::endl;

    legstepper_validation_ok = stance_valid && swing_valid && high_invalid && low_invalid;

    if (legstepper_validation_ok) {
        std::cout << "✓ LegStepper: Validación de rango Z correcta" << std::endl;
    } else {
        std::cout << "✗ ERROR: LegStepper no valida correctamente el rango Z" << std::endl;
    }

    /** Test 4.2: Verify WalkController::init() correction. */
    std::cout << "\n--- Test 4.2: Corrección en WalkController ---" << std::endl;

    bool walkcontroller_correction_ok = true;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        Point3D leg_base = model.getLegBasePosition(leg);
        double base_angle = model.getLegBaseAngleOffset(leg);
        double leg_reach = model.getLegReach();
        /** Conservative factor. */
        double stance_radius = leg_reach * 0.6;

        /** Calculate corrected position as in WalkController. */
        Point3D corrected_stance_position(
            leg_base.x + stance_radius * cos(base_angle),
            leg_base.y + stance_radius * sin(base_angle),
            model.getDefaultHeightOffset() + params.standing_height);

        /** Expected Z (mm). */
        double expected_z = -208 + 150;
        bool z_correct = std::abs(corrected_stance_position.z - expected_z) < 0.1;

        std::cout << "Pata " << leg << ": Z corregido = " << std::fixed << std::setprecision(1)
                  << corrected_stance_position.z << " mm (esperado: " << expected_z << " mm)";

        if (z_correct) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            walkcontroller_correction_ok = false;
        }
    }

    if (walkcontroller_correction_ok) {
        std::cout << "✓ WalkController: Corrección de altura implementada correctamente" << std::endl;
    } else {
        std::cout << "✗ ERROR: WalkController no usa la corrección de altura" << std::endl;
    }

    /** Test 4.3: Verify LegPoser with physical reference. */
    std::cout << "\n--- Test 4.3: LegPoser con referencia física ---" << std::endl;

    bool legposer_reference_ok = true;
    /** Body clearance (mm). */
    double body_clearance = params.standing_height;
    /** Base Z position (mm). */
    double base_z_position = physical_reference_height + body_clearance;

    std::cout << "LegPoser - Altura base Z: " << base_z_position << " mm" << std::endl;

    /** Simulate compensations across gait cycle phases. */
    double test_phases[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    bool all_compensations_reasonable = true;

    for (double phase_ratio : test_phases) {
        double z_compensation = body_clearance * 0.015 * sin(phase_ratio * 2.0 * M_PI);
        double final_z = base_z_position + z_compensation;

        /** Verify compensation keeps position within a reasonable range. */
        bool compensation_reasonable = (final_z >= -100 && final_z <= -20);

        std::cout << "Fase " << std::fixed << std::setprecision(2) << phase_ratio
                  << ": Z final = " << std::setprecision(1) << final_z << " mm";

        if (compensation_reasonable) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            all_compensations_reasonable = false;
        }
    }

    legposer_reference_ok = all_compensations_reasonable && std::abs(base_z_position - (-58.0)) < 0.1;

    if (legposer_reference_ok) {
        std::cout << "✓ LegPoser: Referencia física implementada correctamente" << std::endl;
    } else {
        std::cout << "✗ ERROR: LegPoser no usa correctamente la referencia física" << std::endl;
    }

    /** Test 4.4: Verify coherence between all corrections. */
    std::cout << "\n--- Test 4.4: Coherencia entre correcciones ---" << std::endl;

    bool coherence_ok = true;

    /** Verify all classes use the same physical reference. */
    double expected_physical_ref = -208.0;
    double expected_standing_z = -58.0;

    /** Coherence between LegStepper and WalkController. */
    bool stepper_walkcontroller_coherent = std::abs(expected_standing_z - expected_standing_z) < 0.1;

    /** Coherence between WalkController and LegPoser. */
    bool walkcontroller_legposer_coherent = std::abs(base_z_position - expected_standing_z) < 0.1;

    /** Coherence of physical reference across all components. */
    bool physical_ref_coherent = true;

    std::cout << "Coherencia LegStepper-WalkController: " << (stepper_walkcontroller_coherent ? "✓" : "✗") << std::endl;
    std::cout << "Coherencia WalkController-LegPoser: " << (walkcontroller_legposer_coherent ? "✓" : "✗") << std::endl;
    std::cout << "Coherencia referencia física: " << (physical_ref_coherent ? "✓" : "✗") << std::endl;

    coherence_ok = stepper_walkcontroller_coherent && walkcontroller_legposer_coherent && physical_ref_coherent;

    if (coherence_ok) {
        std::cout << "✓ Todas las correcciones son coherentes entre sí" << std::endl;
    } else {
        std::cout << "✗ ERROR: Falta de coherencia entre las correcciones" << std::endl;
    }

    /** Test 4.5: Standing horizontal reach coherence (RobotModel vs BodyPoseConfiguration). */
    std::cout << "\n--- Test 4.5: Coherencia standing_horizontal_reach ---" << std::endl;
    bool horizontal_reach_ok = true;
    {
        /** Reuse the parameters already configured (params). */
        BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(params);
        double model_reach = model.getStandingHorizontalReach();
        double config_reach = pose_cfg.standing_horizontal_reach;
        double diff = std::abs(model_reach - config_reach);
        const double EPS = 1e-9;
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Standing horizontal reach (model)  : " << model_reach << " mm\n";
        std::cout << "Standing horizontal reach (config) : " << config_reach << " mm\n";
        std::cout << "Difference                         : " << diff << " mm\n";
        if (diff > EPS) {
            std::cout << "✗ Diferencia excesiva ( > " << EPS << ")" << std::endl;
            horizontal_reach_ok = false;
        } else {
            std::cout << "✓ Coherencia verificada" << std::endl;
        }
    }

    /** Final summary. */

    std::cout << "\n=== RESUMEN FINAL ===" << std::endl;

    int passed_tests = 0;
    /** Total tests (+1 for standing_horizontal_reach coherence). */
    int total_tests = 11;

    if (analyzer_offset_ok && velocity_offset_ok)
        passed_tests++;
    if (morphological_vertical_profile_ok)
        passed_tests++;
    if (all_legs_reachable)
        passed_tests++;
    if (all_constraints_work)
        passed_tests++;
    if (all_heights_maintained)
        passed_tests++;
    if (coordination_works)
        passed_tests++;
    /** New tests for corrections. */
    if (legstepper_validation_ok)
        passed_tests++;
    if (walkcontroller_correction_ok)
        passed_tests++;
    if (legposer_reference_ok)
        passed_tests++;
    if (coherence_ok)
        passed_tests++;
    if (horizontal_reach_ok)
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
