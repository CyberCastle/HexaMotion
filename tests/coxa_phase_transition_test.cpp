/**
 * @file coxa_phase_transition_test.cpp
 * @brief Test equivalente a tripod_walk_visualization_test enfocado en coxas.
 *
 * Este test es equivalente al tripod_walk_visualization_test pero muestra
 * únicamente el movimiento de las articulaciones COXA durante las transiciones
 * de fase swing/stance en gait tripod.
 *
 * El test:
 * 1. Inicializa el LocomotionSystem igual que tripod_walk_visualization_test
 * 2. Ejecuta la secuencia de startup completa
 * 3. Monitorea solo los ángulos de coxa durante las transiciones
 * 4. Usa el timing derivado dinámicamente del StepCycle (iteraciones por fase calculadas)
 * 5. Termina cuando todas las patas completan las transiciones requeridas
 *
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 */

#include "../src/body_pose_config_factory.h"
#include "../src/gait_config_factory.h"
#include "../src/locomotion_system.h"
#include "robot_model.h"
#include "test_stubs.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// --------------------------------------------------------------------------------------
// Parámetros por defecto (pueden sobre-escribirse por CLI)
// --------------------------------------------------------------------------------------
static double g_test_velocity = 100.0;            // mm/s
static int g_required_swing_transitions = 5;      // Transiciones STANCE->SWING por pata
static int g_max_steps = 1200;                    // Límite de seguridad
static bool g_show_only_phase_transitions = true; // Modo compacto por defecto
static double g_sym_threshold_stance_deg = 3.0;   // |sum(delta)| máximo permitido en STANCE
static double g_sym_threshold_swing_deg = 4.0;    // |sum(delta)| máximo permitido en SWING (más tolerancia)

// Acumuladores globales de métricas (máximos absolutos observados)
static double g_max_abs_sum_stance_pair[3] = {0, 0, 0}; // pares (0,3) (1,4) (2,5)
static double g_max_abs_sum_swing_pair[3] = {0, 0, 0};
static int g_sym_violations_stance = 0;
static int g_sym_violations_swing = 0;

static void printHelpAndExit() {
    std::cout << "Uso: ./coxa_phase_transition_test [opciones]\n"
              << "  --transitions N    Nº transiciones STANCE->SWING por pata (default 5)\n"
              << "  --velocity V       Velocidad lineal mm/s (default 100)\n"
              << "  --ang-vel W        Velocidad angular rad/s (default 0.25)\n"
              << "  --max-steps M      Máx pasos simulación (default 1200)\n"
              << "  --full             Mostrar TODAS las iteraciones\n"
              << "  --phases-only      Solo transiciones de fase (default)\n"
              << "  --help             Esta ayuda\n";
    std::exit(0);
}

static void parseArgs(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto needVal = [&](const char *flag) {
            if (i + 1 >= argc) {
                std::cerr << "Falta valor para " << flag << std::endl;
                std::exit(1);
            }
        };
        if (a == "--help") {
            printHelpAndExit();
        } else if (a == "--transitions") {
            needVal("--transitions");
            g_required_swing_transitions = std::atoi(argv[++i]);
        } else if (a == "--velocity") {
            needVal("--velocity");
            g_test_velocity = std::atof(argv[++i]);
        } else if (a == "--max-steps") {
            needVal("--max-steps");
            g_max_steps = std::atoi(argv[++i]);
        } else if (a == "--full") {
            g_show_only_phase_transitions = false;
        } else if (a == "--phases-only") {
            g_show_only_phase_transitions = true;
        } else if (a == "--sym-thr-stance") {
            needVal("--sym-thr-stance");
            g_sym_threshold_stance_deg = std::atof(argv[++i]);
        } else if (a == "--sym-thr-swing") {
            needVal("--sym-thr-swing");
            g_sym_threshold_swing_deg = std::atof(argv[++i]);
        } else {
            std::cerr << "Argumento desconocido: " << a << std::endl;
            printHelpAndExit();
        }
    }
    if (g_required_swing_transitions < 1)
        g_required_swing_transitions = 1;
    if (g_max_steps < 200)
        g_max_steps = 200; // seguridad mínima
}

// Utilidad para convertir radianes a grados
static double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

/**
 * @brief Imprime el encabezado del test.
 */
static void printTestHeader() {
    std::cout << "=======================================================================================================" << std::endl;
    std::cout << "                      TRIPOD GAIT COXA MOVEMENT VALIDATION TEST" << std::endl;
    std::cout << "=======================================================================================================" << std::endl;
    if (g_show_only_phase_transitions) {
        std::cout << "Modo: SOLO TRANSICIONES (estado inicial + S->W / W->S)." << std::endl;
    } else {
        std::cout << "Modo: TODAS LAS ITERACIONES (detalle completo)." << std::endl;
    }
    std::cout << "Con timing OpenSHC: Cada fase (stance/swing) usa iteraciones normalizadas dinámicas (no fijo a 52)." << std::endl;
    std::cout << "Objetivo: " << g_required_swing_transitions << " transiciones STANCE->SWING por pata." << std::endl;
    std::cout << "Duración estimada (aprox): ~" << (g_required_swing_transitions * 104) << " pasos (solo referencia)." << std::endl;
    std::cout << "Velocidad: " << g_test_velocity << " mm/s" << std::endl;

    std::cout << std::left << std::setw(8) << "Step"
              << std::setw(6) << "AR"
              << std::setw(6) << "BR"
              << std::setw(6) << "CR"
              << std::setw(6) << "CL"
              << std::setw(6) << "BL"
              << std::setw(6) << "AL"
              << std::setw(12) << "Phases"
              << "Transitions + Metrics" << std::endl;
    std::cout << "       (Coxa angles in degrees)                                    R(S/W)=Radio Stance/Swing  Sym=Simetría(sum,diff)" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
}

/**
 * @brief Imprime el estado de las coxas de todas las patas en un paso.
 * @param sys El sistema LocomotionSystem.
 * @param step El número de paso actual.
 * @param transition_counts Array con conteos de transiciones de cada pata.
 * @param leg_phase_iterations Array con iteraciones actuales de fase de cada pata.
 * @param swing_iterations_per_cycle Iteraciones de swing esperadas por ciclo.
 * @param stance_iterations_per_cycle Iteraciones de stance esperadas por ciclo.
 */
static void printCoxaStates(const LocomotionSystem &sys, int step, const int transition_counts[NUM_LEGS],
                            const int leg_phase_iterations[NUM_LEGS], int swing_iterations_per_cycle,
                            int stance_iterations_per_cycle,
                            const double stance_start_coxa_rad[NUM_LEGS],
                            const double initial_coxa_rad[NUM_LEGS]) {

    std::cout << std::left << std::setw(8) << step;

    // Imprimir ángulos de coxa para cada pata
    const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    double coxa_deg[NUM_LEGS];               // Absolute coxa angle (deg)
    double coxa_delta_initial_deg[NUM_LEGS]; // Delta from initial baseline (deg)
    double coxa_delta_stance_deg[NUM_LEGS];  // Delta from stance start (deg)
    double arc_mm[NUM_LEGS];
    double radius_mm[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles angles = leg.getJointAngles();
        double coxa_angle = angles.coxa; // rad absolute
        coxa_deg[i] = toDegrees(coxa_angle);
        coxa_delta_initial_deg[i] = toDegrees(coxa_angle - initial_coxa_rad[i]);
        coxa_delta_stance_deg[i] = toDegrees(coxa_angle - stance_start_coxa_rad[i]);
        // Radio planar desde la base de la pierna al pie actual (para estimar arco tangencial teórico)
        Point3D base = leg.getBasePosition();
        Point3D tip = leg.getCurrentTipPositionGlobal();
        double dx = tip.x - base.x;
        double dy = tip.y - base.y;
        double r = std::sqrt(dx * dx + dy * dy);
        radius_mm[i] = r;
        double delta_since_stance = coxa_angle - stance_start_coxa_rad[i]; // rad dentro de la fase stance actual (aprox)
        arc_mm[i] = r * delta_since_stance;                                // longitud de arco aproximada
        std::cout << std::setw(6) << std::fixed << std::setprecision(1) << coxa_deg[i];
    }

    // Imprimir fases actuales compactas
    std::cout << std::setw(12);
    std::string phases = "";
    for (int i = 0; i < NUM_LEGS; ++i) {
        StepPhase phase = sys.getLeg(i).getStepPhase();
        phases += (phase == STANCE_PHASE ? "S" : "W");
    }
    std::cout << phases;

    // Imprimir conteos de transiciones
    std::cout << " ";
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << LEG_NAMES[i] << ":" << transition_counts[i] << " ";
    }

    // Añadir métricas adicionales en la misma línea
    // Radios promedio de las patas en stance/swing
    double avg_radius_stance = 0, avg_radius_swing = 0;
    int stance_count = 0, swing_count = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        StepPhase phase = sys.getLeg(i).getStepPhase();
        if (phase == STANCE_PHASE) {
            avg_radius_stance += radius_mm[i];
            stance_count++;
        } else {
            avg_radius_swing += radius_mm[i];
            swing_count++;
        }
    }
    if (stance_count > 0)
        avg_radius_stance /= stance_count;
    if (swing_count > 0)
        avg_radius_swing /= swing_count;

    // Métricas de simetría por pares opuestos (0,3) (1,4) (2,5)
    // Nota: Las métricas originales usaban ángulos absolutos; como las coxas opuestas NO tienen offsets que sumen 0
    // la suma absoluta no es un indicador válido de simetría. Ahora añadimos métricas basadas en deltas respecto
    // al ángulo inicial (baseline) y solo las mostramos cuando AMBAS patas están en la misma fase STANCE.
    auto pairMetricsAbs = [&](int a, int b) {
        double sum = coxa_deg[a] + coxa_deg[b];
        double diff = coxa_deg[a] - coxa_deg[b];
        return std::make_pair(sum, diff);
    };
    auto pairMetricsDelta = [&](int a, int b) {
        double sum = coxa_delta_initial_deg[a] + coxa_delta_initial_deg[b];
        double diff = coxa_delta_initial_deg[a] - coxa_delta_initial_deg[b];
        return std::make_pair(sum, diff);
    };

    auto p03_abs = pairMetricsAbs(0, 3);
    auto p14_abs = pairMetricsAbs(1, 4);
    auto p25_abs = pairMetricsAbs(2, 5);

    // Delta (baseline) metrics – stance y swing se evalúan por separado
    auto bothStance = [&](int a, int b) {
        return sys.getLeg(a).getStepPhase() == STANCE_PHASE && sys.getLeg(b).getStepPhase() == STANCE_PHASE;
    };
    auto bothSwing = [&](int a, int b) {
        return sys.getLeg(a).getStepPhase() == SWING_PHASE && sys.getLeg(b).getStepPhase() == SWING_PHASE;
    };
    std::string p03_delta_str = "--";
    std::string p14_delta_str = "--";
    std::string p25_delta_str = "--";
    std::string p03_delta_swing_str = "--";
    std::string p14_delta_swing_str = "--";
    std::string p25_delta_swing_str = "--";
    if (bothStance(0, 3)) {
        auto m = pairMetricsDelta(0, 3);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p03_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[0] = std::max(g_max_abs_sum_stance_pair[0], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    if (bothStance(1, 4)) {
        auto m = pairMetricsDelta(1, 4);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p14_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[1] = std::max(g_max_abs_sum_stance_pair[1], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    if (bothStance(2, 5)) {
        auto m = pairMetricsDelta(2, 5);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p25_delta_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_stance_pair[2] = std::max(g_max_abs_sum_stance_pair[2], abs_sum);
        if (abs_sum > g_sym_threshold_stance_deg)
            g_sym_violations_stance++;
    }
    // Swing symmetry tracking
    if (bothSwing(0, 3)) {
        auto m = pairMetricsDelta(0, 3);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p03_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[0] = std::max(g_max_abs_sum_swing_pair[0], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }
    if (bothSwing(1, 4)) {
        auto m = pairMetricsDelta(1, 4);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p14_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[1] = std::max(g_max_abs_sum_swing_pair[1], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }
    if (bothSwing(2, 5)) {
        auto m = pairMetricsDelta(2, 5);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << m.first << "," << m.second;
        p25_delta_swing_str = oss.str();
        double abs_sum = std::fabs(m.first);
        g_max_abs_sum_swing_pair[2] = std::max(g_max_abs_sum_swing_pair[2], abs_sum);
        if (abs_sum > g_sym_threshold_swing_deg)
            g_sym_violations_swing++;
    }

    std::cout << " R(S/W):" << std::fixed << std::setprecision(0) << avg_radius_stance << "/" << avg_radius_swing
              << " AbsSym03:" << std::setprecision(1) << p03_abs.first << "," << p03_abs.second
              << " 14:" << p14_abs.first << "," << p14_abs.second
              << " 25:" << p25_abs.first << "," << p25_abs.second
              << " dSym03:" << p03_delta_str
              << " d14:" << p14_delta_str
              << " d25:" << p25_delta_str
              << " dSymW03:" << p03_delta_swing_str
              << " dW14:" << p14_delta_swing_str
              << " dW25:" << p25_delta_swing_str;

    std::cout << std::endl;
}

/**
 * @brief Verifica si todas las patas han completado las transiciones requeridas.
 * @param transition_counts Array con conteos de transiciones de cada pata.
 * @return True si se cumple el objetivo del test, false en caso contrario.
 */
static bool allLegsCompletedTransitions(const int transition_counts[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (transition_counts[i] < g_required_swing_transitions) {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv) {
    parseArgs(argc, argv);
    std::cout << "=== Coxa Phase Transition Test (Equivalente a Tripod Walk Visualization) ===" << std::endl;

    // 1. Inicialización básica
    Parameters p = createDefaultParameters();
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    BodyPoseConfiguration pose_config = getDefaultBodyPoseConfig(p);

    if (!sys.initialize(&imu, &fsr, &servos, pose_config)) {
        std::cerr << "ERROR: Failed to initialize locomotion system." << std::endl;
        return 1;
    }

    // 2. Iniciar en Standing Pose
    if (!sys.setStandingPose()) {
        std::cerr << "ERROR: Failed to set standing pose." << std::endl;
        return 1;
    }
    std::cout << "Robot en standing pose. Todas las patas en fase STANCE." << std::endl;

    // Verificar pose inicial (solo coxas)
    std::cout << "\nVERIFICACIÓN POSE INICIAL (solo ángulos coxa):" << std::endl;
    std::cout << "Pata   Fase   Coxa (grados)" << std::endl;
    std::cout << "-----------------------------" << std::endl;
    const char *LEG_NAMES[NUM_LEGS] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    for (int i = 0; i < NUM_LEGS; ++i) {
        const Leg &leg = sys.getLeg(i);
        JointAngles angles = leg.getJointAngles();
        StepPhase phase = leg.getStepPhase();

        std::cout << std::left << std::setw(7) << LEG_NAMES[i]
                  << std::setw(7) << (phase == STANCE_PHASE ? "S" : "W")
                  << std::fixed << std::setprecision(2) << toDegrees(angles.coxa) << std::endl;
    }
    std::cout << "-----------------------------\n"
              << std::endl;

    // 3. Configurar y iniciar Tripod Gait (idéntico a tripod_walk_visualization_test)
    if (!sys.setGaitType(TRIPOD_GAIT)) {
        std::cerr << "ERROR: Failed to set gait type." << std::endl;
        return 1;
    }

    sys.walkForward(g_test_velocity);
    if (!sys.startWalking()) {
        std::cerr << "ERROR: Failed to start walking (startup sequence)." << std::endl;
        return 1;
    }

    // Ejecutar secuencia de startup
    std::cout << "Ejecutando secuencia de startup..." << std::endl;

    int startup_sequence_attempts = 0;
    const Parameters &startup_params = sys.getParameters();
    double time_delta_startup = startup_params.time_delta;
    double step_frequency_startup = startup_params.step_frequency;
    int horiz_iters = std::max(1, (int)std::round((1.0 / step_frequency_startup) / time_delta_startup));
    int vert_iters = std::max(1, (int)std::round((3.0 / step_frequency_startup) / time_delta_startup));
    int expected_total_iters = horiz_iters + vert_iters;
    const int MAX_STARTUP_SEQUENCE_ATTEMPTS = expected_total_iters + 100;

    std::cout << "Iteraciones startup estimadas: total=" << expected_total_iters
              << ", max attempts=" << MAX_STARTUP_SEQUENCE_ATTEMPTS << std::endl;

    while (sys.isStartupInProgress() && startup_sequence_attempts < MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        if (sys.executeStartupSequence()) {
            std::cout << "Secuencia de startup completada tras " << startup_sequence_attempts << " intentos." << std::endl;
            break;
        }
        startup_sequence_attempts++;

        if (startup_sequence_attempts % 25 == 0) {
            std::cout << "Intento startup " << startup_sequence_attempts
                      << "  Progreso=" << sys.getStartupProgressPercent() << "%" << std::endl;
        }
    }

    if (startup_sequence_attempts >= MAX_STARTUP_SEQUENCE_ATTEMPTS) {
        std::cerr << "ERROR: Secuencia de startup falló tras " << startup_sequence_attempts << " intentos." << std::endl;
        return 1;
    }

    std::cout << "Iniciando análisis de movimiento de coxas..." << std::endl;
    printTestHeader();

    // 4. Loop principal de simulación con verificación de timing
    std::cout << "=== VERIFICACIÓN DE SINCRONIZACIÓN CON trajectory_tip_position_test ===" << std::endl;

    auto first_leg_stepper = sys.getWalkController()->getLegStepper(0);
    if (!first_leg_stepper) {
        std::cerr << "ERROR: No se pudo obtener el LegStepper." << std::endl;
        return 1;
    }

    StepCycle actual_step_cycle = first_leg_stepper->getStepCycle();
    double time_delta = sys.getRobotModel().getTimeDelta();

    // Usar exactamente la misma fórmula que trajectory_tip_position_test
    int swing_iterations_per_cycle = (int)((double(actual_step_cycle.swing_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int stance_iterations_per_cycle = (int)((double(actual_step_cycle.stance_period_) / actual_step_cycle.period_) / (actual_step_cycle.frequency_ * time_delta));
    int total_iterations_per_cycle = swing_iterations_per_cycle + stance_iterations_per_cycle;

    std::cout << "StepCycle activo:" << std::endl;
    std::cout << "  swing_iterations_per_cycle: " << swing_iterations_per_cycle << std::endl;
    std::cout << "  stance_iterations_per_cycle: " << stance_iterations_per_cycle << std::endl;
    std::cout << "  total_iterations_per_cycle: " << total_iterations_per_cycle << std::endl;

    // Validación de coherencia interna: las iteraciones derivadas deben sumar el periodo
    if (swing_iterations_per_cycle + stance_iterations_per_cycle != actual_step_cycle.period_) {
        std::cerr << "ERROR: Inconsistencia de timing: swing(" << swing_iterations_per_cycle << ") + stance("
                  << stance_iterations_per_cycle << ") != period(" << actual_step_cycle.period_ << ")" << std::endl;
        return 1; // Fallar el test: estado interno incoherente
    }

    std::cout << "Iteraciones derivadas: swing=" << swing_iterations_per_cycle << ", stance=" << stance_iterations_per_cycle << std::endl;
    if (swing_iterations_per_cycle != stance_iterations_per_cycle) {
        std::cout << "⚠️  NOTE: swing y stance difieren; validar coherencia de configuración (esto es permitido si las fases tienen distinta duración)." << std::endl;
    }
    std::cout << "(Referencia previa de 52 eliminada; ahora se valida contra StepCycle real)." << std::endl;

    // DEBUG: Mostrar offset multipliers del tripod gait
    std::cout << "\n=== DEBUG: Tripod Gait Offset Multipliers ===" << std::endl;
    auto gait_config = sys.getWalkController()->getCurrentGaitConfig();
    const char *leg_names[] = {"AR", "BR", "CR", "CL", "BL", "AL"};
    std::cout << "Offset multipliers:" << std::endl;
    for (int i = 0; i < 6; i++) {
        int offset = gait_config.offsets.getForLegIndex(i);
        std::cout << "  " << leg_names[i] << ": " << offset << std::endl;
    }
    std::cout << std::endl;

    int step = 0;
    int transition_counts[NUM_LEGS] = {0};
    StepPhase previous_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        previous_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Contador de iteraciones para cada fase por pata
    int leg_phase_iterations[NUM_LEGS] = {0};
    StepPhase leg_current_phases[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        leg_current_phases[i] = sys.getLeg(i).getStepPhase();
    }

    // Mostrar estado inicial
    // Track inicio de stance para estimar arco de yaw: se inicializa en la pose inicial
    double stance_start_coxa_rad[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        stance_start_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
    }

    // Capturar baseline inicial de coxas (para simetría por delta)
    double initial_coxa_rad[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        initial_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
    }

    printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);

    while (step < g_max_steps) {
        // Actualizar sistema
        if (!sys.update()) {
            std::cerr << "WARNING: System update failed at step " << step << std::endl;
            continue;
        }

        // Verificar transiciones y detectar si hay algún cambio de fase
        bool phase_transition_occurred = false;

        for (int i = 0; i < NUM_LEGS; ++i) {
            StepPhase current_phase = sys.getLeg(i).getStepPhase();

            // Detectar transiciones STANCE -> SWING
            if (previous_phases[i] == STANCE_PHASE && current_phase == SWING_PHASE) {
                transition_counts[i]++;
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
            }
            // Detectar transiciones SWING -> STANCE
            else if (previous_phases[i] == SWING_PHASE && current_phase == STANCE_PHASE) {
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
                // Reiniciar referencia de inicio de stance para esta pata
                stance_start_coxa_rad[i] = sys.getLeg(i).getJointAngles().coxa;
            }
            // Si estamos en la misma fase, incrementar contador
            else if (leg_current_phases[i] == current_phase) {
                leg_phase_iterations[i]++;
            }
            // Si hay cambio de fase sin ser transición detectada arriba
            else {
                leg_phase_iterations[i] = 1;
                leg_current_phases[i] = current_phase;
                phase_transition_occurred = true;
            }

            previous_phases[i] = current_phase;
        }

        if (g_show_only_phase_transitions) {
            if (phase_transition_occurred) {
                printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);
            }
        } else {
            printCoxaStates(sys, step, transition_counts, leg_phase_iterations, swing_iterations_per_cycle, stance_iterations_per_cycle, stance_start_coxa_rad, initial_coxa_rad);
        }

        // Verificar completación
        if (allLegsCompletedTransitions(transition_counts)) {
            std::cout << "\nÉXITO: Todas las patas completaron " << g_required_swing_transitions << " transiciones swing." << std::endl;
            break;
        }

        step++;
    }

    if (step == g_max_steps) {
        std::cerr << "\nWARNING: Test alcanzó máximo de pasos (" << g_max_steps << ") antes de completarse." << std::endl;
    }

    // 5. Parar y volver a stance
    std::cout << "\nParando caminar y volviendo a pose de pie..." << std::endl;
    if (!sys.stopWalking()) {
        std::cerr << "WARNING: Failed to initiate stop walking." << std::endl;
    }

    // Ejecutar secuencia de shutdown para transición de RUNNING a READY
    int shutdown_sequence_attempts = 0;
    const int MAX_SHUTDOWN_SEQUENCE_ATTEMPTS = 100;

    while (sys.isShutdownInProgress() && shutdown_sequence_attempts < MAX_SHUTDOWN_SEQUENCE_ATTEMPTS) {
        if (sys.executeShutdownSequence()) {
            std::cout << "Secuencia de shutdown completada tras " << shutdown_sequence_attempts << " intentos." << std::endl;
            break;
        }
        shutdown_sequence_attempts++;

        if (shutdown_sequence_attempts % 10 == 0) {
            std::cout << "Intento shutdown " << shutdown_sequence_attempts << "..." << std::endl;
        }
    }

    if (shutdown_sequence_attempts >= MAX_SHUTDOWN_SEQUENCE_ATTEMPTS) {
        std::cerr << "WARNING: Secuencia de shutdown falló tras " << shutdown_sequence_attempts << " intentos." << std::endl;
    }

    // Resumen final
    std::cout << "\n=== RESUMEN FINAL DEL TEST ===" << std::endl;
    std::cout << "Pasos totales ejecutados: " << step << std::endl;
    std::cout << "Transiciones STANCE->SWING completadas:" << std::endl;
    for (int i = 0; i < NUM_LEGS; ++i) {
        std::cout << "  " << LEG_NAMES[i] << ": " << transition_counts[i] << "/" << g_required_swing_transitions << std::endl;
    }

    bool success = allLegsCompletedTransitions(transition_counts);

    // Evaluar simetría global PASS/FAIL
    bool symmetry_ok = (g_sym_violations_stance == 0 && g_sym_violations_swing == 0);
    if (!symmetry_ok) {
        std::cout << "\n[SIMETRIA] Violaciones detectadas:" << std::endl;
        if (g_sym_violations_stance)
            std::cout << "  STANCE: violaciones=" << g_sym_violations_stance << " (max abs sum pares: ["
                      << g_max_abs_sum_stance_pair[0] << ", " << g_max_abs_sum_stance_pair[1] << ", " << g_max_abs_sum_stance_pair[2] << "]) threshold=" << g_sym_threshold_stance_deg << "°" << std::endl;
        if (g_sym_violations_swing)
            std::cout << "  SWING: violaciones=" << g_sym_violations_swing << " (max abs sum pares: ["
                      << g_max_abs_sum_swing_pair[0] << ", " << g_max_abs_sum_swing_pair[1] << ", " << g_max_abs_sum_swing_pair[2] << "]) threshold=" << g_sym_threshold_swing_deg << "°" << std::endl;
    } else {
        std::cout << "\n[SIMETRIA] PASS: STANCE max=[" << g_max_abs_sum_stance_pair[0] << ", " << g_max_abs_sum_stance_pair[1] << ", " << g_max_abs_sum_stance_pair[2]
                  << "] SWING max=[" << g_max_abs_sum_swing_pair[0] << ", " << g_max_abs_sum_swing_pair[1] << ", " << g_max_abs_sum_swing_pair[2]
                  << "]" << std::endl;
    }
    success = success && symmetry_ok;
    std::cout << "\nResultado: " << (success ? "ÉXITO" : "FALLO") << std::endl;
    std::cout << "Test finalizado." << std::endl;

    return success ? 0 : 1;
}
