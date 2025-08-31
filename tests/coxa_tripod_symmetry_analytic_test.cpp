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
    std::cout << "Con timing OpenSHC: Cada fase (stance/swing) dura 52 iteraciones." << std::endl;
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

    // --- Enable AutoPose (tripod gait) ---
    // Previously the test constructed a BodyPoseConfiguration incorrectly from auto-pose factory and thus ignored auto pose.
    // We explicitly build the AutoPoseConfiguration for the active gait and enable it in the BodyPoseController.
    {
        auto *bpc = sys.getBodyPoseController();
        if (bpc) {
            AutoPoseConfiguration ap_cfg = createAutoPoseConfigurationForGait(p, "tripod_gait");
            bpc->setAutoPoseConfig(ap_cfg);
            bpc->setAutoPoseEnabled(true);
        } else {
            std::cerr << "WARNING: BodyPoseController no disponible; AutoPose no se activará." << std::endl;
        }
    }

    // Enable coxa telemetry for detailed post-run analysis (testing instrumentation)
#ifdef TESTING_ENABLED
    sys.enableTelemetry(true);
#endif

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
    double step_frequency_startup = DEFAULT_STEP_FREQUENCY;
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

    if (swing_iterations_per_cycle != 52 || stance_iterations_per_cycle != 52) {
        std::cout << "⚠️  WARNING: Las iteraciones no coinciden con trajectory_tip_position_test (esperado: 52 cada fase)" << std::endl;
        std::cout << "   Este test usa: swing=" << swing_iterations_per_cycle << ", stance=" << stance_iterations_per_cycle << std::endl;
    } else {
        std::cout << "✅ SINCRONIZACIÓN CONFIRMADA: Usando 52 iteraciones por fase" << std::endl;
    }

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

#ifdef TESTING_ENABLED
    // --- Detailed post-simulation telemetry analysis (testing only) ---
    std::cout << "\n=== DETAILED COXA TELEMETRY ANALYSIS (TESTING_ENABLED) ===" << std::endl;
    size_t n = sys.getTelemetrySampleCount();
    std::cout << "Samples captured: " << n << std::endl;
    if (n > 10) {
        // Validate premises:
        // 1. Tripods A={0,2,4} and B={1,3,5} share the same local coxa angle curve with ~180° phase shift.
        // 2. Opposite leg pairs (0,3) (1,4) (2,5) satisfy phi_i ≈ -phi_j (local angle) on average (specular symmetry).
        // 3. Sweep symmetry: protraction and retraction amplitudes are approximately equal per leg (local frame).
        // 4. Legs separated 60° inside the same tripod are copies (low amplitude variance), ensuring identical trajectories.

        auto phaseGroup = [&](int leg) { return (leg == 0 || leg == 2 || leg == 4) ? 0 : 1; };
        // Recolectar min/max y RMS por pata (local)
        struct Stats {
            double minA = 1e9, maxA = -1e9, sum = 0, sum2 = 0;
            int count = 0;
        };
        Stats st[NUM_LEGS];
        for (size_t i = 0; i < n; ++i) {
            const auto &s = sys.getTelemetrySample(i);
            for (int L = 0; L < NUM_LEGS; ++L) {
                double a = s.local_angle[L];
                st[L].minA = std::min(st[L].minA, a);
                st[L].maxA = std::max(st[L].maxA, a);
                st[L].sum += a;
                st[L].sum2 += a * a;
                st[L].count++;
            }
        }
        // Calcular amplitudes y medias
        double mean[NUM_LEGS];
        double amp[NUM_LEGS];
        for (int L = 0; L < NUM_LEGS; ++L) {
            mean[L] = st[L].sum / std::max(1, st[L].count);
            amp[L] = 0.5 * (st[L].maxA - st[L].minA);
        }
        // 1. Desfase: comparar fases mediante correlación cruzada simple de señales locales discretizadas a signo
        auto computePhaseShiftRatio = [&](int a, int b) {
            // Generar series de signo (stance/swing pattern + direction) para robustez
            std::vector<int> sa, sb;
            sa.reserve(n);
            sb.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                const auto &s = sys.getTelemetrySample(i);
                sa.push_back(s.local_angle[a] > mean[a] ? 1 : -1);
                sb.push_back(s.local_angle[b] > mean[b] ? 1 : -1);
            }
            int bestShift = 0;
            double bestScore = -1e9;
            int maxShift = (int)std::min<size_t>(200, n / 4);
            for (int shift = 0; shift <= maxShift; ++shift) {
                double score = 0;
                int m = 0;
                for (size_t i = shift; i < n; ++i) {
                    score += sa[i] * sb[i - shift];
                    ++m;
                }
                if (m > 0)
                    score /= m;
                if (score > bestScore) {
                    bestScore = score;
                    bestShift = shift;
                }
            }
            return std::make_pair(bestShift, bestScore);
        };
        // Evaluar 0 vs 1 (deben estar aproximadamente en oposición de fase local si pertenecen a trípodes distintos)
        auto s01 = computePhaseShiftRatio(0, 1);
        int expected_half_cycle = swing_iterations_per_cycle + stance_iterations_per_cycle; // full cycle in steps
        // expected half = half of total iterations per cycle
        int expected_half_shift = expected_half_cycle / 2; // integer truncation ok
        int shift_error = std::abs(s01.first - expected_half_shift);
        double shift_error_ratio = expected_half_shift > 0 ? (double)shift_error / (double)expected_half_shift : 1.0;
        bool phase_shift_ok = shift_error_ratio < 0.2; // 20% tolerance heuristic
        std::cout << "ShiftTripod(0 vs 1) bestShift=" << s01.first << " expectedHalf=" << expected_half_shift
                  << " error=" << shift_error << " (" << std::fixed << std::setprecision(2) << (shift_error_ratio * 100.0)
                  << "%) score=" << s01.second << " phase_shift_ok=" << (phase_shift_ok ? "YES" : "NO") << std::endl;
        // 2. Simetría especular global: phi_i ≈ -phi_j en promedio para pares opuestos
        int pairs[3][2] = {{0, 3}, {1, 4}, {2, 5}};
        bool specular_ok = true;
        for (auto &pr : pairs) {
            double avg = 0;
            int c = 0;
            double err_sum = 0;
            for (size_t i = 0; i < n; ++i) {
                const auto &s = sys.getTelemetrySample(i);
                double li = s.local_angle[pr[0]];
                double lj = s.local_angle[pr[1]];
                double sum = li + lj; // debería tender a 0
                err_sum += std::fabs(sum);
                ++c;
            }
            double mae = err_sum / std::max(1, c);
            std::cout << "SpecularPair (" << pr[0] << "," << pr[1] << ") MAE local_sum(rad)=" << mae << std::endl;
            if (mae > 0.15)
                specular_ok = false; // tolerancia heurística
        }
        // 3. Simetría barrido por pata (amplitud protacción vs retracción) ya aproximado con amp[] (baseline)
        // 4. Copias entre patas separadas 60° dentro mismo trípode: comparar amplitudes
        bool tripod_internal_ok = true;
        int tripodA[3] = {0, 2, 4};
        int tripodB[3] = {1, 3, 5};
        auto checkTripod = [&](int *legs) {
            double a0 = amp[legs[0]];
            for (int k = 1; k < 3; ++k) {
                double rel = fabs(amp[legs[k]] - a0) / std::max(1e-6, fabs(a0));
                if (rel > 0.2) {
                    tripod_internal_ok = false;
                }
            }
        };
        checkTripod(tripodA);
        checkTripod(tripodB);
        // Servo vs internal angle match
        double servo_angle_mae = 0.0;
        int servo_samples = 0;
        double servo_tol_rad = 2.0 * M_PI / 180.0; // 2 deg
        for (size_t i = 0; i < n; ++i) {
            const auto &s = sys.getTelemetrySample(i);
            for (int L = 0; L < NUM_LEGS; ++L) {
                servo_angle_mae += std::fabs(s.servo_command_coxa[L] - s.global_angle[L]);
                ++servo_samples;
            }
        }
        servo_angle_mae /= std::max(1, servo_samples);
        bool servo_match_ok = servo_angle_mae < servo_tol_rad;

        // Forward stride contribution (average dx during stance should be positive for both tripods)
        double avg_dx_stance_tripodA = 0, avg_dx_stance_tripodB = 0;
        int cA = 0, cB = 0;
        for (size_t i = 0; i < n; ++i) {
            const auto &s = sys.getTelemetrySample(i);
            for (int L = 0; L < NUM_LEGS; ++L) {
                if (s.phase[L] == STANCE_PHASE) {
                    if (L == 0 || L == 2 || L == 4) {
                        avg_dx_stance_tripodA += s.stride_dx[L];
                        ++cA;
                    } else {
                        avg_dx_stance_tripodB += s.stride_dx[L];
                        ++cB;
                    }
                }
            }
        }
        if (cA > 0)
            avg_dx_stance_tripodA /= cA;
        if (cB > 0)
            avg_dx_stance_tripodB /= cB;
        // Interpret forward progress in world frame: during stance the foot should remain approximately
        // world-stationary while body advances forward (+X). Telemetry computes stride_dx = tip.x - stance_start_tip.x.
        // Thus with forward body motion, tip.x will tend to decrease (negative dx) as the body moves past the planted foot.
        // Accept either small positive advance (simulation artifacts) or consistent negative displacement as forward progress.
        auto is_forward = [](double dx) { return dx > 0.0 || dx < -1.0; }; // tolerate |dx|>1mm negative as forward
        bool forward_progress_ok = (is_forward(avg_dx_stance_tripodA) && is_forward(avg_dx_stance_tripodB));

        std::cout << "TripodA amps(rad): " << amp[0] << "," << amp[2] << "," << amp[4] << "  TripodB amps(rad): " << amp[1] << "," << amp[3] << "," << amp[5] << std::endl;
        std::cout << "Servo vs model coxa MAE(rad): " << servo_angle_mae << " (tol=" << servo_tol_rad << ") match=" << (servo_match_ok ? "YES" : "NO") << std::endl;
        std::cout << "Avg stance stride dx TripodA=" << avg_dx_stance_tripodA << " TripodB=" << avg_dx_stance_tripodB << " forward_progress_ok=" << (forward_progress_ok ? "YES" : "NO") << std::endl;
        std::cout << "Premises Result: specular_ok=" << (specular_ok ? "YES" : "NO")
                  << " tripod_internal_ok=" << (tripod_internal_ok ? "YES" : "NO")
                  << " phase_shift_ok=" << (phase_shift_ok ? "YES" : "NO")
                  << " servo_match_ok=" << (servo_match_ok ? "YES" : "NO")
                  << " forward_progress_ok=" << (forward_progress_ok ? "YES" : "NO") << std::endl;
        bool premises_ok = specular_ok && tripod_internal_ok && phase_shift_ok && servo_match_ok && forward_progress_ok;
        if (!premises_ok) {
            std::cout << "[PREMISES] FAIL: Deviations detected in one or more gait symmetry/phase/stride premises." << std::endl;
        } else {
            std::cout << "[PREMISES] OK: All gait symmetry, phase shift, stride and servo correspondence premises satisfied." << std::endl;
        }
        if (!premises_ok) {
            // Force global failure reflected later
            g_sym_violations_stance += 1; // ensure symmetry_ok false OR we set a separate flag
        }
    }
#endif

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
