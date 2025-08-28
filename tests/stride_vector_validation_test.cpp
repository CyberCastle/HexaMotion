/**
 * @file stride_vector_validation_test.cpp
 * @brief Valida que el cálculo del stride vector (vector de zancada) en HexaMotion
 *        reproduce fielmente la lógica de OpenSHC y, por ende, el movimiento
 *        tangencial esperado de la coxa (componente angular + lineal).
 *
 * Metodología:
 *  1. Construye un LegStepper en estado inicial donde current_tip_pose == default_tip_pose.
 *  2. Aplica velocidades lineales (vx, vy) y angular (omega_z).
 *  3. Llama a updateStride() de HexaMotion.
 *  4. Recalcula el stride vector "esperado" usando la fórmula OpenSHC:
 *       stride_linear  = (vx, vy, 0)
 *       radius         = rechazo de tip sobre eje Z  (=> (x, y, 0) dado z eliminado)
 *       stride_angular = omega_z * k̂  X  radius = (-omega_z * y, omega_z * x, 0)
 *       stride_total   = (stride_linear + stride_angular) * (on_ground_ratio / frequency)
 *  5. Compara cada componente con el resultado de HexaMotion.
 *  6. Repite con varios casos (solo lineal, solo angular, combinado, distintos radios y signos).
 *
 * Éxito: error máximo < 1e-9 (tolerancia estricta porque los cálculos son deterministas).
 */

#include "gait_config.h"
#include "hexamotion_constants.h"
#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

struct StrideTestCase {
    Point3D identity_tip;    // Posición identidad (default/current inicial)
    Point3D linear_velocity; // (vx, vy, 0)
    double angular_velocity; // omega_z (rad/s)
    double frequency;        // step_cycle_.frequency_
    int stance_period;       // step_cycle_.stance_period_
    int swing_period;        // step_cycle_.swing_period_
    std::string name;        // Etiqueta
};

// Forward declaration del cálculo OpenSHC para uso en validateAllLegs
static Point3D computeExpectedOpenSHCStride(const StrideTestCase &tc);

// Ejecuta la misma lógica de validación sobre las 6 patas usando la posición por defecto de cada una.
static bool validateAllLegs(const StrideTestCase &tc, RobotModel &model, const Parameters &params, double tol) {
    std::cout << "  [SubTest] Validación radial y simetría hexagonal (6 coxas)" << std::endl;
    bool all_ok = true;
    // Construir objetos Leg (uno por índice)
    std::vector<std::unique_ptr<Leg>> legs;
    legs.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs.emplace_back(std::make_unique<Leg>(i, model));
    }

    // Para ángulos cero: el alcance planar hasta la punta incluye hexagon_radius + coxa_length + femur_length.
    double r = params.hexagon_radius + params.coxa_length + params.femur_length;
    double z0 = params.default_height_offset;

    // Para cada leg, construir identidad analítica: (r cos(theta_i), r sin(theta_i), z0) usando BASE_THETA_OFFSETS
    // Almacenes para simetría tangencial (se llenan dentro del loop)
    std::vector<Point3D> angular_components(NUM_LEGS, Point3D(0, 0, 0));
    std::vector<double> angular_mags(NUM_LEGS, 0.0);
    std::vector<double> coxa_deltas(NUM_LEGS, 0.0);

    for (int i = 0; i < NUM_LEGS; ++i) {
        double theta = BASE_THETA_OFFSETS[i];
        Point3D analytic_identity(r * std::cos(theta), r * std::sin(theta), z0);

        // Posición obtenida por FK (construida internamente por el modelo con mismos offsets DH)
        Point3D fk_default = model.getLegDefaultPosition(i);
        fk_default.z = z0; // normalizar altura para comparar solo plano XY
        double dx = fk_default.x - analytic_identity.x;
        double dy = fk_default.y - analytic_identity.y;
        double planar_geom_err = std::sqrt(dx * dx + dy * dy);

        // Crear stepper específico
        LegStepper stepper(i, analytic_identity, *legs[i], model);

        StepCycle cycle{};
        cycle.frequency_ = tc.frequency;
        cycle.period_ = tc.stance_period + tc.swing_period;
        cycle.stance_period_ = tc.stance_period;
        cycle.swing_period_ = tc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = tc.stance_period;
        cycle.swing_start_ = tc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepper.setStepCycle(cycle);

        stepper.setDesiredVelocity(tc.linear_velocity, tc.angular_velocity);
        stepper.setWalkPlane(analytic_identity);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepper.updateStride();

        Point3D got = stepper.getStrideVector();

        // Recalcular expected stride usando analytic_identity como radio planar
        StrideTestCase local_tc = tc;
        local_tc.identity_tip = analytic_identity;
        Point3D expected = computeExpectedOpenSHCStride(local_tc);
        Point3D diff = got - expected;
        double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

        double period = tc.stance_period + tc.swing_period;
        double on_ground_ratio = (period > 0) ? (double)tc.stance_period / period : 0.0;
        Point3D scaled_linear = tc.linear_velocity * (on_ground_ratio / tc.frequency);
        Point3D angular_component_expected = expected - scaled_linear;
        Point3D angular_component_got = got - scaled_linear;
        Point3D angular_diff = angular_component_got - angular_component_expected;
        double angular_err = std::sqrt(angular_diff.x * angular_diff.x + angular_diff.y * angular_diff.y + angular_diff.z * angular_diff.z);

        // Coxa delta y validación de traslación tangencial aproximada
        double stance_ratio = on_ground_ratio;
        double coxa_delta_expected = tc.angular_velocity * (stance_ratio / tc.frequency);
        Point3D radius_vec(analytic_identity.x, analytic_identity.y, 0.0);
        double radius_norm = std::sqrt(radius_vec.x * radius_vec.x + radius_vec.y * radius_vec.y);
        double coxa_delta_got = 0.0;
        bool coxa_valid = radius_norm > 1e-9;
        if (coxa_valid) {
            Point3D tangent_unit(-radius_vec.y / radius_norm, radius_vec.x / radius_norm, 0.0);
            double arc_len = angular_component_got.x * tangent_unit.x + angular_component_got.y * tangent_unit.y;
            coxa_delta_got = arc_len / radius_norm;
        }
        double coxa_err = std::fabs(coxa_delta_got - coxa_delta_expected);

        // (Nuevo) Validaciones tangenciales:
        //  a) Ortogonalidad: componente angular ⋅ radio ≈ 0
        //  b) Magnitud: |stride_angular| ≈ |coxa_delta_expected| * radius_norm
        double tangential_dot = angular_component_got.x * radius_vec.x + angular_component_got.y * radius_vec.y; // debería ~0
        double tangential_dot_abs = std::fabs(tangential_dot);
        double expected_arc_len = std::fabs(coxa_delta_expected) * radius_norm;
        double got_arc_len = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        double arc_len_err = std::fabs(got_arc_len - expected_arc_len);
        double tangential_tol = tol * std::max(1.0, radius_norm);
        double arc_len_tol = tol * std::max(1.0, radius_norm);

        // Relación mm -> grados (lineal planar escalada compartida por todas las patas)
        double linear_planar_mm = std::sqrt(scaled_linear.x * scaled_linear.x + scaled_linear.y * scaled_linear.y);
        double coxa_delta_deg_exp = math_utils::radiansToDegrees(coxa_delta_expected);
        double coxa_delta_deg_got = math_utils::radiansToDegrees(coxa_delta_got);

        // Guardar para análisis de simetría (se considera sólo la parte angular pura)
        angular_components[i] = angular_component_got;
        angular_mags[i] = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        coxa_deltas[i] = coxa_delta_got;

        bool tangential_ok = (!coxa_valid) || (tangential_dot_abs <= tangential_tol && arc_len_err <= arc_len_tol);
        bool pass = (err <= tol && angular_err <= tol && (!coxa_valid || coxa_err <= tol) && planar_geom_err <= tol && tangential_ok);
        std::cout << "    Leg " << i
                  << ": stride_err=" << err
                  << " ang_err=" << angular_err
                  << " coxa_err=" << coxa_err
                  << " geom_err=" << planar_geom_err
                  << " tangential_dot=" << tangential_dot_abs
                  << " arc_len_err=" << arc_len_err
                  << " | linear(mm)=" << linear_planar_mm
                  << " coxaΔexp(deg)=" << coxa_delta_deg_exp
                  << " coxaΔgot(deg)=" << coxa_delta_deg_got
                  << (pass ? " ✓" : " ❌") << std::endl;
        if (!pass)
            all_ok = false;
    }

    // ================= Symmetry Validation for Opposite Pairs =================
    // Pares opuestos: (0,3), (1,4), (2,5)
    struct Pair {
        int a;
        int b;
        const char *label;
    };
    Pair pairs[3] = {{0, 3, "(0,3)"}, {1, 4, "(1,4)"}, {2, 5, "(2,5)"}};
    // Contexto geométrico: ángulos base (offset DH) de cada pata en grados para evidenciar estructura hexagonal
    std::cout << "    BaseAngles(deg):";
    for (int i = 0; i < NUM_LEGS; ++i) {
        double deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[i]);
        std::cout << " L" << i << "=" << std::fixed << std::setprecision(1) << deg;
    }
    std::cout << std::endl;
    std::cout << "    Symmetry (pares opuestos, Δθ≈180° ideal):" << std::endl;
    double dir_tol = 1e-6; // tolerancia direccional absoluta
    for (const auto &p : pairs) {
        Point3D va = angular_components[p.a];
        Point3D vb = angular_components[p.b];
        double ma = angular_mags[p.a];
        double mb = angular_mags[p.b];
        // Si ambos son casi cero, se considera trivialmente simétrico
        bool trivial = (ma < 1e-12 && mb < 1e-12);
        double rel_mag_err = 0.0;
        double dir_dot_norm = 0.0;
        bool mag_ok = true, dir_ok = true;
        if (!trivial) {
            rel_mag_err = std::fabs(ma - mb) / std::max(1e-12, (ma + mb) * 0.5);
            if (ma > 0 && mb > 0) {
                dir_dot_norm = (va.x * vb.x + va.y * vb.y + va.z * vb.z) / (ma * mb);
            }
            // Para rotación global pura se espera vectores opuestos (dot≈-1).
            // Para combinaciones con gran componente lineal puede relajarse; aquí mantenemos criterio estricto.
            mag_ok = (rel_mag_err <= 1e-9 || trivial);
            dir_ok = (std::fabs(dir_dot_norm + 1.0) <= dir_tol) || trivial;
        }
        bool pair_ok = (trivial || (mag_ok && dir_ok));
        double theta_a_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.a]);
        double theta_b_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.b]);
        double delta_theta = std::fmod(std::fabs(theta_a_deg - theta_b_deg), 360.0);
        if (delta_theta > 180.0)
            delta_theta = 360.0 - delta_theta; // menor ángulo
        std::cout << "      OppPair " << p.label
                  << " (θa=" << theta_a_deg << ", θb=" << theta_b_deg << ", Δθ=" << delta_theta << ")"
                  << " |mag_a|=" << ma << " |mag_b|=" << mb
                  << " rel_mag_err=" << rel_mag_err
                  << " dir_dot(expect -1)=" << dir_dot_norm
                  << (trivial ? " (trivial: sin rotación)" : "")
                  << (pair_ok ? " ✓" : " ❌") << std::endl;
        if (!pair_ok)
            all_ok = false;
    }

    // Magnitud similar específica entre (0,3) y (2,5) (coherencia hexagonal adicional)
    double avg03 = 0.5 * (angular_mags[0] + angular_mags[3]);
    double avg25 = 0.5 * (angular_mags[2] + angular_mags[5]);
    double rel_block_err = std::fabs(avg03 - avg25) / std::max(1e-12, (avg03 + avg25) * 0.5);
    bool block_ok = (rel_block_err <= 1e-9) || (avg03 < 1e-12 && avg25 < 1e-12);
    std::cout << "      Block magnitudes hexagon ( (0,3) vs (2,5) ) avg03=" << avg03 << " avg25=" << avg25
              << " rel_err=" << rel_block_err
              << " criterio: magnitudes similares por ejes reflexivos del hexágono"
              << (block_ok ? " ✓" : " ❌") << std::endl;
    if (!block_ok)
        all_ok = false;

    return all_ok;
}

// Calcula el stride vector esperado usando la fórmula OpenSHC (idéntica a walk_controller.cpp de OpenSHC).
static Point3D computeExpectedOpenSHCStride(const StrideTestCase &tc) {
    // Componentes lineales
    Point3D stride_linear(tc.linear_velocity.x, tc.linear_velocity.y, 0.0);

    // Radio: rechazo respecto a eje Z (x,y,0) porque current_tip_pose == identity_tip y z se anula.
    Point3D radius(tc.identity_tip.x, tc.identity_tip.y, 0.0);

    // Velocidad angular (0,0,omega)
    Point3D angular_velocity_vec(0.0, 0.0, tc.angular_velocity);

    // Producto cruz: omega_k x radius => (-omega*y, omega*x, 0)
    Point3D stride_angular;
    stride_angular.x = angular_velocity_vec.y * radius.z - angular_velocity_vec.z * radius.y; // = -omega * y
    stride_angular.y = angular_velocity_vec.z * radius.x - angular_velocity_vec.x * radius.z; // =  omega * x
    stride_angular.z = 0.0;                                                                   // planar

    Point3D stride_total = stride_linear + stride_angular;
    double period = tc.stance_period + tc.swing_period;
    double on_ground_ratio = static_cast<double>(tc.stance_period) / period;
    stride_total = stride_total * (on_ground_ratio / tc.frequency);
    return stride_total;
}

int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Stride Vector Validation Test (HexaMotion vs OpenSHC) ===\n";

    // --- Configuración de parámetros mínimos del RobotModel ---
    Parameters params{};
    params.hexagon_radius = 160.0;             // mm
    params.coxa_length = 45.0;                 // mm
    params.femur_length = 90.0;                // mm
    params.tibia_length = 150.0;               // mm
    params.default_height_offset = -150.0;     // mm (igual a -tibia_length para pose vertical neutra)
    params.robot_height = 150.0;               // mm
    params.time_delta = 1.0 / 50.0;            // 50 Hz
    params.standing_height = 120.0;            // mm (para validaciones internas)
    params.enable_workspace_constrain = false; // Desactivar para no alterar stride

    RobotModel model(params);
    model.workspaceAnalyzerInitializer(); // Inicializa analizador (aunque no lo usamos directamente aquí)

    // Creamos un Leg para leg_index 0
    Leg leg0(0, model);

    // Casos de prueba
    std::vector<StrideTestCase> cases = {
        {Point3D(100.0, 0.0, params.default_height_offset), Point3D(50.0, 30.0, 0.0), 0.50, 1.0, 3, 1, "Combinado_PositiveY"},
        {Point3D(80.0, 60.0, params.default_height_offset), Point3D(40.0, -10.0, 0.0), 0.25, 1.2, 3, 1, "AnguloY_PositiveX"},
        {Point3D(120.0, -50.0, params.default_height_offset), Point3D(0.0, 20.0, 0.0), -0.40, 0.8, 2, 2, "SoloAngular_NegOmega"},
        {Point3D(60.0, 40.0, params.default_height_offset), Point3D(30.0, 0.0, 0.0), 0.00, 1.5, 3, 1, "SoloLineal"},
        {Point3D(90.0, -70.0, params.default_height_offset), Point3D(-25.0, 15.0, 0.0), 0.75, 2.0, 4, 2, "AltaFrecuencia"}};

    const double TOL = 1e-9; // Tolerancia estricta
    bool all_passed = true;

    for (const auto &tc : cases) {
        // Preparar LegStepper
        LegStepper stepper(0, tc.identity_tip, leg0, model);

        // Configurar StepCycle
        StepCycle cycle{};
        cycle.frequency_ = tc.frequency;
        cycle.period_ = tc.stance_period + tc.swing_period;
        cycle.stance_period_ = tc.stance_period;
        cycle.swing_period_ = tc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = tc.stance_period; // [0, stance_period)
        cycle.swing_start_ = tc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepper.setStepCycle(cycle);

        // Establecer velocidades deseadas y plano de marcha
        stepper.setDesiredVelocity(tc.linear_velocity, tc.angular_velocity);
        stepper.setWalkPlane(Point3D(tc.identity_tip.x, tc.identity_tip.y, tc.identity_tip.z));
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));

        // updateStride calcula y potencialmente congela stride; llamamos una vez.
        stepper.updateStride();

        Point3D got = stepper.getStrideVector();
        Point3D expected = computeExpectedOpenSHCStride(tc);
        Point3D diff = got - expected;
        double err = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

        // Extra: separar componentes lineal y angular escaladas para diagnóstico del movimiento de la coxa.
        double period = tc.stance_period + tc.swing_period;
        double on_ground_ratio = static_cast<double>(tc.stance_period) / period;
        Point3D scaled_linear = tc.linear_velocity * (on_ground_ratio / tc.frequency);
        Point3D angular_component_expected = expected - scaled_linear;
        Point3D angular_component_got = got - scaled_linear;
        Point3D angular_diff = angular_component_got - angular_component_expected;
        double angular_err = std::sqrt(angular_diff.x * angular_diff.x + angular_diff.y * angular_diff.y + angular_diff.z * angular_diff.z);

        std::cout << "\n[Test] " << tc.name << "\n";
        std::cout << " Identity Tip: (" << tc.identity_tip.x << ", " << tc.identity_tip.y << ", " << tc.identity_tip.z << ")" << std::endl;
        std::cout << " Linear Vel:   (" << tc.linear_velocity.x << ", " << tc.linear_velocity.y << ") mm/s  Angular Vel: " << tc.angular_velocity << " rad/s" << std::endl;
        std::cout << " freq=" << tc.frequency << " stance_period=" << tc.stance_period << " swing_period=" << tc.swing_period << std::endl;
        std::cout << " Expected Stride: (" << expected.x << ", " << expected.y << ", " << expected.z << ")" << std::endl;
        std::cout << " Got Stride:      (" << got.x << ", " << got.y << ", " << got.z << ")" << std::endl;
        std::cout << " |Error|=" << err << "  Angular |Error|=" << angular_err << std::endl;

        // ===================== Comparación de efecto sobre COXA =====================
        // En OpenSHC, la parte angular del stride representa la traslación tangencial de la punta
        // que proviene de una rotación del cuerpo (yaw) durante la fase de apoyo.
        // Sea:
        //   body_yaw_angle = omega_z * (stance_ratio / frequency)
        // La magnitud de la componente angular del stride debe cumplir:
        //   |stride_angular| = |omega_z| * (stance_ratio / frequency) * radius
        // donde radius = sqrt(x^2 + y^2) de la posición identidad (plano XY).
        // Por lo tanto, el delta de ángulo de la coxa esperado es:
        //   coxa_delta_expected = omega_z * (stance_ratio / frequency)
        // Podemos derivar el delta de ángulo a partir del stride obtenido proyectando la
        // componente angular sobre el vector tangente y dividiendo por el radio.

        double stance_ratio = (period > 0.0) ? (static_cast<double>(tc.stance_period) / period) : 0.0;
        double coxa_delta_expected = tc.angular_velocity * (stance_ratio / tc.frequency); // rad

        // Derivar coxa delta a partir del stride calculado por HexaMotion:
        // stride_angular_got ya calculado arriba (angular_component_got)
        Point3D radius_vec(tc.identity_tip.x, tc.identity_tip.y, 0.0);
        double radius_norm = std::sqrt(radius_vec.x * radius_vec.x + radius_vec.y * radius_vec.y);
        double coxa_delta_got = 0.0;
        bool coxa_delta_valid = radius_norm > 1e-9; // evitar división por cero
        if (coxa_delta_valid) {
            // Vector tangente unitario ( -y, x ) / r
            Point3D tangent_unit(-radius_vec.y / radius_norm, radius_vec.x / radius_norm, 0.0);
            // Proyección de la componente angular obtenida sobre el tangente => arco (longitud) recorrida
            double arc_length_got = angular_component_got.x * tangent_unit.x + angular_component_got.y * tangent_unit.y;
            coxa_delta_got = arc_length_got / radius_norm; // rad (con signo)
        }

        double coxa_err = std::fabs(coxa_delta_got - coxa_delta_expected);

        // Métrica adicional: relación desplazamiento lineal (mm) -> grados de coxa
        double linear_planar_mm = std::sqrt(scaled_linear.x * scaled_linear.x + scaled_linear.y * scaled_linear.y);
        double coxa_delta_deg_expected = math_utils::radiansToDegrees(coxa_delta_expected);
        double coxa_delta_deg_got = math_utils::radiansToDegrees(coxa_delta_got);
        // Validaciones tangenciales adicionales (idénticas a sección multi‑leg) para el caso base leg0
        double tangential_dot = angular_component_got.x * radius_vec.x + angular_component_got.y * radius_vec.y;
        double tangential_dot_abs = std::fabs(tangential_dot);
        double expected_arc_len = std::fabs(coxa_delta_expected) * radius_norm;
        double got_arc_len = std::sqrt(angular_component_got.x * angular_component_got.x + angular_component_got.y * angular_component_got.y);
        double arc_len_err = std::fabs(got_arc_len - expected_arc_len);
        std::cout << " Coxa Δexpected(rad): " << coxa_delta_expected
                  << "  Coxa Δgot(rad): " << coxa_delta_got
                  << "  |Δerr|=" << coxa_err << std::endl;
        std::cout << " Tangential: |ω×r|exp=" << expected_arc_len
                  << " |ω×r|got=" << got_arc_len
                  << " arc_len_err=" << arc_len_err
                  << " dot(radius,stride_ang)=" << tangential_dot_abs << std::endl;
        std::cout << " Mapping: linear_planar_scaled=" << linear_planar_mm
                  << " mm -> coxaΔexpected=" << coxa_delta_deg_expected
                  << " deg  coxaΔgot=" << coxa_delta_deg_got << " deg" << std::endl;

        bool tangential_ok = (!coxa_delta_valid) || (tangential_dot_abs <= TOL * std::max(1.0, radius_norm) && std::fabs(got_arc_len - expected_arc_len) <= TOL * std::max(1.0, radius_norm));
        bool pass = (err <= TOL && angular_err <= TOL && (!coxa_delta_valid || coxa_err <= TOL) && tangential_ok);
        if (!pass) {
            std::cout << "  ❌ Mismatch supera tolerancia." << std::endl;
            all_passed = false;
        } else {
            std::cout << "  ✓ OK" << std::endl;
        }

        // Validación adicional en las 6 patas con geometría radial usando sus posiciones por defecto
        bool radial_ok = validateAllLegs(tc, model, params, TOL);
        if (!radial_ok) {
            all_passed = false;
        }
    }

    std::cout << "\nResultado Global: " << (all_passed ? "✓ TODOS LOS CASOS OK" : "❌ FALLÓ ALGÚN CASO") << std::endl;
    return all_passed ? 0 : 1;
}
