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
        // Ya no existen getters de instrumentación de capas (raw/arc_pre/post).
        // Obtenemos directamente el stride final calculado (tras validaciones de seguridad si aplicaran).
        Point3D got = stepper.getStrideVector();

        // Nota: La versión actual de LegStepper::updateStride implementa directamente la fórmula combinada
        // (v + ω×r) * (stance_ratio / frequency) sin etapas intermedias expuestas públicamente; por ello
        // se elimina el análisis de E0/E1/E2 y sólo se compara contra el modelo OpenSHC esperado.

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

    // ================= Symmetry Validation (Oposición 180° y Reflexión 60°) =================
    // NUEVO: Se separan las dos nociones.
    //  A) Oposición (pares realmente separados 180° en el hexágono): (0,5), (1,4), (2,3)
    //  B) Reflexión (pares que sólo invierten el signo de X ó Y respecto a un eje local): (0,3), (2,5) y (1,4) (este último coincide con oposición)

    struct Pair {
        int a;
        int b;
        const char *label;
    };

    // ---- Bloque A: Oposición verdadera (esperamos vectores angulares opuestos -> dot ≈ -1) ----
    Pair opposite_pairs[3] = {{0, 5, "(0,5)"}, {1, 4, "(1,4)"}, {2, 3, "(2,3)"}};
    std::cout << "    BaseAngles(deg):";
    for (int i = 0; i < NUM_LEGS; ++i) {
        double deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[i]);
        std::cout << " L" << i << "=" << std::fixed << std::setprecision(1) << deg;
    }
    std::cout << std::endl;
    std::cout << "    Symmetry Opposite Pairs (Δθ≈180° -> dot≈-1):" << std::endl;
    double dir_tol = 1e-6; // tolerancia direccional estricta
    for (const auto &p : opposite_pairs) {
        Point3D va = angular_components[p.a];
        Point3D vb = angular_components[p.b];
        double ma = angular_mags[p.a];
        double mb = angular_mags[p.b];
        bool trivial = (ma < 1e-12 && mb < 1e-12);
        double rel_mag_err = 0.0, dir_dot_norm = 0.0;
        bool mag_ok = true, dir_ok = true;
        if (!trivial) {
            rel_mag_err = std::fabs(ma - mb) / std::max(1e-12, (ma + mb) * 0.5);
            if (ma > 0 && mb > 0)
                dir_dot_norm = (va.x * vb.x + va.y * vb.y + va.z * vb.z) / (ma * mb);
            mag_ok = (rel_mag_err <= 1e-9 || trivial);
            dir_ok = (std::fabs(dir_dot_norm + 1.0) <= dir_tol) || trivial;
        }
        bool pair_ok = trivial || (mag_ok && dir_ok);
        double theta_a_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.a]);
        double theta_b_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.b]);
        double delta_theta = std::fmod(std::fabs(theta_a_deg - theta_b_deg), 360.0);
        if (delta_theta > 180.0)
            delta_theta = 360.0 - delta_theta;
        std::cout << "      OppPair " << p.label
                  << " (θa=" << theta_a_deg << ", θb=" << theta_b_deg << ", Δθ=" << delta_theta << ")"
                  << " |mag_a|=" << ma << " |mag_b|=" << mb
                  << " rel_mag_err=" << rel_mag_err
                  << " dir_dot(expect -1)=" << dir_dot_norm
                  << (trivial ? " (trivial: sin rotación)" : "")
                  << (pair_ok ? " ✓" : " ❌") << std::endl;
        if (!pair_ok)
            all_ok = false; // Sólo este bloque afecta el resultado global
    }

    // ---- Bloque B: Reflexión (informativo, NO afecta all_ok) ----
    // Para reflexión esperamos magnitudes iguales y direcciones similares (dot≈+1) porque el radio se refleja
    // pero el signo de la componente tangencial puede conservarse según convención (depende de ω y orientación).
    Pair reflection_pairs[3] = {{0, 3, "(0,3)"}, {1, 4, "(1,4)"}, {2, 5, "(2,5)"}};
    std::cout << "    Symmetry Reflection Pairs (Δθ≈60° o 180° -> dot≈+1 esperado):" << std::endl;
    for (const auto &p : reflection_pairs) {
        Point3D va = angular_components[p.a];
        Point3D vb = angular_components[p.b];
        double ma = angular_mags[p.a];
        double mb = angular_mags[p.b];
        bool trivial = (ma < 1e-12 && mb < 1e-12);
        double rel_mag_err = 0.0, dir_dot_norm = 0.0;
        bool mag_ok = true, dir_ok = true;
        if (!trivial) {
            rel_mag_err = std::fabs(ma - mb) / std::max(1e-12, (ma + mb) * 0.5);
            if (ma > 0 && mb > 0)
                dir_dot_norm = (va.x * vb.x + va.y * vb.y + va.z * vb.z) / (ma * mb);
            // criterio relajado: magnitudes casi iguales y dot cercano a +1 (±0.5 margen por mezcla lineal)
            mag_ok = (rel_mag_err <= 1e-9 || trivial);
            dir_ok = (dir_dot_norm >= 0.3) || trivial; // sólo informativo
        }
        double theta_a_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.a]);
        double theta_b_deg = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[p.b]);
        double delta_theta = std::fmod(std::fabs(theta_a_deg - theta_b_deg), 360.0);
        if (delta_theta > 180.0)
            delta_theta = 360.0 - delta_theta;
        std::cout << "      ReflPair " << p.label
                  << " (θa=" << theta_a_deg << ", θb=" << theta_b_deg << ", Δθ=" << delta_theta << ")"
                  << " |mag_a|=" << ma << " |mag_b|=" << mb
                  << " rel_mag_err=" << rel_mag_err
                  << " dir_dot(expect +)=" << dir_dot_norm
                  << (trivial ? " (trivial: sin rotación)" : "")
                  << ((mag_ok && dir_ok) ? " (info ✓)" : " (info ⚠)") << std::endl;
    }

    // Magnitud similar entre bloques axiales (informativo, no afecta all_ok)
    double avg05 = 0.5 * (angular_mags[0] + angular_mags[5]);
    double avg23 = 0.5 * (angular_mags[2] + angular_mags[3]);
    double rel_block_err = std::fabs(avg05 - avg23) / std::max(1e-12, (avg05 + avg23) * 0.5);
    std::cout << "      Block magnitudes opos ( (0,5) vs (2,3) ) avg05=" << avg05 << " avg23=" << avg23
              << " rel_err=" << rel_block_err << " (info)" << std::endl;

    return all_ok;
}

// ---------------------------------------------------------------------------
// Nueva validación: demuestra que el algoritmo actual asume rotación alrededor
// del ORIGEN global en lugar del verdadero centro geométrico del cuerpo.
// Idea: se traslada todo el hexágono por un vector 'shift'. Si la rotación
// estuviera correctamente centrada, el stride angular esperado (ω×r_rel) NO
// cambiaría al añadir un shift constante al centro (porque r_rel = p_i - centro).
// Sin embargo, el código actual usa directamente (x,y) absolutos => aparece un
// sesgo constante bias = ω × shift que se suma a TODAS las patas.
// Verificamos:
//  1) got_stride - expected_center_stride es igual (≈) para todas las patas.
//  2) Ese vector común coincide con bias = ω×shift * (stance_ratio/frequency).
//  3) Tras sustraer bias, cada pata satisface la fórmula centrada correcta.
// Esto confirma: "el stride actual es consistente internamente, pero se asume
// un centro y un frame homogéneo que no se respeta aguas abajo".
// ---------------------------------------------------------------------------
static bool validateFrameCenterAssumption(const StrideTestCase &baseTc, RobotModel &model, const Parameters &params, const Point3D &shift, double tol) {
    std::cout << "\n  [FrameTest] Shift aplicado = (" << shift.x << ", " << shift.y << ") mm" << std::endl;

    // Prepara legs
    std::vector<std::unique_ptr<Leg>> legs;
    legs.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs.emplace_back(std::make_unique<Leg>(i, model));
    }

    double r = params.hexagon_radius + params.coxa_length + params.femur_length; // radio analítico
    double z0 = params.default_height_offset;
    double period = baseTc.stance_period + baseTc.swing_period;
    double stance_ratio = (period > 0.0) ? (double)baseTc.stance_period / period : 0.0;
    double scale = (stance_ratio / baseTc.frequency); // factor temporal común

    // Centro geométrico desplazado (ideal)
    Point3D center = Point3D(shift.x, shift.y, z0);
    // Sesgo teórico inducido por usar origen global incorrecto
    // ω×shift (ω sobre k̂) => (-ω*shift.y, ω*shift.x, 0)
    Point3D bias(-baseTc.angular_velocity * shift.y, baseTc.angular_velocity * shift.x, 0.0);
    bias = bias * scale; // escalado por tiempo en apoyo

    std::vector<Point3D> diffs;
    diffs.reserve(NUM_LEGS);
    bool all_ok = true;

    for (int i = 0; i < NUM_LEGS; ++i) {
        double theta = BASE_THETA_OFFSETS[i];
        Point3D identity_unshifted(r * std::cos(theta), r * std::sin(theta), z0);
        Point3D identity_shifted(identity_unshifted.x + shift.x, identity_unshifted.y + shift.y, z0);

        // Construir test case local para ambos
        StrideTestCase tcUn = baseTc;
        tcUn.identity_tip = identity_unshifted;
        StrideTestCase tcSh = baseTc;
        tcSh.identity_tip = identity_shifted;

        // Stepper unshifted
        LegStepper stepperUn(i, identity_unshifted, *legs[i], model);
        StepCycle cycle{};
        cycle.frequency_ = baseTc.frequency;
        cycle.period_ = baseTc.stance_period + baseTc.swing_period;
        cycle.stance_period_ = baseTc.stance_period;
        cycle.swing_period_ = baseTc.swing_period;
        cycle.stance_start_ = 0;
        cycle.stance_end_ = baseTc.stance_period;
        cycle.swing_start_ = baseTc.stance_period;
        cycle.swing_end_ = cycle.period_;
        stepperUn.setStepCycle(cycle);
        stepperUn.setDesiredVelocity(baseTc.linear_velocity, baseTc.angular_velocity);
        stepperUn.setWalkPlane(identity_unshifted);
        stepperUn.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepperUn.updateStride();
        Point3D strideUn = stepperUn.getStrideVector();

        // Stepper shifted
        LegStepper stepperSh(i, identity_shifted, *legs[i], model);
        stepperSh.setStepCycle(cycle);
        stepperSh.setDesiredVelocity(baseTc.linear_velocity, baseTc.angular_velocity);
        stepperSh.setWalkPlane(identity_shifted);
        stepperSh.setWalkPlaneNormal(Point3D(0, 0, 1));
        stepperSh.updateStride();
        Point3D strideSh = stepperSh.getStrideVector();

        Point3D diff = strideSh - strideUn; // debería ser igual a bias
        diffs.push_back(diff);
        Point3D residual = diff - bias; // cerca de cero
        double diff_err = std::sqrt((diff.x - bias.x) * (diff.x - bias.x) + (diff.y - bias.y) * (diff.y - bias.y));
        double residual_norm = std::sqrt(residual.x * residual.x + residual.y * residual.y + residual.z * residual.z);
        double bias_mag = std::sqrt(bias.x * bias.x + bias.y * bias.y);
        bool pass_local = (diff_err <= tol * std::max(1.0, bias_mag) && residual_norm <= tol * std::max(1.0, bias_mag));
        if (!pass_local)
            all_ok = false;
        std::cout << "    Leg " << i << " diff=(" << diff.x << "," << diff.y << ") bias=(" << bias.x << "," << bias.y << ") residual=(" << residual.x << "," << residual.y << ")" << (pass_local ? " ✓" : " ❌") << std::endl;
    }

    // Verificar que todos los diffs son (casi) iguales entre sí (consistencia interna)
    double uniform_err_max = 0.0;
    Point3D ref = diffs[0];
    for (size_t i = 1; i < diffs.size(); ++i) {
        Point3D d = diffs[i] - ref;
        double dn = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        uniform_err_max = std::max(uniform_err_max, dn);
    }
    bool uniform_ok = uniform_err_max <= tol * 1e3; // tolerancia relajada (el algoritmo puede recortar stride)
    if (!uniform_ok) {
        std::cout << "    ⚠️  Uniformidad NO estricta: uniform_err_max=" << uniform_err_max << std::endl;
        all_ok = false;
    }

    double bias_mag = std::sqrt(bias.x * bias.x + bias.y * bias.y);
    Point3D diff_meas = ref;
    double diff_mag = std::sqrt(diff_meas.x * diff_meas.x + diff_meas.y * diff_meas.y);
    double dot = diff_meas.x * bias.x + diff_meas.y * bias.y;
    double dir_cos = (bias_mag > 1e-12 && diff_mag > 1e-12) ? dot / (bias_mag * diff_mag) : 1.0;
    double angle_deg = math_utils::radiansToDegrees(std::acos(std::max(-1.0, std::min(1.0, dir_cos))));
    double mag_ratio = (bias_mag > 1e-12) ? diff_mag / bias_mag : 0.0;
    bool bias_detected = (diff_mag > tol * 100.0); // el medido debe ser significativo
    bool direction_ok = angle_deg <= 15.0;         // alineación direccional razonable
    std::cout << "    Bias teórico=(" << bias.x << "," << bias.y << ") mag=" << bias_mag
              << " | diff_medido=(" << diff_meas.x << "," << diff_meas.y << ") mag=" << diff_mag
              << " angle_diff_deg=" << angle_deg << " mag_ratio=" << mag_ratio << std::endl;

    if (uniform_ok && bias_detected && direction_ok) {
        std::cout << "    ✓ Confirmado: stride interno consistente + dependencia del origen (diff uniforme alineado a ω×shift)." << std::endl;
        return true;
    } else {
        std::cout << "    ❌ No se confirma completamente (uniform_ok=" << uniform_ok
                  << ", bias_detected=" << bias_detected << ", direction_ok=" << direction_ok << ")" << std::endl;
        return false;
    }
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
        // Nota: Los errores de simetría ya reflejan que la desviación proviene de E0 en la mayoría de casos.

        // --- Nueva prueba de frame/centro: sólo si hay componente angular (para observar bias) ---
        if (std::fabs(tc.angular_velocity) > 1e-9) {
            // Shift artificial (desplaza el centro real).
            Point3D shiftA(25.0, -25.0, 0.0);
            std::cout << "\n  [FrameTestSuite] Caso: " << tc.name << " (con shiftA)" << std::endl;
            bool frame_ok = validateFrameCenterAssumption(tc, model, params, shiftA, 1e-9);
            if (!frame_ok)
                all_passed = false;
        }
    }

    std::cout << "\nResultado Global: " << (all_passed ? "✓ TODOS LOS CASOS OK" : "❌ FALLÓ ALGÚN CASO") << std::endl;
    return all_passed ? 0 : 1;
}
