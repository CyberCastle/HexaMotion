/**
 * @file swing_coxa_orientation_test.cpp
 * @brief Detecta si la generación de la trayectoria de SWING (target_tip_pose + nodos Bezier)
 *        ignora el ángulo actual de la coxa (hipótesis: se asume coxa=0° al calcular el objetivo).
 *
 * Metodología:
 *  - Fijar un LegStepper con identidad y default_tip_pose conocidos.
 *  - Aplicar una velocidad lineal (stride) para forzar un target en swing.
 *  - Para varios ángulos de coxa (-20, 0, +20 grados) modificar el joint de la pata
 *    antes de invocar updateTipPositionIterative() en estado SWING (iteración 1).
 *  - Registrar:
 *      * current_tip_pose inicial (post rotación coxa)
 *      * target_tip_pose_ congelado
 *      * swing_2_nodes_[4] (nodo final de swing)
 *      * Dirección y magnitud planares base->target y base->final_swing
 *  - Calcular referencia OpenSHC: raw_target = default_tip_pose + 0.5 * stride_vector
 *  - Si los vectores base->target son idénticos para todos los ángulos (diferencia angular ~0)
 *    indica que el ángulo actual de coxa NO se incorporó explícitamente al cálculo del objetivo.
 */

#include "leg_stepper.h"
#include "math_utils.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct CoxaAngleCase {
    double coxa_deg;
};

static double planarAngle(const Point3D &v) { return std::atan2(v.y, v.x); }
static double planarNorm(const Point3D &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Swing Coxa Orientation Influence Test ===\n";

    // Parámetros mínimos coherentes con AGENTS.md
    Parameters params{};
    params.hexagon_radius = 160.0;
    params.coxa_length = 45.0;
    params.femur_length = 90.0;
    params.tibia_length = 150.0;
    params.default_height_offset = -params.tibia_length;
    params.robot_height = 150.0;
    params.time_delta = 1.0 / 50.0;
    params.standing_height = 120.0;
    params.enable_workspace_constrain = false;

    RobotModel model(params);

    // Configuración de StepCycle común
    StepCycle cycle{};
    cycle.frequency_ = 1.0;
    cycle.period_ = 4;
    cycle.stance_period_ = 2;
    cycle.swing_period_ = 2;
    cycle.stance_start_ = 0;
    cycle.stance_end_ = 2;
    cycle.swing_start_ = 2;
    cycle.swing_end_ = 4;

    // Datos acumulados por pata
    struct LegSummary {
        int leg;
        double max_delta_target_deg;
        double max_delta_swing_end_deg;
    };
    std::vector<LegSummary> leg_summaries;

    // Recorremos las 6 patas
    for (int leg_index = 0; leg_index < 6; ++leg_index) {
        Leg leg(leg_index, model);

        // Construir identidad radial para la pata usando su base theta
        double base_theta = model.getLegBaseAngleOffset(leg_index);                         // radians
        double planar_r = params.hexagon_radius + params.coxa_length + params.femur_length; // mismo razonamiento que stride test
        Point3D identity_tip(planar_r * std::cos(base_theta), planar_r * std::sin(base_theta), params.default_height_offset);

        LegStepper stepper(leg_index, identity_tip, leg, model);
        stepper.setStepCycle(cycle);

        // Velocidades que generan stride (idénticas para todas, se proyectan vía stride update)
        Point3D linear_vel(60.0, 20.0, 0.0);
        double angular_vel = 0.3; // yaw
        stepper.setDesiredVelocity(linear_vel, angular_vel);
        stepper.setWalkPlane(identity_tip);
        stepper.setWalkPlaneNormal(Point3D(0, 0, 1));

        std::vector<CoxaAngleCase> cases = {{-20.0}, {0.0}, {20.0}};
        struct Result {
            double coxa_deg;
            double target_dir;
            double swing_end_dir;
            double target_norm;
            double swing_end_norm;
            Point3D target;
            Point3D swing_end;
            Point3D tip_before;
        };
        std::vector<Result> results;

        // Referencia raw_target para esta pata
        double stance_ratio = double(cycle.stance_period_) / double(cycle.period_);
        Point3D radius(identity_tip.x, identity_tip.y, 0.0);
        Point3D angular_component(-angular_vel * radius.y, angular_vel * radius.x, 0.0);
        Point3D stride_total = (linear_vel + angular_component) * (stance_ratio / cycle.frequency_);
        Point3D stride_half = stride_total * 0.5;
        Point3D reference_raw_target = identity_tip + stride_half;

        for (auto cs : cases) {
            JointAngles ja = leg.getJointAngles();
            ja.coxa = cs.coxa_deg;
            leg.setJointAngles(ja);
            stepper.setCurrentTipPose(leg.getCurrentTipPositionGlobal());
            stepper.setStepState(STEP_SWING);
            stepper.setPhase(cycle.swing_start_);
            stepper.updateTipPositionIterative(1, params.time_delta, false, false);
            Point3D target = stepper.getTargetTipPose();
            Point3D swing_end = stepper.getSwing2ControlNode(4);
            Point3D base = model.getLegBasePosition(leg_index);
            Point3D base_to_target(target.x - base.x, target.y - base.y, 0.0);
            Point3D base_to_swing_end(swing_end.x - base.x, swing_end.y - base.y, 0.0);
            results.push_back(Result{cs.coxa_deg, planarAngle(base_to_target), planarAngle(base_to_swing_end), planarNorm(base_to_target), planarNorm(base_to_swing_end), target, swing_end, leg.getCurrentTipPositionGlobal()});
        }

        // Reporte por pata
        std::cout << "\n[Pata " << leg_index << "] base_theta(deg)=" << math_utils::radiansToDegrees(base_theta) << " raw_target=(" << reference_raw_target.x << ", " << reference_raw_target.y << ")" << "\n";
        const double DEG = math_utils::radiansToDegrees(1.0);
        if (!results.empty()) {
            double base_dir = results[0].target_dir;
            for (auto &r : results) {
                double dtheta_target = (r.target_dir - base_dir) * DEG;
                double dtheta_swing_end = (r.swing_end_dir - base_dir) * DEG;
                std::cout << " Coxa=" << r.coxa_deg
                          << "° target_dir(deg)=" << r.target_dir * DEG
                          << " Δtarget_dir(deg)=" << dtheta_target
                          << " swing_end_dir(deg)=" << r.swing_end_dir * DEG
                          << " Δswing_end_dir(deg)=" << dtheta_swing_end
                          << " target_norm=" << r.target_norm
                          << " swing_end_norm=" << r.swing_end_norm
                          << " tip_before=(" << r.tip_before.x << ", " << r.tip_before.y << ")"
                          << " target=(" << r.target.x << ", " << r.target.y << ")"
                          << " swing_end=(" << r.swing_end.x << ", " << r.swing_end.y << ")"
                          << "\n";
            }
        }

        double max_delta_target_deg = 0.0, max_delta_swing_end_deg = 0.0;
        for (size_t i = 1; i < results.size(); ++i) {
            double dt = std::fabs(math_utils::radiansToDegrees(results[i].target_dir - results[0].target_dir));
            double ds = std::fabs(math_utils::radiansToDegrees(results[i].swing_end_dir - results[0].swing_end_dir));
            if (dt > max_delta_target_deg)
                max_delta_target_deg = dt;
            if (ds > max_delta_swing_end_deg)
                max_delta_swing_end_deg = ds;
        }
        leg_summaries.push_back(LegSummary{leg_index, max_delta_target_deg, max_delta_swing_end_deg});
    }

    // Resumen global
    const double THRESH = 1e-3; // 0.001 deg
    std::cout << "\n=== Resumen Global ===\n";
    for (auto &ls : leg_summaries) {
        std::cout << "Leg " << ls.leg
                  << " maxΔ target_dir(deg)=" << ls.max_delta_target_deg
                  << " maxΔ swing_end_dir(deg)=" << ls.max_delta_swing_end_deg
                  << (ls.max_delta_target_deg <= THRESH ? " [INV]" : " [VAR]")
                  << (ls.max_delta_swing_end_deg <= THRESH ? " [INV]" : " [VAR]")
                  << "\n";
    }
    bool all_invariant = true;
    for (auto &ls : leg_summaries) {
        if (ls.max_delta_target_deg > THRESH || ls.max_delta_swing_end_deg > THRESH) {
            all_invariant = false;
            break;
        }
    }
    if (all_invariant)
        std::cout << "Conclusión: target_tip_pose y swing_end invariables al ángulo de coxa en TODAS las patas (escenario probado).\n";
    else
        std::cout << "Conclusión: Se detectó variación en alguna pata.\n";
    std::cout << "Umbral usado (deg): " << THRESH << "\n";

    return 0;
}