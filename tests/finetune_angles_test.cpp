#include "robot_model.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// Estructura para almacenar una solución completa
struct AngleSolution {
    double femur_deg;
    double tibia_deg;
    double coxa_deg;
    Point3D global_position;
    double height_error;
    double tibia_angle;
    double score;

    AngleSolution() : femur_deg(0), tibia_deg(0), coxa_deg(0),
                      global_position(0, 0, 0), height_error(999),
                      tibia_angle(0), score(999) {}
};

// Calcula el ángulo de la tibia respecto al suelo (plano XY)
double getTibiaAngleToGround(const RobotModel &model, int leg, const JointAngles &angles) {
    Eigen::Matrix4d transform = model.legTransform(leg, angles);
    Eigen::Vector3d tibia_direction(transform(0, 2), transform(1, 2), transform(2, 2));
    Eigen::Vector3d ground_normal(0, 0, 1);
    double dot_product = tibia_direction.dot(ground_normal) / (tibia_direction.norm() * ground_normal.norm());
    double angle_rad = acos(std::max(-1.0, std::min(1.0, dot_product))); // Clamp para evitar errores numéricos
    double angle_deg = angle_rad * 180.0 / M_PI;
    // Ángulo respecto al plano XY: 90° cuando la tibia está perpendicular al suelo
    return std::abs(90.0 - angle_deg);
}

// Verifica si la tibia está perpendicular al suelo
bool isTibiaPerpendicularToGround(const RobotModel &model, int leg, const JointAngles &angles, double tolerance_deg = 2.0) {
    double angle = getTibiaAngleToGround(model, leg, angles);
    return std::abs(angle - 90.0) <= tolerance_deg;
}

// Encuentra los mejores ángulos para una altura dada usando un buscador numérico
std::vector<AngleSolution> findOptimalAnglesForHeight(const RobotModel &model, double target_height,
                                                      double height_tolerance = 1.0) {
    std::vector<AngleSolution> solutions;
    const Parameters &p = model.getParams();

    std::cout << "Buscando solución optimizada para altura " << target_height << "mm..." << std::endl;
    std::cout << "Limitaciones reales del robot (AGENTS.md):" << std::endl;
    std::cout << "Rango coxa: [" << p.coxa_angle_limits[0] << "°, " << p.coxa_angle_limits[1] << "°]" << std::endl;
    std::cout << "Rango fémur: [" << p.femur_angle_limits[0] << "°, " << p.femur_angle_limits[1] << "°]" << std::endl;
    std::cout << "Rango tibia: [" << p.tibia_angle_limits[0] << "°, " << p.tibia_angle_limits[1] << "°]" << std::endl;

    // Conversión de límites a radianes
    double femur_min = p.femur_angle_limits[0] * M_PI / 180.0;
    double femur_max = p.femur_angle_limits[1] * M_PI / 180.0;
    double tibia_min = p.tibia_angle_limits[0] * M_PI / 180.0;
    double tibia_max = p.tibia_angle_limits[1] * M_PI / 180.0;

    double coxa_deg = 30.0; // Coxa fija

    // Estimación inicial cercana al rango medio
    JointAngles angles(coxa_deg * M_PI / 180.0, 0.0, 0.0);

    auto cost = [&](const JointAngles &q) {
        Point3D fk = model.forwardKinematicsGlobalCoordinates(0, q);
        double h_err = std::abs(std::abs(fk.z) - target_height);
        double t_angle = getTibiaAngleToGround(model, 0, q);
        double o_err = t_angle - 90.0;
        return h_err * h_err + o_err * o_err;
    };

    auto clamp = [&](JointAngles &q) {
        q.femur = model.constrainAngle(q.femur, femur_min, femur_max);
        q.tibia = model.constrainAngle(q.tibia, tibia_min, tibia_max);
    };

    const double eps = 0.001; // Paso para gradiente numérico
    const double lr = 0.5;    // Tasa de aprendizaje
    for (int i = 0; i < 200; ++i) {
        double base_cost = cost(angles);

        JointAngles q_fp = angles;
        q_fp.femur += eps;
        double grad_f = (cost(q_fp) - base_cost) / eps;

        JointAngles q_tp = angles;
        q_tp.tibia += eps;
        double grad_t = (cost(q_tp) - base_cost) / eps;

        angles.femur -= lr * grad_f;
        angles.tibia -= lr * grad_t;
        clamp(angles);

        if (std::sqrt(base_cost) <= height_tolerance) {
            break;
        }
    }

    Point3D fk_res = model.forwardKinematicsGlobalCoordinates(0, angles);
    double height_error = std::abs(std::abs(fk_res.z) - target_height);
    double tibia_angle = getTibiaAngleToGround(model, 0, angles);

    AngleSolution sol;
    sol.femur_deg = angles.femur * 180.0 / M_PI;
    sol.tibia_deg = angles.tibia * 180.0 / M_PI;
    sol.coxa_deg = coxa_deg;
    sol.global_position = fk_res;
    sol.height_error = height_error;
    sol.tibia_angle = tibia_angle;
    sol.score = 10.0 * height_error + std::abs(tibia_angle - 90.0);

    solutions.push_back(sol);
    return solutions;
}

// Estructura para almacenar información de una pata
struct LegConfiguration {
    int leg_index;
    double base_angle_deg;
    JointAngles joint_angles;
    Point3D global_position;
    double height_error;
    double tibia_angle;
    double score;
};

// Calcula las configuraciones para las 6 patas usando los offsets angulares de BASE_THETA_OFFSETS
std::vector<LegConfiguration> calculateAllLegsConfiguration(const RobotModel &model, double target_height,
                                                            const AngleSolution &base_solution) {
    std::vector<LegConfiguration> leg_configs;

    // Offsets angulares para cada pata (BASE_THETA_OFFSETS en grados)
    const double BASE_THETA_OFFSETS_DEG[6] = {-30.0, -90.0, -150.0, 150.0, 90.0, 30.0};

    std::cout << "\nCalculando configuraciones para las 6 patas:" << std::endl;
    std::cout << "Pata | Base°  | Coxa° | Femur° | Tibia° | Pos_X   | Pos_Y   | Pos_Z   | H_Error | T_Angle" << std::endl;
    std::cout << "-----|--------|-------|--------|--------|---------|---------|---------|---------|--------" << std::endl;

    for (int leg = 0; leg < 6; leg++) {
        LegConfiguration config;
        config.leg_index = leg;
        config.base_angle_deg = BASE_THETA_OFFSETS_DEG[leg];

        // Usar la solución base para fémur y tibia, coxa permanece en 0°
        config.joint_angles = JointAngles(
            base_solution.coxa_deg * M_PI / 180.0,
            base_solution.femur_deg * M_PI / 180.0,
            base_solution.tibia_deg * M_PI / 180.0);

        // Calcular la posición global para esta pata
        config.global_position = model.forwardKinematicsGlobalCoordinates(leg, config.joint_angles);

        // Calcular métricas de calidad
        config.height_error = std::abs(std::abs(config.global_position.z) - target_height);
        config.tibia_angle = getTibiaAngleToGround(model, leg, config.joint_angles);
        config.score = 10.0 * config.height_error + std::abs(config.tibia_angle - 90.0);

        leg_configs.push_back(config);

        // Mostrar información de la pata
        std::cout << std::setw(4) << leg << " | "
                  << std::setw(6) << std::fixed << std::setprecision(1) << config.base_angle_deg << " | "
                  << std::setw(5) << std::setprecision(1) << base_solution.coxa_deg << " | "
                  << std::setw(6) << std::setprecision(1) << base_solution.femur_deg << " | "
                  << std::setw(6) << std::setprecision(1) << base_solution.tibia_deg << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.x << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.y << " | "
                  << std::setw(7) << std::setprecision(2) << config.global_position.z << " | "
                  << std::setw(7) << std::setprecision(2) << config.height_error << " | "
                  << std::setw(7) << std::setprecision(2) << config.tibia_angle << std::endl;
    }

    return leg_configs;
}

int main() {
    // Configuración de parámetros del robot según AGENTS.md
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    // Limitaciones reales del robot según AGENTS.md
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = 50;
    p.tibia_angle_limits[1] = 50;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== BÚSQUEDA DE ÁNGULOS ÓPTIMOS PARA ALTURA MÍNIMA ESPECÍFICA ===" << std::endl;
    std::cout << "Parámetros del robot:" << std::endl;
    std::cout << "  - Radio hexágono: " << p.hexagon_radius << "mm" << std::endl;
    std::cout << "  - Longitud coxa: " << p.coxa_length << "mm" << std::endl;
    std::cout << "  - Longitud fémur: " << p.femur_length << "mm" << std::endl;
    std::cout << "  - Longitud tibia: " << p.tibia_length << "mm" << std::endl;
    std::cout << "  - Altura robot: " << p.robot_height << "mm" << std::endl;

    // Definir las alturas objetivo a probar - ajustadas para las limitaciones reales
    std::vector<double> target_heights = {140.0, 208.0}; // Rango de alturas a probar

    for (double target_height : target_heights) {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "ALTURA OBJETIVO: " << target_height << "mm" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Buscar soluciones para esta altura con tolerancia aumentada
        std::vector<AngleSolution> solutions = findOptimalAnglesForHeight(model, target_height, 0.1); // Tolerancia aumentada

        if (solutions.empty()) {
            std::cout << "No se encontraron soluciones para altura " << target_height << "mm" << std::endl;
            continue;
        }

        std::cout << "Solución encontrada" << std::endl;

        // Usar la mejor solución para calcular las 6 patas
        const AngleSolution &best_solution = solutions[0];

        std::cout << "\n--- MEJOR SOLUCIÓN PARA PATA DE REFERENCIA ---" << std::endl;
        std::cout << "Ángulos articulares:" << std::endl;
        std::cout << "  - Coxa: " << best_solution.coxa_deg << "°" << std::endl;
        std::cout << "  - Fémur: " << best_solution.femur_deg << "°" << std::endl;
        std::cout << "  - Tibia: " << best_solution.tibia_deg << "°" << std::endl;
        std::cout << "Posición global del extremo (pata 0):" << std::endl;
        std::cout << "  - X: " << best_solution.global_position.x << "mm" << std::endl;
        std::cout << "  - Y: " << best_solution.global_position.y << "mm" << std::endl;
        std::cout << "  - Z: " << best_solution.global_position.z << "mm" << std::endl;
        std::cout << "Métricas de calidad:" << std::endl;
        std::cout << "  - Error de altura: " << best_solution.height_error << "mm" << std::endl;
        std::cout << "  - Ángulo tibia al suelo: " << best_solution.tibia_angle << "°" << std::endl;
        std::cout << "  - Puntuación: " << best_solution.score << std::endl;

        // Calcular configuraciones para las 6 patas
        std::vector<LegConfiguration> all_legs = calculateAllLegsConfiguration(model, target_height, best_solution);

        // Calcular estadísticas
        double avg_height_error = 0.0;
        double avg_tibia_angle = 0.0;
        double max_height_error = 0.0;
        double min_height_error = std::numeric_limits<double>::max();

        for (const auto &leg : all_legs) {
            avg_height_error += leg.height_error;
            avg_tibia_angle += leg.tibia_angle;
            max_height_error = std::max(max_height_error, leg.height_error);
            min_height_error = std::min(min_height_error, leg.height_error);
        }
        avg_height_error /= 6.0;
        avg_tibia_angle /= 6.0;

        std::cout << "\nEstadísticas de las 6 patas:" << std::endl;
        std::cout << "  - Error de altura promedio: " << avg_height_error << "mm" << std::endl;
        std::cout << "  - Error de altura mín/máx: " << min_height_error << "/" << max_height_error << "mm" << std::endl;
        std::cout << "  - Ángulo tibia promedio: " << avg_tibia_angle << "°" << std::endl;
        std::cout << "  - Todas las patas usan los mismos ángulos articulares" << std::endl;
        std::cout << "  - Diferencias en posición se deben a BASE_THETA_OFFSETS" << std::endl;

        // Verificar cinemática inversa para la pata 0
        JointAngles best_angles(best_solution.coxa_deg * M_PI / 180.0,
                                best_solution.femur_deg * M_PI / 180.0,
                                best_solution.tibia_deg * M_PI / 180.0);
        Point3D verification = model.forwardKinematicsGlobalCoordinates(0, best_angles);
        std::cout << "\nVerificación FK (pata 0):" << std::endl;
        std::cout << "  - Posición calculada: (" << verification.x << ", " << verification.y << ", " << verification.z << ")" << std::endl;
        std::cout << "  - Diferencia: (" << (verification.x - best_solution.global_position.x) << ", "
                  << (verification.y - best_solution.global_position.y) << ", "
                  << (verification.z - best_solution.global_position.z) << ")" << std::endl;
    }

    return 0;
}
