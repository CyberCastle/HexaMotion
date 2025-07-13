#include "body_pose_config.h"
#include "body_pose_config_factory.h"
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

// Valida la configuración de una pata usando cinemática inversa
bool validateLegConfigurationWithIK(const RobotModel &model, const LegConfiguration &config,
                                    double tolerance_mm = 1.0) {
    // Obtener la posición objetivo desde la configuración
    Point3D target_position = config.global_position;
    JointAngles target_angles = config.joint_angles;

    // Aplicar cinemática inversa para obtener ángulos calculados
    Point3D target = model.forwardKinematicsGlobalCoordinates(config.leg_index, target_angles);
    JointAngles ik = model.inverseKinematicsCurrentGlobalCoordinates(config.leg_index, target_angles, target);
    // Calcular la posición usando los ángulos IK
    Point3D fk = model.forwardKinematicsGlobalCoordinates(config.leg_index, ik);

    // Calcular error de posición
    double position_error = std::sqrt(
        std::pow(target.x - fk.x, 2) +
        std::pow(target.y - fk.y, 2) +
        std::pow(target.z - fk.z, 2));

    // Verificar si la solución es válida
    return position_error <= tolerance_mm;
}

// Encuentra los mejores ángulos para una altura dada del robot usando fuerza bruta
std::vector<AngleSolution> findOptimalAnglesForHeight(const RobotModel &model, double target_height,
                                                      double height_tolerance = 1.0) {
    std::vector<AngleSolution> solutions;
    const Parameters &p = model.getParams();

    std::cout << "Buscando soluciones para altura " << target_height << "mm..." << std::endl;
    std::cout << "Limitaciones reales del robot (AGENTS.md):" << std::endl;
    std::cout << "Rango coxa: [" << p.coxa_angle_limits[0] << "°, " << p.coxa_angle_limits[1] << "°]" << std::endl;
    std::cout << "Rango fémur: [" << p.femur_angle_limits[0] << "°, " << p.femur_angle_limits[1] << "°]" << std::endl;
    std::cout << "Rango tibia: [" << p.tibia_angle_limits[0] << "°, " << p.tibia_angle_limits[1] << "°]" << std::endl;

    int total_combinations = 0;
    int valid_combinations = 0;

    // Estrategia de fuerza bruta: recorrer solo fémur y tibia (coxa = 30°)
    double femur_step = 0.1; // Paso más pequeño para mayor precisión
    double tibia_step = 0.1;
    double coxa_deg = 0.0; // Coxa fija en 0°

    // Queremos la altura minima, entonces evaluamos desde el limite inferior hasta 0, lo que implica que el fémur esté apuntado hacia arriba.
    for (double femur_deg = p.femur_angle_limits[0]; femur_deg <= 1; femur_deg += femur_step) {
        // Queremos la altura mñinima, entonces evaluamos desde 0 hasta el limite superior de la tibia.
        for (double tibia_deg = -1; tibia_deg <= p.tibia_angle_limits[1]; tibia_deg += tibia_step) {
            total_combinations++;

            JointAngles test_angles(coxa_deg * M_PI / 180.0,
                                    femur_deg * M_PI / 180.0,
                                    tibia_deg * M_PI / 180.0);

            // Calcular posición global
            Point3D fk_result = model.forwardKinematicsGlobalCoordinates(0, test_angles);
            // Comparar magnitud de altura con target
            // Error de altura: magnitud de Z contra altura objetivo
            double height_error = std::abs(std::abs(fk_result.z) - target_height);

            // Solo considerar si la altura está dentro de la tolerancia
            if (height_error <= height_tolerance) {

                double tibia_angle = getTibiaAngleToGround(model, 0, test_angles);
                bool is_perpendicular = isTibiaPerpendicularToGround(model, 0, test_angles, 0.0); // Tolerancia aumentada

                // Solo guardar si la tibia está perpendicular
                if (is_perpendicular) {
                    AngleSolution solution;
                    solution.femur_deg = femur_deg;
                    solution.tibia_deg = tibia_deg;
                    solution.coxa_deg = coxa_deg;
                    solution.global_position = fk_result;
                    solution.height_error = height_error;
                    solution.tibia_angle = tibia_angle;
                    solution.score = 10.0 * height_error + std::abs(tibia_angle - 90.0);

                    solutions.push_back(solution);
                    valid_combinations++;
                }
            }
        }
    }

    std::cout << "Evaluadas " << total_combinations << " combinaciones, encontradas "
              << valid_combinations << " válidas" << std::endl;

    // Refinar las mejores soluciones con búsqueda más fina
    if (!solutions.empty()) {
        std::sort(solutions.begin(), solutions.end(),
                  [](AngleSolution &a, AngleSolution &b) {
                      return a.score < b.score;
                  });

        std::cout << "Refinando las mejores soluciones..." << std::endl;

        // Tomar las mejores 3 soluciones y refinarlas (reducido por rangos más pequeños)
        std::vector<AngleSolution> refined_solutions;
        for (int i = 0; i < std::min(3, (int)solutions.size()); i++) {
            AngleSolution &base = solutions[i];

            // Búsqueda fina alrededor de la mejor solución (solo fémur y tibia)
            for (double df = -2.0; df <= 2.0; df += 0.5) {
                for (double dt = -2.0; dt <= 2.0; dt += 0.5) {
                    double new_femur = base.femur_deg + df;
                    double new_tibia = base.tibia_deg + dt;
                    double new_coxa = 0.0; // Coxa fija en 0°

                    if (new_femur >= p.femur_angle_limits[0] && new_femur <= p.femur_angle_limits[1] &&
                        new_tibia >= p.tibia_angle_limits[0] && new_tibia <= p.tibia_angle_limits[1]) {

                        JointAngles test_angles(new_coxa * M_PI / 180.0,
                                                new_femur * M_PI / 180.0,
                                                new_tibia * M_PI / 180.0);

                        Point3D fk_result = model.forwardKinematicsGlobalCoordinates(0, test_angles);
                        // Error de altura: magnitud de Z contra altura objetivo
                        double height_error = std::abs(std::abs(fk_result.z) - target_height);

                        if (height_error <= height_tolerance) {
                            double tibia_angle = getTibiaAngleToGround(model, 0, test_angles);
                            bool is_perpendicular = isTibiaPerpendicularToGround(model, 0, test_angles, 3.0);

                            if (is_perpendicular) {
                                AngleSolution solution;
                                solution.femur_deg = new_femur;
                                solution.tibia_deg = new_tibia;
                                solution.coxa_deg = new_coxa;
                                solution.global_position = fk_result;
                                solution.height_error = height_error;
                                solution.tibia_angle = tibia_angle;
                                solution.score = 10.0 * height_error + std::abs(tibia_angle - 90.0);

                                refined_solutions.push_back(solution);
                            }
                        }
                    }
                }
            }
        }

        // Combinar y reordenar todas las soluciones
        solutions.insert(solutions.end(), refined_solutions.begin(), refined_solutions.end());

        std::cout << "Refinamiento completado. Total de soluciones: " << solutions.size() << std::endl;
    }

    // Ordenar por puntuación final
    std::sort(solutions.begin(), solutions.end(),
              [](AngleSolution &a, AngleSolution &b) {
                  return a.score < b.score;
              });

    return solutions;
}

// Calcula las configuraciones para las 6 patas usando los offsets angulares de BASE_THETA_OFFSETS
std::vector<LegConfiguration> calculateAllLegsConfiguration(const RobotModel &model, double target_height,
                                                            AngleSolution &base_solution) {
    std::vector<LegConfiguration> leg_configs;

    // Offsets angulares para cada pata (BASE_THETA_OFFSETS en grados)
    const double BASE_THETA_OFFSETS_DEG[6] = {-30.0, -90.0, -150.0, 150.0, 90.0, 30.0};

    std::cout << "\nCalculando configuraciones para las 6 patas:" << std::endl;
    std::cout << "Pata | Coxa° | Femur° | Tibia° | Pos_X   | Pos_Y   | Pos_Z   | H_Error | T_Angle" << std::endl;
    std::cout << "-----|-------|--------|--------|---------|---------|---------|---------|--------" << std::endl;

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
                  << std::setw(5) << std::setprecision(2) << base_solution.coxa_deg << " | "
                  << std::setw(6) << std::setprecision(2) << base_solution.femur_deg << " | "
                  << std::setw(6) << std::setprecision(2) << base_solution.tibia_deg << " | "
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
    p.tibia_angle_limits[0] = -50;
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
    std::vector<double> target_heights = {140.0}; // Rango de alturas a probar

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

        std::cout << "Encontradas " << solutions.size() << " soluciones válidas" << std::endl;

        // Usar la mejor solución para calcular las 6 patas
        AngleSolution &best_solution = solutions[0];

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

        // Validar las configuraciones con cinemática inversa
        std::cout << "\n--- VALIDACIÓN CON CINEMÁTICA INVERSA ---" << std::endl;
        int patas_validadas = 0;
        for (const auto &leg_config : all_legs) {
            bool ik_valid = validateLegConfigurationWithIK(model, leg_config, 1.0);
            if (ik_valid) {
                patas_validadas++;
            }
            std::cout << "Pata " << leg_config.leg_index << " (offset " << leg_config.base_angle_deg << "°): ";
            std::cout << "IK: " << (ik_valid ? "PASÓ" : "FALLÓ")
                      << std::endl;
        }
        std::cout << "Total: " << patas_validadas << "/6 patas validadas" << std::endl;

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
