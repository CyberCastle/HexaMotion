/**
 * @file kinematics_validation_test.cpp
 * @brief Validación de la implementación OpenSHC-style vs angle_calculus.cpp
 * @author HexaMotion Team
 * @version 2.0
 * @date 2024
 *
 * Este test compara la nueva implementación OpenSHC-style en HexaMotion
 * con la lógica geométrica precisa de angle_calculus.cpp
 */

#include "body_pose_config_factory.h"
#include "robot_model.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

// Constantes de angle_calculus.cpp
constexpr double A_COXA = 50.0;   // mm
constexpr double B_FEMUR = 101.0; // mm
constexpr double C_TIBIA = 208.0; // mm

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// Estructura para ángulos de angle_calculus
struct CalcAngles {
    double theta1; // °  coxa-fémur
    double theta2; // °  fémur-tibia
    bool valid;    // solución dentro de límites
};

constexpr double FEMUR_MIN_DEG = -75.0; // °  límites de servo
constexpr double FEMUR_MAX_DEG = 75.0;  // °  límites de servo
constexpr double TIBIA_MIN_DEG = -45.0; // °  límites de servo
constexpr double TIBIA_MAX_DEG = 45.0;  // °  límites de servo

/**
 * @brief  Calcula los ángulos de servo para una altura dada, asegurando que la tibia
 *         sea perpendicular al suelo (α = 0).
 *
 * @param H_mm  Altura desde el plano de la base de la pierna hasta la punta [mm].
 * @return      Estructura con los ángulos θ₁ y θ₂ en grados, y un booleano valid.
 */
CalcAngles calcLegAngles(double H_mm) {
    CalcAngles best{0, 0, false};
    double bestErr = std::numeric_limits<double>::infinity();

    // ---------------------------------------
    // 1) RANGOS (radianes)
    const double alphaMin = -75.0 * DEG2RAD;
    const double alphaMax = 75.0 * DEG2RAD;
    const double betaMin = -45.0 * DEG2RAD;
    const double betaMax = 45.0 * DEG2RAD;
    const double dBeta = 0.1 * DEG2RAD; // paso, ajústalo si quieres más precisión

    // 2) LONGITUDES
    const double A = A_COXA;
    const double B = B_FEMUR;
    const double C = C_TIBIA;

    // 3) BARRIDO de β
    for (double beta = betaMin; beta <= betaMax; beta += dBeta) {
        // discriminante de la misma ecuación que tenías:
        //    C·cosα - (A + B·cosβ)·sinα  = H_mm
        double sum = A + B * std::cos(beta);
        double discriminant = sum * sum - (H_mm * H_mm - C * C);
        if (discriminant < 0.0)
            continue;

        double sqrtD = std::sqrt(discriminant);
        // dos raíces en α
        for (int sign : {-1, +1}) {
            double t = (-sum + sign * sqrtD) / (H_mm + C);
            double alpha = 2.0 * std::atan(t);
            if (alpha < alphaMin || alpha > alphaMax)
                continue;

            //  Medir error de verticalidad real
            double err = std::fabs(alpha + beta);
            if (err >= bestErr)
                continue;

            // printf("DEBUG: α=%.2f°, β=%.2f° -> err=%.2f\n", alpha * RAD2DEG, beta * RAD2DEG, err);
            // 4) Paso a grados y a ángulos de servo
            double theta1 = (alpha - beta) * RAD2DEG; // θ₁ = α - β
            double theta2 = -beta * RAD2DEG;          // θ₂ = −β

            // 5) LÍMITES de servo
            if (theta1 < -75.0 || theta1 > 75.0)
                continue;
            if (theta2 < -45.0 || theta2 > 45.0)
                continue;

            // 6) ERROR de verticalidad: α debe ser 0 para tibia perpendicular
            bestErr = err;
            best = {theta1, theta2, true};
        }
    }

    return best;
}

/**
 * @brief  Calcula la altura del extremo del pie por debajo de la base de la pierna
 *         (eje de unión coxa–fémur), a partir de los dos ángulos de servo.
 *
 * @param theta1_deg  Ángulo del servo de fémur en grados.
 *                    0° → fémur horizontal; + → levanta el pie.
 * @param theta2_deg  Ángulo del servo de tibia en grados.
 *                    0° → tibia vertical; + → inclina la tibia hacia atrás.
 * @param[out] valid  Se pone true si ambos ángulos estaban dentro de sus límites.
 * @return            Altura desde el plano de la base de la pierna hasta la punta [mm].
 */
double calcHeight(double theta1_deg, double theta2_deg, bool &valid) {
    valid = false;

    // Comprobar límites de los ángulos relativos
    if (theta1_deg < -75.0 || theta1_deg > 75.0)
        return 0.0;
    if (theta2_deg < -45.0 || theta2_deg > 45.0)
        return 0.0;

    // Convertir a radianes
    double theta1 = theta1_deg * DEG2RAD;
    double theta2 = theta2_deg * DEG2RAD;

    // Relaciones geométricas empleadas en la inversa:
    //   θ₂ = −β     ⇒  β = −θ₂
    //   θ₁ = β − α  ⇒  α = β − θ₁
    double beta = -theta2;
    double alpha = beta - theta1;

    // Chequeo opcional de los límites absolutos de α y β
    if (alpha < -75.0 * DEG2RAD || alpha > 75.0 * DEG2RAD)
        return 0.0;
    if (beta < -45.0 * DEG2RAD || beta > 45.0 * DEG2RAD)
        return 0.0;

    // Altura alcanzada (positivo hacia abajo)
    double H_mm = A_COXA * std::sin(alpha) + B_FEMUR * std::sin(beta) + C_TIBIA;

    valid = true;
    return H_mm;
}

class KinematicsValidator {
  private:
    Parameters params;
    std::unique_ptr<RobotModel> model;

  public:
    KinematicsValidator() {
        setupParameters();
        model = std::make_unique<RobotModel>(params);
    }

    void setupParameters() {
        // Usar las mismas dimensiones que angle_calculus.cpp
        params.hexagon_radius = 200.0f;
        params.coxa_length = A_COXA;
        params.femur_length = B_FEMUR;
        params.tibia_length = C_TIBIA;
        params.robot_height = 120.0f;

        // Límites de angle_calculus.cpp
        params.coxa_angle_limits[0] = -75.0f;
        params.coxa_angle_limits[1] = 75.0f;
        params.femur_angle_limits[0] = -75.0f;
        params.femur_angle_limits[1] = 75.0f;
        params.tibia_angle_limits[0] = -45.0f;
        params.tibia_angle_limits[1] = 45.0f;

        // Configuración IK
        params.ik.max_iterations = 50;
        params.ik.pos_threshold_mm = 0.5f;

        // Asegurar que use parámetros DH por defecto (no custom)
        params.use_custom_dh_parameters = false; // Use analytic IK/FK for validation

        // Debug: verificar que los parámetros se establecen correctamente
        std::cout << "DEBUG params setup: hexagon=" << params.hexagon_radius
                  << ", coxa=" << params.coxa_length
                  << ", femur=" << params.femur_length
                  << ", tibia=" << params.tibia_length << std::endl;
    }

    void validateVerticalReach() {
        std::cout << "\n=== VALIDACIÓN: ÁNGULOS SERVO FEMUR Y TIBIA ===" << std::endl;
        std::cout << "Comparando alturas: angle_calculus.cpp vs calculateServoAnglesForHeight (OpenSHC)" << std::endl;
        std::cout << std::string(120, '-') << std::endl;
        std::cout << "Height | angle_calculus (femur,tibia,alpha,beta) | calculateServoAnglesForHeight (femur,tibia,alpha,beta) | Δfemur | Δtibia | Status" << std::endl;
        std::cout << "(mm)   | (deg, deg, deg, deg)                   | (deg, deg, deg, deg)                                 | (deg)  | (deg)   |" << std::endl;
        std::cout << std::string(120, '-') << std::endl;

        // Test heights from angle_calculus range
        std::vector<double> test_heights = {
            120.0, 140.0, 160.0, 180.0, 200.0, 220.0, 240.0, 280.0, 320.0, 350.0};

        int passed = 0, total = 0;

        for (double height : test_heights) {
            total++;
            // 1. Obtener ángulos de referencia con angle_calculus.cpp
            CalcAngles ref_solution = calcLegAngles(height);
            CalculatedServoAngles calc_solution = calculateServoAnglesForHeight(height, params);

            if (!ref_solution.valid) {
                std::cout << std::setw(6) << height << " | INVALID SOLUTION             | INVALID SOLUTION                        |   N/A  |   N/A   | SKIP" << std::endl;
                total--;
                continue;
            }

            // Mostrar ambos resultados explícitamente
            double femur1 = ref_solution.theta1; // angle_calculus femur
            double tibia1 = ref_solution.theta2; // angle_calculus tibia
            double theta1_rad = femur1 * DEG2RAD;
            double theta2_rad = tibia1 * DEG2RAD;
            double beta1 = -theta2_rad;         // β = −θ₂
            double alpha1 = theta1_rad + beta1; // α = θ₁ + β
            double alpha1_deg = alpha1 * RAD2DEG;
            double beta1_deg = beta1 * RAD2DEG;

            double femur2 = calc_solution.femur; // calculateServoAnglesForHeight femur
            double tibia2 = calc_solution.tibia; // calculateServoAnglesForHeight tibia
            double theta1b_rad = femur2 * DEG2RAD;
            double theta2b_rad = tibia2 * DEG2RAD;
            double beta2 = -theta2b_rad;
            double alpha2 = theta1b_rad + beta2;
            double alpha2_deg = alpha2 * RAD2DEG;
            double beta2_deg = beta2 * RAD2DEG;

            double dfemur = std::abs(femur1 - femur2);
            double dtibia = std::abs(tibia1 - tibia2);
            bool match = (dfemur < 0.1 && dtibia < 0.1);
            if (match)
                passed++;

            std::cout << std::setw(6) << height << " | "
                      << std::setw(8) << femur1 << ", " << std::setw(8) << tibia1 << ", " << std::setw(8) << alpha1_deg << ", " << std::setw(8) << beta1_deg << " | "
                      << std::setw(8) << femur2 << ", " << std::setw(8) << tibia2 << ", " << std::setw(8) << alpha2_deg << ", " << std::setw(8) << beta2_deg << " | "
                      << std::setw(6) << dfemur << " | "
                      << std::setw(6) << dtibia << " | "
                      << (match ? "MATCH" : "DIFF") << std::endl;
        }
        std::cout << std::string(120, '-') << std::endl;
        std::cout << "Resultados: " << passed << "/" << total << " tests coinciden ("
                  << std::fixed << std::setprecision(1) << (100.0 * passed / total) << "%)" << std::endl;
    }

    void validateAngleConsistency() {
        std::cout << "\n=== VALIDACIÓN: CONSISTENCIA FORWARD/INVERSE ===" << std::endl;
        std::cout << "Probando que FK(IK(punto)) = punto (altura relativa a base de pierna)" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        // Test angle pairs from angle_calculus valid range
        std::vector<std::pair<double, double>> test_angles = {
            {0.0, 0.0}, {10.0, -10.0}, {-10.0, 10.0}, {20.0, -20.0}, {-20.0, 20.0}, {30.0, -30.0}, {-30.0, 30.0}, {45.0, -45.0}};

        int passed = 0, total = 0;

        std::cout << "θ1     θ2     | AngleCalc Height | HexaMotion Height | Error   | Status" << std::endl;
        std::cout << "(deg)  (deg)  | (mm)             | (mm)              | (mm)    |" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        for (auto angles : test_angles) {
            total++;

            double theta1 = angles.first;
            double theta2 = angles.second;

            // angle_calculus forward kinematics
            bool valid;
            double ref_height = calcHeight(theta1, theta2, valid);

            if (!valid) {
                std::cout << std::setw(6) << theta1 << " " << std::setw(6) << theta2
                          << " | INVALID          | N/A               | N/A     | SKIP" << std::endl;
                total--; // No contar tests inválidos
                continue;
            }

            // Convertir a target point en frame global del robot
            // angle_calculus.cpp asume que el leg se mueve verticalmente desde el cuerpo
            // Necesitamos crear un target que represente la altura deseada
            JointAngles zero_angles(0, 0, 0);
            Point3D base_global = model->getAnalyticLegBasePosition(0);

            // Target en la misma posición X,Y que la base, pero con la altura deseada
            Point3D target_global;
            target_global.x = base_global.x;
            target_global.y = base_global.y;
            target_global.z = base_global.z - ref_height; // Altura hacia abajo

            Point3D base_globa2 = model->transformLocalToGlobalCoordinates(0, target_global, zero_angles);

            // HexaMotion IK -> FK (los ángulos se manejan internamente en radianes)
            JointAngles hexa_angles = model->inverseKinematicsGlobalCoordinates(0, base_globa2);
            Point3D hexa_tip_global = model->forwardKinematicsGlobalCoordinates(0, hexa_angles);

            // Calcular altura relativa a la base de la pierna
            double hexa_height = hexa_tip_global.z;

            // Error en altura (principal métrica para comparación con angle_calculus)
            double height_error = std::abs(hexa_height - ref_height);

            bool test_passed = height_error < 10.0f; // Tolerancia de 10mm
            if (test_passed)
                passed++;

            std::cout << std::setw(6) << theta1 << " " << std::setw(6) << theta2
                      << " | " << std::setw(15) << ref_height
                      << " | " << std::setw(16) << hexa_height
                      << " | " << std::setw(6) << height_error
                      << "  | " << (test_passed ? "PASS" : "FAIL") << std::endl;
        }

        std::cout << std::string(70, '-') << std::endl;
        std::cout << "Resultados: " << passed << "/" << total << " tests pasaron ("
                  << std::fixed << std::setprecision(1) << (100.0 * passed / total) << "%)" << std::endl;
    }

    void validateWorkspaceComparison() {
        std::cout << "\n=== VALIDACIÓN: COMPARACIÓN DE WORKSPACE ===" << std::endl;
        std::cout << "Analizando límites de workspace entre implementaciones (altura relativa a base de pierna)" << std::endl;
        std::cout << std::string(50, '-') << std::endl;

        // Calcular rango de alturas con angle_calculus
        double min_height = 1000.0, max_height = 0.0;
        int valid_solutions = 0;

        for (double h = 100.0; h <= 400.0; h += 5.0) {
            CalcAngles sol = calcLegAngles(h);
            if (sol.valid) {
                min_height = std::min(min_height, h);
                max_height = std::max(max_height, h);
                valid_solutions++;
            }
        }

        std::cout << "angle_calculus.cpp workspace:" << std::endl;
        std::cout << "  Altura mínima: " << min_height << " mm" << std::endl;
        std::cout << "  Altura máxima: " << max_height << " mm" << std::endl;
        std::cout << "  Rango total: " << (max_height - min_height) << " mm" << std::endl;
        std::cout << "  Soluciones válidas: " << valid_solutions << "/61" << std::endl;

        // Calcular estadísticas de HexaMotion
        int hexa_valid = 0;
        double hexa_min_error = 1000.0f, hexa_max_error = 0.0f;
        double total_error = 0.0f;

        // Obtener posición de la base de la pierna
        JointAngles zero_angles(0, 0, 0);
        Point3D base_global = model->getDHLegBasePosition(0);

        for (double h = min_height; h <= max_height; h += 5.0) {
            // Target en la misma posición X,Y que la base, pero con la altura deseada
            Point3D target_global;
            target_global.x = base_global.x;
            target_global.y = base_global.y;
            target_global.z = base_global.z - h; // Altura hacia abajo

            JointAngles hexa_sol = model->inverseKinematicsGlobalCoordinates(0, target_global);
            Point3D hexa_tip_global = model->forwardKinematicsGlobalCoordinates(0, hexa_sol);

            // Calcular altura relativa a la base de la pierna
            double hexa_height = -(hexa_tip_global.z - base_global.z);

            double error = std::abs(hexa_height - h); // Error en altura relativa
            if (error < 20.0f) {                      // Tolerancia razonable
                hexa_valid++;
                hexa_min_error = std::min(hexa_min_error, error);
                hexa_max_error = std::max(hexa_max_error, error);
                total_error += error;
            }
        }

        int hexa_total = (max_height - min_height) / 5.0 + 1;
        std::cout << "\nHexaMotion workspace (altura relativa a base de pierna):" << std::endl;
        std::cout << "  Soluciones válidas: " << hexa_valid << "/" << hexa_total << std::endl;
        std::cout << "  Error mínimo: " << hexa_min_error << " mm" << std::endl;
        std::cout << "  Error máximo: " << hexa_max_error << " mm" << std::endl;
        std::cout << "  Error promedio: " << (hexa_valid > 0 ? total_error / hexa_valid : 0.0f) << " mm" << std::endl;

        double coverage = 100.0f * hexa_valid / hexa_total;
        std::cout << "  Cobertura de workspace: " << std::fixed << std::setprecision(1) << coverage << "%" << std::endl;
    }

    void runFullValidation() {
        std::cout << "===== VALIDACIÓN COMPLETA: HEXAMOTION vs ANGLE_CALCULUS =====" << std::endl;
        std::cout << "Dimensiones del robot:" << std::endl;
        std::cout << "  Coxa: " << A_COXA << " mm" << std::endl;
        std::cout << "  Fémur: " << B_FEMUR << " mm" << std::endl;
        std::cout << "  Tibia: " << C_TIBIA << " mm" << std::endl;
        std::cout << "  Radio hexágono: " << params.hexagon_radius << " mm" << std::endl;

        validateVerticalReach();
        validateAngleConsistency();
        validateWorkspaceComparison();

        std::cout << "\n===== VALIDACIÓN COMPLETA =====" << std::endl;
    }
};

int main() {
    KinematicsValidator validator;
    validator.runFullValidation();
    return 0;
}
