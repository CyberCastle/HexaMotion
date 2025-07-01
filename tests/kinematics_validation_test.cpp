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

#include "HexaModel.h"
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
struct AngleCalcAngles {
    double theta1; // °  coxa-fémur
    double theta2; // °  fémur-tibia
    bool valid;    // solución dentro de límites
};

// Implementación analítica equivalente a angle_calculus.cpp
AngleCalcAngles calcLegAngles(double H_mm) {
    const double alphaMin = -75.0 * DEG2RAD;
    const double alphaMax = 75.0 * DEG2RAD;
    const double betaMin = -45.0 * DEG2RAD;
    const double betaMax = 45.0 * DEG2RAD;

    AngleCalcAngles best{0, 0, false};
    double bestScore = 1e9;
    double bestOrient = std::numeric_limits<double>::max();

    for (double beta = betaMin; beta <= betaMax; beta += 0.5 * DEG2RAD) {
        double sum = A_COXA + B_FEMUR * std::cos(beta);
        double discriminant =
            sum * sum - (H_mm * H_mm - C_TIBIA * C_TIBIA);
        if (discriminant < 0.0)
            continue;

        double sqrt_disc = std::sqrt(discriminant);
        for (int sign = -1; sign <= 1; sign += 2) {
            double t = (-sum + sign * sqrt_disc) / (H_mm + C_TIBIA);
            double alpha = 2.0 * std::atan(t);

            if (alpha < alphaMin || alpha > alphaMax)
                continue;

            double theta1 = (alpha - beta) * RAD2DEG;
            double theta2 = -beta * RAD2DEG;

            if (theta1 < -75.0 || theta1 > 75.0)
                continue;
            if (theta2 < -45.0 || theta2 > 45.0)
                continue;

            double orient_err = std::fabs(alpha);
            double score = std::fabs(alpha) + std::fabs(beta);
            if (orient_err < bestOrient ||
                (std::abs(orient_err - bestOrient) < 1e-6 && score < bestScore)) {
                best = {theta1, theta2, true};
                bestScore = score;
                bestOrient = orient_err;
            }
        }
    }
    return best;
}

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

    // Relaciones geométricas del modelo DH
    double beta = -theta2;        // β = −θ₂
    double alpha = theta1 + beta; // α = θ₁ + β

    if (alpha < -75.0 * DEG2RAD || alpha > 75.0 * DEG2RAD)
        return 0.0;
    if (beta < -45.0 * DEG2RAD || beta > 45.0 * DEG2RAD)
        return 0.0;

    // Altura según la cadena DH
    double H_mm = C_TIBIA * std::cos(alpha) -
                  A_COXA * std::sin(alpha) -
                  B_FEMUR * std::sin(alpha) * std::cos(beta);

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
        std::cout << "Comparando alturas: angle_calculus.cpp vs HexaMotion FK (altura relativa a base de pierna)" << std::endl;
        std::cout << std::string(90, '-') << std::endl;

        // Test heights from angle_calculus range
        std::vector<double> test_heights = {
            120.0, 140.0, 160.0, 180.0, 200.0, 220.0, 240.0, 280.0, 320.0, 350.0};

        int passed = 0, total = 0;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Height | angle_calculus        | HexaMotion FK Local  | Error     | Status" << std::endl;
        std::cout << "(mm)   | θ1     θ2     α    β  | α      β     H_calc  | H_err     |" << std::endl;
        std::cout << std::string(90, '-') << std::endl;

        for (double height : test_heights) {
            total++;

            // 1. Obtener ángulos de referencia con angle_calculus.cpp
            AngleCalcAngles ref_solution = calcLegAngles(height);

            if (!ref_solution.valid) {
                std::cout << std::setw(6) << height << " | "
                          << "INVALID SOLUTION       | N/A                   | N/A       | SKIP" << std::endl;
                total--;
                continue;
            }

            // 2. Convertir ángulos relativos de angle_calculus a ángulos absolutos DH
            double theta1_rad = ref_solution.theta1 * DEG2RAD;
            double theta2_rad = ref_solution.theta2 * DEG2RAD;

            double beta = -theta2_rad;        // β = −θ₂
            double alpha = theta1_rad + beta; // α = θ₁ + β

            // Convertir a grados para mostrar y comparar
            double alpha_deg = alpha * RAD2DEG; // Ángulo femur
            double beta_deg = beta * RAD2DEG;   // Ángulo tibia

            // 3. PRUEBA DIRECTA: Usar los ángulos convertidos directamente en HexaMotion FK
            JointAngles test_angles;
            test_angles.coxa = 0.0f;       // Sin rotación horizontal
            test_angles.femur = alpha_deg; // Ángulo femur absoluto
            test_angles.tibia = beta_deg;  // Ángulo tibia absoluto

            // 4. Calcular la posición de la punta con FK en frame global del robot
            Point3D tip_global = model->forwardKinematics(0, test_angles);

            // 5. Calcular la posición de la base de la pierna (con ángulos cero)
            JointAngles zero_angles(0, 0, 0);
            Point3D base_global = model->getDHLegBasePosition(0);

            // 6. Calcular la altura relativa a la base de la pierna
            // La altura es la diferencia en Z (negativa hacia abajo)
            double hexa_height = -(tip_global.z - base_global.z);
            double height_error = std::abs(hexa_height - height);

            // 7. Verificar que la fórmula de angle_calculus coincida
            double expected_height = C_TIBIA * std::cos(alpha) -
                                     A_COXA * std::sin(alpha) -
                                     B_FEMUR * std::sin(alpha) * std::cos(beta);
            double formula_error = std::abs(expected_height - height);

            // El test pasa si las alturas coinciden
            bool height_match = height_error < 2.0f;   // ±2mm tolerancia
            bool formula_match = formula_error < 0.1f; // La fórmula debe ser exacta

            bool test_passed = height_match && formula_match;
            if (test_passed)
                passed++;

            std::cout << std::setw(6) << height << " | "
                      << std::setw(6) << ref_solution.theta1 << " "
                      << std::setw(6) << ref_solution.theta2 << " "
                      << std::setw(5) << alpha_deg << " "
                      << std::setw(4) << beta_deg << " | "
                      << std::setw(6) << alpha_deg << " "
                      << std::setw(5) << beta_deg << " "
                      << std::setw(7) << hexa_height << " | "
                      << std::setw(8) << height_error << "  | "
                      << (test_passed ? "PASS" : "FAIL") << std::endl;

            // Debug detallado para casos problemáticos o muestras
            if (!test_passed || height == 120.0 || height == 200.0) {
                std::cout << "  DEBUG altura " << height << ":" << std::endl;
                std::cout << "    angle_calculus: θ1=" << ref_solution.theta1
                          << "°, θ2=" << ref_solution.theta2 << "°" << std::endl;
                std::cout << "    Conversión: α=" << alpha_deg
                          << "°, β=" << beta_deg << "°" << std::endl;
                std::cout << "    Fórmula angle_calculus: " << expected_height << "mm" << std::endl;
                std::cout << "    HexaMotion base global: (" << base_global.x << ", " << base_global.y
                          << ", " << base_global.z << ")" << std::endl;
                std::cout << "    HexaMotion tip global: (" << tip_global.x << ", " << tip_global.y
                          << ", " << tip_global.z << ")" << std::endl;
                std::cout << "    HexaMotion altura relativa: " << hexa_height << "mm" << std::endl;
                std::cout << "    Error altura: " << height_error << "mm" << std::endl;
                std::cout << "    Error fórmula: " << formula_error << "mm" << std::endl;
            }
        }

        std::cout << std::string(90, '-') << std::endl;
        std::cout << "Resultados: " << passed << "/" << total << " tests pasaron ("
                  << std::fixed << std::setprecision(1) << (100.0 * passed / total) << "%)" << std::endl;

        if (passed == total) {
            std::cout << "✅ ÉXITO: La cinemática directa (FK) de HexaMotion coincide con angle_calculus.cpp" << std::endl;
            std::cout << "   Esto confirma que el cálculo de altura es correcto." << std::endl;
        } else {
            std::cout << "❌ ERROR: Discrepancias en el cálculo de altura." << std::endl;
            std::cout << "   Revisar los parámetros DH o la implementación de FK." << std::endl;
        }
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
            Point3D base_global = model->getDHLegBasePosition(0);

            // Target en la misma posición X,Y que la base, pero con la altura deseada
            Point3D target_global;
            target_global.x = base_global.x;
            target_global.y = base_global.y;
            target_global.z = base_global.z - ref_height; // Altura hacia abajo

            // HexaMotion IK -> FK
            JointAngles hexa_angles = model->inverseKinematics(0, target_global);
            Point3D hexa_tip_global = model->forwardKinematics(0, hexa_angles);

            // Calcular altura relativa a la base de la pierna
            double hexa_height = -(hexa_tip_global.z - base_global.z);

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
            AngleCalcAngles sol = calcLegAngles(h);
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

            JointAngles hexa_sol = model->inverseKinematics(0, target_global);
            Point3D hexa_tip_global = model->forwardKinematics(0, hexa_sol);

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
        // validateAngleConsistency();
        // validateWorkspaceComparison();

        std::cout << "\n===== VALIDACIÓN COMPLETA =====" << std::endl;
    }
};

int main() {
    KinematicsValidator validator;
    validator.runFullValidation();
    return 0;
}
