#include "body_pose_config_factory.h"
#include "robot_model.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    // Configuración de parámetros del robot según AGENTS.md
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.standing_height = 150;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== VALIDACIÓN DE calculateServoAnglesForHeight ANALÍTICA ===" << std::endl;
    std::cout << "Parámetros del robot:" << std::endl;
    std::cout << "  - standing_height: " << p.standing_height << "mm" << std::endl;
    std::cout << "  - robot_height: " << p.robot_height << "mm" << std::endl;
    std::cout << "  - coxa_length: " << p.coxa_length << "mm" << std::endl;
    std::cout << "  - femur_length: " << p.femur_length << "mm" << std::endl;
    std::cout << "  - tibia_length: " << p.tibia_length << "mm" << std::endl;

    // Test para altura objetivo de 150mm
    std::cout << "\n=== PRUEBA PARA ALTURA 150mm ===" << std::endl;
    CalculatedServoAngles calc = calculateServoAnglesForHeight(150.0, p);

    std::cout << "Función calculateServoAnglesForHeight:" << std::endl;
    std::cout << "  - Válido: " << (calc.valid ? "SÍ" : "NO") << std::endl;
    std::cout << "  - Coxa: " << calc.coxa << "°" << std::endl;
    std::cout << "  - Femur: " << calc.femur << "°" << std::endl;
    std::cout << "  - Tibia: " << calc.tibia << "°" << std::endl;

    if (calc.valid) {
        // Verificar con cinemática directa
        JointAngles angles(calc.coxa * M_PI / 180.0, calc.femur * M_PI / 180.0, calc.tibia * M_PI / 180.0);
        Point3D fk = model.forwardKinematicsGlobalCoordinates(0, angles);

        std::cout << "\nVerificación con cinemática directa:" << std::endl;
        std::cout << "  - Posición FK: (" << fk.x << ", " << fk.y << ", " << fk.z << ")" << std::endl;
        std::cout << "  - Altura absoluta: " << std::abs(fk.z) << "mm" << std::endl;
        std::cout << "  - Altura objetivo: " << 150.0 << "mm" << std::endl;
        std::cout << "  - Error altura: " << (std::abs(fk.z) - 150.0) << "mm" << std::endl;

        // Comparar con valores conocidos del finetune_angles_test
        std::cout << "\nComparación con finetune_angles_test:" << std::endl;
        std::cout << "  - Femur esperado: -35.00°" << std::endl;
        std::cout << "  - Femur calculado: " << calc.femur << "°" << std::endl;
        std::cout << "  - Diferencia femur: " << (calc.femur - (-35.0)) << "°" << std::endl;

        std::cout << "  - Tibia esperada: 35.00°" << std::endl;
        std::cout << "  - Tibia calculada: " << calc.tibia << "°" << std::endl;
        std::cout << "  - Diferencia tibia: " << (calc.tibia - 35.0) << "°" << std::endl;

        // Verificar que es una solución válida
        bool success = true;
        if (std::abs(std::abs(fk.z) - 150.0) > 0.1) {
            std::cout << "  ❌ ERROR: Altura no coincide con objetivo" << std::endl;
            success = false;
        }
        if (std::abs(calc.femur - (-35.0)) > 0.1) {
            std::cout << "  ❌ ERROR: Ángulo femur no coincide con esperado" << std::endl;
            success = false;
        }
        if (std::abs(calc.tibia - 35.0) > 0.1) {
            std::cout << "  ❌ ERROR: Ángulo tibia no coincide con esperado" << std::endl;
            success = false;
        }

        if (success) {
            std::cout << "  ✅ ÉXITO: Función analítica funciona correctamente" << std::endl;
        }
    } else {
        std::cout << "  ❌ ERROR: Función no devolvió una solución válida" << std::endl;
    }

    // Test para verificar que la función es mucho más rápida
    std::cout << "\n=== PRUEBA DE VELOCIDAD ===" << std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
        CalculatedServoAngles test_calc = calculateServoAnglesForHeight(150.0, p);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Tiempo para 1000 cálculos: " << duration.count() << " microsegundos" << std::endl;
    std::cout << "Tiempo promedio por cálculo: " << (duration.count() / 1000.0) << " microsegundos" << std::endl;
    std::cout << "Comparación: finetune_angles_test tarda ~60 segundos (60,000,000 microsegundos)" << std::endl;
    std::cout << "Aceleración: ~" << (60000000.0 / (duration.count() / 1000.0)) << "x más rápido" << std::endl;

    return 0;
}
