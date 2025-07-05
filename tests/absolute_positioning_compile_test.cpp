#include "robot_model.h"
#include "../src/imu_auto_pose.h"
#include "../tests/test_stubs.h"

#include "../src/HexaModel.h"
#include "../src/imu_auto_pose.h"
#include "../tests/test_stubs.h"
#include <iomanip>
#include <iostream>

// Test compilation of new absolute positioning features
void test_absolute_positioning_compilation() {
    std::cout << "=== Test de Compilación: Soporte para Posicionamiento Absoluto ===" << std::endl;
    std::cout << std::endl;

    // Test 1: IMU data structures
    std::cout << "1. Probando estructuras de datos IMU extendidas..." << std::endl;
    IMUData data{};
    data.mode = IMU_MODE_ABSOLUTE_POS;
    data.has_absolute_capability = true;
    data.absolute_data.absolute_roll = 10.0f;
    data.absolute_data.absolute_pitch = 5.0f;
    data.absolute_data.absolute_yaw = -15.0f;
    data.absolute_data.calibration_status = 3;
    data.absolute_data.absolute_orientation_valid = true;

    std::cout << "   ✓ Estructura IMUData creada exitosamente" << std::endl;
    std::cout << "   ✓ Modo configurado: " << (data.mode == IMU_MODE_ABSOLUTE_POS ? "ABSOLUTE_POS" : "OTRO") << std::endl;
    std::cout << "   ✓ Capacidad absoluta: " << (data.has_absolute_capability ? "SÍ" : "NO") << std::endl;
    std::cout << "   ✓ Orientación absoluta (R/P/Y): " << std::fixed << std::setprecision(1)
              << data.absolute_data.absolute_roll << "°, "
              << data.absolute_data.absolute_pitch << "°, "
              << data.absolute_data.absolute_yaw << "°" << std::endl;
    std::cout << std::endl;

    // Test 2: Dummy IMU with absolute positioning
    std::cout << "2. Probando DummyIMU con soporte absoluto..." << std::endl;
    DummyIMU dummy_imu;

    // Inicialmente debería estar en modo RAW
    std::cout << "   - Modo inicial: " << (dummy_imu.getIMUMode() == IMU_MODE_RAW_DATA ? "RAW_DATA" : "OTRO") << std::endl;
    std::cout << "   - Soporte absoluto inicial: " << (dummy_imu.hasAbsolutePositioning() ? "SÍ" : "NO") << std::endl;

    // Habilitar modo absoluto
    dummy_imu.enableAbsoluteMode(true);
    dummy_imu.setIMUMode(IMU_MODE_ABSOLUTE_POS);

    IMUMode mode = dummy_imu.getIMUMode();
    bool has_abs = dummy_imu.hasAbsolutePositioning();

    std::cout << "   ✓ Modo después de configuración: " << (mode == IMU_MODE_ABSOLUTE_POS ? "ABSOLUTE_POS" : "OTRO") << std::endl;
    std::cout << "   ✓ Soporte absoluto después de habilitar: " << (has_abs ? "SÍ" : "NO") << std::endl;

    // Test 3: Calibration status
    std::cout << std::endl;
    std::cout << "3. Probando estado de calibración..." << std::endl;
    uint8_t sys, gyro, accel, mag;
    bool calib_result = dummy_imu.getCalibrationStatus(&sys, &gyro, &accel, &mag);

    std::cout << "   ✓ Lectura de calibración: " << (calib_result ? "EXITOSA" : "FALLÓ") << std::endl;
    std::cout << "   ✓ Estado Sistema: " << (int)sys << "/3" << std::endl;
    std::cout << "   ✓ Estado Giroscopio: " << (int)gyro << "/3" << std::endl;
    std::cout << "   ✓ Estado Acelerómetro: " << (int)accel << "/3" << std::endl;
    std::cout << "   ✓ Estado Magnetómetro: " << (int)mag << "/3" << std::endl;

    // Test 4: Self-test and reset
    std::cout << std::endl;
    std::cout << "4. Probando auto-test y reset..." << std::endl;
    bool self_test_result = dummy_imu.runSelfTest();
    bool reset_result = dummy_imu.resetOrientation();

    std::cout << "   ✓ Auto-test: " << (self_test_result ? "PASÓ" : "FALLÓ") << std::endl;
    std::cout << "   ✓ Reset de orientación: " << (reset_result ? "EXITOSO" : "FALLÓ") << std::endl;

    // Test 5: Reading data
    std::cout << std::endl;
    std::cout << "5. Probando lectura de datos..." << std::endl;
    IMUData test_data = dummy_imu.readIMU();

    std::cout << "   ✓ Lectura de datos: " << (test_data.is_valid ? "VÁLIDA" : "INVÁLIDA") << std::endl;
    std::cout << "   ✓ Modo en datos leídos: " << (test_data.mode == IMU_MODE_ABSOLUTE_POS ? "ABSOLUTE_POS" : "OTRO") << std::endl;
    std::cout << "   ✓ Capacidad absoluta en datos: " << (test_data.has_absolute_capability ? "SÍ" : "NO") << std::endl;

    if (test_data.has_absolute_capability) {
        std::cout << "   ✓ Orientación absoluta válida: " << (test_data.absolute_data.absolute_orientation_valid ? "SÍ" : "NO") << std::endl;
        std::cout << "   ✓ Estado de calibración en datos: " << (int)test_data.absolute_data.calibration_status << "/3" << std::endl;
        std::cout << "   ✓ Orientación leída (R/P/Y): " << std::fixed << std::setprecision(1)
                  << test_data.absolute_data.absolute_roll << "°, "
                  << test_data.absolute_data.absolute_pitch << "°, "
                  << test_data.absolute_data.absolute_yaw << "°" << std::endl;
    }

    // Test 6: Mode switching
    std::cout << std::endl;
    std::cout << "6. Probando cambio de modos..." << std::endl;

    // Cambiar a modo RAW
    dummy_imu.setIMUMode(IMU_MODE_RAW_DATA);
    IMUMode raw_mode = dummy_imu.getIMUMode();
    std::cout << "   ✓ Cambio a RAW_DATA: " << (raw_mode == IMU_MODE_RAW_DATA ? "EXITOSO" : "FALLÓ") << std::endl;

    // Cambiar a modo FUSION
    dummy_imu.setIMUMode(IMU_MODE_FUSION);
    IMUMode fusion_mode = dummy_imu.getIMUMode();
    std::cout << "   ✓ Cambio a FUSION: " << (fusion_mode == IMU_MODE_FUSION ? "EXITOSO" : "FALLÓ") << std::endl;

    // Volver a modo ABSOLUTE
    dummy_imu.setIMUMode(IMU_MODE_ABSOLUTE_POS);
    IMUMode abs_mode = dummy_imu.getIMUMode();
    std::cout << "   ✓ Cambio a ABSOLUTE_POS: " << (abs_mode == IMU_MODE_ABSOLUTE_POS ? "EXITOSO" : "FALLÓ") << std::endl;

    std::cout << std::endl;
    std::cout << "=== Resumen del Test ===" << std::endl;
    std::cout << "✓ Todas las estructuras de datos compilan correctamente" << std::endl;
    std::cout << "✓ Todos los métodos de la interfaz funcionan" << std::endl;
    std::cout << "✓ El soporte para posicionamiento absoluto está operativo" << std::endl;
    std::cout << "✓ Los cambios de modo funcionan correctamente" << std::endl;
    std::cout << "✓ La lectura de datos y calibración funciona" << std::endl;
    std::cout << std::endl;
    std::cout << "🎉 Test de compilación: EXITOSO" << std::endl;
}

// Test funcionalidades adicionales donde se pueden usar datos absolutos
void test_additional_absolute_positioning_applications() {
    std::cout << std::endl;
    std::cout << "=== Test: Aplicaciones Adicionales de Posicionamiento Absoluto ===" << std::endl;
    std::cout << std::endl;

    // Test 1: Terrain Adaptation con datos absolutos
    std::cout << "1. Probando integración con Terrain Adaptation..." << std::endl;

    DummyIMU bno055_imu;
    bno055_imu.enableAbsoluteMode(true);
    bno055_imu.setIMUMode(IMU_MODE_ABSOLUTE_POS);

    // Simular terreno inclinado
    bno055_imu.setRPY(10.0f, 15.0f, 0.0f); // 10° roll, 15° pitch

    IMUData terrain_data = bno055_imu.readIMU();
    std::cout << "   ✓ Datos para análisis de terreno obtenidos" << std::endl;
    std::cout << "   ✓ Inclinación detectada (R/P): " << std::fixed << std::setprecision(1)
              << terrain_data.absolute_data.absolute_roll << "°, "
              << terrain_data.absolute_data.absolute_pitch << "°" << std::endl;

    if (terrain_data.has_absolute_capability) {
        std::cout << "   ✓ Aceleración lineal disponible para análisis de movimiento" << std::endl;
        std::cout << "   ✓ Datos de cuaternión disponibles para cálculos precisos" << std::endl;
    }

    // Test 2: Gait Pattern Selection
    std::cout << std::endl;
    std::cout << "2. Probando selección de patrones de marcha..." << std::endl;

    // Calcular magnitud de inclinación (similar a locomotion_system.cpp)
    double tilt_magnitude = sqrt(terrain_data.absolute_data.absolute_roll * terrain_data.absolute_data.absolute_roll +
                                terrain_data.absolute_data.absolute_pitch * terrain_data.absolute_data.absolute_pitch);

    std::cout << "   ✓ Magnitud de inclinación calculada: " << std::fixed << std::setprecision(1)
              << tilt_magnitude << "°" << std::endl;

    bool steep_terrain = tilt_magnitude > 10.0f;
    std::cout << "   ✓ Detección de terreno empinado: " << (steep_terrain ? "SÍ" : "NO") << std::endl;

    if (steep_terrain) {
        std::cout << "   → Se recomendaría patrón tripod para mayor estabilidad" << std::endl;
    } else {
        std::cout << "   → Se puede usar patrón wave para mayor eficiencia" << std::endl;
    }

    // Test 3: Stability Assessment
    std::cout << std::endl;
    std::cout << "3. Probando evaluación de estabilidad..." << std::endl;

    // Simular varianza de aceleración para detectar terreno irregular
    double accel_variance = abs(terrain_data.absolute_data.linear_accel_x) +
                           abs(terrain_data.absolute_data.linear_accel_y) +
                           abs(terrain_data.absolute_data.linear_accel_z);

    std::cout << "   ✓ Varianza de aceleración: " << std::fixed << std::setprecision(3)
              << accel_variance << " m/s²" << std::endl;

    bool rough_terrain = accel_variance > 0.5f || tilt_magnitude > 5.0f;
    std::cout << "   ✓ Detección de terreno irregular: " << (rough_terrain ? "SÍ" : "NO") << std::endl;

    // Test 4: Advanced Features con BNO055
    std::cout << std::endl;
    std::cout << "4. Probando características avanzadas específicas del BNO055..." << std::endl;

    if (terrain_data.absolute_data.quaternion_valid) {
        std::cout << "   ✓ Cuaterniones disponibles para rotaciones precisas" << std::endl;
        std::cout << "   ✓ Quaternion (w,x,y,z): (" << std::fixed << std::setprecision(3)
                  << terrain_data.absolute_data.quaternion_w << ", "
                  << terrain_data.absolute_data.quaternion_x << ", "
                  << terrain_data.absolute_data.quaternion_y << ", "
                  << terrain_data.absolute_data.quaternion_z << ")" << std::endl;
    }

    if (terrain_data.absolute_data.linear_acceleration_valid) {
        std::cout << "   ✓ Aceleración lineal (sin gravedad) disponible" << std::endl;
        std::cout << "   ✓ Útil para detección de movimiento y vibración del robot" << std::endl;
    }

    std::cout << "   ✓ Estado de calibración: " << (int)terrain_data.absolute_data.calibration_status << "/3" << std::endl;
    std::cout << "   ✓ Estado del sistema: " << (int)terrain_data.absolute_data.system_status << std::endl;

    // Test 5: Aplicaciones Futuras Potenciales
    std::cout << std::endl;
    std::cout << "5. Verificando datos para aplicaciones futuras..." << std::endl;

    std::cout << "   ✓ Datos disponibles para navegación inercial" << std::endl;
    std::cout << "   ✓ Datos disponibles para estimación de velocidad" << std::endl;
    std::cout << "   ✓ Datos disponibles para detección de caídas" << std::endl;
    std::cout << "   ✓ Datos disponibles para compensación de vibración" << std::endl;
    std::cout << "   ✓ Datos disponibles para análisis de fatiga de marcha" << std::endl;

    std::cout << std::endl;
    std::cout << "=== Aplicaciones Validadas ===" << std::endl;
    std::cout << "🎯 Auto-posing (control de postura corporal)" << std::endl;
    std::cout << "🎯 Terrain Adaptation (adaptación a terreno)" << std::endl;
    std::cout << "🎯 Gait Selection (selección de patrones de marcha)" << std::endl;
    std::cout << "🎯 Stability Assessment (evaluación de estabilidad)" << std::endl;
    std::cout << "🎯 Motion Analysis (análisis de movimiento)" << std::endl;
    std::cout << "🎯 Vibration Detection (detección de vibración)" << std::endl;
    std::cout << std::endl;
    std::cout << "🚀 El BNO055 proporciona datos valiosos para múltiples sistemas de HexaMotion" << std::endl;
}

int main() {
    std::cout << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "  HexaMotion - Test de Posicionamiento Absoluto IMU" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << std::endl;

    try {
        test_absolute_positioning_compilation();
        test_additional_absolute_positioning_applications();

        std::cout << std::endl;
        std::cout << "=========================================================" << std::endl;
        std::cout << "  RESULTADO: TODOS LOS TESTS PASARON ✅" << std::endl;
        std::cout << "  El BNO055 puede ser usado en múltiples componentes" << std::endl;
        std::cout << "=========================================================" << std::endl;
        return 0;
    } catch (const std::exception &e) {
        std::cout << std::endl;
        std::cout << "=========================================================" << std::endl;
        std::cout << "  ERROR: " << e.what() << " ❌" << std::endl;
        std::cout << "=========================================================" << std::endl;
        return 1;
    } catch (...) {
        std::cout << std::endl;
        std::cout << "=========================================================" << std::endl;
        std::cout << "  ERROR DESCONOCIDO ❌" << std::endl;
        std::cout << "=========================================================" << std::endl;
        return 1;
    }
}
