# Soporte para IMUs con Posicionamiento Absoluto en HexaMotion

## Introducción

HexaMotion ahora incluye soporte completo para IMUs avanzados como el **BNO055** que proporcionan información de posición absoluta y algoritmos de fusión de sensores integrados. Esta funcionalidad permite elegir entre usar los algoritmos del sensor o los de la librería para obtener la mejor precisión según las necesidades específicas.

## Características Implementadas

### 1. Modos de Operación IMU

El sistema ahora soporta tres modos de operación:

-   **`IMU_MODE_RAW_DATA`**: Utiliza datos crudos del sensor con algoritmos de la librería
-   **`IMU_MODE_FUSION`**: Utiliza la fusión de sensores interna del IMU
-   **`IMU_MODE_ABSOLUTE_POS`**: Utiliza el posicionamiento absoluto del sensor (ej. BNO055)

### 2. Estructura de Datos Extendida

#### `IMUAbsoluteData`

```cpp
struct IMUAbsoluteData {
    float absolute_roll, absolute_pitch, absolute_yaw;  // Orientación absoluta en grados
    float linear_accel_x, linear_accel_y, linear_accel_z;  // Aceleración lineal (sin gravedad)
    float quaternion_w, quaternion_x, quaternion_y, quaternion_z;  // Cuaternión de orientación
    bool absolute_orientation_valid;  // Validez de orientación absoluta
    bool linear_acceleration_valid;   // Validez de aceleración lineal
    bool quaternion_valid;            // Validez de datos de cuaternión
    uint8_t calibration_status;       // Estado de calibración (0-3, 3=completamente calibrado)
    uint8_t system_status;            // Estado del sistema del sensor
    uint8_t self_test_result;         // Resultado de auto-test
};
```

#### `IMUData` Actualizada

```cpp
struct IMUData {
    // Datos básicos IMU (siempre disponibles)
    float roll, pitch, yaw;           // Ángulos de Euler en grados
    float accel_x, accel_y, accel_z;  // Aceleración cruda en m/s²
    float gyro_x, gyro_y, gyro_z;     // Velocidad angular en rad/s
    bool is_valid;                    // Validez de datos básicos

    // Datos extendidos para IMUs avanzados
    IMUAbsoluteData absolute_data;    // Datos de posición absoluta (cuando está disponible)
    IMUMode mode;                     // Modo de operación actual
    bool has_absolute_capability;     // Si el IMU soporta posicionamiento absoluto
};
```

### 3. Interfaz IIMUInterface Extendida

La interfaz ahora incluye métodos adicionales:

```cpp
class IIMUInterface {
public:
    // Métodos existentes
    virtual bool initialize() = 0;
    virtual IMUData readIMU() = 0;
    virtual bool calibrate() = 0;
    virtual bool isConnected() = 0;

    // Nuevos métodos para soporte avanzado
    virtual bool setIMUMode(IMUMode mode) = 0;
    virtual IMUMode getIMUMode() const = 0;
    virtual bool hasAbsolutePositioning() const = 0;
    virtual bool getCalibrationStatus(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag) = 0;
    virtual bool runSelfTest() = 0;
    virtual bool resetOrientation() = 0;
};
```

### 4. Configuración de Auto-Pose Mejorada

#### Nuevos Parámetros

```cpp
struct IMUPoseParams {
    // Parámetros existentes...
    bool use_absolute_data;    // Usar posicionamiento absoluto si está disponible
    bool prefer_sensor_fusion; // Preferir fusión del sensor sobre algoritmos de librería
};
```

#### Nuevos Métodos de Control

```cpp
// Configurar modo de operación IMU
bool configureIMUMode(bool use_absolute_data, bool prefer_fusion);

// Obtener estado de calibración
uint8_t getIMUCalibrationStatus() const;

// Verificar si está usando datos absolutos
bool isUsingAbsoluteData() const;

// Obtener modo IMU actual
IMUMode getIMUMode() const;
```

## Uso Básico

### 1. Implementación de IMU BNO055

```cpp
class BNO055IMU : public IIMUInterface {
private:
    IMUMode current_mode_;
    // ... otros miembros privados

public:
    bool initialize() override {
        // Inicializar hardware BNO055
        // ...
        setIMUMode(IMU_MODE_ABSOLUTE_POS); // Usar modo absoluto por defecto
        return true;
    }

    IMUData readIMU() override {
        IMUData data{};

        // Llenar datos básicos
        data.roll = /* leer del BNO055 */;
        data.pitch = /* leer del BNO055 */;
        data.yaw = /* leer del BNO055 */;
        // ... otros datos básicos

        data.mode = current_mode_;
        data.has_absolute_capability = true;

        // Llenar datos absolutos específicos del BNO055
        data.absolute_data.absolute_roll = /* orientación absoluta */;
        data.absolute_data.absolute_pitch = /* orientación absoluta */;
        data.absolute_data.absolute_yaw = /* orientación absoluta */;
        data.absolute_data.linear_accel_x = /* aceleración sin gravedad */;
        data.absolute_data.linear_accel_y = /* aceleración sin gravedad */;
        data.absolute_data.linear_accel_z = /* aceleración sin gravedad */;

        // Estados de validez
        data.absolute_data.absolute_orientation_valid = /* verificar calibración */;
        data.absolute_data.linear_acceleration_valid = /* verificar calibración */;
        data.absolute_data.calibration_status = /* leer estado de calibración */;

        return data;
    }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;
        // Configurar el BNO055 según el modo
        switch (mode) {
            case IMU_MODE_RAW_DATA:
                // Configurar para datos crudos
                break;
            case IMU_MODE_FUSION:
                // Configurar para fusión de sensores
                break;
            case IMU_MODE_ABSOLUTE_POS:
                // Configurar para posicionamiento absoluto
                break;
        }
        return true;
    }

    bool hasAbsolutePositioning() const override {
        return true; // BNO055 soporta posicionamiento absoluto
    }

    // ... implementar otros métodos requeridos
};
```

### 2. Configuración del Sistema de Locomotion

```cpp
void setup() {
    BNO055IMU bno055_imu;
    // ... otros componentes

    // Inicializar sistema de locomoción
    locomotion_system.initialize(&bno055_imu, &fsr_sensors, &servo_controller);

    // Configurar auto-pose para usar datos absolutos
    IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        // Usar datos absolutos con fusión del sensor
        auto_pose->configureIMUMode(true, true);

        // Configurar parámetros optimizados para datos absolutos
        IMUAutoPose::IMUPoseParams params;
        params.use_absolute_data = true;
        params.prefer_sensor_fusion = true;
        params.orientation_gain = 0.7f; // Mayor ganancia para datos precisos
        params.response_speed = 0.2f;   // Respuesta más rápida
        params.deadzone_degrees = 1.0f; // Zona muerta más pequeña
        auto_pose->setIMUPoseParams(params);

        // Activar auto-pose
        auto_pose->setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
        auto_pose->setEnabled(true);
    }
}
```

### 3. Monitoreo del Estado

```cpp
void loop() {
    // Actualizar sistema
    locomotion_system.update();

    // Verificar estado del IMU
    IMUAutoPose* auto_pose = locomotion_system.getIMUAutoPose();
    if (auto_pose) {
        // Verificar si está usando datos absolutos
        if (auto_pose->isUsingAbsoluteData()) {
            Serial.println("Usando posicionamiento absoluto del IMU");
        } else {
            Serial.println("Usando algoritmos de la librería");
        }

        // Verificar calibración
        uint8_t calibration = auto_pose->getIMUCalibrationStatus();
        if (calibration < 3) {
            Serial.print("Advertencia: Calibración incompleta (");
            Serial.print(calibration);
            Serial.println("/3)");
        }

        // Obtener modo actual
        IMUMode mode = auto_pose->getIMUMode();
        Serial.print("Modo IMU: ");
        switch (mode) {
            case IMU_MODE_RAW_DATA:
                Serial.println("DATOS CRUDOS");
                break;
            case IMU_MODE_FUSION:
                Serial.println("FUSIÓN DE SENSORES");
                break;
            case IMU_MODE_ABSOLUTE_POS:
                Serial.println("POSICIONAMIENTO ABSOLUTO");
                break;
        }
    }
}
```

## Ventajas del Posicionamiento Absoluto

### 1. Mayor Precisión

-   Los algoritmos del BNO055 están optimizados específicamente para el hardware
-   Calibración automática y compensación de derivas
-   Fusión de 9 DOF (acelerómetro, giroscopio, magnetómetro)

### 2. Menor Carga Computacional

-   Los cálculos complejos se realizan en el chip del sensor
-   Liberando recursos del procesador principal para otras tareas

### 3. Datos de Aceleración Lineal

-   Aceleración con gravedad ya removida
-   Útil para detección de movimiento y control dinámico

### 4. Orientación Absoluta

-   Referencias a campo magnético terrestre
-   Mantenimiento de orientación a largo plazo sin deriva

## Configuraciones Recomendadas

### Para Máxima Precisión

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.orientation_gain = 0.8f;
params.response_speed = 0.15f;
params.deadzone_degrees = 0.5f;
```

### Para Máxima Estabilidad

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.orientation_gain = 0.5f;
params.response_speed = 0.1f;
params.deadzone_degrees = 2.0f;
```

### Para Terreno Irregular

```cpp
params.use_absolute_data = true;
params.prefer_sensor_fusion = true;
params.adaptive_gains = true;
params.stabilization_gain = 1.5f;
```

### Modo de Compatibilidad (IMUs Básicos)

```cpp
params.use_absolute_data = false;
params.prefer_sensor_fusion = false;
// Se usarán algoritmos de la librería con datos crudos
```

## Consideraciones de Implementación

### 1. Calibración

-   El BNO055 requiere calibración de 4 componentes: sistema, giroscopio, acelerómetro y magnetómetro
-   La calibración debe completarse antes de usar datos absolutos
-   Monitorear `calibration_status` continuamente

### 2. Validez de Datos

-   Verificar siempre los flags de validez antes de usar datos absolutos
-   Tener un fallback a datos crudos si los datos absolutos no son válidos

### 3. Configuración de Hardware

-   Asegurarse de que el IMU esté montado correctamente
-   Evitar interferencias magnéticas cerca del sensor
-   Configurar correctamente la orientación de ejes

## Ejemplo Completo

Ver el archivo `examples/bno055_absolute_positioning_example.ino` para una implementación completa que demuestra:

-   Configuración de BNO055 simulado
-   Cambio dinámico entre modos
-   Monitoreo de calibración
-   Optimización de parámetros

## Compatibilidad

Esta implementación es completamente compatible con versiones anteriores. Los IMUs que no soportan posicionamiento absoluto continuarán funcionando normalmente usando los algoritmos de la librería.
