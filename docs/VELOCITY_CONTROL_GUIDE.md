# HexaMotion Velocity Control Guide

Este documento describe las diferentes formas de ajustar la velocidad de desplazamiento en HexaMotion, basándose en el análisis del código y la arquitectura del sistema.

## Resumen Ejecutivo

HexaMotion proporciona múltiples métodos para controlar y ajustar la velocidad de desplazamiento del robot hexápodo:

1. **Comandos de velocidad cartesiana directa**
2. **Parámetros del gait (paso y frecuencia)**
3. **Velocidad de servos por defecto**
4. **Límites de velocidad dinámicos**
5. **Modo de velocidad crucero**

## 1. Comandos de Velocidad Cartesiana

### Método Principal: setDesiredVelocity

```cpp
// Usando StateController
Eigen::Vector2f linear_vel(30.0f, 10.0f);  // x=30mm/s, y=10mm/s
float angular_vel = 15.0f;                  // 15°/s rotation
state_controller.setDesiredVelocity(linear_vel, angular_vel);
```

### Métodos Directos en LocomotionSystem

```cpp
// Movimiento hacia adelante
bool walkForward(float velocity);                    // velocity en mm/s
bool walkForward(float velocity, float duration);    // con duración

// Movimiento hacia atrás
bool walkBackward(float velocity);
bool walkBackward(float velocity, float duration);

// Movimiento lateral
bool walkSideways(float velocity, bool right_direction = true);
bool walkSideways(float velocity, float duration, bool right_direction = true);

// Giro en el lugar
bool turnInPlace(float angular_velocity);           // angular_velocity en °/s

// Control combinado (método más flexible)
bool planGaitSequence(float vx, float vy, float omega);
```

### Ejemplo de Uso Básico

```cpp
void setup() {
    // Inicialización del sistema...
    state_controller.requestRobotState(ROBOT_READY);
}

void loop() {
    if (state_controller.isReadyForOperation()) {
        // Movimiento directo
        locomotion_system.walkForward(25.0f);     // 25 mm/s hacia adelante
        delay(2000);
        locomotion_system.turnInPlace(30.0f);     // 30°/s giro
        delay(1000);
        locomotion_system.stopMovement();
    }

    state_controller.update();
    locomotion_system.update();
}
```

## 2. Parámetros del Gait

### Configuración de Paso (Step Length)

El `step_length` se calcula dinámicamente basándose en varios factores:

```cpp
// En LocomotionSystem::updateStepParameters()
switch (current_gait) {
case TRIPOD_GAIT:
    // Pasos más largos para velocidad
    step_length = leg_reach * params.gait_factors.tripod_length_factor;
    break;
case WAVE_GAIT:
    // Pasos más cortos para estabilidad
    step_length = leg_reach * params.gait_factors.wave_length_factor;
    break;
// ...
}
```

### Factores que Afectan step_length

```cpp
float LocomotionSystem::getStepLength() const {
    float base_step_length = step_length;
    float leg_reach = calculateLegReach();
    float max_safe_step = leg_reach * params.gait_factors.max_length_factor;

    // Ajuste por estabilidad
    float stability_factor = 1.0f;
    if (stability_index < 0.5f) {
        stability_factor = 0.7f + 0.3f * stability_index;
    }

    // Ajuste por terreno (usando IMU)
    float terrain_factor = 1.0f;
    if (terrain_complexity > threshold) {
        terrain_factor *= adjustment_factor;
    }

    return base_step_length * stability_factor * terrain_factor;
}
```

### Configuración de Frecuencia

La frecuencia del gait se puede ajustar usando parámetros:

```cpp
// En OpenSHC (parámetros dinámicos)
params_.step_frequency.current_value = new_frequency;  // Hz

// En HexaMotion equivalente
gait_config.frequency = 1.5f;                         // 1.5 Hz
walk_controller.updateVelocityLimits(
    gait_config.frequency,
    gait_config.stance_ratio,
    gait_config.time_to_max_stride
);
```

## 3. Velocidad de Servos por Defecto

### Configuración Global

```cpp
// En HexaModel.h - parámetros de configuración
struct ServoConfig {
    float default_servo_speed = 100.0f;  // Velocidad por defecto
    float max_servo_speed = 200.0f;      // Velocidad máxima
    // ...
};

// Usar la nueva interfaz unificada
servo_interface->setJointAngleAndSpeed(angle, speed);
```

### Relación con Velocidad Cartesiana

La velocidad de los servos está relacionada con la velocidad cartesiana a través de la **matriz Jacobiana**:

```
joint_velocities = J^(-1) * cartesian_velocities
```

En HexaMotion, esta relación se maneja implícitamente mediante:

-   Cálculo de posiciones target usando cinemática inversa
-   Ajuste del timing del gait y frecuencia de comandos
-   Aplicación de límites de velocidad calculados dinámicamente

## 4. Límites de Velocidad Dinámicos

### Sistema VelocityLimits

HexaMotion implementa un sistema sofisticado de límites de velocidad:

```cpp
class VelocityLimits {
public:
    struct LimitValues {
        float linear_x;      // Velocidad lineal en X (mm/s)
        float linear_y;      // Velocidad lineal en Y (mm/s)
        float angular_z;     // Velocidad angular en Z (°/s)
    };

    struct WorkspaceConfig {
        float leg_length;           // Longitud total de la pata
        float body_radius;          // Radio del cuerpo del robot
        float safety_margin;        // Margen de seguridad (0-1)
        float min_ground_clearance; // Altura mínima sobre el suelo
    };
};
```

### Cálculo de Límites

```cpp
// En VelocityLimits::calculateMaxLinearSpeed()
float calculateMaxLinearSpeed(float walkspace_radius,
                              float on_ground_ratio,
                              float frequency) const {
    if (on_ground_ratio <= 0.0f || frequency <= 0.0f) {
        return 0.0f;
    }

    float cycle_time = on_ground_ratio / frequency;
    float max_speed = (walkspace_radius * 2.0f) / cycle_time;

    return std::min(max_speed, 5.0f); // Cap de seguridad: 5 m/s
}
```

### Aplicación de Límites

```cpp
// En WalkController
VelocityLimits::LimitValues WalkController::applyVelocityLimits(
    float vx, float vy, float omega) const {

    auto limits = getVelocityLimits();

    VelocityLimits::LimitValues result;
    result.linear_x = std::clamp(vx, -limits.linear_x, limits.linear_x);
    result.linear_y = std::clamp(vy, -limits.linear_y, limits.linear_y);
    result.angular_z = std::clamp(omega, -limits.angular_z, limits.angular_z);

    return result;
}
```

## 5. Modo de Velocidad Crucero

### Configuración

```cpp
// En StateController
void setCruiseVelocity(const Eigen::Vector3f& velocity) {
    cruise_velocity_ = velocity;  // x, y, angular
    cruise_mode_enabled_ = true;
}

void enableCruiseMode(bool enable) {
    cruise_mode_enabled_ = enable;
}
```

### Uso en Control de Velocidad

```cpp
void StateController::updateVelocityControl() {
    float linear_x, linear_y, angular_z;

    if (cruise_mode_enabled_) {
        // Usar velocidad crucero predefinida
        linear_x = cruise_velocity_.x();
        linear_y = cruise_velocity_.y();
        angular_z = cruise_velocity_.z();
    } else {
        // Usar entrada directa de velocidad
        linear_x = desired_linear_velocity_.x();
        linear_y = desired_linear_velocity_.y();
        angular_z = desired_angular_velocity_;
    }

    // Aplicar control de velocidad
    if (abs(linear_x) > 0.01f || abs(linear_y) > 0.01f || abs(angular_z) > 0.01f) {
        locomotion_system_.planGaitSequence(linear_x, linear_y, angular_z);
    }
}
```

## 6. Cadena de Control de Velocidad

### Flujo de Datos

```
Usuario/Aplicación
       ↓
StateController::setDesiredVelocity()
       ↓
StateController::updateVelocityControl()
       ↓
LocomotionSystem::planGaitSequence()
       ↓
WalkController::planGaitSequence()
       ↓
WalkController::applyVelocityLimits()
       ↓
LocomotionSystem::update()
       ↓
IServoInterface::setJointAngleAndSpeed()
```

### Componentes Clave

1. **VelocityLimits**: Calcula límites dinámicos basados en geometría y gait
2. **WalkController**: Aplica límites y valida comandos de velocidad
3. **LocomotionSystem**: Ejecuta patrones de gait y coordina movimiento
4. **StateController**: Gestiona estados y proporciona interfaz de alto nivel

## 7. Ejemplos Prácticos

### Ejemplo 1: Control de Velocidad Básico

```cpp
void basicVelocityControl() {
    // Configuración inicial
    state_controller.requestRobotState(ROBOT_READY);

    while (!state_controller.isReadyForOperation()) {
        state_controller.update();
        delay(10);
    }

    // Movimiento controlado
    Eigen::Vector2f linear(25.0f, 0.0f);  // 25 mm/s hacia adelante
    float angular = 0.0f;

    state_controller.setDesiredVelocity(linear, angular);

    // Ejecutar por 3 segundos
    for (int i = 0; i < 300; i++) {
        state_controller.update();
        locomotion_system.update();
        delay(10);
    }

    // Detener
    state_controller.setDesiredVelocity(Eigen::Vector2f(0.0f, 0.0f), 0.0f);
}
```

### Ejemplo 2: Ajuste Dinámico de Velocidad

```cpp
void adaptiveVelocityControl() {
    float base_speed = 30.0f;  // mm/s

    while (true) {
        // Obtener datos del sensor
        if (imu_interface && imu_interface->isConnected()) {
            IMUData imu_data = imu_interface->readIMU();

            // Ajustar velocidad según inclinación del terreno
            float tilt = sqrt(imu_data.roll * imu_data.roll +
                             imu_data.pitch * imu_data.pitch);

            float speed_factor = 1.0f;
            if (tilt > 10.0f) {  // Reducir velocidad en terreno inclinado
                speed_factor = std::max(0.5f, 1.0f - (tilt - 10.0f) / 20.0f);
            }

            float adjusted_speed = base_speed * speed_factor;

            Eigen::Vector2f linear(adjusted_speed, 0.0f);
            state_controller.setDesiredVelocity(linear, 0.0f);
        }

        state_controller.update();
        locomotion_system.update();
        delay(50);
    }
}
```

### Ejemplo 3: Control de Velocidad Crucero

```cpp
void cruiseControlDemo() {
    // Configurar velocidad crucero
    Eigen::Vector3f cruise_vel(20.0f, 5.0f, 10.0f);  // x, y, angular
    state_controller.setCruiseVelocity(cruise_vel);

    // Activar modo crucero
    state_controller.enableCruiseMode(true);

    // El robot mantendrá esta velocidad automáticamente
    while (cruiseActive) {
        state_controller.update();
        locomotion_system.update();
        delay(20);
    }

    // Desactivar crucero
    state_controller.enableCruiseMode(false);
}
```

## 8. Configuración Avanzada

### Ajuste de Parámetros de Gait

```cpp
// Configurar factores de gait para diferentes velocidades
struct GaitFactors {
    float tripod_length_factor = 0.8f;   // Para velocidad alta
    float wave_length_factor = 0.6f;     // Para estabilidad
    float ripple_length_factor = 0.7f;   // Balanceado

    float max_length_factor = 0.9f;      // Límite de seguridad
};

// Ajustar según condiciones
if (high_speed_mode) {
    params.gait_factors.tripod_length_factor = 0.9f;
    locomotion_system.setGaitType(TRIPOD_GAIT);
} else if (stability_required) {
    params.gait_factors.wave_length_factor = 0.5f;
    locomotion_system.setGaitType(WAVE_GAIT);
}
```

### Configuración de Límites de Velocidad

```cpp
// Configurar workspace y límites
VelocityLimits::WorkspaceConfig workspace;
workspace.leg_length = 150.0f;           // mm
workspace.body_radius = 100.0f;          // mm
workspace.safety_margin = 0.8f;          // 80% del workspace
workspace.min_ground_clearance = 20.0f;  // mm

// Configurar gait para límites
VelocityLimits::GaitConfig gait;
gait.frequency = 1.5f;                   // Hz
gait.stance_ratio = 0.6f;                // 60% stance, 40% swing
gait.time_to_max_stride = 2.0f;          // s

// Aplicar configuración
walk_controller.updateVelocityLimits(
    gait.frequency,
    gait.stance_ratio,
    gait.time_to_max_stride
);
walk_controller.setVelocitySafetyMargin(workspace.safety_margin);
```

## 9. Resolución de Problemas

### Problema: Robot no alcanza la velocidad deseada

**Posibles causas:**

1. Comandos de velocidad exceden los límites calculados
2. Parámetros de gait restrictivos
3. Terreno o condiciones que reducen los límites

**Soluciones:**

```cpp
// Verificar límites actuales
auto limits = walk_controller.getVelocityLimits();
Serial.println("Max linear X: " + String(limits.linear_x));
Serial.println("Max linear Y: " + String(limits.linear_y));
Serial.println("Max angular Z: " + String(limits.angular_z));

// Ajustar parámetros si es necesario
walk_controller.setVelocitySafetyMargin(0.9f);  // Aumentar margen utilizable
walk_controller.updateVelocityLimits(2.0f, 0.5f, 1.5f);  // Frecuencia más alta
```

### Problema: Movimiento inestable a altas velocidades

**Soluciones:**

```cpp
// Cambiar a gait más estable
locomotion_system.setGaitType(WAVE_GAIT);

// Reducir step_length
params.gait_factors.wave_length_factor = 0.5f;

// Aumentar frecuencia manteniendo velocidad
walk_controller.updateVelocityLimits(2.5f, 0.7f, 1.0f);
```

## 10. Comparación con OpenSHC

| Aspecto                  | HexaMotion                                  | OpenSHC                                         |
| ------------------------ | ------------------------------------------- | ----------------------------------------------- |
| **Límites de Velocidad** | Calculados dinámicamente por VelocityLimits | Calculados por WalkController::generateLimits() |
| **Parámetros**           | Estructuras configurables                   | Parámetros ajustables en tiempo real            |
| **Interfaz**             | StateController::setDesiredVelocity()       | WalkController::updateWalk()                    |
| **Gait**                 | Enum con tipos predefinidos                 | Parámetros de timing flexibles                  |
| **Servo Speed**          | setJointAngleAndSpeed() unificado           | Parámetros separados de velocidad               |

## Conclusión

HexaMotion proporciona un sistema completo y flexible para el control de velocidad que permite:

1. **Control directo** a través de comandos de velocidad cartesiana
2. **Configuración avanzada** mediante parámetros de gait y workspace
3. **Seguridad** a través de límites dinámicos y validación
4. **Adaptabilidad** con ajustes automáticos según condiciones del terreno
5. **Flexibilidad** con múltiples métodos de ajuste según las necesidades

El sistema está diseñado para ser tanto fácil de usar para casos básicos como potente para aplicaciones avanzadas que requieren control fino sobre el comportamiento de locomoción.
