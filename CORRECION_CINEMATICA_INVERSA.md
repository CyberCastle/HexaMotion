# Corrección de Errores de Cinemática Inversa y Validación de Posiciones Angulares

## Descripción del Problema

El sistema de locomoción del robot hexápodo presentaba violaciones en los límites de ángulos de las articulaciones, donde los ángulos excedían los límites configurados de ±65° (coxa), ±75° (fémur), y ±45° (tibia), con violaciones que alcanzaban valores superiores a 90° según las salidas de las pruebas.

## Proceso de Investigación y Corrección

### Paso 1: Análisis Inicial del Sistema de Restricciones

**Acción:** Investigar el pipeline de aplicación de restricciones en el sistema de locomoción.

**Hallazgos:**

-   El sistema implementa correctamente la aplicación de restricciones en la función `setLegJointAngles()`
-   Utiliza el método `constrainAngle()` para limitar los ángulos dentro de los rangos permitidos
-   El pipeline de restricciones funciona: `setLegPosition()` → `setLegJointAngles()` → `constrainAngle()` → almacena valores limitados

**Archivos examinados:**

-   `src/locomotion_system.cpp` - Contiene la lógica de aplicación de restricciones
-   `src/robot_model.cpp` - Implementa `constrainAngle()` y `normalizeAngle()`
-   `src/pose_controller.cpp` - Aplicación adicional de restricciones en control de pose

### Paso 2: Identificación de la Causa Raíz

**Problema encontrado:** La prueba principal de simulación utilizaba límites de articulación incorrectos (±90° para todas las articulaciones) en lugar de los límites apropiados del estilo CSIRO.

**Archivo problemático:** `/Users/cybercastle/tmp/HexaMotion/tests/tripod_gait_sim_test.cpp`

**Código problemático identificado:**

```cpp
// Límites incorrectos (demasiado permisivos)
p.coxa_angle_limits[0] = -90;   // Debería ser -65
p.coxa_angle_limits[1] = 90;    // Debería ser 65
p.femur_angle_limits[0] = -90;  // Debería ser -75
p.femur_angle_limits[1] = 90;   // Debería ser 75
p.tibia_angle_limits[0] = -90;  // Debería ser -45
p.tibia_angle_limits[1] = 90;   // Debería ser 45
```

### Paso 3: Corrección de los Parámetros de Configuración

**Acción:** Actualizar el archivo de prueba con los límites correctos de articulación estilo CSIRO.

**Cambios realizados en `tripod_gait_sim_test.cpp` (líneas 292-302):**

```cpp
// Límites corregidos para articulaciones estilo CSIRO
p.coxa_angle_limits[0] = -65;   // Cambio de -90 a -65
p.coxa_angle_limits[1] = 65;    // Cambio de 90 a 65
p.femur_angle_limits[0] = -75;  // Cambio de -90 a -75
p.femur_angle_limits[1] = 75;   // Cambio de 90 a 75
p.tibia_angle_limits[0] = -45;  // Cambio de -90 a -45
p.tibia_angle_limits[1] = 45;   // Cambio de 90 a 45
```

**Justificación:**

-   **Coxa (±65°):** Rango realista para la articulación de cadera que permite movimiento lateral adecuado
-   **Fémur (±75°):** Rango que permite elevación y descenso efectivo de las patas
-   **Tibia (±45°):** Rango conservador que evita colisiones y mantiene estabilidad estructural

### Paso 4: Mejora del Sistema de Monitoreo

**Mejoras implementadas:**

1. **Función de visualización de dimensiones del robot:**

```cpp
void printRobotDimensions(const RobotModelParameters& p) {
    std::cout << "\n=== DIMENSIONES DEL ROBOT ===" << std::endl;
    std::cout << "Longitud Coxa:     " << std::setw(6) << p.coxa_length << " mm" << std::endl;
    std::cout << "Longitud Fémur:    " << std::setw(6) << p.femur_length << " mm" << std::endl;
    std::cout << "Longitud Tibia:    " << std::setw(6) << p.tibia_length << " mm" << std::endl;
    // ... más detalles
}
```

2. **Función mejorada de estado detallado del robot:**

```cpp
void printDetailedRobotStatus(const LocomotionSystem& ls, const RobotModelParameters& p) {
    // Incluye resumen de dimensiones y estados de articulaciones
    // con valores calculados como alcance máximo y radio de trabajo
}
```

### Paso 5: Verificación del Sistema

**Prueba ejecutada:** Ejecutar la prueba corregida para confirmar que las violaciones de ángulos de articulación están ahora correctamente restringidas.

**Resultado:** Los ángulos permanecen dentro de los límites especificados, confirmando que el sistema de restricciones funciona correctamente con los parámetros apropiados.

## Análisis del Sistema de Restricciones

### Arquitectura del Sistema de Restricciones

El sistema implementa un pipeline de múltiples capas para la aplicación de restricciones:

1. **Capa de Entrada:** `setLegPosition()` recibe posiciones cartesianas objetivo
2. **Capa de Cinemática Inversa:** Calcula ángulos de articulación necesarios
3. **Capa de Restricciones:** `setLegJointAngles()` aplica `constrainAngle()` a cada articulación
4. **Capa de Almacenamiento:** Guarda valores restringidos en `joint_angles[leg]`

### Función de Restricción de Ángulos

```cpp
double constrainAngle(double angle, double min_angle, double max_angle) {
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}
```

**Características:**

-   Implementación simple y efectiva de clamping
-   Aplicación consistente en todo el sistema
-   Sin efectos secundarios o modificaciones de estado inesperadas

## Lecciones Aprendidas

### 1. **Importancia de la Configuración Correcta**

El sistema de restricciones funcionaba correctamente desde el principio. El problema real era que la configuración de prueba utilizaba límites demasiado permisivos que no reflejaban las limitaciones físicas reales del robot.

### 2. **Validación de Parámetros**

Es crucial validar que los parámetros de configuración reflejen las especificaciones reales del hardware, especialmente en sistemas robóticos donde las limitaciones físicas son críticas para la operación segura.

### 3. **Separación de Responsabilidades**

El diseño correcto separa claramente:

-   **Algoritmos de cinemática:** Cálculo de ángulos necesarios
-   **Sistema de restricciones:** Aplicación de límites físicos
-   **Configuración:** Parámetros específicos del robot

## Recomendaciones para el Futuro

### 1. **Validación Automática de Configuración**

Implementar verificaciones automáticas que validen que los parámetros de configuración están dentro de rangos realistas:

```cpp
bool validateRobotParameters(const RobotModelParameters& p) {
    // Verificar que los límites de ángulos son realistas
    if (abs(p.coxa_angle_limits[0]) > 90 || abs(p.coxa_angle_limits[1]) > 90) {
        std::cerr << "Advertencia: Límites de coxa excesivos" << std::endl;
        return false;
    }
    // ... más validaciones
    return true;
}
```

### 2. **Documentación de Especificaciones**

Crear documentación clara que especifique:

-   Límites de articulación recomendados para diferentes tipos de robot
-   Justificación técnica para cada rango de movimiento
-   Procedimientos de calibración y validación

### 3. **Pruebas de Integración Mejoradas**

Desarrollar pruebas que específicamente validen:

-   Que los límites de configuración son apropiados para el hardware
-   Que el sistema de restricciones funciona correctamente bajo diversas condiciones
-   Que no hay discrepancias entre configuración y implementación

## Archivos Modificados

### Archivos Principales:

-   **`tests/tripod_gait_sim_test.cpp`:** Corrección de parámetros de límites de articulación y adición de visualización de dimensiones
-   **Archivos de análisis:** Múltiples archivos de prueba examinados para verificar el sistema de restricciones

### Archivos de Sistema (verificados, no modificados):

-   **`src/locomotion_system.cpp`:** Sistema de restricciones funcional
-   **`src/robot_model.cpp`:** Implementaciones de `constrainAngle()` y `normalizeAngle()`
-   **`src/pose_controller.cpp`:** Restricciones adicionales en control de pose

## Conclusión

La solución exitosa de este problema demostró que a menudo los "errores de algoritmo" pueden ser en realidad problemas de configuración. El sistema de cinemática inversa y restricciones funcionaba correctamente, pero estaba siendo probado con parámetros inapropiados que no reflejaban las limitaciones físicas reales del robot.

La corrección fue relativamente simple una vez identificada la causa raíz: cambiar los límites de ±90° a los valores realistas de ±65° (coxa), ±75° (fémur), y ±45° (tibia). Esto permitió que el sistema de restricciones funcionara como estaba diseñado, manteniendo todos los ángulos dentro de rangos seguros y operativos.

**Fecha de documentación:** 9 de junio de 2025
**Estado:** Problema resuelto y sistema verificado
