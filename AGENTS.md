# AGENT Instructions

Este archivo define las pautas para colaborar en HexaMotion.

## Objetivo

Esta librería proporciona control de locomoción para un robot hexápodo basado en Arduino Giga R1.
El cuerpo del robot es un hexágono con las patas separadas 60° entre sí.
Incluye cinemática inversa con parámetros DH y jacobianos, control de orientación y pose, planificador de marchas y manejo de errores.
Las interfaces `IIMUInterface`, `IFSRInterface` e `IServoInterface` deben implementarse para conectar el IMU, los sensores FSR y los servos inteligentes.

## Estilo de código

-   Utilizar C++17.
-   Indentación de 4 espacios sin tabulaciones.
-   Colocar la llave de apertura en la misma línea que la declaración.
-   Documentar funciones públicas con comentarios estilo Doxygen (`/** ... */`).

## Desarrollo

-   Clonar el repositorio completo, incluyendo todos los submódulos.
-   Al momento de implementar alguna funcionalidad, analizar la carpeta OpenSHC antes,
    de tal forma que la implementación sea equivalente a la de OpenSHC.

## Pruebas

-   Antes de enviar cambios, ejecutar las pruebas unitarias.
-   Instalar la dependencia Eigen ejecutando `tests/setup.sh` si es necesario.
-   Construir las pruebas con `make` dentro del directorio `tests`.

```bash
cd tests
./setup.sh
make
```

Cada ejecutable de prueba puede ejecutarse de forma individual.

### Parámetros de prueba

Usar la siguiente configuración de `Parameters` en los archivos de prueba:

```cpp
Parameters p{};
p.hexagon_radius = 400;
p.coxa_length = 50;
p.femur_length = 101;
p.tibia_length = 208;
p.robot_height = 90;
p.control_frequency = 50;
p.coxa_angle_limits[0] = -65;
p.coxa_angle_limits[1] = 65;
p.femur_angle_limits[0] = -75;
p.femur_angle_limits[1] = 75;
p.tibia_angle_limits[0] = -45;
p.tibia_angle_limits[1] = 45;
```

## Mensajes de commit

-   Usar tiempo imperativo en inglés. Ejemplos: "Add new gait option".
-   Mantener el resumen por debajo de 72 caracteres.

## Solicitudes de extracción

-   Incluir un resumen de los cambios realizados.
-   Mencionar cualquier limitación conocida o pasos adicionales.
