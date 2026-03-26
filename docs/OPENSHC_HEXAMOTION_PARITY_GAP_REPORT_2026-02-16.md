# OpenSHC vs HexaMotion — Reporte de brechas para paridad 1:1

Fecha: 2026-02-16

## Alcance y criterio de comparación

Este análisis se realizó con los siguientes criterios:

- **Contexto normativo**: `AGENTS.md` (objetivo de paridad 1:1 con OpenSHC, excluyendo ROS transport).
- **Exclusiones explícitas aceptadas**:
    - Sin ROS nativo.
    - Solo 3DOF por pierna.
    - Solo 6 patas.
    - `AMBLE_GAIT` no soportado.
    - Planner mode no portado en HexaMotion; comportamiento equivalente debe vivir en software externo usando la API de `LocomotionSystem`.
    - Sin YAML/dynamic config.
- **Fuentes comparadas** (únicamente código):
    - OpenSHC: `OpenSHC/include/syropod_highlevel_controller/*`, `OpenSHC/src/*`
    - HexaMotion: `src/*`
- **Importante**: se **ignoró** intencionalmente todo el contenido bajo `docs/`.

## Mapeo de componentes equivalentes

| OpenSHC                         | HexaMotion                                                  | Estado                      |
| ------------------------------- | ----------------------------------------------------------- | --------------------------- |
| `Model` / `Leg` / `Pose`        | `RobotModel` / `Leg` / `Pose` (+ `AnalyticRobotModel`)      | Implementado, refactorizado |
| `StateController`               | `StateController` + `LocomotionSystem` (facade sin ROS)     | Implementado                |
| `WalkController` + `LegStepper` | `WalkController` + `LegStepper`                             | Implementado                |
| `PoseController`                | `BodyPoseController` (+ `ManualBodyPoseController`)         | Implementado                |
| `AdmittanceController`          | `AdmittanceController`                                      | Implementado                |
| `parameters_and_states.h`       | `hexamotion_constants.h`, `gait_types.h`, structs de config | Parcialmente equivalente    |

## Paridad confirmada (sin brecha crítica detectada)

1. **Secuencia de control principal (pipeline) presente**
    - HexaMotion ejecuta un flujo equivalente de alto nivel: actualización de marcha, actualización de pose/stance, IK y salida a actuadores.
    - Evidencia: `src/locomotion_system.cpp` (bloque `runControlPipelineStep`).

2. **Máquina de estados de marcha base equivalente**
    - Estados `STARTING/MOVING/STOPPING/STOPPED` y ciclo de fase global implementados.
    - Evidencia: `src/walk_controller.cpp`.

3. **Control de admittance integrado al loop**
    - Integración ODE por pierna + actualización dinámica de stiffness están presentes.
    - Evidencia: `src/admittance_controller.cpp`, `src/locomotion_system.cpp`.

4. **Orquestación de transición de robot (pack/ready/running) implementada**
    - Existe lógica de transición y control de estado operativo en HexaMotion.
    - Evidencia: `src/state_controller.cpp`.

## Brechas de paridad 1:1 (hallazgos)

## 1) CRÍTICA — Orden de actualización manual vs walk no equivalente

- **OpenSHC**: aplica `updateWalk(...)` y después `updateManual(...)` en el mismo tick.
    - Evidencia: `OpenSHC/src/state_controller.cpp` (`updateState()` / bloque principal).
- **HexaMotion**: aplica entradas manuales antes de ejecutar el pipeline, y luego `walk_ctrl->updateWalk(...)` puede sobrescribir resultado manual.
    - Evidencia: `src/state_controller.cpp` (`context_.applyManualLegInputs(...)`), `src/locomotion_system.cpp` (`walk_ctrl->updateWalk(...)`).
- **Impacto**: comportamiento distinto al manipular patas manualmente durante ejecución.
- **Acción recomendada**: mover aplicación manual a después de `updateWalk` dentro del mismo ciclo (o reaplicar manual inmediatamente después de marcha).

## 2) CRÍTICA — Falta guardia de estado de pierna antes de caminar

- **OpenSHC**: aborta actualización de marcha si alguna pierna no está en `WALKING`.
    - Evidencia: `OpenSHC/src/walk_controller.cpp` (check explícito de `leg->getLegState() != WALKING`).
- **HexaMotion**: no existe gate equivalente al inicio de `WalkController::updateWalk(...)`.
    - Evidencia: `src/walk_controller.cpp` (iteración de steppers sin early-return por `LegState`).
- **Impacto**: HexaMotion puede continuar avanzando gait con patas en modo manual.
- **Acción recomendada**: añadir validación de `LegState` por pierna y retorno temprano equivalente a OpenSHC.

## 3) ALTA — Cambio de gait sin refresco explícito de auto-pose asociado

- **OpenSHC**: al cambiar gait, reinicializa parámetros y actualiza auto-pose (`setAutoPoseParams`).
    - Evidencia: `OpenSHC/src/state_controller.cpp` (`changeGait()`).
- **HexaMotion**: `changeGait(...)` crea/setea gait config, pero no refresca explícitamente auto-pose por gait.
    - Evidencia: `src/state_controller.cpp` (`changeGait`), `src/locomotion_system.cpp` (`setGaitConfiguration`).
- **Impacto**: puede quedar desfasado el comportamiento de compensación automática tras cambio de gait.
- **Acción recomendada**: regenerar configuración de auto-pose al confirmar gait nuevo.

## 4) MEDIA — Semántica de `PosingMode` divergente

- **OpenSHC**: cambio de posing mode es principalmente informativo; la compuerta principal de control está delegada externamente.
    - Evidencia: comentario en `OpenSHC/src/state_controller.cpp` (`posingModeCallback`: “Used only for user message...”).
- **HexaMotion**: `updatePoseControl()` usa `PosingMode` como compuerta activa por ejes.
    - Evidencia: `src/state_controller.cpp` (`updatePoseControl`, `applyBodyPositionControl`, `applyBodyOrientationControl`).
- **Impacto**: misma entrada de modo puede producir conducta distinta.
- **Acción recomendada**: introducir modo de compatibilidad OpenSHC (semántica informativa) o alinear flujo exacto.

## 5) MEDIA — Modelo de `SystemState` no homólogo al de OpenSHC

- **OpenSHC**: `SUSPENDED` / `OPERATIONAL` a nivel sistema; robot state separado.
    - Evidencia: `OpenSHC/include/syropod_highlevel_controller/parameters_and_states.h`.
- **HexaMotion**: `SYSTEM_UNKNOWN/PACKED/READY/RUNNING` fusiona semántica sistema/robot.
    - Evidencia: `src/hexamotion_constants.h`.
- **Impacto**: difiere contrato de orquestación top-level y su gating.
- **Acción recomendada**: recuperar gate de sistema equivalente (`SUSPENDED/OPERATIONAL`), crear/mantener `RobotState` separado (`PACKED/READY/RUNNING`) y usar `RobotState` para validaciones de transición/ejecución (incluyendo tests), no `SystemState`.

## 6) BAJA — Proyección de delta de admittance distinta

- **OpenSHC**: proyecta `admittance_delta` sobre el eje de la punta (`tip X-axis`).
    - Evidencia: `OpenSHC/include/syropod_highlevel_controller/model.h` (`setAdmittanceDelta`).
- **HexaMotion**: guarda delta vectorial directo (sin proyección).
    - Evidencia: `src/leg.h` (`setAdmittanceDelta`), `src/admittance_controller.cpp` (comentario de simplificación).
- **Impacto**: diferencias sutiles de compliance en orientaciones fuera de caso planar.
- **Acción recomendada**: recuperar proyección equivalente cuando exista base de orientación de tip.

## Matriz de riesgo de paridad

| ID  | Brecha                              | Severidad | Riesgo funcional                    |
| --- | ----------------------------------- | --------- | ----------------------------------- |
| G1  | Orden manual vs walk                | Crítica   | Alto (control manual inconsistente) |
| G2  | Sin guardia leg-state para caminar  | Crítica   | Alto (gait no bloqueado en manual)  |
| G3  | Gait change sin refresh auto-pose   | Alta      | Medio/Alto                          |
| G4  | `PosingMode` con semántica distinta | Media     | Medio                               |
| G5  | `SystemState` no homólogo           | Media     | Medio                               |
| G6  | Delta admittance sin proyección     | Baja      | Bajo/Medio                          |

## Orden recomendado de ejecución (para cerrar paridad)

1. **G1 + G2** (bloque crítico de locomoción/manual).
2. **G3** (refresh auto-pose al cambio de gait).
3. **G4 + G5** (alinear contrato de orquestación y modos).
4. **G6** (fino de admittance / vector projection).

## Definición de “paridad 1:1 alcanzada” (criterio de salida)

Se considera alcanzada cuando:

- El orden de actualización por tick reproduce OpenSHC en los caminos equivalentes permitidos por el alcance.
- La marcha se bloquea cuando existan patas fuera de `WALKING` igual que en OpenSHC.
- Cambio de gait sincroniza parámetros de auto-pose asociados.
- La semántica de modos y estado top-level no altera comportamiento equivalente.
- `SystemState` se usa sólo como estado top-level (`SUSPENDED/OPERATIONAL`) y `RobotState` se usa para ciclo operativo del robot (`PACKED/READY/RUNNING`).
- Las validaciones de startup/shutdown y los tests de transición verifican `RobotState`.
- Admittance mantiene misma proyección física que OpenSHC en el frame de tip.

---

Siguiente artefacto recomendado: una **checklist función-a-función** (OpenSHC → HexaMotion) con estado `OK / DIVERGE / MISSING` para seguimiento incremental por PR.
