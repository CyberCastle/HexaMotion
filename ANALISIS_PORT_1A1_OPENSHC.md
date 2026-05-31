# Verificación de la hipótesis: "HexaMotion is a 1:1 port of OpenSHC without ROS support"

**Fecha:** 31 de mayo de 2026
**Alcance:** Análisis completo de la implementación en `src/` frente a la referencia en `OpenSHC/`.
**Nota:** Se ignoró deliberadamente el contenido de `docs/` por estar desactualizado.

---

## Veredicto

**La hipótesis se cumple PARCIALMENTE.**

HexaMotion es un port **funcionalmente fiel** de OpenSHC en lo esencial (cinemática, generación de
trayectorias Bézier, máquinas de estado, control de pose y admitancia) y la eliminación de ROS está
bien resuelta. Las exclusiones declaradas en `AGENTS.md` (tip-orientation, planner, cruise,
ExternalTarget, velocity_input_mode, ignore_IK_warnings, ParameterSelection/adjustParameter, YAML,
AMBLE_GAIT, estrategia de workspace) **se respetan correctamente**.

Sin embargo, **NO es un port estrictamente "1:1"**: HexaMotion introduce lógica de control adicional
(no presente en OpenSHC) que altera el cálculo de trayectorias y la coordinación de estados, además
de algunas relocalizaciones de algoritmos y simplificaciones que **no figuran en la lista de
diferencias declaradas**. Estas divergencias se detallan abajo.

La etiqueta más precisa sería: **"reimplementación equivalente y orientada a MCU, sin ROS, con
mejoras de estabilidad añadidas"** más que un port verbatim 1:1.

---

## 1. Lo que SÍ es 1:1 (paridad confirmada)

| Subsistema                                                                 | Equivalencia                                                                                                        | Evidencia                                                                                                                            |
| -------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| Cinemática directa (DH)                                                    | Equivalente                                                                                                         | [robot_model.cpp](src/robot_model.cpp) `forwardKinematicsGlobalCoordinates` ↔ OpenSHC `Leg::applyFK`                                 |
| Cinemática inversa (DLS + nullspace Fahimi 2008)                           | Núcleo DLS idéntico (coef. 0.02, peso límites 0.1, mezcla 0.25/0.75); estrategia de convergencia diverge (ver §2.5) | [robot_model.cpp](src/robot_model.cpp#L116) ↔ [model.cpp](OpenSHC/src/model.cpp#L726)                                                |
| Generación de nodos Bézier swing/stance                                    | 1:1                                                                                                                 | [leg_stepper.cpp](src/leg_stepper.cpp#L237) ↔ [walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1295)                           |
| `iteratePhase` / máquina swing-stance                                      | 1:1                                                                                                                 | [leg_stepper.cpp](src/leg_stepper.cpp#L854) ↔ [walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L871)                            |
| `updateWalk` (STOPPED→STARTING→MOVING→STOPPING)                            | 1:1                                                                                                                 | [walk_controller.cpp](src/walk_controller.cpp#L292) ↔ [walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L440)                    |
| Máquina de estados robot (UNKNOWN→PACKED→READY→RUNNING)                    | Equivalente                                                                                                         | [state_controller.cpp](src/state_controller.cpp#L691) ↔ [state_controller.cpp](OpenSHC/src/state_controller.cpp#L197)                |
| Secuencias de pose (`executeSequence`, pack/unpack, startup/shutdown)      | 1:1                                                                                                                 | [body_pose_controller.cpp](src/body_pose_controller.cpp#L1061) ↔ [pose_controller.cpp](OpenSHC/src/pose_controller.cpp#L144)         |
| AutoPoser (gravity_amplitudes, Bézier cuártico, fases)                     | 1:1                                                                                                                 | [auto_poser.h](src/auto_poser.h#L54) ↔ [pose_controller.cpp](OpenSHC/src/pose_controller.cpp#L1352)                                  |
| LegPoser (`stepToPosition`, `transitionConfiguration`)                     | 1:1                                                                                                                 | [leg_poser.cpp](src/leg_poser.cpp#L30)                                                                                               |
| `updateInclinationPose`, `updateIKErrorPose`, `calculateDefaultPose`       | Portados                                                                                                            | [body_pose_controller.cpp](src/body_pose_controller.cpp#L332)                                                                        |
| AdmittanceController (sistema masa-resorte-amortiguador, 30 sub-pasos RK4) | Equivalente                                                                                                         | [admittance_controller.cpp](src/admittance_controller.cpp#L165) ↔ [admittance_controller.cpp](OpenSHC/src/admittance_controller.cpp) |
| Operaciones de Pose (transform, addPose, removePose, interpolate, inverse) | Equivalentes                                                                                                        | [pose.h](src/pose.h) ↔ [pose.h](OpenSHC/include/syropod_highlevel_controller/pose.h)                                                 |
| `calculateStanceSpanChange` (lógica XOR de bearing 90/270)                 | 1:1                                                                                                                 | [leg_stepper.cpp](src/leg_stepper.cpp#L743)                                                                                          |
| Rol orquestador del `main.cpp` ROS → `LocomotionSystem`                    | Cubierto (adquisición de joints, loop, salida a servos)                                                             | [locomotion_system.cpp](src/locomotion_system.cpp) ↔ [main.cpp](OpenSHC/src/main.cpp)                                                |

**Las exclusiones declaradas en `AGENTS.md` están respetadas:** ROS, `updateTipRotation`,
`gravity_aligned_tips`/`updateTipAlignPose`, rotation-constrained IK retry, planner, cruise,
`ExternalTarget`, `velocity_input_mode`, `ignore_IK_warnings`, `ParameterSelection`/`adjustParameter`,
sin YAML, AMBLE_GAIT y la estrategia `WorkspaceAnalyzer` sobre modelo vivo. Ninguna de ellas debe
considerarse inconsistencia.

---

## 2. Inconsistencias con "1:1" NO declaradas en AGENTS.md

> Estas son las divergencias relevantes que **no aparecen** en la lista de "Key differences" de
> `AGENTS.md` y que hacen que el port no sea estrictamente 1:1. Varias están gobernadas por
> parámetros, pero introducen comportamiento ausente en OpenSHC.

### 2.1 Lógica de estabilidad añadida en `LegStepper` (no existe en OpenSHC)

En `updateTipPositionIterative` HexaMotion incorpora bloques de control que OpenSHC **no tiene**:

- **Congelación de stride (stride freezing):** el vector de stride se congela en la primera
  iteración de la fase (`frozen_stride_vector_linear_/total_`). OpenSHC recalcula el stride cada
  iteración. → [leg_stepper.cpp](src/leg_stepper.cpp#L132)
- **Anti-drift híbrido:** al entrar en stance se mide el offset respecto a la pose por defecto y se
  aplica _hard reset_, _soft blend_ o corrección lateral según umbrales
  (`drift_hard_threshold_mm`, `drift_soft_threshold_mm`, `drift_soft_blend_alpha`). OpenSHC no
  realiza ninguna corrección de deriva. → [leg_stepper.cpp](src/leg_stepper.cpp#L646)
- **Corrección de residuo lateral** proyectada en el eje de la pata (`lateral_unit_`), inexistente
  en OpenSHC. → [leg_stepper.cpp](src/leg_stepper.cpp#L600)
- **Phase-end snap:** al final de fase (`step_progress_ >= 0.999`) se fuerza/interpola la pose hacia
  el target congelado (`enable_phase_end_snap`, `phase_end_snap_alpha`). El propio código lo
  comenta como _"documented difference from vanilla OpenSHC"_. → [leg_stepper.cpp](src/leg_stepper.cpp#L702)
- **Validación/constrain de workspace durante el gait** (`validateTargetTipPose`,
  `constrainToWorkspace`, `calculateSafeTarget`, `validateAndFixControlNodes`). OpenSHC no protege
  la trayectoria contra poses inalcanzables dentro de `updateTipPosition`. → [leg_stepper.cpp](src/leg_stepper.cpp#L873)

**Impacto:** aunque mejoran la robustez en MCU, alteran la trayectoria del pie respecto a OpenSHC.
Son la divergencia más sustancial respecto a "1:1". `AGENTS.md` menciona los parámetros de drift y
snap de pasada, pero **no declara** que el algoritmo de trayectoria incorpora comportamiento extra
respecto a OpenSHC.

### 2.2 Refactor del cálculo de trayectoria a forma iterativa explícita

OpenSHC integra la trayectoria con base en `swing_progress_/stance_progress_` derivados de la fase.
HexaMotion la reescribe con contadores de iteración explícitos
(`swing_iterations_`, `stance_iterations_`, `swing_delta_t_`, `stance_delta_t_`) en
`updateTipPositionIterative`. → [leg_stepper.cpp](src/leg_stepper.cpp#L403)

Además, el redondeo de timing diverge:

- OpenSHC redondea a par **solo** las iteraciones de swing.
- HexaMotion fuerza par en **swing y stance** y aplica un **mínimo de 10 iteraciones** de stance.
  → [leg_stepper.cpp](src/leg_stepper.cpp#L186)

### 2.3 `StateController`: conmutación de patas manuales no simultánea

OpenSHC puede transicionar **dos patas a la vez** (primary + secondary) con banderas independientes
(`toggle_primary_leg_state_`, `toggle_secondary_leg_state_`). HexaMotion procesa **una pata por vez**
mediante `toggle_leg_index_` / `toggle_leg_state_pending_`. → [state_controller.cpp](src/state_controller.cpp#L827)

No está cubierto por ninguna exclusión declarada; es una reducción funcional respecto a OpenSHC.

### 2.4 Estimación del plano de marcha relocalizada y con distinto conjunto de puntos

OpenSHC calcula el plano de marcha en `WalkController::updateWalkPlane` por mínimos cuadrados sobre
las posiciones _default_ de todas las patas, y lo expone vía `LegStepper::getWalkPlaneNormal`.
HexaMotion lo mueve a `BodyPoseController` y lo calcula por mínimos cuadrados sobre las patas
**actualmente en stance** (`calculateWalkPlaneNormal`/`calculateWalkPlaneHeight`).
→ [body_pose_controller.cpp](src/body_pose_controller.cpp#L615)

Es equivalente en régimen cuasi-estático, pero el origen de los datos y la ubicación difieren y **no
está declarado** en `AGENTS.md` (solo se declara la relocalización de `estimateGravity`).

### 2.5 IK: bucle iterativo interno, clamp por iteración y semilla analítica añadidos

> **Corregido el 31-may-2026 tras verificación línea por línea.** La versión previa de esta sección
> contenía valores erróneos (`IK_MAX_ANGLE_STEP = 15°`, `IK_TOLERANCE = 0.5 mm`,
> `damping_lambda = 30.0` en uso). Los valores reales se detallan a continuación.

- **Divergencia algorítmica principal (NUEVO hallazgo):** OpenSHC `Leg::solveIK` ejecuta **un único
  paso DLS por ciclo de control** (`applyIK` la llama una vez por ciclo) y converge a lo largo de
  muchos ciclos del lazo de control de alta frecuencia. HexaMotion `RobotModel::solveIK` ejecuta un
  **bucle iterativo interno** (hasta `IK_DEFAULT_MAX_ITERATIONS = 30` iteraciones) hasta converger
  **dentro de una sola llamada**. Esta es la divergencia de IK más sustancial y no estaba
  documentada. → [robot_model.cpp](src/robot_model.cpp#L116) ↔ [model.cpp](OpenSHC/src/model.cpp#L726) / [model.cpp](OpenSHC/src/model.cpp#L861)
- HexaMotion **limita el cambio angular por iteración** a `IK_MAX_ANGLE_STEP` = **5.0°** (no 15°).
  OpenSHC no limita el paso angular directo: clampa la **velocidad** articular (`max_angular_speed_`)
  en `updateJointPositions`. → [robot_model.cpp](src/robot_model.cpp#L116)
- HexaMotion añade una **semilla analítica** (ley de cosenos + atan2, candidatos coxa directo/π y
  tibia elbow-up/down) antes de refinar con DLS (`inverseKinematicsGlobalCoordinates`). OpenSHC
  parte siempre de la pose articular actual (siempre incremental). → [robot_model.cpp](src/robot_model.cpp#L280)
- **Constantes verificadas (varias son IDÉNTICAS, no divergentes):**
    - `dls_coefficient` = **0.02** en HexaMotion (`IK_DLS_COEFFICIENT`) IDÉNTICO a `DLS_COEFFICIENT`
      = 0.02 de OpenSHC. El campo `params.ik.damping_lambda = 30.0f` **existe pero NO se usa** en
      `solveIK` (parámetro muerto). → [hexamotion_constants.h](src/hexamotion_constants.h#L148) ↔ [model.h](OpenSHC/include/syropod_highlevel_controller/model.h#L17)
    - Mezcla del gradiente nullspace Fahimi 2008 (`0.25·posición + 0.75·velocidad`): IDÉNTICA.
    - `JOINT_LIMIT_COST_WEIGHT` = **0.1**: IDÉNTICO en ambos.
    - Tolerancia: HexaMotion `IK_TOLERANCE` = **1.0 mm**; OpenSHC `IK_TOLERANCE` = **0.005 m = 5 mm**.
      HexaMotion es **más estricto** (no "0.5 mm" como decía la versión previa).

**Resumen:** el núcleo DLS (coeficiente, gradiente, peso de límites) es idéntico; lo que diverge es
**la estrategia de convergencia** (bucle interno vs un paso por ciclo), el **clamp angular por
iteración** y la **semilla analítica** inicial.

### 2.6 `rough_terrain_mode` simplificado / no portado por completo

El parámetro/lógica `rough_terrain_mode` de OpenSHC
([parameters_and_states.h](OpenSHC/include/syropod_highlevel_controller/parameters_and_states.h))
y la fusión de sensores externos para `step_plane_pose` en `updateTipPosition`
([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1093)) **no tienen equivalente directo**.
HexaMotion lo sustituye por una clase separada `TerrainAdaptation` con lógica simplificada y un flag
local. No está listado como exclusión. → [terrain_adaptation.cpp](src/terrain_adaptation.cpp)

---

## 3. Divergencias arquitectónicas (NO rompen equivalencia funcional)

Se documentan por completitud; son decisiones de diseño consistentes con un target MCU sin ROS y
mayormente alineadas con el espíritu de `AGENTS.md`, aunque algunas no estén explícitamente listadas:

- **Modelo de objetos colapsado:** OpenSHC usa objetos `Joint`/`Link`/`Tip` (~30 objetos);
  HexaMotion los tabula en arrays (`JointAngles`, `dh_transforms[leg][row][param]`) con ~6 objetos
  `Leg`. Menor RAM, sin pérdida de información cinemática. → [leg.h](src/leg.h), [robot_model.h](src/robot_model.h)
- **Admitancia desacoplada:** OpenSHC mezcla `admittance_delta_`/`virtual_mass_` dentro de `Leg`;
  HexaMotion lo aísla en `AdmittanceController`. Equivalente, mejor separación de responsabilidades.
- **Modularización:** `VelocityLimits`, `WorkspaceAnalyzer`, `TerrainAdaptation`, `GaitConfigFactory`
  y `BodyPoseController` separan lógica que en OpenSHC vive inline en `WalkController`/`Model`.
- **Callbacks ROS → setters de API:** los ~32 callbacks ROS de `StateController` se sustituyen por
  métodos setter directos. Esperado por la eliminación de ROS.
- **FK devuelve `Point3D`** (solo posición) en lugar de `Pose` (posición + rotación); la velocidad de
  tip se delega al `LegStepper` en vez de calcularse en el modelo. Coherente con el límite 3DOF.
- **Sin publishers ROS** (`publishLegState`, `publishPose`, TF, RViz, `debug_visualiser`): excluidos
  intencionalmente.

---

## 4. Funcionalidad añadida en HexaMotion (no existe en OpenSHC)

- Filtrado de contacto FSR con histéresis (`updateFSRHistory`, `getFilteredContactState`). → [leg.h](src/leg.h)
- `notifyRobotReady()` para forzar el estado READY externamente. → [state_controller.cpp](src/state_controller.cpp)
- Manejo de errores explícito: `getLastErrorMessage()`, `getDiagnosticInfo()`, `emergencyStop()`,
  `reset()`.
- `default_height_offset` configurable y `packed_pose_steps[NUM_LEGS][MAX_PACK_STEPS]`.
- `AnalyticRobotModel` para verificación de paridad FK/Jacobiano en tests.
- APIs de alto nivel en `LocomotionSystem` (forward/backward/turn/stop) y setters específicos
  (`setStepFrequency`, `setSwingHeight`, etc.), tal como prevé `AGENTS.md`.

---

## 4.bis Verificación línea por línea (complemento — 31 de mayo de 2026)

> Esta sección complementa el análisis original con una **comparación directa línea por línea** de
> los archivos fuente de HexaMotion frente a OpenSHC. Confirma, corrige o amplía los hallazgos
> previos con la evidencia exacta leída en el código.

### A. IK: un paso DLS por ciclo (OpenSHC) vs bucle interno hasta 30 iteraciones (HexaMotion)

Confirmado leyendo `OpenSHC/src/model.cpp` (`Leg::solveIK` L726 → un solo paso DLS;
`Leg::applyIK` L861 → una llamada por ciclo) frente a `src/robot_model.cpp` (`solveIK` L116 → itera
hasta `IK_DEFAULT_MAX_ITERATIONS = 30` con criterio de parada `position_error.norm() < IK_TOLERANCE`).
Detalle completo y corrección de constantes en la **sección 2.5** (revisada).

### B. Stride freezing confirmado (afecta a los nodos de stance)

`src/leg_stepper.cpp` `updateStride` (L132) congela el stride en la primera iteración de fase
(`if (!stride_frozen_) { frozen_stride_vector_total_ = ...; stride_frozen_ = true; }`), y
`generateStanceControlNodes` usa explícitamente `stride_vector_to_use = stride_frozen_ ?
frozen_stride_vector_total_ : stride_vector_`. OpenSHC `updateStride`
([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L921)) **recalcula** el stride en cada llamada
(`stride_vector_ *= (on_ground_ratio / step.frequency_)`) y `generateStanceControlNodes` siempre usa
el `stride_vector_` vivo. Divergencia real confirmada.

### C. Redondeo de timing (confirmado, con matiz sobre el mínimo)

Verificado en `src/leg_stepper.cpp` `calculateSwingTiming` (L200) frente a
`OpenSHC/src/walk_controller.cpp` `updateTipPosition` (L1035):

- OpenSHC: `swing_iterations = roundToEvenInt(...)` (par **solo en swing**); `stance_iterations`
  **no** se fuerza a par ni tiene mínimo (`stance_delta_t_ = 1.0 / stance_iterations`).
- HexaMotion: fuerza par en **swing y stance**, y aplica un **mínimo de 10 iteraciones a ambas**
  (`if (swing_iterations_ < 10) swing_iterations_ = 10;` y `if (stance_iterations_ < 10) ...`).
  (El documento original mencionaba el mínimo solo para stance; aplica a las dos.)

### D. Nodos Bézier: 1:1 salvo dos detalles

Comparando `generatePrimarySwingControlNodes`, `generateSecondarySwingControlNodes` y
`generateStanceControlNodes` ([src/leg_stepper.cpp](src/leg_stepper.cpp#L264) ↔
[OpenSHC/src/walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1238)), las fórmulas de los 5
nodos son **idénticas**. Dos diferencias añadidas en HexaMotion:

1. **Snap de Z en `swing_2_nodes_[4]`:** HexaMotion fuerza `swing_2_nodes_[4].z = default_tip_pose_.z`
   para eliminar deriva numérica en Z al final del swing. OpenSHC deja `swing_2_nodes_[4] = target`
   sin ajuste. (Extensión anti-drift, coherente con la sección 2.1.)
2. **`forceNormalTouchdown` con separación de nodo simplificada (NUEVO hallazgo):** OpenSHC usa
   `stance_node_seperation = 0.25 · final_tip_velocity · (timeDelta / swing_delta_t_)` con
   `final_tip_velocity = -stride · (stance_delta_t_ / timeDelta)`. HexaMotion lo reduce a
   `final_tip_velocity = stride · (-1/stance_iterations_)` y `sep = final_tip_velocity · 0.25`,
   omitiendo el factor `(timeDelta / swing_delta_t_)`. Matemáticamente difieren en un factor
   ≈ `swing_iterations/2`, por lo que la curva de aterrizaje normal forzado **no es 1:1** cuando
   `force_normal_touchdown` está activo. → [leg_stepper.cpp](src/leg_stepper.cpp#L344) ↔ [walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L1313)

### E. Plano de marcha y toggle de patas (confirmados)

- **Plano de marcha:** OpenSHC `WalkController::updateWalkPlane`
  ([walk_controller.cpp](OpenSHC/src/walk_controller.cpp#L748)) ajusta el plano por
  pseudo-inversa `(AᵀA)⁻¹Aᵀ` sobre las poses **default de TODAS las patas**. HexaMotion lo calcula
  en `BodyPoseController` sobre las patas **en stance**. Confirma la sección 2.4.
- **Toggle de patas:** OpenSHC `legStateToggle`
  ([state_controller.cpp](OpenSHC/src/state_controller.cpp#L542)) gestiona dos banderas
  pendientes independientes (`toggle_primary_leg_state_` / `toggle_secondary_leg_state_`).
  HexaMotion mantiene **una sola** solicitud pendiente (`toggle_leg_index_` /
  `toggle_leg_state_pending_`). Ambos permiten hasta `MAX_MANUAL_LEGS` patas en estado MANUAL
  simultáneamente, pero HexaMotion **no encola** dos toggles a la vez. Confirma la sección 2.3.

---

## 5. Resumen de inconsistencias respecto a "1:1" | ¿Declarada en AGENTS.md? | Severidad |

| --- | --------------------------------------------------------------- | -------------------------------------- | ---------- |
| 1 | Anti-drift híbrido + corrección lateral en `LegStepper` | No (solo se nombran parámetros) | Alta |
| 2 | Congelación de stride (stride freezing) | No | Alta |
| 3 | Phase-end snap | Parcial (parámetro, no comportamiento) | Media |
| 4 | Validación/constrain de workspace durante gait | No | Media |
| 5 | Refactor a integración iterativa explícita + redondeo de stance | No | Media |
| 6 | Toggle de patas manuales 1-a-la-vez (vs 2 simultáneas) | No | Media |
| 7 | Plano de marcha relocalizado y sobre patas en stance | No | Baja-Media |
| 8 | IK: bucle iterativo interno (vs un paso DLS/ciclo) + clamp 5° + semilla analítica | No | Media |
| 9 | `rough_terrain_mode` simplificado / parcial | No | Baja-Media |
| 10 | `forceNormalTouchdown`: separación de nodo simplificada (≠ factor) | No | Baja |

**Conclusión:** la afirmación _"1:1 port of OpenSHC without ROS support"_ es **demasiado fuerte**.
Es exacta para la cinemática, las máquinas de estado, el control de pose/admitancia y la generación
base de Bézier, pero **inexacta** para la capa de generación de trayectoria del `LegStepper` y la
coordinación de patas manuales, donde HexaMotion **añade comportamiento de estabilidad y aplica
simplificaciones** que OpenSHC no posee. Se recomienda reformular la descripción a algo como
_"reimplementación equivalente de OpenSHC para MCU, sin ROS, con extensiones de estabilidad"_ o bien
**documentar explícitamente** en `AGENTS.md` las nueve divergencias de la sección 5.
