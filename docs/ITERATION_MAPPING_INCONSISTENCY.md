# Swing/Stance Iteration Mapping — Inconsistencia 1:1 con OpenSHC

## Resumen

La implementación de `LegStepper::updateTipPositionIterative()` en HexaMotion no es equivalente 1:1 a `LegStepper::updateTipPosition()` de OpenSHC. Existen dos divergencias estructurales que se compensan mutuamente en producción pero rompen cualquier invocación que no replique exactamente el flujo del `WalkController`.

---

## 1. Divergencia en el orden de llamadas

### OpenSHC (`walk_controller.cpp` líneas 637–639)

```cpp
leg_stepper->updateTipPosition();   // usa phase_ actual
leg_stepper->updateTipRotation();
leg_stepper->iteratePhase();        // phase_ = (phase_ + 1) % period_
```

**`phase_` se usa primero y se incrementa después.** La primera iteración de swing recibe `phase_ = swing_start_` y la primera de stance recibe `phase_ = stance_start_`.

### HexaMotion (`walk_controller.cpp` líneas 562–621)

```cpp
int current_phase = leg_stepper->getPhase();
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);           // phase_ incrementado
leg_stepper->updateStepStateFromPhase();         // determina SWING/STANCE
// ...
leg_stepper->updateTipPosition(time_delta, ...); // usa phase_ ya incrementado
```

**`phase_` se incrementa primero y se usa después.** Cada valor de `phase_` que entra a `updateTipPositionIterative` lleva un desfase de +1 respecto a OpenSHC.

---

## 2. Divergencia en el layout del ciclo de fase

### OpenSHC — Layout envolvente (wrapping stance)

El ciclo se organiza como `[stance₁ | SWING | stance₂]`, donde la fase stance envuelve el borde del periodo:

| Campo           | Valor (tripod 50/50, period=52) |
| --------------- | ------------------------------- |
| `stance_end_`   | 13                              |
| `swing_start_`  | 13                              |
| `swing_end_`    | 39                              |
| `stance_start_` | 39                              |

Stance ocupa fases `[39..51] ∪ [0..12]` (envolvente). Por eso OpenSHC necesita `mod()` con offset para re-indexar la iteración de stance.

### HexaMotion — Layout contiguo

El ciclo se genera en `GaitConfiguration::generateStepCycle()` como `[STANCE | SWING]`:

| Campo           | Valor (tripod 50/50, period=52) |
| --------------- | ------------------------------- |
| `stance_start_` | 0                               |
| `stance_end_`   | 26                              |
| `swing_start_`  | 26                              |
| `swing_end_`    | 52                              |

Stance ocupa fases `[0..25]` y swing `[26..51]`. No hay wraparound.

---

## 3. Efecto combinado sobre las fórmulas de iteración

### SWING

**OpenSHC** (`phase_` sin incrementar aún):

```cpp
int iteration = phase_ - step.swing_start_ + 1;
// phase_=13 → 13-13+1 = 1     (primera iteración)
// phase_=38 → 38-13+1 = 26    (última iteración)
// Rango: [1..26]  ✓
```

**HexaMotion** (`phase_` ya incrementado, `iteration = phase_`):

```cpp
int swing_iteration = iteration % swing_iterations_;
if (swing_iteration == 0) swing_iteration = swing_iterations_;
// iteration=26 → 26%26=0 → corregido a 26   (¡última iteración en vez de primera!)
// iteration=27 → 27%26=1                      (segunda)
// iteration=51 → 51%26=25                     (penúltima)
// Rango: [26, 1, 2, ..., 25]  ❌
```

La primera iteración de swing produce `swing_iteration=26` (`time_input=1.0`), que debería ser 1 (`time_input=swing_delta_t_`).

**¿Por qué no se nota en producción?** Porque `initializeSwingPeriod()` resetea `swing_origin` a la posición actual, y los nodos de control se regeneran cada iteración desde ese mismo origen. El `quarticBezierDot` en `t=1.0` con origen ≈ posición actual produce un `delta_pos ≈ 0`. Es un "paso fantasma" que se tolera silenciosamente.

### STANCE

**OpenSHC** (`phase_` sin incrementar, layout envolvente):

```cpp
int iteration = mod(phase_ + (step.period_ - modified_stance_start), step.period_) + 1;
// modified_stance_start=39, period_=52
// phase_=39 → mod(39+13, 52)+1 = 0+1 = 1       (primera)
// phase_=12 → mod(12+13, 52)+1 = 25+1 = 26      (última)
// Rango: [1..26]  ✓
```

**HexaMotion** (`phase_` ya incrementado, layout contiguo):

```cpp
int stance_iteration = (iteration % stance_iterations_) + 1;
// iteration=phase_=0 → (0%26)+1 = 1       (primera)
// iteration=phase_=25 → (25%26)+1 = 26    (última)
// Rango: [1..26]  ✓
```

En producción, el desfase de +1 aterriza en `phase_=0` al inicio de stance (porque `(51+1)%52=0`), y `(0%26)+1=1`. **El mapeo de stance es correcto en producción por compensación entre el pre-incremento y el layout contiguo desde 0.**

---

## 4. Tabla resumen de equivalencia

| Aspecto                  | OpenSHC                                  | HexaMotion                                | ¿1:1? |
| ------------------------ | ---------------------------------------- | ----------------------------------------- | ----- |
| Orden phase/update       | update → phase++                         | phase++ → update                          | ❌    |
| Layout stance            | Envolvente `[start..P) ∪ [0..end)`       | Contiguo `[0..N)`                         | ❌    |
| Swing iteration fórmula  | `phase_ - swing_start_ + 1` → [1..N]     | `phase_ % N`, fix 0→N → [N, 1..N-1]       | ❌    |
| Stance iteration fórmula | `mod(phase_ + offset, P) + 1` → [1..N]   | `(phase_ % N) + 1` → [1..N]               | ✅\*  |
| Swing `time_input`       | `swing_delta_t_ * iter` con iter∈[1..N]  | Primer tick: t=1.0 ("paso fantasma")      | ❌    |
| Stance `time_input`      | `iter * stance_delta_t_` con iter∈[1..N] | `iter * stance_delta_t_` con iter∈[1..N]  | ✅    |
| Swing origin init        | `if (iteration == 1)`                    | fallback por `iteration < last_iteration` | ~✅   |
| Stance origin init       | `if (iteration == 1)`                    | `if (stance_iteration == 1)`              | ✅\*  |

**(\*)** Funciona en producción por compensación implícita. Falla si se invoca con iteraciones que no sigan la convención `phase_` de producción (ej: tests con iteraciones 1-based).

---

## 5. Impacto observable

### En producción

- **Swing**: el paso fantasma en la primera iteración produce `delta_pos ≈ 0`, que es tolerado. La trayectoria se desarrolla correctamente a partir de la segunda iteración.
- **Stance**: funciona correctamente por compensación.
- **No se reportan errores** porque el flujo del `WalkController` siempre invoca con la convención exact de `phase_`.

### En tests

- Cualquier test que pase iteraciones 1-based (ej: `trajectory_tip_position_test`) observa:
    - Stance: `iteration=swing_iterations+1` causa `stance_iteration` desfasado; la inicialización del origen se ejecuta en la **última** iteración en vez de la primera, produciendo una discontinuidad comprobable.
    - Swing: `iteration=1` produce `swing_iteration=1` correctamente por coincidencia (1%26=1).

---

## 6. Acciones correctivas

### Acción 1: Alinear el orden de llamadas con OpenSHC

Modificar `WalkController::updateWalk()` para que `updateTipPosition` se llame **antes** de incrementar `phase_`, idéntico a OpenSHC.

**Archivo:** `src/walk_controller.cpp`

**Cambio:** Restructurar el bucle para que la secuencia sea:

```cpp
// ANTES (actual)
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();
// ... estado STARTING/STOPPING ...
leg_stepper->updateTipPosition(time_delta, ...);

// DESPUÉS (propuesto - equivalente a OpenSHC)
leg_stepper->updateTipPosition(time_delta, ...);
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();
// ... estado STARTING/STOPPING ...
```

**Riesgo:** Alto. `updateStepStateFromPhase()` determina SWING/STANCE antes de llamar a `updateTipPosition`, y ese resultado es usado por el stepper para decidir qué rama ejecutar. Si se mueve después, `step_state_` no estaría actualizado al momento de generar la trayectoria.

**Mitigación:** Separar la determinación de estado de la iteración de fase. Opción: llamar `updateStepStateFromPhase()` al inicio del tick (sin incrementar phase*), luego `updateTipPosition`, luego `iteratePhase` (phase*++ + updateStepState):

```cpp
leg_stepper->updateStepStateFromPhase();        // estado basado en phase_ actual
leg_stepper->updateTipPosition(time_delta, ...); // usa phase_ actual
// ... estado STARTING/STOPPING ...
current_phase = (current_phase + 1) % step_cycle.period_;
leg_stepper->setPhase(current_phase);
leg_stepper->updateStepStateFromPhase();          // prepara estado para siguiente tick
```

### Acción 2: Reescribir fórmulas de swing iteration como en OpenSHC

Usar una fórmula explícita `phase_ - swing_start_ + 1` en lugar del módulo genérico.

**Archivo:** `src/leg_stepper.cpp`, rama `STEP_SWING` de `updateTipPositionIterative`

**Cambio:**

```cpp
// ANTES (actual)
int swing_iteration = iteration % swing_iterations_;
if (swing_iteration == 0)
    swing_iteration = swing_iterations_;

// DESPUÉS (propuesto — equivalente a OpenSHC)
int swing_iteration = iteration - step_cycle_.swing_start_ + 1;
// Clamp por seguridad
if (swing_iteration < 1) swing_iteration = 1;
if (swing_iteration > swing_iterations_) swing_iteration = swing_iterations_;
```

**Precondición:** Requiere que la Acción 1 esté implementada (para que `iteration = phase_` aún no esté incrementado al entrar).

### Acción 3: Reescribir fórmula de stance iteration como en OpenSHC

Usar la fórmula con `mod` y `modified_stance_start` para ser robusto ante cualquier layout.

**Archivo:** `src/leg_stepper.cpp`, rama `STEP_STANCE` de `updateTipPositionIterative`

**Cambio:**

```cpp
// ANTES (actual)
int stance_iteration = (iteration % std::max(1, stance_iterations_)) + 1;

// DESPUÉS (propuesto — equivalente a OpenSHC)
int modified_stance_start = (step_state_ == STEP_SWING || completed_first_step_)
    ? step_cycle_.stance_start_
    : static_cast<int>(phase_offset_ * step_cycle_.period_);
int stance_iteration = math_utils::mod(iteration + (step_cycle_.period_ - modified_stance_start),
                                        step_cycle_.period_) + 1;
```

**Precondición:** Requiere que la Acción 1 esté implementada. Además, necesita que `math_utils::mod` implemente el módulo siempre-positivo: `(a % b + b) % b`.

### Acción 4: Añadir `math_utils::mod` (módulo siempre-positivo)

**Archivo:** `src/math_utils.h`

```cpp
/** Always-positive modulo, matching OpenSHC's mod() template. */
inline int mod(int a, int b) { return ((a % b) + b) % b; }
```

### Acción 5: Corregir el test `trajectory_tip_position_test.cpp`

Una vez implementadas las acciones 1–4, el test debe invocar `updateTipPositionIterative` con valores de `phase_` equivalentes a los de producción, sin necesidad de convenciones ad-hoc.

Alternativamente, como medida intermedia **sin modificar producción**, el test puede simular el flujo exacto de producción:

```cpp
// En vez de:
for (int iteration = 1; iteration <= swing_iterations; iteration++) {
    stepper.updateTipPositionIterative(iteration, ...);
}

// Usar el flujo de producción:
for (int phase = step_cycle.swing_start_; phase < step_cycle.swing_end_; phase++) {
    stepper.setPhase(phase);
    stepper.updateStepStateFromPhase();
    stepper.updateTipPosition(time_delta, false, false);
}
```

---

## 7. Orden de implementación recomendado

| Prioridad | Acción                              | Dependencias  | Impacto                             |
| --------- | ----------------------------------- | ------------- | ----------------------------------- |
| 1         | Acción 4 — `math_utils::mod`        | Ninguna       | Bajo (aditivo)                      |
| 2         | Acción 1 — Reordenar phase/update   | Acción 4      | **Alto** (modifica flujo principal) |
| 3         | Acción 2 — Swing iteration fórmula  | Acción 1      | Medio                               |
| 4         | Acción 3 — Stance iteration fórmula | Acciones 1, 4 | Medio                               |
| 5         | Acción 5 — Actualizar tests         | Acciones 1–4  | Bajo                                |

### Alternativa conservadora (solo tests, sin tocar producción)

Si el riesgo de modificar el flujo principal es inaceptable en este momento:

1. Documentar que `updateTipPositionIterative` espera valores de `phase_` con la convención de producción (pre-incrementados, 0-based).
2. Los tests deben replicar el flujo exacto de `WalkController`: incrementar phase\_, luego updateStepState, luego updateTipPosition.
3. Aceptar el "paso fantasma" de swing como comportamiento conocido.

---

## 8. Verificación

Tras aplicar las correcciones, los siguientes criterios deben cumplirse:

1. **Swing**: `time_input` del primer tick de swing debe ser `swing_delta_t_` (≈0.077), no 1.0.
2. **Stance**: `stance_iteration == 1` exactamente en la primera iteración de stance (no en la última).
3. **Test**: `trajectory_tip_position_test` — la validación de stance debe reportar coxa como articulación dominante: `abs(total_coxa_change) > abs(total_tibia_change)`.
4. **Regresión**: todos los tests existentes (`make` en `tests/`) deben seguir pasando.
