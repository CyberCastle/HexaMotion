# Stride-Induced Joint Angle Deviation Analysis

# Análisis de Desviación Angular Inducida por el Stride

---

## 1. Summary / Resumen

**EN:** The joint angle deviations observed at phase transitions during tripod gait are **not a bug**. They are the expected geometric consequence of applying a uniform global stride vector to legs mounted at different base angles on a hexagonal body. The analytical IK model reproduces the observed angles with < 0.005° error (2-decimal precision) across all six legs.

**ES:** Las desviaciones angulares observadas en las transiciones de fase durante la marcha trípode **no son un error**. Son la consecuencia geométrica esperada de aplicar un stride vector global uniforme a patas montadas con diferentes ángulos base en un cuerpo hexagonal. El modelo analítico de IK reproduce los ángulos observados con < 0.005° de error (precisión de 2 decimales) en las seis patas.

---

## 2. Observed Data / Datos Observados

At steady-state velocity (Step 129, stride/2 ≈ 52mm), the test measured:

En velocidad estacionaria (Step 129, stride/2 ≈ 52mm), el test midió:

| Leg | Name | Base Angle | Femur (°) | Tibia (°) | ΔFemur | ΔTibia  |
| --- | ---- | ---------- | --------- | --------- | ------ | ------- |
| 1   | AR   | -30°       | -31.87    | 19.70     | +3.18° | -15.35° |
| 2   | BR   | -90°       | -34.89    | 32.23     | +0.16° | -2.82°  |
| 3   | CR   | -150°      | -31.79    | 44.12     | +3.26° | +9.07°  |
| 4   | CL   | +150°      | -31.79    | 44.12     | +3.26° | +9.07°  |
| 5   | BL   | +90°       | -34.89    | 32.23     | +0.16° | -2.82°  |
| 6   | AL   | +30°       | -31.87    | 19.70     | +3.18° | -15.35° |

Reference / Referencia: femur = -35.05°, tibia = 35.05° (at standing pose, stride = 0).

---

## 3. Root Cause / Causa Raíz

### 3.1 The Stride Formula / La Fórmula del Stride

**EN:** The target tip position for each gait phase is computed as:

**ES:** La posición objetivo del tip para cada fase de marcha se calcula como:

```cpp
target_tip_pose_ = default_tip_pose_ + stride_vector * 0.5;
```

This formula exists identically in both OpenSHC and HexaMotion:

Esta fórmula existe de forma idéntica en OpenSHC y HexaMotion:

- **OpenSHC:** [`walk_controller.cpp` line 1044](https://github.com/csiro-robotics/syropod_highlevel_controller) — `target_tip_pose_.position_ = default_tip_pose_.position_ + 0.5 * stride_vector_;`
- **HexaMotion:** `leg_stepper.cpp` line 489 — `Point3D raw_target = default_tip_pose_ + active_stride * 0.5;`

### 3.2 Why It Affects Each Leg Differently / Por Qué Afecta Diferente a Cada Pata

**EN:** The stride vector is a **uniform global vector** in the direction of travel (X axis for forward walking). However, each leg is mounted at a different base angle on the hexagonal body. The stride displaces the foot tip in X, but the IK solution depends on the **radial distance** from the hip (coxa pivot) to the foot, not on the absolute X position.

The first-order approximation of the radial change is:

$$\Delta r_{radial} \approx \frac{stride}{2} \cdot \cos(\theta_{base})$$

However, the exact radial change uses full Euclidean geometry (see Section 3.3), which includes second-order effects — particularly significant when the stride is tangential to the leg axis.

Note: the coxa (yaw) joint absorbs the tangential component of the stride. Since the femur and tibia operate in the sagittal plane, their angles depend only on the radial distance $R$ and height $Z$, making the radial effect the dominant factor for femur/tibia deviations.

**ES:** El stride vector es un **vector global uniforme** en la dirección de avance (eje X para caminar hacia adelante). Sin embargo, cada pata está montada con un ángulo base diferente en el cuerpo hexagonal. El stride desplaza el punto del pie en X, pero la solución IK depende de la **distancia radial** desde la cadera (pivote de coxa) hasta el pie, no de la posición X absoluta.

La aproximación de primer orden del cambio radial es:

$$\Delta r_{radial} \approx \frac{stride}{2} \cdot \cos(\theta_{base})$$

Sin embargo, el cambio radial exacto usa geometría euclidiana completa (ver Sección 3.3), que incluye efectos de segundo orden — particularmente significativos cuando el stride es tangencial al eje de la pata.

Nota: la coxa (yaw) absorbe la componente tangencial del stride. Como fémur y tibia operan en el plano sagital, sus ángulos dependen solo de la distancia radial $R$ y la altura $Z$, haciendo que el efecto radial sea el factor dominante para las desviaciones de fémur/tibia.

### 3.3 Effect Per Leg Group / Efecto por Grupo de Patas

The "Radial Effect" column uses **exact Euclidean geometry** — the actual change in hip-to-tip distance $\Delta r = \|\mathbf{tip}_{displaced} - \mathbf{hip}\| - reach$ — not the first-order linear approximation $s \cdot \cos\theta$:

| Legs           | Base θ | cos(θ) | Radial Effect (exact)  | Linear approx | IK Result                        |
| -------------- | ------ | ------ | ---------------------- | ------------- | -------------------------------- |
| 1 (AR), 6 (AL) | ±30°   | +0.866 | **Extension** +46.9mm  | +45.0mm       | Tibia **decreases** to 19.7°     |
| 2 (BR), 5 (BL) | ±90°   | 0.000  | **Tangential** +9.8mm  | 0.0mm         | Tibia **barely changes** (32.2°) |
| 3 (CR), 4 (CL) | ±150°  | -0.866 | **Retraction** -41.3mm | -45.0mm       | Tibia **increases** to 44.1°     |

The ±90° case is definitive: $\sqrt{132.69^2 + 52^2} - 132.69 = 9.8$ mm. A purely tangential displacement still changes the Euclidean distance (Pythagorean second-order effect).

**EN:** Front legs (±30°) reach further outward → the 2-link mechanism (femur+tibia) must straighten → tibia angle decreases. Rear-diagonal legs (±150°) retract inward → the mechanism folds more → tibia angle increases. Lateral legs (±90°) receive mostly tangential displacement → minimal angle change.

**ES:** Las patas frontales (±30°) se extienden más hacia afuera → el mecanismo de 2 eslabones (fémur+tibia) debe estirarse → el ángulo de tibia disminuye. Las patas traseras-diagonales (±150°) se retraen hacia adentro → el mecanismo se pliega más → el ángulo de tibia aumenta. Las patas laterales (±90°) reciben desplazamiento mayormente tangencial → cambio angular mínimo.

### 3.4 Progressive Growth / Crecimiento Progresivo

**EN:** The deviations grow progressively across gait cycles because the `WalkController` applies an acceleration ramp from 0 to the commanded velocity. As velocity increases, the stride vector grows proportionally:

$$stride = v \cdot \frac{stance\_period / period}{frequency}$$

**Dimensional analysis:** $[mm/s] \cdot [dimensionless] / [Hz] = [mm/s] \cdot [s] = [mm]$. The ratio `stance_period / period` is dimensionless (the on-ground fraction), and dividing by frequency $[1/s]$ yields the ground contact time per step cycle.

**ES:** Las desviaciones crecen progresivamente a través de los ciclos de marcha porque el `WalkController` aplica una rampa de aceleración de 0 a la velocidad comandada. A medida que la velocidad aumenta, el stride vector crece proporcionalmente:

$$stride = v \cdot \frac{stance\_period / period}{frequency}$$

**Análisis dimensional:** $[mm/s] \cdot [adimensional] / [Hz] = [mm/s] \cdot [s] = [mm]$. La relación `stance_period / period` es adimensional (la fracción de apoyo), y dividir por la frecuencia $[1/s]$ da el tiempo de contacto con el suelo por ciclo de paso.

| Step | stride/2 (mm) | Tibia Leg 1 (°) | ΔTibia |
| ---- | ------------- | --------------- | ------ |
| 0    | ~1            | ~35.0           | ~0°    |
| 77   | ~27.6         | 27.6            | -7.4°  |
| 129  | ~52.0         | 19.7            | -15.3° |

### 3.5 Perfect Symmetry / Simetría Perfecta

**EN:** Opposite leg pairs (1↔6, 2↔5, 3↔4) show identical deviations because their base angles are opposite in sign (+θ, -θ). Since cosine has even symmetry — cos(-θ) = cos(θ) — both legs in each pair experience the same radial displacement and therefore the same IK solution. This mirror symmetry is an intrinsic property of the hexagonal geometry with a linear stride vector.

**ES:** Los pares de patas opuestas (1↔6, 2↔5, 3↔4) muestran desviaciones idénticas porque sus ángulos base son opuestos en signo (+θ, -θ). Como el coseno tiene simetría par — cos(-θ) = cos(θ) — ambas patas de cada par experimentan el mismo desplazamiento radial y por tanto la misma solución IK. Esta simetría especular es una propiedad intrínseca de la geometría hexagonal con un stride vector lineal.

---

## 4. Numerical Verification / Verificación Numérica

**EN:** An analytical 2-link IK model (closed-form asin + atan2 in the sagittal plane) was used to predict joint angles from the stride-displaced tip positions. The HexaMotion test uses a different path: full DH 4×4 chain with Jacobian-based DLS iterative solver. Despite being genuinely different calculation methods, the results agree within **< 0.005°** (2-decimal precision) across all six legs:

**ES:** Se utilizó un modelo analítico de IK de 2 eslabones (forma cerrada asin + atan2 en el plano sagital) para predecir los ángulos articulares desde las posiciones desplazadas por el stride. El test de HexaMotion usa un camino diferente: cadena DH 4×4 completa con solver iterativo DLS basado en Jacobiano. A pesar de ser métodos de cálculo genuinamente distintos, los resultados coinciden dentro de **< 0.005°** (precisión de 2 decimales) en las seis patas:

```
Leg1(AR): calc=(-31.87, 19.70)  obs=(-31.87, 19.70)  err=0.00°
Leg2(BR): calc=(-34.89, 32.23)  obs=(-34.89, 32.23)  err=0.00°
Leg3(CR): calc=(-31.79, 44.12)  obs=(-31.79, 44.12)  err=0.00°
Leg4(CL): calc=(-31.79, 44.12)  obs=(-31.79, 44.12)  err=0.00°
Leg5(BL): calc=(-34.89, 32.23)  obs=(-34.89, 32.23)  err=0.00°
Leg6(AL): calc=(-31.87, 19.70)  obs=(-31.87, 19.70)  err=0.00°
```

---

## 5. References / Referencias

### 5.1 OpenSHC Source Code / Código Fuente de OpenSHC

1. **Target generation formula** — `OpenSHC/src/walk_controller.cpp` line 1044:

    ```cpp
    target_tip_pose_.position_ = default_tip_pose_.position_ + 0.5 * stride_vector_;
    ```

    This is the exact formula that produces the stride-displaced target position. The HexaMotion implementation (`leg_stepper.cpp` line 489) is a 1:1 port.

    **ES:** Esta es la fórmula exacta que produce la posición objetivo desplazada por el stride. La implementación de HexaMotion (`leg_stepper.cpp` línea 489) es un port 1:1.

2. **Global stride vector computation** — `OpenSHC/src/walk_controller.cpp` lines 930-940:

    ```cpp
    Eigen::Vector3d stride_vector_linear(Eigen::Vector3d(velocity[0], velocity[1], 0.0));
    Eigen::Vector3d stride_vector_angular = angular_velocity.cross(radius);
    stride_vector_ = stride_vector_linear + stride_vector_angular;
    stride_vector_ *= (on_ground_ratio / step.frequency_);
    ```

    The stride is a **uniform global vector** (same for all legs in pure linear mode). Each leg receives the same X displacement, but its kinematics project it differently based on its base angle.

    **ES:** El stride es un **vector global uniforme** (el mismo para todas las patas en modo lineal puro). Cada pata recibe el mismo desplazamiento en X, pero su cinemática lo proyecta de forma diferente según su ángulo base.

3. **Bearing-dependent velocity limits** — `OpenSHC/src/walk_controller.cpp` lines 420-430:

    ```cpp
    Eigen::Vector2d stride_vector = linear_velocity_input + angular_velocity_input * rotation_normal;
    int bearing = mod(roundToInt(radiansToDegrees(atan2(stride_vector[1], stride_vector[0]))), 360);
    ```

    OpenSHC computes velocity/acceleration limits per bearing direction, **precisely because** different stride directions produce different radial reach effects per leg. This confirms the angular-dependent behavior is a known design consideration.

    **ES:** OpenSHC calcula los límites de velocidad/aceleración por dirección de bearing, **precisamente porque** diferentes direcciones de stride producen diferentes efectos de alcance radial por pata. Esto confirma que el comportamiento dependiente del ángulo es una consideración de diseño conocida.

4. **Walkspace radius limiting** — `OpenSHC/src/walk_controller.cpp` lines 295-310:

    ```cpp
    double max_speed = (walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
    ```

    Maximum stride is designed as `2 × walkspace_radius` per direction. The walkspace radius varies by bearing to ensure the stride-displaced tip stays within the reachable workspace of each leg.

    **ES:** El stride máximo está diseñado como `2 × walkspace_radius` por dirección. El radio del walkspace varía según el bearing para asegurar que el tip desplazado por el stride permanezca dentro del espacio de trabajo alcanzable de cada pata.

### 5.2 Published Paper / Artículo Publicado

> B. Tam, F. Talbot, R. Steindl, A. Elfes and N. Kottege, _"OpenSHC: A Versatile Multilegged Robot Controller,"_ in IEEE Access, vol. 8, pp. 188908-188926, 2020, doi: [10.1109/ACCESS.2020.3031019](https://doi.org/10.1109/ACCESS.2020.3031019).

**EN:** This paper formally describes the OpenSHC locomotion controller, including quartic Bézier trajectory generation and stride vector computation. The `target = default + stride/2` formula is part of the documented gait planner design. Section IV-C covers the walk controller and stride generation architecture.

**ES:** Este artículo describe formalmente el controlador de locomoción OpenSHC, incluyendo la generación de trayectorias con Bézier cuártico y el cálculo del stride vector. La fórmula `target = default + stride/2` es parte del diseño documentado del planificador de marcha. La Sección IV-C cubre la arquitectura del walk controller y la generación de stride.

### 5.3 Kinematic Principle / Principio Cinemático

**EN:** For any hexapod robot with radially distributed legs and a uniform global stride vector, the foot displacement relative to the hip has a radial component that varies with the cosine of the leg's base angle. This is a direct result of projecting a Cartesian vector onto a radial direction — a fundamental geometric property documented in standard robotics textbooks:

> Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). _Robotics: Modelling, Planning and Control._ Springer. — Chapter 3 (Differential Kinematics) covers how Cartesian displacements project onto joint-space configurations through the Jacobian, producing configuration-dependent angle changes for the same Cartesian offset.

**ES:** Para cualquier robot hexápodo con patas distribuidas radialmente y un stride vector global uniforme, el desplazamiento del pie relativo a la cadera tiene una componente radial que varía con el coseno del ángulo base de la pata. Esto es el resultado directo de proyectar un vector cartesiano sobre una dirección radial — una propiedad geométrica fundamental documentada en libros de texto estándar de robótica:

> Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). _Robotics: Modelling, Planning and Control._ Springer. — El Capítulo 3 (Cinemática Diferencial) cubre cómo los desplazamientos cartesianos se proyectan en configuraciones del espacio articular a través del Jacobiano, produciendo cambios de ángulo dependientes de la configuración para el mismo offset cartesiano.

---

## 6. Conclusion / Conclusión

**EN:** The reference angles of -35.05°/35.05° (femur/tibia) are valid **only at the standing pose** (stride = 0). During active walking, the stride displacement modifies the radial distance from hip to foot differently for each leg based on its angular mounting position, producing correct but different IK solutions. **There is no error or numerical drift — this is the physics of hexapod locomotion with a uniform global stride.**

**ES:** Los ángulos de referencia de -35.05°/35.05° (fémur/tibia) son válidos **solo en la posición de reposo** (stride = 0). Durante la marcha activa, el desplazamiento del stride modifica la distancia radial de la cadera al pie de forma diferente para cada pata según su posición angular de montaje, produciendo soluciones IK correctas pero distintas del nominal. **No hay error ni drift numérico — es la física de la locomoción hexápoda con un stride global uniforme.**
