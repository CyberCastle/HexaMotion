# Stride-Induced Joint Angle Deviation

## 1. Summary

The joint angle deviations observed at phase transitions during tripod gait are **not a bug**. They are the expected geometric consequence of applying a uniform global stride vector to legs mounted at different base angles on a hexagonal body. The analytical IK model reproduces the observed angles with < 0.005° error (2-decimal precision) across all six legs.

## 2. Observed Data

At steady-state velocity (Step 129, stride/2 ≈ 52 mm), the test measured:

| Leg | Name | Base Angle | Femur (°) | Tibia (°) | ΔFemur | ΔTibia  |
| --- | ---- | ---------- | --------- | --------- | ------ | ------- |
| 1   | AR   | -30°       | -31.87    | 19.70     | +3.18° | -15.35° |
| 2   | BR   | -90°       | -34.89    | 32.23     | +0.16° | -2.82°  |
| 3   | CR   | -150°      | -31.79    | 44.12     | +3.26° | +9.07°  |
| 4   | CL   | +150°      | -31.79    | 44.12     | +3.26° | +9.07°  |
| 5   | BL   | +90°       | -34.89    | 32.23     | +0.16° | -2.82°  |
| 6   | AL   | +30°       | -31.87    | 19.70     | +3.18° | -15.35° |

Reference: femur = -35.05°, tibia = 35.05° (at standing pose, stride = 0).

## 3. Root Cause

### 3.1 The Stride Formula

The target tip position for each gait phase is computed as:

```cpp
target_tip_pose_ = default_tip_pose_ + stride_vector * 0.5;
```

This formula exists identically in both OpenSHC and HexaMotion:

- **OpenSHC:** `OpenSHC/src/walk_controller.cpp` line 1044 — `target_tip_pose_.position_ = default_tip_pose_.position_ + 0.5 * stride_vector_;`
- **HexaMotion:** `src/leg_stepper.cpp` line 489 — `Point3D raw_target = default_tip_pose_ + active_stride * 0.5;`

### 3.2 Why It Affects Each Leg Differently

The stride vector is a **uniform global vector** in the direction of travel (X axis for forward walking). However, each leg is mounted at a different base angle on the hexagonal body. The stride displaces the foot tip in X, but the IK solution depends on the **radial distance** from the hip (coxa pivot) to the foot, not on the absolute X position.

The first-order approximation of the radial change is:

$$\Delta r_{radial} \approx \frac{stride}{2} \cdot \cos(\theta_{base})$$

The exact radial change uses full Euclidean geometry (see Section 3.3), which includes second-order effects — particularly significant when the stride is tangential to the leg axis.

Note: the coxa (yaw) joint absorbs the tangential component of the stride. Since the femur and tibia operate in the sagittal plane, their angles depend only on the radial distance $R$ and height $Z$, making the radial effect the dominant factor for femur/tibia deviations.

### 3.3 Effect Per Leg Group

The "Radial Effect" column uses **exact Euclidean geometry** — the actual change in hip-to-tip distance $\Delta r = \|\mathbf{tip}_{displaced} - \mathbf{hip}\| - reach$ — not the first-order linear approximation $s \cdot \cos\theta$:

| Legs           | Base θ | cos(θ) | Radial Effect (exact)  | Linear approx | IK Result                        |
| -------------- | ------ | ------ | ---------------------- | ------------- | -------------------------------- |
| 1 (AR), 6 (AL) | ±30°   | +0.866 | **Extension** +46.9mm  | +45.0mm       | Tibia **decreases** to 19.7°     |
| 2 (BR), 5 (BL) | ±90°   | 0.000  | **Tangential** +9.8mm  | 0.0mm         | Tibia **barely changes** (32.2°) |
| 3 (CR), 4 (CL) | ±150°  | -0.866 | **Retraction** -41.3mm | -45.0mm       | Tibia **increases** to 44.1°     |

The ±90° case is definitive: $\sqrt{132.69^2 + 52^2} - 132.69 = 9.8$ mm. A purely tangential displacement still changes the Euclidean distance (Pythagorean second-order effect).

Front legs (±30°) reach further outward → the 2-link mechanism (femur+tibia) must straighten → tibia angle decreases. Rear-diagonal legs (±150°) retract inward → the mechanism folds more → tibia angle increases. Lateral legs (±90°) receive mostly tangential displacement → minimal angle change.

### 3.4 Progressive Growth

The deviations grow progressively across gait cycles because the `WalkController` applies an acceleration ramp from 0 to the commanded velocity. As velocity increases, the stride vector grows proportionally:

$$stride = v \cdot \frac{stance\_period / period}{frequency}$$

**Dimensional analysis:** $[mm/s] \cdot [dimensionless] / [Hz] = [mm/s] \cdot [s] = [mm]$. The ratio `stance_period / period` is dimensionless (the on-ground fraction), and dividing by frequency $[1/s]$ yields the ground contact time per step cycle.

| Step | stride/2 (mm) | Tibia Leg 1 (°) | ΔTibia |
| ---- | ------------- | --------------- | ------ |
| 0    | ~1            | ~35.0           | ~0°    |
| 77   | ~27.6         | 27.6            | -7.4°  |
| 129  | ~52.0         | 19.7            | -15.3° |

### 3.5 Perfect Symmetry

Opposite leg pairs (1↔6, 2↔5, 3↔4) show identical deviations because their base angles are opposite in sign (+θ, -θ). Since cosine has even symmetry — cos(-θ) = cos(θ) — both legs in each pair experience the same radial displacement and therefore the same IK solution. This mirror symmetry is an intrinsic property of the hexagonal geometry with a linear stride vector.

## 4. Numerical Verification

An analytical 2-link IK model (closed-form asin + atan2 in the sagittal plane) was used to predict joint angles from the stride-displaced tip positions. The HexaMotion test uses a different path: full DH 4×4 chain with Jacobian-based DLS iterative solver. Despite being genuinely different calculation methods, the results agree within **< 0.005°** (2-decimal precision) across all six legs:

```
Leg1(AR): calc=(-31.87, 19.70)  obs=(-31.87, 19.70)  err=0.00°
Leg2(BR): calc=(-34.89, 32.23)  obs=(-34.89, 32.23)  err=0.00°
Leg3(CR): calc=(-31.79, 44.12)  obs=(-31.79, 44.12)  err=0.00°
Leg4(CL): calc=(-31.79, 44.12)  obs=(-31.79, 44.12)  err=0.00°
Leg5(BL): calc=(-34.89, 32.23)  obs=(-34.89, 32.23)  err=0.00°
Leg6(AL): calc=(-31.87, 19.70)  obs=(-31.87, 19.70)  err=0.00°
```

## 5. Solution Strategies

The following strategies from the legged-locomotion literature address this phenomenon, categorized by whether the goal is navigation precision (OpenSHC/HexaMotion approach) or pure joint symmetry (biomimetic approach irrelevant to precise odometry).

### Solution A: Cartesian Priority (Standard in Navigation Robotics)

**Method:** Maintain stride generation in the body frame and accept the resulting joint deviations.
**Application:** OpenSHC, HexaMotion, ROS Hexapod Stack.

- **Rationale:** The primary goal is for the robot to move from point A to point B in a straight line.
- **Mechanism:** The gait planner computes `target_tip = current_tip + stride_vector`. Inverse Kinematics (IK) solves for the angles needed to reach that exact position.
- **Result:** The legs move asymmetrically from a joint perspective, but the body moves precisely in the world frame.
- **Advantage:** Maximizes odometry precision and avoids lateral slippage.
- **Recommendation:** This is the correct and current HexaMotion implementation. No changes required.

### Solution B: Body Orientation Compensation (Active Heading / Crabbing)

**Method:** Actively rotate the robot body to align the motion vector with the legs' most favorable work planes.

- **Rationale:** Legs have greater range of motion and linearity when movement is purely radial (extension/retraction) or purely tangential (coxa sweep).
- **Mechanism:** If the robot must travel a long distance along the X axis, the controller rotates the body (Yaw) so the stride vector is optimal for the majority of legs (e.g., a "crab" gait at 90°).
- **Implementation:** Requires a higher-level planning layer that decides the chassis orientation ($\psi_{body}$) independently of the velocity vector ($\vec{v}$).

### Solution C: Joint-Space Gait Generation

**Method:** Generate identical cyclic trajectories (sine/cosine/splines) for all joints, ignoring Cartesian linearity.
**Application:** Simple biomimetic robots, toys, highly irregular terrain where precision does not matter.

- **Mechanism:** `angle(t) = Amplitude * sin(omega * t + phase_offset)`. All legs use the same amplitude.
- **Result:** The legs trace arcs on the ground instead of straight lines; the body advances with an oscillating motion ("wobble").
- **Critical Disadvantage:** Incompatible with precise navigation or SLAM systems, as odometry becomes erratic. **Not recommended for HexaMotion.**

### Solution D: Dynamic Stride Limiting

**Method:** Reduce stride length based on the "worst-case" leg.

- **Mechanism:** Compute the maximum allowable stride such that the angular deviation of the most disadvantaged leg does not exceed a threshold (e.g., $\Delta\theta < 5^\circ$).
- **Result:** Smoother, more symmetric movement, at the cost of significantly reduced maximum speed.

### Solution E: Morphology Change (Rectangular vs Hexagonal)

**Method:** Modify the physical layout of the legs.

- **Mechanism:** A rectangular (in-line / parallel) configuration aligns all legs with the main direction of travel, producing identical joint displacements for a forward stride.
- **Cost:** The omnidirectionality of the hexagon is lost (the robot turns and walks sideways less gracefully).

## 6. Conclusion

The reference angles of -35.05°/35.05° (femur/tibia) are valid **only at the standing pose** (stride = 0). During active walking, the stride displacement modifies the radial distance from hip to foot differently for each leg based on its angular mounting position, producing correct but different IK solutions. **There is no error or numerical drift — this is the physics of hexapod locomotion with a uniform global stride.**

For HexaMotion (functional parity with OpenSHC for research/navigation), the recommended approach is **Solution A (Cartesian Priority)**. The behavior should be documented as "Expected by Design"; do not attempt to force equal angles, as this would break straight-line locomotion.

## 7. References

- B. Tam, F. Talbot, R. Steindl, A. Elfes and N. Kottege, _"OpenSHC: A Versatile Multilegged Robot Controller,"_ IEEE Access, vol. 8, pp. 188908-188926, 2020, doi: [10.1109/ACCESS.2020.3031019](https://doi.org/10.1109/ACCESS.2020.3031019).
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). _Robotics: Modelling, Planning and Control._ Springer. (Chapter 3: Differential Kinematics).
- Waldron, K. J. (1984). _"Configuration design of the adaptive suspension vehicle."_ The International Journal of Robotics Research.
- Cruse, H. (1990). _"What mechanisms coordinate leg movement in walking arthropods?"_ Trends in Neurosciences.

### Relevant OpenSHC Source

- **Global stride vector** — `OpenSHC/src/walk_controller.cpp` lines 930-940: stride is a uniform global vector scaled by `on_ground_ratio / step.frequency_`.
- **Bearing-dependent velocity limits** — `OpenSHC/src/walk_controller.cpp` lines 420-430: limits are computed per bearing direction precisely because different stride directions produce different radial reach effects per leg.
- **Walkspace radius limiting** — `OpenSHC/src/walk_controller.cpp` lines 295-310: maximum stride is `2 × walkspace_radius` per direction, with the walkspace radius varying by bearing.
