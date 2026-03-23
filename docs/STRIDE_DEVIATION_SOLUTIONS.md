# Solutions and Strategies for Stride-Induced Joint Angle Deviation

---

## 1. Executive Summary

Academic and technical research confirms that the **Stride-Induced Joint Angle Deviation** documented in _STRIDE_ANGLE_DEVIATION_ANALYSIS.md_ is not an implementation defect, but a **necessary geometric consequence** of omnidirectional rectilinear locomotion in a hexapod robot with a circular/hexagonal body.

Although one might intuitively expect all legs to perform the same joint cycle, this is mathematically impossible if the body is to move in a perfect straight line (Cartesianly perfect) with a uniform velocity vector.

This document presents the standard academic strategies to address this phenomenon, categorized by whether the goal is navigation precision (OpenSHC/HexaMotion approach) or pure joint symmetry (biomimetic approach irrelevant to precise odometry).

---

## 2. Phenomenon Analysis

In a hexapod with radially distributed legs (base angles of ±30°, ±90°, ±150°), applying a uniform stride vector $\vec{v}_{global}$ (e.g., advancing along X) produces:

1.  **Different projections** in each leg's local frame.
2.  **Unequal radial changes** (extension/contraction) to generate the same linear displacement.
3.  **Divergent joint angles** (Femur/Tibia) between leg groups.

Attempting to force identical angles on all legs would inevitably result in curved or deviated trajectories in Cartesian space, causing foot slippage and errors in the robot's position estimation.

---

## 3. Academic Solution Strategies

The following details the approaches found in the legged locomotion literature for handling this discrepancy.

### Solution A: Cartesian Priority (Standard in Navigation Robotics)

**Method:** Maintain stride generation in the body frame and accept the resulting joint deviations.
**Application:** OpenSHC, HexaMotion, ROS Hexapod Stack.

- **Rationale:** The primary goal is for the robot to move from point A to point B in a perfect straight line.
- **Mechanism:** The gait planner computes `target_tip = current_tip + stride_vector`. Inverse Kinematics (IK) solves for the angles needed to reach that _exact_ position.
- **Result:** The legs move in an "ugly" (asymmetric) way from a joint perspective, but the body moves "perfectly" in the world.
- **Advantage:** Maximizes odometry precision and avoids lateral slippage.
- **Recommendation:** **This is the correct and current HexaMotion implementation.** No changes required.

### Solution B: Body Orientation Compensation (Active Heading / Crabbing)

**Method:** Actively rotate the robot body to align the motion vector with the legs' most favorable work planes.

- **Rationale:** Legs have greater range of motion and linearity when movement is purely radial (extension/retraction) or purely tangential (coxa sweep), depending on the design.
- **Mechanism:** If the robot must travel a long distance along the X axis, the controller rotates the body (Yaw) to "point" the legs so that the stride vector is optimal for the majority.
    - _Example:_ A circular hexapod sometimes walks more efficiently "sideways" (Crab gait, stride at 90°), where 4 legs (the diagonal ones) work almost purely tangentially.
- **Implementation:** Requires a higher-level planning layer (Path Planning) that decides the chassis orientation ($\psi_{body}$) independently of the velocity vector ($\vec{v}$).

### Solution C: Joint-Space Gait Generation

**Method:** Generate identical cyclic trajectories (sine/cosine/splines) for all joints, ignoring Cartesian linearity.
**Application:** Simple biomimetic robots, toys, highly irregular terrain where precision does not matter.

- **Rationale:** Maximize symmetry and simplicity of low-level control.
- **Mechanism:** `angle(t) = Amplitude * sin(omega * t + phase_offset)`. All legs use the same amplitude.
- **Result:** The legs trace arcs on the ground instead of straight lines. The robot body advances with an oscillating motion ("wobble") and the global trajectory is difficult to predict accurately.
- **Critical Disadvantage:** Incompatible with precise navigation or SLAM systems, as odometry becomes erratic. **Not recommended for HexaMotion.**

### Solution D: Dynamic Stride Limiting

**Method:** Reduce stride length based on the "worst-case" leg.

- **Rationale:** Minimize angular deviation by staying in the linear zone of the inverse kinematics curve (near the center of the range).
- **Mechanism:** Compute the maximum allowable stride such that the angular deviation of the most disadvantaged leg (typically front/rear legs during forward advancement) does not exceed a threshold (e.g., $\Delta\theta < 5^\circ$).
- **Result:** Smoother and visually more symmetric movement, but at the cost of significantly reducing the robot's maximum speed.

### Solution E: Morphology Change (Rectangular vs Hexagonal)

**Method:** Modify the physical layout of the legs.

- **Rationale:** The asymmetry stems from the hexagonal radial arrangement.
- **Mechanism:** A **Rectangular (In-line / Parallel)** configuration aligns all legs with the main direction of travel. In this configuration, a forward stride produces identical joint displacements in all legs.
- **Cost:** The ideal omnidirectionality of the hexagon is lost (the robot becomes "clumsy" at turning or walking sideways).

---

## 4. Conclusion and Recommendation

For the **HexaMotion** project, which seeks functional parity with **OpenSHC** (a high-level controller aimed at research and navigation):

1.  **The "Solution" is Solution A (Cartesian Priority).**
    The observed angular asymmetry is the price to pay for precise Cartesian locomotion. **One should not attempt to "fix" the angles** by forcing them to be equal, as this would break the core functionality of the gait controller (moving the body to the desired target in a straight line).

2.  **Academic Validation:**
    Textbooks such as _Siciliano et al. (Robotics: Modelling, Planning and Control)_ clearly explain that inverse kinematics (IK) absorbs geometric nonlinearities to produce linear Cartesian motion. The observation that "different joint configurations produce the same Cartesian displacement" is a fundamental property, not a bug.

3.  **Recommended Action:**
    Document the behavior as "Expected by Design" and close any bug reports related to "asymmetric angles during straight-line gait".

---

## 5. References

- **Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009).** _Robotics: Modelling, Planning and Control._ Springer. (Chapter 3: Kinematics).
- **Tam, B., Talbot, F., Steindl, R., Elfes, A., & Kottege, N. (2020).** _"OpenSHC: A Versatile Multilegged Robot Controller,"_ IEEE Access.
- **Waldron, K. J.** (1984). _"Configuration design of the adaptive suspension vehicle."_ The International Journal of Robotics Research.
- **Cruse, H.** (1990). _"What mechanisms coordinate leg movement in walking arthropods?"_ Trends in Neurosciences.
