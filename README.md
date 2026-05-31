# HexaMotion

**HexaMotion** is a near-complete 1:1 port of [OpenSHC (Syropod High-level Controller)](https://github.com/csiro-robotics/syropod_highlevel_controller) — CSIRO's versatile multilegged robot controller — rewritten **without ROS** to run on MCU targets such as the STM32H7 (Arduino Giga R1).

OpenSHC is a powerful ROS-based C++ controller for quasi-static multilegged robots, developed by [CSIRO's Robotics and Autonomous Systems Group](https://research.csiro.au/robotics/). HexaMotion brings that same locomotion logic to embedded hardware, replacing the ROS transport and configuration layer with a direct C++ API and a `Parameters` structure, while preserving the core algorithms and control flow as faithfully as possible.

> **AI-assisted development:** The porting of OpenSHC's logic to HexaMotion was done almost entirely with AI, using **GitHub Copilot** powered by the **GPT** and **Claude Opus** large language models, and to a lesser extent **Gemini**. The AI agents handled code translation, architecture adaptation, test generation and iterative debugging throughout the project.

## Overview

HexaMotion targets hexapod robots with a **hexagonal body**, **six legs** spaced 60° apart, and **three joints (3DOF) per leg**. It includes:

- Inverse kinematics using DH parameters and Jacobians.
- Body pose and orientation control via IMU feedback.
- Multiple gait planners: tripod, wave, ripple and metachronal.
- Terrain adaptation and inclination compensation.
- Admittance control and cartesian velocity control.
- FSR (Force Sensitive Resistor) input for contact detection.
- Smart servo interface for precise joint control.
- Error reporting, diagnostics and self-tests.

The library relies on [ArduinoEigen](https://github.com/arduino-libraries/ArduinoEigen) for matrix math and is designed for use with the **Arduino Giga R1**.

## Relationship with OpenSHC

HexaMotion is a **port**, not a fork. The original OpenSHC codebase (included in the `OpenSHC/` directory for reference) was systematically translated into a standalone C++ library. The core state machine, gait planning, body pose control, walk controller sequencing and inverse kinematics logic are preserved 1:1 from OpenSHC.

### Key differences and limitations

| Area                   | OpenSHC                                        | HexaMotion                                                    |
| ---------------------- | ---------------------------------------------- | ------------------------------------------------------------- |
| **Leg count**          | Configurable (any)                             | Fixed to 6 legs                                               |
| **DOF per leg**        | Configurable                                   | Fixed to 3DOF                                                 |
| **ROS**                | Required                                       | Not supported — replaced by direct API                        |
| **Configuration**      | YAML files + dynamic parameters                | `Parameters` struct + explicit setter APIs                    |
| **Workspace strategy** | Full model copy for isolation                  | Decoupled `WorkspaceAnalyzer` over live model (MCU-optimized) |
| **Gravity estimation** | Orientation-based (`Model::estimateGravity()`) | Accelerometer-based (simpler, noisier under dynamic accel.)   |

**Intentionally not ported:**

- **Tip orientation control** (`updateTipRotation`, gravity-aligned tips, rotation-constrained IK retries) — not applicable with 3DOF legs where tip orientation is not an independent task variable.
- **AMBLE_GAIT** — not supported with current morphology/constraints.
- **Planner mode** — equivalent planning should be implemented externally via `LocomotionSystem` API.
- **Cruise control** — equivalent cruise behavior should be implemented externally.
- **`ExternalTarget`** — externally-driven tip targets should use the `LocomotionSystem` API.
- **`velocity_input_mode`** — throttle-vs-real input should be handled externally by pre-scaling commands.
- **`ignore_IK_warnings`** — suppressing IK warnings would interfere with HexaMotion's diagnostic flow.
- **Runtime/dynamic parameter adjustment** (`ParameterSelection`, `adjustParameter()`) — replaced by explicit setter APIs (`setStepFrequency`, `setSwingHeight`, etc.) and/or direct `Parameters` updates before runtime.

**Architectural changes:**

- `LocomotionSystem` acts as a ROS-less facade around `StateController`, replacing OpenSHC's external ROS graph/script role. It routes external inputs, runs the control pipeline (sensors → walk update → IK → servo output) and exposes high-level convenience methods (forward, backward, turn, stop).
- OpenSHC logic is split into more focused classes for readability and maintainability.
- Class/data naming follows a semantic, self-documenting pattern — some names differ from OpenSHC while keeping 1:1 logic.
- Configurations use factory patterns where appropriate.

### Parity & deliberate divergences

HexaMotion targets a near-complete 1:1 reproduction of OpenSHC. A master switch
`Parameters::strict_openshc_parity` (default **false**) disables every HexaMotion-only stability
extension in `LegStepper` (stride freezing, hybrid anti-drift, lateral residual cleanup, phase-end
snap, in-gait workspace constraining and swing-end Z snapping) to obtain OpenSHC-verbatim
trajectories. It has the highest precedence over the fine-grained extension flags.

One intentional divergence is retained for MCU effectiveness: inverse kinematics uses a bounded
internal DLS convergence loop (≤30 iterations, 1 mm early-exit) with an analytic seed and a
5°/iteration clamp, instead of OpenSHC's single DLS step per control cycle. It converges within one
~50 Hz MCU cycle and the analytic seed lowers the average iteration count. Rough-terrain swing-target
adaptation (proactive step-plane projection and reactive step-depth probing) lives in
`TerrainAdaptation` rather than inline in `LegStepper`, keeping HexaMotion's class decomposition. The
iterative timing rounding, dual manual-leg toggle, all-legs walk-plane fit and `forceNormalTouchdown`
node-separation formula are aligned 1:1 with OpenSHC.

## Prerequisites

- Arduino IDE with board support for **Arduino Giga R1**.
- Install the **ArduinoEigen** library using the Library Manager or by copying it into your `libraries` folder.

## Including the library

Place this repository inside your Arduino `libraries` directory. In your sketch include the unified header:

```cpp
#include <HexaMotion.h>
```

## Basic usage

Create hardware interface classes that implement `IIMUInterface`, `IFSRInterface` and `IServoInterface`. Then pass instances of these classes to `LocomotionSystem` together with your robot parameters.

**Note**: HexaMotion supports advanced IMUs like the BNO055 with enhanced terrain adaptation and locomotion capabilities. The system automatically detects and leverages absolute positioning data while maintaining backward compatibility with basic IMUs.

```cpp
#include <Arduino.h>
#include <HexaMotion.h>

class MyIMU : public IIMUInterface {
    // Implement basic methods and optionally support absolute positioning
    // For BNO055-style IMUs, implement hasAbsolutePositioning() to return true
    /* ... */
};
class MyFSR : public IFSRInterface {
    // Implement all required methods including the new update() method
    // that uses AdvancedAnalog DMA for simultaneous FSR readings
    bool update() override { /* Update internal ADC registers */ }
    /* ... other methods ... */
};
class MyServo : public IServoInterface { /* ... */ };

Parameters params;            // fill your robot parameters
params.coxa_angle_limits[0] = -90;
params.coxa_angle_limits[1] = 90;
params.femur_angle_limits[0] = -90;
params.femur_angle_limits[1] = 90;
params.tibia_angle_limits[0] = -90;
params.tibia_angle_limits[1] = 90;
LocomotionSystem robot(params);
MyIMU imu;
MyFSR fsr;
MyServo servos;

void setup() {
    // Create a body pose configuration (factory function)
    BodyPoseConfiguration pose_cfg = getDefaultBodyPoseConfig(params);

    // Initialize hardware + controllers
    robot.initialize(&imu, &fsr, &servos, pose_cfg);

    // Move to calibrated standing pose
    robot.setStandingPose();

    // Select gait (TRIPOD_GAIT, WAVE_GAIT, RIPPLE_GAIT, METACHRONAL_GAIT)
    robot.selectGait(TRIPOD_GAIT);

    // Start continuous forward motion (mm/s), or use turnInPlace / walkSideways
    robot.walkForward(30.0);
    // robot.turnInPlace(0.3);      // alternative rotation (rad/s)
}

void loop() {
    // Run control loop (~50 Hz). Consider using a timing guard for real hardware.
    robot.update();
}
```

Make sure the joint limit arrays (`coxa_angle_limits`, `femur_angle_limits` and
`tibia_angle_limits`) are populated with valid ranges before creating the
`LocomotionSystem` instance. If these values remain at their defaults the system
will flag `KINEMATICS_ERROR` and skip sending servo commands.

Debug logging can be enabled by defining the `DEBUG_LOGGING` macro before
including the library. When active, certain events such as state transitions
will be printed to the serial console.

## Configurable parameters

The `Parameters` structure (defined in `src/robot_model.h`) defines the physical dimensions and control limits of the robot. Key fields include:

- `hexagon_radius`, `coxa_length`, `femur_length`, `tibia_length`.
- `robot_height`, `standing_height`, `default_height_offset`.
- Joint angle limits for coxa, femur and tibia.
- FSR contact thresholds (`fsr_touchdown_threshold`, `fsr_liftoff_threshold`, `fsr_min_pressure`).
- Gait tuning (`step_frequency`, `body_velocity_scaler`), control frequency (`time_delta`) and servo speed (`default_servo_speed`).
- IK solver settings (`ik`), body compensation (`body_comp`), admittance control (`admittance`) and workspace tuning (`workspace_tuning`).

## Running tests

The test suite depends on the Eigen library. A helper script in the
`tests/` directory installs this dependency automatically on both Linux and
macOS. Ensure you have **Homebrew** installed when running on macOS.
The accompanying `Makefile` will detect the Eigen installation path on
macOS using `brew --prefix eigen`, so no manual configuration is required.
After executing the script, build the tests using `make`:

```bash
cd tests
./setup.sh
make
```

Each test binary can be executed individually once the build completes.

## IMU Integration

Advanced IMUs (e.g. BNO055) automatically enable: absolute orientation usage, improved terrain adaptation, gravity-free linear acceleration, quaternion-based terrain analysis and multi-factor stability estimation. When an IMU does not provide absolute positioning, the system gracefully falls back to the baseline algorithms without code changes required. Implement `hasAbsolutePositioning()` accordingly in your `IIMUInterface` implementation.

## References

- **OpenSHC repository:** [csiro-robotics/syropod_highlevel_controller](https://github.com/csiro-robotics/syropod_highlevel_controller)
- **OpenSHC paper:** B. Tam, F. Talbot, R. Steindl, A. Elfes and N. Kottege, "OpenSHC: A Versatile Multilegged Robot Controller," in _IEEE Access_, vol. 8, pp. 188908-188926, 2020, doi: [10.1109/ACCESS.2020.3031019](https://doi.org/10.1109/ACCESS.2020.3031019).
- **OpenSHC tutorials:** [csiro-robotics/shc_tutorials](https://github.com/csiro-robotics/shc_tutorials)

## License

See [LICENSE](LICENSE).
