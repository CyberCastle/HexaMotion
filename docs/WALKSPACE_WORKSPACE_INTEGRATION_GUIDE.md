# WorkspaceAnalyzer Integration Guide

## Overview

HexaMotion consolidates OpenSHC-style walkspace generation and workspace validation into a single class: `WorkspaceAnalyzer`.

- OpenSHC concept split: walkspace generation + workspace validation
- HexaMotion equivalent: `WorkspaceAnalyzer` (unified API)

This document reflects the current API surface used by `WalkController`, `VelocityLimits`, and `LocomotionSystem`.

## OpenSHC Parity Model

- `generateWorkspace()` provides bearing-based walkspace limits used by gait/velocity logic.
- `getWorkplane(leg, height)` provides interpolated 2D workplanes by height.
- Reachability/constraint checks (`isPositionReachable`, `constrainToValidWorkspace`) replace ad-hoc external checks.
- Validation config (`ValidationConfig`) centralizes collision/joint-limit safety behavior.

## Access Pattern

```cpp
RobotModel &model = locomotion_system.getRobotModel();
WorkspaceAnalyzer &analyzer = model.getWorkspaceAnalyzer();

analyzer.generateWorkspace();
```

## Core APIs (Current)

### Workspace generation and maps

```cpp
analyzer.generateWorkspace();
const std::map<int, double> &walkspace = analyzer.getWalkspaceMap();
```

### Workplanes and bounds

```cpp
Workplane wp = analyzer.getWorkplane(0, 0.0); // leg 0, reference height
WorkspaceBounds bounds = analyzer.getWorkspaceBounds(0);
```

### Reachability and constraints

```cpp
Point3D current_leg_positions[NUM_LEGS] = {};
Point3D target(280.0, -120.0, -150.0);

bool reachable = analyzer.isPositionReachable(0, target, true);
Point3D constrained = analyzer.constrainToValidWorkspace(0, target, current_leg_positions);
ValidationResult vr = analyzer.validateTarget(0, target, current_leg_positions, true);
```

### Velocity constraints

```cpp
VelocityConstraints c = analyzer.calculateVelocityConstraints(
    0,      // leg index
    45.0,   // bearing degrees
    1.0,    // gait frequency (Hz)
    0.6);   // stance ratio
```

### Runtime analysis controls

```cpp
analyzer.enableAnalysis(true);
bool enabled = analyzer.isAnalysisEnabled();

const WorkspaceAnalyzer::AnalysisInfo &info = analyzer.getAnalysisInfo();
std::string info_text = analyzer.getAnalysisInfoString();
analyzer.resetAnalysisStats();
```

## WalkController Relationship

`WalkController` does not expose old `WalkspaceAnalyzer` helper methods. Instead:

- `WalkController::generateWalkspace()` pulls data from `WorkspaceAnalyzer`.
- `WalkController` velocity limiting uses `VelocityLimits`, which internally relies on analyzer-derived constraints.
- `WalkController::updateWalk(...)` requires body pose inputs:

```cpp
walk_controller.updateWalk(
    Point3D(vx, vy, 0.0),
    omega,
    current_body_position,
    current_body_orientation);
```

## ValidationConfig (Current Fields)

```cpp
ValidationConfig cfg;
cfg.enable_collision_checking = true;
cfg.enable_joint_limit_checking = true;
cfg.enable_terrain_adaptation = true;
cfg.safety_margin = 20.0;
cfg.angular_velocity_scaling = 0.8;
cfg.max_velocity_scale = 1.2;
cfg.workspace_margin_factor = 0.95;
cfg.collision_safety_margin = 25.0;
cfg.safety_margin_factor = 0.9;
cfg.minimum_reach_factor = 0.1;

analyzer.updateConfig(cfg);
```

## Minimal End-to-End Example

```cpp
RobotModel &model = locomotion_system.getRobotModel();
WorkspaceAnalyzer &analyzer = model.getWorkspaceAnalyzer();

analyzer.enableAnalysis(true);
analyzer.generateWorkspace();

Point3D tips[NUM_LEGS] = {};
WorkspaceAnalyzer::WalkspaceResult result = analyzer.analyzeWalkspace(tips);

if (!result.is_stable) {
    // Reduce command velocity externally before sending to planGaitSequence
}
```

## Notes

- This guide intentionally avoids legacy methods that no longer exist in `WalkController`.
- For OpenSHC parity discussions, treat `WorkspaceAnalyzer` as the maintained equivalent of older split components.
