# HexaMotion State Controller Integration Guide

## Overview

The HexaMotion StateController provides a complete OpenSHC-equivalent state machine implementation for comprehensive hexapod robot control. This guide demonstrates how to integrate and use the StateController with your HexaMotion-based hexapod robot.

## Features

The StateController implements all major OpenSHC state management capabilities:

### Hierarchical State Management

- **System States**: SYSTEM_PACKED, SYSTEM_READY, SYSTEM_RUNNING
- **Robot States**: PACKED, READY, RUNNING, UNKNOWN, OFF
- **Walk States**: WALK_STARTING, WALK_MOVING, WALK_STOPPING, WALK_STOPPED
- **Leg States**: WALKING, MANUAL, WALKING_TO_MANUAL, MANUAL_TO_WALKING

### Operational Modes

- **Posing Modes**: X_Y, PITCH_ROLL, Z_YAW, EXTERNAL positioning
- **Pose Reset**: Various axis combinations for pose reset
- **Manual Leg Control**: Individual leg manipulation (up to 2 legs simultaneously)

### Advanced Features

- **Startup/Shutdown Sequences**: Multi-step state transitions
- **Pack/Unpack Sequences**: Safe robot storage and deployment
- **State Transition Validation**: Prevents invalid state changes
- **Progress Tracking**: Real-time transition progress monitoring
- **Error Handling**: Comprehensive error detection and recovery

## Quick Start

### 1. Basic Setup

```cpp
#include <locomotion_system.h>
#include <state_controller.h>
#include <body_pose_config_factory.h>

// Configure your robot parameters
Parameters params;
params.hexagon_radius = 150;
params.coxa_length = 50;
params.femur_length = 100;
params.tibia_length = 120;
params.robot_height = 80;

// Set joint limits (REQUIRED!)
params.coxa_angle_limits[0] = -90;
params.coxa_angle_limits[1] = 90;
params.femur_angle_limits[0] = -90;
params.femur_angle_limits[1] = 90;
params.tibia_angle_limits[0] = -90;
params.tibia_angle_limits[1] = 90;

// Initialize locomotion system
LocomotionSystem locomotion(params);
BodyPoseConfiguration body_pose_config = getDefaultBodyPoseConfig(params);
locomotion.initialize(&imu, &fsr, &servos, body_pose_config);

// Configure state machine
StateMachineConfig config;
config.enable_startup_sequence = true;
config.transition_timeout = 10.0f;

// Create and initialize state controller
StateController state_controller(locomotion, config);
state_controller.initialize(body_pose_config);
```

### 2. Main Control Loop

```cpp
void loop() {
    float dt = 0.02f; // 50Hz control loop

    // Update state machine
    state_controller.update(dt);

    // Update locomotion
    locomotion.update();
}
```

## Core State Management

### Robot State Transitions

```cpp
// Basic state progression
state_controller.requestRobotState(ROBOT_READY);    // Unpack and prepare
state_controller.requestRobotState(ROBOT_RUNNING);  // Enable full operation
state_controller.requestRobotState(ROBOT_PACKED);   // Pack for storage
```

### Checking State Transition Progress

```cpp
if (state_controller.isTransitioning()) {
    Serial.println(state_controller.getDiagnosticInfo());
    if (state_controller.hasErrors()) {
        Serial.println("Error: " + state_controller.getLastErrorMessage());
    }
}
```

## Locomotion Control

### Basic Movement

```cpp
if (state_controller.isReadyForOperation()) {
    // Set linear and angular velocity
    Eigen::Vector2d linear_vel(30.0, 10.0);  // x=30mm/s, y=10mm/s
    double angular_vel = 15.0;                // 15°/s rotation

    state_controller.setDesiredVelocity(linear_vel, angular_vel);
}
```

> **Note:** HexaMotion does not implement cruise control. Per `AGENTS.md`, constant-velocity / cruise behavior is the responsibility of external software, which should maintain and re-issue velocity commands through the API.

### Gait Control

```cpp
// Change gait type
state_controller.changeGait(WAVE_GAIT);    // Switch to wave gait
state_controller.changeGait(TRIPOD_GAIT);  // Switch to tripod gait
state_controller.changeGait(RIPPLE_GAIT);  // Switch to ripple gait
```

## Body Pose Control

### Manual Posing

```cpp
// Enable specific posing mode
state_controller.setPosingMode(POSING_X_Y);        // X-Y translation
state_controller.setPosingMode(POSING_PITCH_ROLL); // Pitch-roll rotation
state_controller.setPosingMode(POSING_Z_YAW);      // Z translation + yaw

// Set desired pose
Eigen::Vector3d position(10.0, 5.0, -5.0);      // mm
Eigen::Vector3d orientation(5.0, -3.0, 15.0);   // degrees
state_controller.setDesiredPose(position, orientation);

// Reset pose
state_controller.setPoseResetMode(POSE_RESET_ALL);
state_controller.setPosingMode(POSING_NONE);
```

## Advanced Leg Control

### Manual Leg Manipulation

```cpp
// Set individual leg to manual mode
if (state_controller.setLegState(0, LEG_MANUAL)) {
    // Control leg tip velocity
    Eigen::Vector3d tip_velocity(10.0, 0.0, 5.0); // mm/s
    state_controller.setLegTipVelocity(0, tip_velocity);

    // Return to walking mode
    state_controller.setLegState(0, LEG_WALKING);
}
```

### Leg State Monitoring

```cpp
// Check individual leg states
for (int i = 0; i < 6; i++) {
    LegState leg_state = state_controller.getLegState(i);
    Serial.println("Leg " + String(i) + " state: " + String(leg_state));
}

// Check manual leg count
int manual_count = state_controller.getManualLegCount();
Serial.println("Manual legs: " + String(manual_count) + "/2");
```

## State Monitoring

### Real-time Status

```cpp
void printSystemStatus() {
    Serial.println("=== System Status ===");
    Serial.println("System: " + String(state_controller.getSystemState()));
    Serial.println("Robot: " + String(state_controller.getRobotState()));
    Serial.println("Walk: " + String(state_controller.getWalkState()));
    Serial.println("Posing: " + String(state_controller.getPosingMode()));
    Serial.println("Ready: " + String(state_controller.isReadyForOperation()));
    Serial.println("Manual Legs: " + String(state_controller.getManualLegCount()));
}
```

### Error Handling

```cpp
// Check for state controller errors
if (state_controller.hasErrors()) {
    String error_msg = state_controller.getLastErrorMessage();
    Serial.println("State Controller Error: " + error_msg);

    // Clear error after handling
    state_controller.clearError();
}

// Emergency stop
if (emergency_condition) {
    state_controller.emergencyStop();
}
```

## Configuration Options

### StateMachineConfig Structure

```cpp
StateMachineConfig config;
config.enable_startup_sequence = true;      // Multi-step startup (false = direct mode)
config.transition_timeout = 10.0f;          // Max transition time (seconds)
config.pack_unpack_time = 2.0f;             // Pack/unpack duration
config.enable_auto_posing = false;          // Automatic posing
config.enable_manual_posing = true;         // Manual posing
config.max_manual_legs = 2;                 // Max simultaneous manual legs
```

## Integration Patterns

### State-Based Control

```cpp
void handleRobotControl() {
    switch (state_controller.getRobotState()) {
        case ROBOT_PACKED:
            // Robot is in storage mode
            handlePackedState();
            break;

        case ROBOT_READY:
            // Robot is ready for operation
            handleReadyState();
            break;

        case ROBOT_RUNNING:
            // Robot is fully operational
            handleRunningState();
            break;

        case ROBOT_UNKNOWN:
            // Determine robot state
            detectRobotState();
            break;
    }
}
```

### Event-Driven Control

```cpp
void onStateTransition(RobotState from_state, RobotState to_state) {
    Serial.println("State transition: " + String(from_state) + " -> " + String(to_state));

    // Handle specific transitions
    if (to_state == ROBOT_RUNNING) {
        // Enable sensors, start walking controller, etc.
        enableFullOperation();
    } else if (to_state == ROBOT_PACKED) {
        // Disable non-essential systems
        enterStorageMode();
    }
}
```

## Best Practices

### 1. State Machine Safety

- Always check `isReadyForOperation()` before commanding movement
- Use transition progress monitoring for time-critical operations
- Implement proper error handling for all state changes

### 2. Timing Considerations

- Call `update()` at consistent intervals (recommended: 50Hz)
- Allow sufficient time for state transitions to complete
- Use transition timeouts to prevent hanging states

### 3. Resource Management

- Limit manual leg control to `StateMachineConfig::max_manual_legs` (default: 2)
- Monitor system resources during complex operations
- Implement graceful degradation for sensor failures

### 4. Integration Testing

- Test all state transitions systematically
- Validate emergency stop functionality
- Verify pose and velocity limits are respected

## Troubleshooting

### Common Issues

1. **State Transitions Fail**
    - Check if transition is valid using `isValidStateTransition()`
    - Ensure locomotion system is initialized and enabled
    - Verify joint limits are properly configured

2. **Manual Leg Control Not Working**
    - Confirm robot is in ROBOT_RUNNING state
    - Check if maximum manual leg count is exceeded
    - Verify leg index is valid (0-5)

3. **Pose Control Issues**
    - Enable appropriate posing mode first
    - Check pose limits in locomotion system configuration
    - Ensure robot is stable before applying poses

### Debug Output

Enable debug logging for detailed state machine information:

```cpp
// Monitor transition state
if (state_controller.isTransitioning()) {
    Serial.println(state_controller.getDiagnosticInfo());
}
```

## Advanced Topics

### Custom State Sequences

- Implement custom startup/shutdown sequences
- Define application-specific state transitions
- Create complex choreographed movements

### Sensor Integration

- Use FSR data for ground contact detection
- Integrate IMU feedback for stability control
- Implement terrain adaptation algorithms

### Performance Optimization

- Tune state machine timing parameters
- Optimize transition sequences for speed
- Balance safety checks with performance requirements

## API Quick Reference

### Core State Control

- `requestSystemState(SystemState)` - Request system state change
- `requestRobotState(RobotState)` - Request robot state change
- `isReadyForOperation()` - Check if robot ready for commands

### Motion Control

- `setDesiredVelocity(linear, angular)` - Set movement velocity
- `setDesiredPose(position, orientation)` - Set body pose

### Leg Control

- `setLegState(leg_index, state)` - Set individual leg state
- `setLegTipVelocity(leg_index, velocity)` - Control leg tip movement
- `getManualLegCount()` - Get number of manual legs

### Status Monitoring

- `getSystemState()`, `getRobotState()`, `getWalkState()` - Get current states
- `isTransitioning()` - Check if state transition in progress
- `getDiagnosticInfo()` - Get formatted diagnostic information
- `hasErrors()`, `getLastErrorMessage()` - Error status and messages

This comprehensive state machine provides all the functionality of OpenSHC's complex state management system while integrating seamlessly with the HexaMotion architecture.
