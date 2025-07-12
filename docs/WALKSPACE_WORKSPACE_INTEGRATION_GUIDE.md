# WalkspaceAnalyzer and WorkspaceValidator Integration Guide

## Overview

This guide describes the OpenSHC-equivalent integration of `WalkspaceAnalyzer` and `WorkspaceValidator` in the `WalkController`, providing runtime control and external access to analysis information while maintaining separate logic for each component.

## Architecture

The integration follows OpenSHC principles with the following key components:

### 1. WalkspaceAnalyzer

-   **Purpose**: Real-time walkspace analysis and stability assessment
-   **Features**:
    -   Runtime enable/disable control
    -   Performance monitoring and statistics
    -   External access to analysis information
    -   Adaptive control based on stability scores

### 2. WorkspaceValidator

-   **Purpose**: Workspace validation and collision avoidance
-   **Features**:
    -   Position validation and constraint
    -   Collision risk assessment
    -   Velocity constraint calculation
    -   Safety margin management

### 3. WalkController Integration

-   **Purpose**: Unified control interface
-   **Features**:
    -   Runtime control of analysis components
    -   Real-time adaptive control
    -   External API for analysis information
    -   Performance monitoring

## Key Features

### Runtime Control

Both `WalkspaceAnalyzer` and `WorkspaceValidator` can be enabled/disabled at runtime:

```cpp
// Enable/disable walkspace analysis
walk_controller->enableWalkspaceAnalysis(true);

// Check if analysis is enabled
bool enabled = walk_controller->isWalkspaceAnalysisEnabled();
```

### External Access to Analysis Information

Comprehensive analysis information is available for external systems:

```cpp
// Get complete analysis information
const WalkspaceAnalyzer::AnalysisInfo& info = walk_controller->getWalkspaceAnalysisInfo();

// Get specific stability metrics
double stability_margin = walk_controller->getStabilityMargin();
double stability_score = walk_controller->getOverallStabilityScore();
bool is_stable = walk_controller->isCurrentlyStable();

// Get leg reachability scores
std::map<int, double> reachability = walk_controller->getLegReachabilityScores();

// Get walkspace map
const std::map<int, double>& walkspace_map = walk_controller->getCurrentWalkspaceMap();
```

### Performance Monitoring

Built-in performance tracking and statistics:

```cpp
// Get analysis statistics
const WalkspaceAnalyzer::AnalysisInfo& info = walk_controller->getWalkspaceAnalysisInfo();
Serial.print("Analysis count: "); Serial.println(info.analysis_count);
Serial.print("Average time: "); Serial.print(info.average_analysis_time_ms); Serial.println(" ms");
Serial.print("Total time: "); Serial.print(info.total_analysis_time_ms); Serial.println(" ms");

// Reset statistics
walk_controller->resetWalkspaceAnalysisStats();
```

### Real-time Adaptive Control

The system automatically adapts control based on stability analysis:

```cpp
// The system automatically:
// - Reduces velocity when stability is compromised
// - Boosts velocity when stability is high
// - Adjusts step parameters based on terrain
// - Prevents collisions between legs

// Manual velocity commands are automatically constrained
Point3D velocity(50.0, 20.0, 0.0);
double angular_velocity = 10.0;
walk_controller->updateWalk(velocity, angular_velocity);
```

## API Reference

### WalkspaceAnalyzer Control Methods

#### `enableWalkspaceAnalysis(bool enabled)`

Enable or disable real-time walkspace analysis.

**Parameters:**

-   `enabled`: True to enable analysis, false to disable

**Example:**

```cpp
walk_controller->enableWalkspaceAnalysis(true);
```

#### `isWalkspaceAnalysisEnabled()`

Check if walkspace analysis is currently enabled.

**Returns:** True if analysis is enabled

**Example:**

```cpp
if (walk_controller->isWalkspaceAnalysisEnabled()) {
    // Analysis is active
}
```

#### `getWalkspaceAnalysisInfo()`

Get comprehensive analysis information.

**Returns:** Reference to `WalkspaceAnalyzer::AnalysisInfo` structure

**Example:**

```cpp
const auto& info = walk_controller->getWalkspaceAnalysisInfo();
Serial.print("Stability score: "); Serial.println(info.overall_stability_score);
```

#### `getWalkspaceAnalysisInfoString()`

Get analysis information as a formatted string.

**Returns:** Formatted string with analysis information

**Example:**

```cpp
String analysis_string = walk_controller->getWalkspaceAnalysisInfoString();
Serial.println(analysis_string);
```

#### `resetWalkspaceAnalysisStats()`

Reset analysis statistics and performance counters.

**Example:**

```cpp
walk_controller->resetWalkspaceAnalysisStats();
```

### Analysis Information Access Methods

#### `getStabilityMargin()`

Get current stability margin in millimeters.

**Returns:** Stability margin value

**Example:**

```cpp
double margin = walk_controller->getStabilityMargin();
Serial.print("Stability margin: "); Serial.print(margin); Serial.println(" mm");
```

#### `getOverallStabilityScore()`

Get overall stability score (0.0 to 1.0).

**Returns:** Stability score between 0.0 and 1.0

**Example:**

```cpp
double score = walk_controller->getOverallStabilityScore();
Serial.print("Stability score: "); Serial.println(score, 3);
```

#### `isCurrentlyStable()`

Check if the robot is currently in a stable configuration.

**Returns:** True if stable, false otherwise

**Example:**

```cpp
if (walk_controller->isCurrentlyStable()) {
    Serial.println("Robot is stable");
} else {
    Serial.println("Robot is unstable");
}
```

#### `getLegReachabilityScores()`

Get reachability scores for all legs.

**Returns:** Map of leg index to reachability score (0.0 to 1.0)

**Example:**

```cpp
auto reachability = walk_controller->getLegReachabilityScores();
for (const auto& leg_score : reachability) {
    Serial.print("Leg "); Serial.print(leg_score.first);
    Serial.print(": "); Serial.println(leg_score.second, 3);
}
```

### Walkspace Map Methods

#### `generateWalkspaceMap()`

Generate a new walkspace map.

**Returns:** True if successful, false otherwise

**Example:**

```cpp
if (walk_controller->generateWalkspaceMap()) {
    Serial.println("Walkspace map generated successfully");
}
```

#### `isWalkspaceMapGenerated()`

Check if walkspace map has been generated.

**Returns:** True if map is available

**Example:**

```cpp
if (walk_controller->isWalkspaceMapGenerated()) {
    // Use walkspace map
}
```

#### `getCurrentWalkspaceMap()`

Get the current walkspace map.

**Returns:** Reference to map of bearing to radius values

**Example:**

```cpp
const auto& walkspace_map = walk_controller->getCurrentWalkspaceMap();
for (const auto& entry : walkspace_map) {
    Serial.print(entry.first); Serial.print("°: ");
    Serial.print(entry.second); Serial.println(" mm");
}
```

#### `getWalkspaceRadius(double bearing_degrees)`

Get walkspace radius for a specific bearing.

**Parameters:**

-   `bearing_degrees`: Bearing angle in degrees (0-360)

**Returns:** Maximum walkspace radius at bearing

**Example:**

```cpp
double radius_forward = walk_controller->getWalkspaceRadius(0.0);
double radius_right = walk_controller->getWalkspaceRadius(90.0);
Serial.print("Forward radius: "); Serial.println(radius_forward);
Serial.print("Right radius: "); Serial.println(radius_right);
```

## Analysis Information Structure

The `WalkspaceAnalyzer::AnalysisInfo` structure contains comprehensive analysis data:

```cpp
struct AnalysisInfo {
    WalkspaceResult current_result;        // Current analysis result
    std::map<int, WorkspaceBounds> leg_bounds; // Workspace bounds per leg
    std::map<int, double> leg_reachability;    // Reachability scores per leg
    double overall_stability_score;            // Overall stability score (0-1)
    bool analysis_enabled;                     // Analysis enabled flag
    unsigned long last_analysis_time;          // Last analysis timestamp
    int analysis_count;                        // Number of analyses performed
    double average_analysis_time_ms;           // Average analysis time
    double total_analysis_time_ms;             // Total analysis time
    double min_analysis_time_ms;               // Minimum analysis time
    double max_analysis_time_ms;               // Maximum analysis time
    std::map<int, double> walkspace_radii;     // Current walkspace radii
    bool walkspace_map_generated;              // Map generation status
};
```

## Usage Examples

### Basic Integration Example

```cpp
#include "walk_controller.h"

// Initialize walk controller
RobotModel model;
model.initialize(params);
WalkController walk_controller(model);
walk_controller.init();

// Enable walkspace analysis
walk_controller.enableWalkspaceAnalysis(true);

// Generate initial walkspace map
walk_controller.generateWalkspaceMap();

// Main control loop
void loop() {
    // Update walking with velocity commands
    Point3D velocity(30.0, 10.0, 0.0);
    double angular_velocity = 5.0;
    walk_controller.updateWalk(velocity, angular_velocity);

    // Check stability
    if (!walk_controller.isCurrentlyStable()) {
        Serial.println("Warning: Robot unstable!");
    }

    // Print analysis info periodically
    static unsigned long last_print = 0;
    if (millis() - last_print > 2000) {
        String analysis_string = walk_controller.getWalkspaceAnalysisInfoString();
        Serial.println(analysis_string);
        last_print = millis();
    }
}
```

### Advanced Control Example

```cpp
// Advanced control with stability-based adaptation
void advancedControl() {
    // Get current stability information
    double stability_score = walk_controller.getOverallStabilityScore();
    double stability_margin = walk_controller.getStabilityMargin();

    // Adaptive velocity control
    double base_velocity = 50.0;
    double adaptive_velocity = base_velocity;

    if (stability_score < 0.5) {
        // Reduce velocity when stability is low
        adaptive_velocity *= stability_score;
    } else if (stability_score > 0.8) {
        // Boost velocity when stability is high
        adaptive_velocity *= (1.0 + (stability_score - 0.8) * 0.5);
    }

    // Apply velocity constraints
    adaptive_velocity = std::min(adaptive_velocity, 100.0);

    // Update walking
    Point3D velocity(adaptive_velocity, 0.0, 0.0);
    walk_controller.updateWalk(velocity, 0.0);

    // Monitor leg reachability
    auto reachability = walk_controller.getLegReachabilityScores();
    for (const auto& leg_score : reachability) {
        if (leg_score.second < 0.3) {
            Serial.print("Warning: Leg "); Serial.print(leg_score.first);
            Serial.println(" has low reachability");
        }
    }
}
```

### Performance Monitoring Example

```cpp
// Performance monitoring and optimization
void monitorPerformance() {
    const auto& analysis_info = walk_controller.getWalkspaceAnalysisInfo();

    // Check analysis performance
    if (analysis_info.average_analysis_time_ms > 10.0) {
        Serial.println("Warning: Analysis taking too long");
        // Consider reducing analysis frequency or precision
    }

    // Print performance statistics
    Serial.print("Analysis count: "); Serial.println(analysis_info.analysis_count);
    Serial.print("Average time: "); Serial.print(analysis_info.average_analysis_time_ms, 2);
    Serial.println(" ms");
    Serial.print("Min/Max time: "); Serial.print(analysis_info.min_analysis_time_ms, 2);
    Serial.print("/"); Serial.print(analysis_info.max_analysis_time_ms, 2);
    Serial.println(" ms");

    // Reset statistics periodically
    if (analysis_info.analysis_count > 1000) {
        walk_controller.resetWalkspaceAnalysisStats();
        Serial.println("Analysis statistics reset");
    }
}
```

## Configuration

### WalkspaceAnalyzer Configuration

The `WalkspaceAnalyzer` can be configured with different precision levels:

```cpp
// Low precision (fast, less accurate)
ComputeConfig low_config = ComputeConfig::low();

// Medium precision (balanced)
ComputeConfig medium_config = ComputeConfig::medium();

// High precision (slow, very accurate)
ComputeConfig high_config = ComputeConfig::high();

// Apply configuration
walkspace_analyzer->setPrecisionConfig(medium_config);
```

### WorkspaceValidator Configuration

The `WorkspaceValidator` can be configured with safety parameters:

```cpp
ValidationConfig config;
config.safety_margin = 20.0f;           // Safety margin (mm)
config.enable_collision_checking = true;
config.enable_joint_limit_checking = true;
config.angular_velocity_scaling = 0.8f;

workspace_validator->updateConfig(config);
```

## Testing

The integration includes comprehensive tests:

```bash
# Run the integration test
cd tests
make walkspace_workspace_integration_test
./walkspace_workspace_integration_test

# Run all tests
make all
```

## Performance Considerations

### Analysis Frequency

-   **Real-time control**: 20-50 Hz recommended
-   **Monitoring only**: 1-5 Hz sufficient
-   **Debugging**: 0.1-1 Hz acceptable

### Memory Usage

-   **WalkspaceAnalyzer**: ~2-5 KB for analysis data
-   **WorkspaceValidator**: ~1-2 KB for validation data
-   **Total integration**: ~5-10 KB additional memory

### Computational Cost

-   **Low precision**: ~0.1-0.5 ms per analysis
-   **Medium precision**: ~0.5-2.0 ms per analysis
-   **High precision**: ~2.0-10.0 ms per analysis

## Troubleshooting

### Common Issues

1. **Analysis taking too long**

    - Reduce precision level
    - Disable analysis during critical operations
    - Increase analysis interval

2. **Memory issues**

    - Reset analysis statistics periodically
    - Use lower precision configuration
    - Limit walkspace map resolution

3. **Stability false positives**

    - Adjust stability threshold
    - Check sensor calibration
    - Verify robot parameters

4. **Performance degradation**
    - Monitor analysis times
    - Reset statistics when needed
    - Optimize analysis frequency

### Debug Information

Enable debug output to troubleshoot issues:

```cpp
// Get detailed analysis information
String debug_info = walk_controller->getWalkspaceAnalysisInfoString();
Serial.println(debug_info);

// Check individual component status
Serial.print("Analysis enabled: "); Serial.println(walk_controller->isWalkspaceAnalysisEnabled());
Serial.print("Map generated: "); Serial.println(walk_controller->isWalkspaceMapGenerated());
Serial.print("Currently stable: "); Serial.println(walk_controller->isCurrentlyStable());
```

## Conclusion

The WalkspaceAnalyzer and WorkspaceValidator integration provides OpenSHC-equivalent functionality with:

-   **Runtime control** of analysis components
-   **External access** to comprehensive analysis information
-   **Real-time adaptive control** based on stability assessment
-   **Performance monitoring** and optimization
-   **Separate logic** maintained for each component

This integration enables advanced hexapod control with real-time stability analysis and adaptive behavior while maintaining clean separation of concerns between workspace analysis and validation components.
