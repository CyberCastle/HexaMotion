#ifndef VELOCITY_LIMITS_H
#define VELOCITY_LIMITS_H

#include "HexaModel.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

/**
 * Dynamic velocity limits system equivalent to OpenSHC's velocity limiting mechanism.
 * This system dynamically calculates maximum linear and angular velocities based on:
 * - Workspace constraints and geometry
 * - Gait parameters (frequency, stance ratio)
 * - Robot kinematics and leg positioning
 * - Bearing-based directional limits
 * - Acceleration constraints
 */
class VelocityLimits {
  public:
    /**
     * @brief Velocity limits for a specific bearing.
     */
    struct LimitValues {
        float linear_x;     // Maximum linear velocity in X direction (m/s)
        float linear_y;     // Maximum linear velocity in Y direction (m/s)
        float angular_z;    // Maximum angular velocity around Z axis (rad/s)
        float acceleration; // Maximum acceleration (m/s²)

        LimitValues() : linear_x(0.0f), linear_y(0.0f), angular_z(0.0f), acceleration(0.0f) {}
        LimitValues(float lx, float ly, float az, float acc)
            : linear_x(lx), linear_y(ly), angular_z(az), acceleration(acc) {}
    };

    /**
     * @brief Map of velocity limits for 0-359 degree bearings.
     */
    struct LimitMap {
        std::array<LimitValues, 360> limits; // Limits for each degree (0-359)

        LimitMap() {
            // Initialize all limits to zero
            for (auto &limit : limits) {
                limit = LimitValues();
            }
        }
    };

    /**
     * @brief Workspace parameters used for limit generation.
     */
    struct WorkspaceConfig {
        float walkspace_radius; // Effective workspace radius for walking
        float stance_radius;    // Radius for angular velocity calculations
        float overshoot_x;      // Overshoot compensation in X direction
        float overshoot_y;      // Overshoot compensation in Y direction
        float safety_margin;    // Safety factor for workspace limits

        WorkspaceConfig() : walkspace_radius(0.0f), stance_radius(0.0f),
                            overshoot_x(0.0f), overshoot_y(0.0f), safety_margin(0.85f) {}
    };

    /**
     * @brief Gait parameters affecting limit calculations.
     */
    struct GaitConfig {
        float frequency;          // Step frequency (Hz)
        float stance_ratio;       // Ratio of stance phase (0.0 - 1.0)
        float swing_ratio;        // Ratio of swing phase (0.0 - 1.0)
        float time_to_max_stride; // Time to reach maximum stride (s)

        GaitConfig() : frequency(1.0f), stance_ratio(0.6f), swing_ratio(0.4f),
                       time_to_max_stride(2.0f) {}
    };

    explicit VelocityLimits(const RobotModel &model);
    ~VelocityLimits(); // Needed for PIMPL idiom

    // Main velocity limiting functions (equivalent to OpenSHC's generateLimits/getLimit)
    void generateLimits(const GaitConfig &gait_config);
    LimitValues getLimit(float bearing_degrees) const;

    // Workspace generation and calculation
    void calculateWorkspace(const GaitConfig &gait_config);
    const WorkspaceConfig &getWorkspaceConfig() const;

    // Velocity scaling and validation
    LimitValues scaleVelocityLimits(const LimitValues &input_velocities,
                                    float angular_velocity_percentage = 1.0f) const;
    bool validateVelocityInputs(float vx, float vy, float omega) const;

    // Bearing-based interpolation (equivalent to OpenSHC's bearing interpolation)
    LimitValues interpolateLimits(float bearing_degrees) const;

    // Acceleration limiting
    LimitValues applyAccelerationLimits(const LimitValues &target_velocities,
                                        const LimitValues &current_velocities,
                                        float dt) const;

    // Overshoot compensation
    void calculateOvershoot(const GaitConfig &gait_config);
    float getOvershootX() const;
    float getOvershootY() const;

    // Configuration and parameter updates
    void updateGaitParameters(const GaitConfig &gait_config);
    void setSafetyMargin(float margin);
    void setAngularVelocityScaling(float scaling);

    // Debug and analysis functions
    LimitValues getMaxLimits() const;
    LimitValues getMinLimits() const;
    std::vector<LimitValues> getAllLimits() const;

    // Utility functions
    static float normalizeBearing(float bearing_degrees);
    static float calculateBearing(float vx, float vy);

  private:
    // PIMPL idiom to hide WorkspaceValidator dependency
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    // Utility functions that don't require PIMPL access
    float interpolateValue(float value1, float value2, float factor) const;
    int getBearingIndex(float bearing_degrees) const;

    // Internal calculation methods (now implemented via WorkspaceValidator)
    float calculateMaxLinearSpeed(float walkspace_radius, float on_ground_ratio,
                                  float frequency) const;
    float calculateMaxAngularSpeed(float max_linear_speed, float stance_radius) const;
    float calculateMaxAcceleration(float max_speed, float time_to_max) const;
    LimitValues calculateLimitsForBearing(float bearing_degrees,
                                          const GaitConfig &gait_config) const;
};

#endif // VELOCITY_LIMITS_H
