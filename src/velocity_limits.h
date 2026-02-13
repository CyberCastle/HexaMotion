#ifndef VELOCITY_LIMITS_H
#define VELOCITY_LIMITS_H

#include "robot_model.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

// Forward declaration
struct GaitConfiguration;

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
        double linear_x;      // Maximum linear velocity in X direction (mm/s)
        double linear_y;      // Maximum linear velocity in Y direction (mm/s)
        double angular_z;     // Maximum angular velocity around Z axis (rad/s)
        double acceleration;  // Maximum acceleration (mm/s²)
        double angular_accel; // Maximum angular acceleration (rad/s²) - added for symmetry

        LimitValues() : linear_x(0.0f), linear_y(0.0f), angular_z(0.0f), acceleration(0.0f), angular_accel(0.0f) {}
        LimitValues(double lx, double ly, double az, double acc)
            : linear_x(lx), linear_y(ly), angular_z(az), acceleration(acc), angular_accel(0.0f) {}
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
        double walkspace_radius;        // Effective workspace radius for walking (mm)
        double stance_radius;           // Radius for angular velocity calculations (mm)
        double scaled_walkspace_radius; // Walkspace radius after dynamic overshoot deductions (compat mode)
        double overshoot_x;             // Overshoot compensation in X direction
        double overshoot_y;             // Overshoot compensation in Y direction
        double safety_margin;           // Safety factor for workspace limits
        double reference_height;        // Physical reference height (z = getDefaultHeightOffset())

        WorkspaceConfig() : walkspace_radius(0.0f), stance_radius(0.0f), scaled_walkspace_radius(0.0f),
                            overshoot_x(0.0f), overshoot_y(0.0f), safety_margin(0.85f),
                            reference_height(0.0f) {}
    };

    explicit VelocityLimits(const RobotModel &model);
    ~VelocityLimits(); // Needed for PIMPL idiom

    // Main velocity limiting functions (equivalent to OpenSHC's generateLimits/getLimit)
    void generateLimits(const GaitConfiguration &gait_config);
    LimitValues getLimit(double bearing_degrees) const;

    // Angular acceleration map access (separate from general LimitValues for focused queries)
    double getAngularAcceleration(double bearing_degrees) const; // Interpolated angular acceleration
    std::array<double, 360> getAngularAccelerationMap() const;   // Raw per-bearing angular acceleration

    // Workspace generation and calculation
    void calculateWorkspace(const GaitConfiguration &gait_config);
    const WorkspaceConfig &getWorkspaceConfig() const;

    // Physical robot configuration
    double getPhysicalReferenceHeight() const;

    // Velocity scaling and validation
    LimitValues scaleVelocityLimits(const LimitValues &input_velocities,
                                    double angular_velocity_percentage = 1.0f) const;
    bool validateVelocityInputs(double vx, double vy, double omega) const;

    // Bearing-based interpolation (equivalent to OpenSHC's bearing interpolation)
    LimitValues interpolateLimits(double bearing_degrees) const;

    // Acceleration limiting
    LimitValues applyAccelerationLimits(const LimitValues &target_velocities,
                                        const LimitValues &current_velocities,
                                        double dt) const;

    // Overshoot compensation
    void calculateOvershoot(const GaitConfiguration &gait_config);
    double getOvershootX() const;
    double getOvershootY() const;

    // Configuration and parameter updates
    void updateGaitParameters(const GaitConfiguration &gait_config);
    void setSafetyMargin(double margin);
    void setAngularVelocityScaling(double scaling);

    // Debug and analysis functions
    LimitValues getMaxLimits() const;
    LimitValues getMinLimits() const;
    std::vector<LimitValues> getAllLimits() const;

    // Utility functions
    static double normalizeBearing(double bearing_degrees);
    static double calculateBearing(double vx, double vy);

  private:
    // PIMPL idiom to hide implementation details
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    // Utility functions that don't require PIMPL access
    double interpolateValue(double value1, double value2, double factor) const;
    int getBearingIndex(double bearing_degrees) const;

    // Internal calculation methods
    // Use gait_config.step_length directly (already derived from standing horizontal reach)
    double calculateMaxLinearSpeed(double walkspace_radius, const GaitConfiguration &gait_config) const;
    double calculateMaxAngularSpeed(double max_linear_speed, double stance_radius) const;
    double calculateMaxAcceleration(double max_speed, double time_to_max) const;
    LimitValues calculateLimitsForBearing(double bearing_degrees,
                                          const GaitConfiguration &gait_config) const;
};

#endif // VELOCITY_LIMITS_H
