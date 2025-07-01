#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H

#include "HexaModel.h"
#include "hexamotion_constants.h"
#include <algorithm>
#include "velocity_limits.h"
#include <array>
#include <memory>

// Forward declaration to avoid circular dependency
class WorkspaceValidator;

/**
 * @brief Cartesian velocity controller equivalent to OpenSHC's velocity control system.
 *
 * This controller maps Cartesian velocity commands (linear and angular) to servo speeds,
 * allowing servo velocity adjustments to directly influence Cartesian motion velocity.
 * The system dynamically calculates appropriate servo speeds based on:
 * - Commanded linear velocity (vx, vy)
 * - Commanded angular velocity (omega)
 * - Current gait parameters
 * - Joint workspace constraints
 * - Individual leg kinematics
 */
class CartesianVelocityController {
  public:
    /**
     * @brief Servo speed configuration for all joints.
     */
    struct ServoSpeedConfig {
        double base_speed = SERVO_SPEED_DEFAULT;           // Base servo speed multiplier (0.1-3.0)
        double velocity_scaling = SERVO_SPEED_DEFAULT;     // Velocity-dependent scaling factor
        double angular_compensation = SERVO_SPEED_DEFAULT; // Angular velocity compensation
        double gait_adjustment = SERVO_SPEED_DEFAULT;      // Gait-specific adjustment

        /**
         * @brief Calculate final servo speed from all factors.
         * @return Computed servo speed (clamped to valid range)
         */
        double getEffectiveSpeed() const {
            double speed = base_speed * velocity_scaling * angular_compensation * gait_adjustment;
            return std::clamp<double>(speed, SERVO_SPEED_MIN, SERVO_SPEED_MAX);
        }
    };

    /**
     * @brief Per-leg servo speed configuration.
     */
    struct LegServoSpeeds {
        ServoSpeedConfig coxa;
        ServoSpeedConfig femur;
        ServoSpeedConfig tibia;
    };

    /**
     * @brief Velocity scaling parameters.
     */
    struct VelocityScaling {
        double linear_velocity_scale = DEFAULT_LINEAR_VELOCITY_SCALE;   // Linear velocity to speed scaling factor
        double angular_velocity_scale = DEFAULT_ANGULAR_VELOCITY_SCALE; // Angular velocity to speed scaling factor
        double minimum_speed_ratio = DEFAULT_MIN_SPEED_RATIO;           // Minimum speed as ratio of max (20%)
        double maximum_speed_ratio = DEFAULT_MAX_SPEED_RATIO;           // Maximum speed as ratio of max (180%)
        bool enable_adaptive_scaling = true;                           // Enable adaptive scaling based on leg workspace
    };

    /**
     * @brief Gait-specific speed modifiers.
     */
    struct GaitSpeedModifiers {
        double tripod_speed_factor = DEFAULT_TRIPOD_SPEED_FACTOR;           // Tripod gait speed multiplier
        double wave_speed_factor = DEFAULT_WAVE_SPEED_FACTOR;               // Wave gait speed multiplier
        double ripple_speed_factor = DEFAULT_RIPPLE_SPEED_FACTOR;           // Ripple gait speed multiplier
        double metachronal_speed_factor = DEFAULT_METACHRONAL_SPEED_FACTOR; // Metachronal gait speed multiplier
        double adaptive_speed_factor = DEFAULT_ADAPTIVE_SPEED_FACTOR;       // Adaptive gait speed multiplier
    };

    /**
     * @brief Construct velocity controller.
     * @param model Reference to robot model for kinematics.
     */
    explicit CartesianVelocityController(const RobotModel &model);
    ~CartesianVelocityController(); // Needed for unique_ptr with forward declaration

    /**
     * @brief Update servo speeds based on commanded Cartesian velocity.
     * @param linear_velocity_x Linear velocity in X direction (m/s)
     * @param linear_velocity_y Linear velocity in Y direction (m/s)
     * @param angular_velocity Angular velocity around Z axis (rad/s)
     * @param current_gait Active gait type
     * @return True if servo speeds were successfully calculated
     */
    bool updateServoSpeeds(double linear_velocity_x, double linear_velocity_y,
                           double angular_velocity, GaitType current_gait);

    /**
     * @brief Get calculated servo speed for a specific joint.
     * @param leg_index Leg index (0-5)
     * @param joint_index Joint index (0=coxa, 1=femur, 2=tibia)
     * @return Calculated servo speed
     */
    double getServoSpeed(int leg_index, int joint_index) const;

    /**
     * @brief Get servo speeds for an entire leg.
     * @param leg_index Leg index (0-5)
     * @return Servo speed configuration for the leg
     */
    const LegServoSpeeds &getLegServoSpeeds(int leg_index) const;

    /**
     * @brief Set velocity scaling parameters.
     * @param scaling New scaling configuration
     */
    void setVelocityScaling(const VelocityScaling &scaling);

    /**
     * @brief Set gait-specific speed modifiers.
     * @param modifiers New gait speed modifiers
     */
    void setGaitSpeedModifiers(const GaitSpeedModifiers &modifiers);

    /**
     * @brief Enable or disable velocity-based servo speed control.
     * @param enable True to enable velocity control, false for constant speed
     */
    void setVelocityControlEnabled(bool enable);

    /**
     * @brief Check if velocity control is currently enabled.
     * @return True if velocity control is active
     */
    bool isVelocityControlEnabled() const { return velocity_control_enabled_; }

    /**
     * @brief Get current Cartesian velocity magnitude.
     * @return Total velocity magnitude in m/s
     */
    double getCurrentVelocityMagnitude() const;

    /**
     * @brief Reset servo speeds to default configuration.
     */
    void resetToDefaults();

  private:
    const RobotModel &model_;
    std::unique_ptr<WorkspaceValidator> workspace_validator_; // Workspace validation
    std::array<LegServoSpeeds, NUM_LEGS> leg_servo_speeds_;
    VelocityScaling velocity_scaling_;
    GaitSpeedModifiers gait_modifiers_;

    // Current velocity state
    double current_linear_vx_ = 0.0f;
    double current_linear_vy_ = 0.0f;
    double current_angular_velocity_ = 0.0f;
    GaitType current_gait_ = TRIPOD_GAIT;

    bool velocity_control_enabled_ = true;

    /**
     * @brief Calculate velocity scaling factor for linear motion.
     * @param velocity_magnitude Linear velocity magnitude (m/s)
     * @return Scaling factor for servo speeds
     */
    double calculateLinearVelocityScale(double velocity_magnitude) const;

    /**
     * @brief Calculate velocity scaling factor for angular motion.
     * @param angular_velocity Angular velocity magnitude (rad/s)
     * @return Scaling factor for servo speeds
     */
    double calculateAngularVelocityScale(double angular_velocity) const;

    /**
     * @brief Calculate gait-specific speed adjustment.
     * @param gait Current gait type
     * @return Gait speed multiplier
     */
    double calculateGaitSpeedAdjustment(GaitType gait) const;

    /**
     * @brief Calculate per-leg speed compensation based on leg position and velocity direction.
     * @param leg_index Leg index (0-5)
     * @param linear_vx Linear velocity in X
     * @param linear_vy Linear velocity in Y
     * @param angular_vel Angular velocity
     * @return Speed compensation factor for this leg
     */
    double calculateLegSpeedCompensation(int leg_index, double linear_vx,
                                        double linear_vy, double angular_vel) const;

    /**
     * @brief Apply workspace constraints to servo speed.
     * @param leg_index Leg index
     * @param joint_index Joint index
     * @param base_speed Base calculated speed
     * @return Workspace-constrained speed
     */
    double applyWorkspaceConstraints(int leg_index, int joint_index, double base_speed) const;
};

#endif // CARTESIAN_VELOCITY_CONTROLLER_H
