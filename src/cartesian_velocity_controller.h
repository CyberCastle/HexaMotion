#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H

#include "HexaModel.h"
#include "velocity_limits.h"
#include <array>

// Velocity control constants
#define SERVO_SPEED_MIN 0.1f                  // Minimum servo speed multiplier
#define SERVO_SPEED_MAX 3.0f                  // Maximum servo speed multiplier
#define SERVO_SPEED_DEFAULT 1.0f              // Default servo speed multiplier
#define VELOCITY_SCALE_MIN 0.1f               // Minimum velocity scaling factor
#define VELOCITY_SCALE_MAX 5.0f               // Maximum velocity scaling factor
#define SPEED_RATIO_MIN 0.05f                 // Minimum speed ratio (5%)
#define SPEED_RATIO_MAX_VALIDATION 0.95f      // Maximum for minimum_speed_ratio validation
#define SPEED_RATIO_MIN_VALIDATION 1.05f      // Minimum for maximum_speed_ratio validation
#define SPEED_RATIO_MAX 3.0f                  // Maximum speed ratio (300%)
#define GAIT_MODIFIER_MIN 0.1f                // Minimum gait speed modifier
#define GAIT_MODIFIER_MAX 2.0f                // Maximum gait speed modifier
#define LEG_COMPENSATION_MIN 0.5f             // Minimum leg speed compensation
#define LEG_COMPENSATION_MAX 2.0f             // Maximum leg speed compensation
#define DEFAULT_LINEAR_VELOCITY_SCALE 2.0f    // Default linear velocity scaling factor
#define DEFAULT_ANGULAR_VELOCITY_SCALE 1.5f   // Default angular velocity scaling factor
#define DEFAULT_MIN_SPEED_RATIO 0.2f          // Default minimum speed ratio (20%)
#define DEFAULT_MAX_SPEED_RATIO 1.8f          // Default maximum speed ratio (180%)
#define DEFAULT_TRIPOD_SPEED_FACTOR 1.2f      // Default tripod gait speed factor
#define DEFAULT_WAVE_SPEED_FACTOR 0.8f        // Default wave gait speed factor
#define DEFAULT_RIPPLE_SPEED_FACTOR 0.9f      // Default ripple gait speed factor
#define DEFAULT_METACHRONAL_SPEED_FACTOR 1.0f // Default metachronal gait speed factor
#define DEFAULT_ADAPTIVE_SPEED_FACTOR 1.1f    // Default adaptive gait speed factor
#define COXA_SPEED_FACTOR 0.9f                // Coxa joint speed adjustment (10% slower)
#define FEMUR_SPEED_FACTOR 1.0f               // Femur joint speed adjustment (normal)
#define TIBIA_SPEED_FACTOR 1.1f               // Tibia joint speed adjustment (10% faster)
#define FEMUR_VELOCITY_MULTIPLIER 1.1f        // Femur velocity scaling multiplier
#define TIBIA_VELOCITY_MULTIPLIER 1.2f        // Tibia velocity scaling multiplier

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
        float base_speed = SERVO_SPEED_DEFAULT;           // Base servo speed multiplier (0.1-3.0)
        float velocity_scaling = SERVO_SPEED_DEFAULT;     // Velocity-dependent scaling factor
        float angular_compensation = SERVO_SPEED_DEFAULT; // Angular velocity compensation
        float gait_adjustment = SERVO_SPEED_DEFAULT;      // Gait-specific adjustment

        /**
         * @brief Calculate final servo speed from all factors.
         * @return Computed servo speed (clamped to valid range)
         */
        float getEffectiveSpeed() const {
            float speed = base_speed * velocity_scaling * angular_compensation * gait_adjustment;
            return std::max(SERVO_SPEED_MIN, std::min(SERVO_SPEED_MAX, speed)); // Clamp to servo limits
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
        float linear_velocity_scale = DEFAULT_LINEAR_VELOCITY_SCALE;   // Linear velocity to speed scaling factor
        float angular_velocity_scale = DEFAULT_ANGULAR_VELOCITY_SCALE; // Angular velocity to speed scaling factor
        float minimum_speed_ratio = DEFAULT_MIN_SPEED_RATIO;           // Minimum speed as ratio of max (20%)
        float maximum_speed_ratio = DEFAULT_MAX_SPEED_RATIO;           // Maximum speed as ratio of max (180%)
        bool enable_adaptive_scaling = true;                           // Enable adaptive scaling based on leg workspace
    };

    /**
     * @brief Gait-specific speed modifiers.
     */
    struct GaitSpeedModifiers {
        float tripod_speed_factor = DEFAULT_TRIPOD_SPEED_FACTOR;           // Tripod gait speed multiplier
        float wave_speed_factor = DEFAULT_WAVE_SPEED_FACTOR;               // Wave gait speed multiplier
        float ripple_speed_factor = DEFAULT_RIPPLE_SPEED_FACTOR;           // Ripple gait speed multiplier
        float metachronal_speed_factor = DEFAULT_METACHRONAL_SPEED_FACTOR; // Metachronal gait speed multiplier
        float adaptive_speed_factor = DEFAULT_ADAPTIVE_SPEED_FACTOR;       // Adaptive gait speed multiplier
    };

    /**
     * @brief Construct velocity controller.
     * @param model Reference to robot model for kinematics.
     */
    explicit CartesianVelocityController(const RobotModel &model);

    /**
     * @brief Update servo speeds based on commanded Cartesian velocity.
     * @param linear_velocity_x Linear velocity in X direction (m/s)
     * @param linear_velocity_y Linear velocity in Y direction (m/s)
     * @param angular_velocity Angular velocity around Z axis (rad/s)
     * @param current_gait Active gait type
     * @return True if servo speeds were successfully calculated
     */
    bool updateServoSpeeds(float linear_velocity_x, float linear_velocity_y,
                           float angular_velocity, GaitType current_gait);

    /**
     * @brief Get calculated servo speed for a specific joint.
     * @param leg_index Leg index (0-5)
     * @param joint_index Joint index (0=coxa, 1=femur, 2=tibia)
     * @return Calculated servo speed
     */
    float getServoSpeed(int leg_index, int joint_index) const;

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
    float getCurrentVelocityMagnitude() const;

    /**
     * @brief Reset servo speeds to default configuration.
     */
    void resetToDefaults();

  private:
    const RobotModel &model_;
    std::array<LegServoSpeeds, NUM_LEGS> leg_servo_speeds_;
    VelocityScaling velocity_scaling_;
    GaitSpeedModifiers gait_modifiers_;

    // Current velocity state
    float current_linear_vx_ = 0.0f;
    float current_linear_vy_ = 0.0f;
    float current_angular_velocity_ = 0.0f;
    GaitType current_gait_ = TRIPOD_GAIT;

    bool velocity_control_enabled_ = true;

    /**
     * @brief Calculate velocity scaling factor for linear motion.
     * @param velocity_magnitude Linear velocity magnitude (m/s)
     * @return Scaling factor for servo speeds
     */
    float calculateLinearVelocityScale(float velocity_magnitude) const;

    /**
     * @brief Calculate velocity scaling factor for angular motion.
     * @param angular_velocity Angular velocity magnitude (rad/s)
     * @return Scaling factor for servo speeds
     */
    float calculateAngularVelocityScale(float angular_velocity) const;

    /**
     * @brief Calculate gait-specific speed adjustment.
     * @param gait Current gait type
     * @return Gait speed multiplier
     */
    float calculateGaitSpeedAdjustment(GaitType gait) const;

    /**
     * @brief Calculate per-leg speed compensation based on leg position and velocity direction.
     * @param leg_index Leg index (0-5)
     * @param linear_vx Linear velocity in X
     * @param linear_vy Linear velocity in Y
     * @param angular_vel Angular velocity
     * @return Speed compensation factor for this leg
     */
    float calculateLegSpeedCompensation(int leg_index, float linear_vx,
                                        float linear_vy, float angular_vel) const;

    /**
     * @brief Apply workspace constraints to servo speed.
     * @param leg_index Leg index
     * @param joint_index Joint index
     * @param base_speed Base calculated speed
     * @return Workspace-constrained speed
     */
    float applyWorkspaceConstraints(int leg_index, int joint_index, float base_speed) const;
};

#endif // CARTESIAN_VELOCITY_CONTROLLER_H
