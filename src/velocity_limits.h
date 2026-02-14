#ifndef VELOCITY_LIMITS_H
#define VELOCITY_LIMITS_H

#include "gait_config.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "robot_model.h"
#include <map>

/**
 * @brief Generates bearing-based velocity and acceleration limit maps and provides
 *        runtime limit interpolation (OpenSHC equivalent).
 *
 * Computes per-bearing maximum linear/angular speeds and accelerations based on
 * walkspace radii, gait parameters and step cycle timing.
 *
 * In OpenSHC, both generateLimits() and getLimit() live on WalkController.
 * HexaMotion intentionally consolidates all velocity-limit logic here for
 * architectural coherence; getLimit() receives per-leg tip positions as an
 * explicit parameter instead of accessing leg steppers directly.
 *
 * @see OpenSHC_GAP_REPORT.md — "VelocityLimits::getLimit relocation"
 */
class VelocityLimits {
  public:
    typedef std::map<int, double> LimitMap;

    /** @brief Construct with a reference to the robot model. */
    explicit VelocityLimits(const RobotModel &model);

    /** @brief Default destructor. */
    ~VelocityLimits() = default;

    /** @brief Set the walkspace map used for limit generation. */
    void setWalkspace(const LimitMap &walkspace);

    /**
     * @brief Generate velocity limits for a given step cycle and gait configuration.
     *
     * When all four output pointers are NULL, the results are stored in the
     * internal member maps (accessible via the getMax*Map() accessors).
     *
     * @param step          Pre-computed step cycle timing.
     * @param gait_config   Active gait configuration.
     * @param max_linear_speed_ptr        Output map for linear speed limits (may be NULL).
     * @param max_angular_speed_ptr       Output map for angular speed limits (may be NULL).
     * @param max_linear_acceleration_ptr Output map for linear acceleration limits (may be NULL).
     * @param max_angular_acceleration_ptr Output map for angular acceleration limits (may be NULL).
     */
    void generateLimits(StepCycle step,
                        const GaitConfiguration &gait_config,
                        LimitMap *max_linear_speed_ptr = NULL,
                        LimitMap *max_angular_speed_ptr = NULL,
                        LimitMap *max_linear_acceleration_ptr = NULL,
                        LimitMap *max_angular_acceleration_ptr = NULL);

    /**
     * @brief Convenience overload that generates a StepCycle from the gait configuration.
     *
     * @param gait_config   Active gait configuration (used to derive StepCycle internally).
     * @param max_linear_speed_ptr        Output map for linear speed limits (may be NULL).
     * @param max_angular_speed_ptr       Output map for angular speed limits (may be NULL).
     * @param max_linear_acceleration_ptr Output map for linear acceleration limits (may be NULL).
     * @param max_angular_acceleration_ptr Output map for angular acceleration limits (may be NULL).
     */
    void generateLimits(const GaitConfiguration &gait_config,
                        LimitMap *max_linear_speed_ptr = NULL,
                        LimitMap *max_angular_speed_ptr = NULL,
                        LimitMap *max_linear_acceleration_ptr = NULL,
                        LimitMap *max_angular_acceleration_ptr = NULL);

    /**
     * @brief Get interpolated limit for a velocity command from a bearing-based limit map
     *        (OpenSHC WalkController::getLimit equivalent).
     *
     * Calculates per-leg stride bearing from combined linear + angular velocity,
     * interpolates the limit map bounding that bearing, and returns the minimum
     * across all legs.
     *
     * @note In OpenSHC this method lives on WalkController and accesses leg steppers
     *       directly.  HexaMotion passes tip positions explicitly so that all
     *       velocity-limit logic is co-located in VelocityLimits.
     *
     * @param linear_velocity_input  Desired linear body velocity (XY plane).
     * @param angular_velocity_input Desired angular body velocity.
     * @param limit                  Bearing-based limit map (e.g. max_linear_speed).
     * @param tip_positions          Current tip pose for each leg (array of NUM_LEGS).
     * @return Smallest interpolated limit across all legs.
     */
    double getLimit(const Eigen::Vector2d &linear_velocity_input,
                    double angular_velocity_input,
                    const LimitMap &limit,
                    const Point3D tip_positions[NUM_LEGS]) const;

    /** @brief Read-only accessor for the generated linear speed limit map. */
    const LimitMap &getMaxLinearSpeedMap() const { return max_linear_speed_; }

    /** @brief Read-only accessor for the generated angular speed limit map. */
    const LimitMap &getMaxAngularSpeedMap() const { return max_angular_speed_; }

    /** @brief Read-only accessor for the generated linear acceleration limit map. */
    const LimitMap &getMaxLinearAccelerationMap() const { return max_linear_acceleration_; }

    /** @brief Read-only accessor for the generated angular acceleration limit map. */
    const LimitMap &getMaxAngularAccelerationMap() const { return max_angular_acceleration_; }

  private:
    const RobotModel &model_;
    LimitMap walkspace_;
    LimitMap max_linear_speed_;
    LimitMap max_angular_speed_;
    LimitMap max_linear_acceleration_;
    LimitMap max_angular_acceleration_;
};

#endif // VELOCITY_LIMITS_H
