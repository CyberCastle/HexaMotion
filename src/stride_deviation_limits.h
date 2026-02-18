#ifndef STRIDE_DEVIATION_LIMITS_H
#define STRIDE_DEVIATION_LIMITS_H

#include "gait_config.h"
#include "robot_model.h"
#include "velocity_limits.h"
#include <utility>

/**
 * @brief Computes stride/deviation-aware velocity caps and morphology-based kinematic limits.
 *
 * This class consolidates utility calculations that are already present across OpenSHC-equivalent
 * components in HexaMotion:
 * - Velocity map generation and bearing interpolation policy (`VelocityLimits`)
 * - Body height range from joint limits (`RobotModel::calculateHeightRange`)
 * - Standing horizontal reach and default height offset (`RobotModel` helpers)
 *
 * The "minimal angular deviation" velocity is estimated by evaluating femur/tibia IK deviation
 * under forward stride displacement and selecting the maximum linear speed that satisfies a
 * user-specified deviation bound.
 */
class StrideDeviationLimits {
  public:
    /** @brief Construct with a robot model reference. */
    explicit StrideDeviationLimits(const RobotModel &model);

    /** @brief Default destructor. */
    ~StrideDeviationLimits() = default;

    /**
     * @brief Compute the worst femur/tibia absolute angular deviation for a forward linear speed.
     *
     * Uses the same stride definition as OpenSHC/HexaMotion:
     * `stride = v * (stance_period / period) / frequency`.
     *
     * @param gait_config Active gait configuration.
     * @param linear_speed_mm_s Forward linear speed in mm/s.
     * @return Worst absolute deviation (degrees) across all legs and both femur/tibia joints.
     */
    double calculateWorstFemurTibiaDeviationDeg(const GaitConfiguration &gait_config,
                                                double linear_speed_mm_s) const;

    /**
     * @brief Compute maximum linear speed with bounded stride-induced angular deviation.
     *
     * The result is found by binary search on speed and can optionally be clamped by
     * OpenSHC-equivalent walkspace speed limits.
     *
     * @param gait_config Active gait configuration.
     * @param max_allowed_deviation_deg Allowed maximum femur/tibia deviation in degrees.
     * @param walkspace Optional pointer to walkspace radius map (bearing -> radius). If provided,
     *                  the result is clamped by OpenSHC-equivalent generated speed limits.
     * @return Maximum speed in mm/s that satisfies the deviation bound.
     */
    double calculateMaxLinearSpeedForMinimalAngularDeviation(const GaitConfiguration &gait_config,
                                                             double max_allowed_deviation_deg,
                                                             const VelocityLimits::LimitMap *walkspace = NULL) const;

    /**
     * @brief Compute OpenSHC-equivalent conservative maximum linear speed from walkspace.
     * @param gait_config Active gait configuration.
     * @param walkspace Walkspace radius map (bearing -> radius).
     * @return Minimum generated linear speed limit across all bearings (mm/s).
     */
    double calculateOpenSHCMaxLinearSpeedFromWalkspace(const GaitConfiguration &gait_config,
                                                       const VelocityLimits::LimitMap &walkspace) const;

    /**
     * @brief Compute OpenSHC-equivalent conservative maximum angular speed from walkspace.
     * @param gait_config Active gait configuration.
     * @param walkspace Walkspace radius map (bearing -> radius).
     * @return Minimum generated angular speed limit across all bearings (rad/s).
     */
    double calculateOpenSHCMaxAngularSpeedFromWalkspace(const GaitConfiguration &gait_config,
                                                        const VelocityLimits::LimitMap &walkspace) const;

    /**
     * @brief Compute body height range from configured joint limits.
     * @return Pair {min_height_mm, max_height_mm}.
     */
    std::pair<double, double> calculateBodyHeightRange() const;

    /** @brief @return Minimum reachable body height in millimeters. */
    double calculateMinimumBodyHeight() const;

    /** @brief @return Maximum reachable body height in millimeters. */
    double calculateMaximumBodyHeight() const;

    /**
     * @brief Compute minimum planar hip-to-tip reach from joint limits.
     * @return Minimum planar reach in millimeters, or -1.0 on invalid configuration.
     */
    double calculateMinimumPlanarReach() const;

    /**
     * @brief Compute maximum planar hip-to-tip reach from joint limits.
     * @return Maximum planar reach in millimeters, or -1.0 on invalid configuration.
     */
    double calculateMaximumPlanarReach() const;

    /** @brief @return Standing horizontal reach (mm), morphology-aware helper. */
    double calculateStandingHorizontalReach() const;

    /** @brief @return Configured default height offset (mm). */
    double calculateDefaultHeightOffset() const;

  private:
    const RobotModel &model_;

    Point3D getReferenceTipForLimits() const;
    JointAngles getNominalStandingAngles() const;
    double getFiniteMapMinimum(const VelocityLimits::LimitMap &limits) const;
    std::pair<double, double> calculatePlanarReachRange() const;
};

#endif // STRIDE_DEVIATION_LIMITS_H
