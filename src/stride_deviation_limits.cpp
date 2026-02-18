#include "stride_deviation_limits.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

StrideDeviationLimits::StrideDeviationLimits(const RobotModel &model)
    : model_(model) {
}

double StrideDeviationLimits::calculateWorstFemurTibiaDeviationDeg(const GaitConfiguration &gait_config,
                                                                   double linear_speed_mm_s) const {
    if (linear_speed_mm_s <= 0.0) {
        return 0.0;
    }

    StepCycle step_cycle = gait_config.generateStepCycle();
    if (step_cycle.period_ <= 0 || step_cycle.frequency_ <= 0.0) {
        return 0.0;
    }

    double on_ground_ratio = static_cast<double>(step_cycle.stance_period_) / step_cycle.period_;
    double stride = linear_speed_mm_s * (on_ground_ratio / step_cycle.frequency_);
    double half_stride = 0.5 * stride;

    JointAngles nominal = getNominalStandingAngles();
    double worst_deg = 0.0;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D nominal_tip = model_.forwardKinematicsGlobalCoordinates(leg, nominal);
        Point3D target_tip = nominal_tip;
        target_tip.x += half_stride;

        JointAngles solved = model_.inverseKinematicsCurrentGlobalCoordinates(leg, nominal, target_tip);

        double femur_dev = std::fabs(math_utils::radiansToDegrees(solved.femur - nominal.femur));
        double tibia_dev = std::fabs(math_utils::radiansToDegrees(solved.tibia - nominal.tibia));
        if (!std::isfinite(femur_dev) || !std::isfinite(tibia_dev)) {
            return std::numeric_limits<double>::infinity();
        }

        worst_deg = std::max(worst_deg, std::max(femur_dev, tibia_dev));
    }

    return worst_deg;
}

double StrideDeviationLimits::calculateMaxLinearSpeedForMinimalAngularDeviation(
    const GaitConfiguration &gait_config,
    double max_allowed_deviation_deg,
    const VelocityLimits::LimitMap *walkspace) const {
    if (max_allowed_deviation_deg <= 0.0) {
        return 0.0;
    }

    double upper_speed = (model_.getParams().max_velocity > 0.0) ? model_.getParams().max_velocity : 1000.0;

    if (walkspace && !walkspace->empty()) {
        double open_shc_cap = calculateOpenSHCMaxLinearSpeedFromWalkspace(gait_config, *walkspace);
        if (open_shc_cap > 0.0) {
            upper_speed = std::min(upper_speed, open_shc_cap);
        }
    }

    if (upper_speed <= 0.0) {
        return 0.0;
    }

    if (calculateWorstFemurTibiaDeviationDeg(gait_config, upper_speed) <= max_allowed_deviation_deg) {
        return upper_speed;
    }

    double low = 0.0;
    double high = upper_speed;
    for (int i = 0; i < 50; ++i) {
        double mid = 0.5 * (low + high);
        double deviation = calculateWorstFemurTibiaDeviationDeg(gait_config, mid);
        if (deviation <= max_allowed_deviation_deg) {
            low = mid;
        } else {
            high = mid;
        }
    }

    return low;
}

double StrideDeviationLimits::calculateOpenSHCMaxLinearSpeedFromWalkspace(const GaitConfiguration &gait_config,
                                                                          const VelocityLimits::LimitMap &walkspace) const {
    if (walkspace.empty()) {
        return 0.0;
    }

    VelocityLimits limits(model_);
    limits.setWalkspace(walkspace);
    limits.setReferenceTipPosition(getReferenceTipForLimits());

    VelocityLimits::LimitMap linear_map;
    limits.generateLimits(gait_config, &linear_map, NULL, NULL, NULL);
    return getFiniteMapMinimum(linear_map);
}

double StrideDeviationLimits::calculateOpenSHCMaxAngularSpeedFromWalkspace(const GaitConfiguration &gait_config,
                                                                           const VelocityLimits::LimitMap &walkspace) const {
    if (walkspace.empty()) {
        return 0.0;
    }

    VelocityLimits limits(model_);
    limits.setWalkspace(walkspace);
    limits.setReferenceTipPosition(getReferenceTipForLimits());

    VelocityLimits::LimitMap angular_map;
    limits.generateLimits(gait_config, NULL, &angular_map, NULL, NULL);
    return getFiniteMapMinimum(angular_map);
}

std::pair<double, double> StrideDeviationLimits::calculateBodyHeightRange() const {
    return model_.calculateHeightRange();
}

double StrideDeviationLimits::calculateMinimumBodyHeight() const {
    return calculateBodyHeightRange().first;
}

double StrideDeviationLimits::calculateMaximumBodyHeight() const {
    return calculateBodyHeightRange().second;
}

double StrideDeviationLimits::calculateMinimumPlanarReach() const {
    return calculatePlanarReachRange().first;
}

double StrideDeviationLimits::calculateMaximumPlanarReach() const {
    return calculatePlanarReachRange().second;
}

double StrideDeviationLimits::calculateStandingHorizontalReach() const {
    return model_.getStandingHorizontalReach();
}

double StrideDeviationLimits::calculateDefaultHeightOffset() const {
    return model_.getDefaultHeightOffset();
}

Point3D StrideDeviationLimits::getReferenceTipForLimits() const {
    JointAngles nominal = getNominalStandingAngles();
    return model_.forwardKinematicsGlobalCoordinates(0, nominal);
}

JointAngles StrideDeviationLimits::getNominalStandingAngles() const {
    const Parameters &params = model_.getParams();
    CalculatedServoAngles calc = RobotModel::calculateServoAnglesForHeight(params.standing_height, params);
    if (calc.valid) {
        return JointAngles(0.0, calc.femur, calc.tibia);
    }
    return JointAngles(0.0, 0.0, 0.0);
}

double StrideDeviationLimits::getFiniteMapMinimum(const VelocityLimits::LimitMap &limits) const {
    double min_value = std::numeric_limits<double>::infinity();
    for (VelocityLimits::LimitMap::const_iterator it = limits.begin(); it != limits.end(); ++it) {
        double value = it->second;
        if (std::isfinite(value) && value < UNASSIGNED_VALUE) {
            min_value = std::min(min_value, value);
        }
    }
    if (!std::isfinite(min_value)) {
        return 0.0;
    }
    return min_value;
}

std::pair<double, double> StrideDeviationLimits::calculatePlanarReachRange() const {
    const Point3D base = model_.getLegBasePosition(0);
    const double coxa_step = (model_.getCoxaAngleLimitRad(1) - model_.getCoxaAngleLimitRad(0)) / WORKSPACE_RESOLUTION;
    const double femur_step = (model_.getFemurAngleLimitRad(1) - model_.getFemurAngleLimitRad(0)) / WORKSPACE_RESOLUTION;
    const double tibia_step = (model_.getTibiaAngleLimitRad(1) - model_.getTibiaAngleLimitRad(0)) / WORKSPACE_RESOLUTION;

    double min_reach = std::numeric_limits<double>::infinity();
    double max_reach = 0.0;

    for (int i = 0; i <= WORKSPACE_RESOLUTION; ++i) {
        for (int j = 0; j <= WORKSPACE_RESOLUTION; ++j) {
            for (int k = 0; k <= WORKSPACE_RESOLUTION; ++k) {
                JointAngles q(model_.getCoxaAngleLimitRad(0) + i * coxa_step,
                              model_.getFemurAngleLimitRad(0) + j * femur_step,
                              model_.getTibiaAngleLimitRad(0) + k * tibia_step);

                if (!model_.checkJointLimits(0, q)) {
                    continue;
                }

                Point3D tip = model_.forwardKinematicsGlobalCoordinates(0, q);
                double planar = std::sqrt((tip.x - base.x) * (tip.x - base.x) +
                                          (tip.y - base.y) * (tip.y - base.y));

                min_reach = std::min(min_reach, planar);
                max_reach = std::max(max_reach, planar);
            }
        }
    }

    if (!std::isfinite(min_reach)) {
        return std::make_pair(-1.0, -1.0);
    }
    return std::make_pair(min_reach, max_reach);
}
