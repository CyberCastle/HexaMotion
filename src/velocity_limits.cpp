#include "velocity_limits.h"
#include "hexamotion_constants.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

VelocityLimits::VelocityLimits(const RobotModel &model)
    : model_(model) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC equivalent: WalkController::getLimit
// Relocated to VelocityLimits for architectural coherence (see OpenSHC_GAP_REPORT.md).
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double VelocityLimits::getLimit(const Eigen::Vector2d &linear_velocity_input,
                                double angular_velocity_input,
                                const LimitMap &limit,
                                const Point3D tip_positions[NUM_LEGS]) const {
    double min_limit = UNASSIGNED_VALUE;
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D tip_position = tip_positions[i];
        Eigen::Vector2d rotation_normal(-tip_position.y, tip_position.x);
        Eigen::Vector2d stride_vector = linear_velocity_input + angular_velocity_input * rotation_normal;

        int bearing = math_utils::mod(math_utils::roundToInt(math_utils::radiansToDegrees(
                                          atan2(stride_vector[1], stride_vector[0]))),
                                      360);

        int upper_bound = limit.lower_bound(bearing)->first;
        int lower_bound = math_utils::mod(upper_bound - BEARING_STEP, 360);
        bearing += (bearing < lower_bound) ? 360 : 0;
        upper_bound += (upper_bound < lower_bound) ? 360 : 0;
        double control_input = static_cast<double>(bearing - lower_bound) / (upper_bound - lower_bound);
        double limit_interpolation = math_utils::interpolate(limit.at(lower_bound),
                                                             limit.at(math_utils::mod(upper_bound, 360)),
                                                             control_input);
        min_limit = std::min(min_limit, limit_interpolation);
    }
    return min_limit;
}

void VelocityLimits::setWalkspace(const LimitMap &walkspace) {
    walkspace_ = walkspace;
}

void VelocityLimits::generateLimits(const GaitConfiguration &gait_config,
                                    LimitMap *max_linear_speed_ptr,
                                    LimitMap *max_angular_speed_ptr,
                                    LimitMap *max_linear_acceleration_ptr,
                                    LimitMap *max_angular_acceleration_ptr) {
    generateLimits(gait_config.generateStepCycle(), gait_config,
                   max_linear_speed_ptr, max_angular_speed_ptr,
                   max_linear_acceleration_ptr, max_angular_acceleration_ptr);
}

void VelocityLimits::generateLimits(StepCycle step,
                                    const GaitConfiguration &gait_config,
                                    LimitMap *max_linear_speed_ptr,
                                    LimitMap *max_angular_speed_ptr,
                                    LimitMap *max_linear_acceleration_ptr,
                                    LimitMap *max_angular_acceleration_ptr) {
    int base_step_period = gait_config.phase_config.stance_phase + gait_config.phase_config.swing_phase;
    int normaliser = step.period_ / base_step_period;
    int base_step_offset = gait_config.phase_config.phase_offset * normaliser;

    bool set_limits = (!max_linear_speed_ptr && !max_linear_acceleration_ptr &&
                       !max_angular_speed_ptr && !max_angular_acceleration_ptr);
    if (set_limits) {
        max_linear_speed_ptr = &max_linear_speed_;
        max_angular_speed_ptr = &max_angular_speed_;
        max_linear_acceleration_ptr = &max_linear_acceleration_;
        max_angular_acceleration_ptr = &max_angular_acceleration_;
    }

    if (max_linear_speed_ptr)
        max_linear_speed_ptr->clear();
    if (max_linear_acceleration_ptr)
        max_linear_acceleration_ptr->clear();
    if (max_angular_speed_ptr)
        max_angular_speed_ptr->clear();
    if (max_angular_acceleration_ptr)
        max_angular_acceleration_ptr->clear();

    int max_stance_extension = 0;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        int multiplier = gait_config.offsets.getForLegIndex(leg);
        int step_offset = (base_step_offset * multiplier) % step.period_;
        if (step_offset > step.swing_start_ && step_offset < step.swing_end_) {
            max_stance_extension = std::max(max_stance_extension, step.swing_end_ - step_offset);
        }
    }

    const Parameters &params = model_.getParams();
    double time_to_max_stride = (max_stance_extension + step.stance_period_ + step.swing_period_) * params.time_delta;

    LimitMap::iterator it;
    for (it = walkspace_.begin(); it != walkspace_.end(); ++it) {
        double walkspace_radius = it->second;
        double on_ground_ratio = double(step.stance_period_) / step.period_;
        double max_speed = (walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
        double max_acceleration = max_speed / time_to_max_stride;

        double stance_overshoot = 0.0;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            int multiplier = gait_config.offsets.getForLegIndex(leg);
            double step_offset = (base_step_offset * multiplier) % step.period_;
            double t = step_offset * params.time_delta;
            double time_to_swing_end = time_to_max_stride - t;
            double v0 = max_acceleration * time_to_swing_end;
            double stride_length = v0 * (on_ground_ratio / step.frequency_);
            double d0 = -stride_length / 2.0;
            double d1 = d0 + v0 * t + 0.5 * max_acceleration * t * t;
            double d2 = max_speed * (step.stance_period_ * params.time_delta - t);
            stance_overshoot = std::max(stance_overshoot, d1 + d2 - walkspace_radius);
        }

        double swing_overshoot = 0.5 * max_speed * step.swing_period_ / (2.0 * step.period_ * step.frequency_);
        double scaled_walkspace_radius =
            (walkspace_radius / (walkspace_radius + stance_overshoot + swing_overshoot)) * walkspace_radius;

        Point3D reference_tip = model_.getLegDefaultPosition(0);
        double stance_radius = Eigen::Vector2d(reference_tip.x, reference_tip.y).norm();

        double max_linear_speed = (scaled_walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
        double max_linear_acceleration = max_linear_speed / time_to_max_stride;
        double max_angular_speed = max_linear_speed / stance_radius;
        double max_angular_acceleration = max_angular_speed / time_to_max_stride;

        if (walkspace_radius == 0.0) {
            max_linear_speed = 0.0;
            max_linear_acceleration = UNASSIGNED_VALUE;
            max_angular_speed = 0.0;
            max_angular_acceleration = UNASSIGNED_VALUE;
        }

        if (max_linear_speed_ptr)
            max_linear_speed_ptr->insert(LimitMap::value_type(it->first, max_linear_speed));
        if (max_linear_acceleration_ptr)
            max_linear_acceleration_ptr->insert(LimitMap::value_type(it->first, max_linear_acceleration));
        if (max_angular_speed_ptr)
            max_angular_speed_ptr->insert(LimitMap::value_type(it->first, max_angular_speed));
        if (max_angular_acceleration_ptr)
            max_angular_acceleration_ptr->insert(LimitMap::value_type(it->first, max_angular_acceleration));
    }
}
