#include "tripod_gait_controller.h"
#include <algorithm>
#include <cmath>

static constexpr double STAND_HEIGHT = 150.0;          // mm
static constexpr double DEFAULT_FREQUENCY = 1.0;       // Hz
static constexpr double DEFAULT_STEP_LENGTH = 40.0;    // mm
static constexpr double DEFAULT_STEP_HEIGHT = 30.0;    // mm
static constexpr double BASE_RADIAL = 151.0;           // coxa + femur

TripodGaitController::TripodGaitController(const Parameters &params)
    : params_(params), model_(params), phase_(0.0), frequency_(DEFAULT_FREQUENCY),
      step_length_(DEFAULT_STEP_LENGTH), step_height_(DEFAULT_STEP_HEIGHT),
      body_yaw_(0.0), linear_sign_(1.0), angular_step_(0.0), walking_(false) {}

void TripodGaitController::start() { walking_ = true; }

void TripodGaitController::stop() { walking_ = false; }

void TripodGaitController::moveForward(double step) {
    step_length_ = std::abs(step);
    linear_sign_ = 1.0;
}

void TripodGaitController::moveBackward(double step) {
    step_length_ = std::abs(step);
    linear_sign_ = -1.0;
}

void TripodGaitController::rotateLeft(double angle_step) { angular_step_ = angle_step; }

void TripodGaitController::rotateRight(double angle_step) { angular_step_ = -angle_step; }

Point3D TripodGaitController::computeFootPosition(int leg, double x_off, double z_off) const {
    double angle = BASE_THETA_OFFSETS[leg] + body_yaw_;
    double c = std::cos(angle);
    double s = std::sin(angle);

    double radial = BASE_RADIAL + linear_sign_ * x_off;
    double x = params_.hexagon_radius * c + radial * c;
    double y = params_.hexagon_radius * s + radial * s;
    double z = -STAND_HEIGHT + z_off;
    return Point3D{x, y, z};
}

JointAngles TripodGaitController::solveIK(int leg, const Point3D &target) const {
    double base_angle = BASE_THETA_OFFSETS[leg] + body_yaw_;
    double c = std::cos(base_angle);
    double s = std::sin(base_angle);

    double bx = params_.hexagon_radius * c;
    double by = params_.hexagon_radius * s;

    double xr = target.x - bx;
    double yr = target.y - by;
    double x_local = c * xr + s * yr;
    double y_local = -s * xr + c * yr;
    double z_local = target.z;

    double q1 = std::atan2(y_local, x_local);

    double r_coxa = std::sqrt(x_local * x_local + y_local * y_local);
    double px = r_coxa - params_.coxa_length;
    double pz = z_local;

    double r = std::sqrt(px * px + pz * pz);
    double K = (params_.femur_length * params_.femur_length -
                params_.tibia_length * params_.tibia_length - r * r) /
               (2.0 * params_.tibia_length);
    double phi = std::atan2(pz, px);
    double arg = K / r;
    arg = std::clamp(arg, -1.0, 1.0);
    double q3 = std::asin(arg) - phi;

    double cosq2 = (px + params_.tibia_length * std::sin(q3)) / params_.femur_length;
    double sinq2 = -(pz + params_.tibia_length * std::cos(q3)) / params_.femur_length;
    double q2 = std::atan2(sinq2, cosq2);

    return JointAngles{q1, q2, q3};
}

void TripodGaitController::update(double dt, std::array<JointAngles, NUM_LEGS> &angles) {
    if (!walking_) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            Point3D target = computeFootPosition(i, 0.0, 0.0);
            angles[i] = solveIK(i, target);
        }
        return;
    }

    body_yaw_ += angular_step_ * dt;
    phase_ += frequency_ * dt;
    if (phase_ >= 1.0)
        phase_ -= 1.0;

    for (int i = 0; i < NUM_LEGS; ++i) {
        bool group_a = (i % 2 == 0);
        double leg_phase = phase_ + (group_a ? 0.0 : 0.5);
        if (leg_phase >= 1.0)
            leg_phase -= 1.0;

        double x_off = 0.0;
        double z_off = 0.0;
        if (leg_phase < 0.5) {
            double t = leg_phase * 2.0;
            x_off = (0.5 - t) * step_length_;
            z_off = 0.0;
        } else {
            double t = (leg_phase - 0.5) * 2.0;
            x_off = (-0.5 + t) * step_length_;
            z_off = step_height_ * std::sin(M_PI * t);
        }

        Point3D target = computeFootPosition(i, x_off, z_off);
        angles[i] = solveIK(i, target);
    }
}

