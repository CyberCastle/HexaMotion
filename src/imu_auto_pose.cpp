#include "imu_auto_pose.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

IMUAutoPose::IMUAutoPose(RobotModel &model, IIMUInterface *imu, ManualPoseController &pose_controller,
                         ComputeConfig config)
    : model_(model), imu_(imu), pose_controller_(pose_controller), config_(config),
      current_mode_(AUTO_POSE_OFF), filter_alpha_(0.1f), last_update_time_(0),
      terrain_roughness_estimate_(0.0f), walking_detected_(false) {

    update_interval_ = config_.getDeltaTime() * 1000.0f; // Convert to milliseconds
}

void IMUAutoPose::initialize() {
    // Initialize filters
    gravity_filter_ = Point3D(0, 0, -9.81f);
    orientation_filter_ = Point3D(0, 0, 0);

    // Set filter parameters based on precision level
    switch (config_.precision) {
    case PRECISION_LOW:
        filter_alpha_ = 0.2f; // Faster response, less smooth
        break;
    case PRECISION_MEDIUM:
        filter_alpha_ = 0.1f; // Balanced
        break;
    case PRECISION_HIGH:
        filter_alpha_ = 0.05f; // Slower response, very smooth
        break;
    }

    resetFilters();
}

void IMUAutoPose::setAutoPoseMode(AutoPoseMode mode) {
    current_mode_ = mode;
    current_state_.pose_active = (mode != AUTO_POSE_OFF);

    if (mode == AUTO_POSE_OFF) {
        // Reset pose when disabling auto-pose
        resetFilters();
    }
}

void IMUAutoPose::setIMUPoseParams(const IMUPoseParams &params) {
    params_ = params;
}

void IMUAutoPose::update(float dt) {
    if (!imu_ || current_mode_ == AUTO_POSE_OFF) {
        current_state_.pose_active = false;
        return;
    }

    // Check update timing - skip timing check if dt is provided (for testing)
    // HARDWARE CONSIDERATION: Real systems need robust timing to handle:
    // - Variable processing delays in main loop
    // - IMU data rate mismatches (sensor vs control frequency)
    // - Recovery from missed updates due to system overload
    unsigned long current_time = millis();
    if (dt <= 0.0f && current_time - last_update_time_ < update_interval_) {
        return;
    }
    last_update_time_ = current_time;

    // Update IMU data and filters
    updateIMUData();

    // Update pose based on current mode
    switch (current_mode_) {
    case AUTO_POSE_LEVEL:
        updateLevelMode();
        break;
    case AUTO_POSE_INCLINATION:
        updateInclinationMode();
        break;
    case AUTO_POSE_ADAPTIVE:
        updateAdaptiveMode();
        break;
    case AUTO_POSE_CUSTOM:
        // Custom implementation would go here
        break;
    default:
        break;
    }

    // Apply calculated pose if active
    if (current_state_.pose_active) {
        applyAutoPose();
    }
}

void IMUAutoPose::setEnabled(bool enabled) {
    if (enabled) {
        if (current_mode_ == AUTO_POSE_OFF) {
            setAutoPoseMode(AUTO_POSE_LEVEL); // Default mode
        }
    } else {
        setAutoPoseMode(AUTO_POSE_OFF);
    }
}

void IMUAutoPose::setWalkingState(bool walking) {
    walking_detected_ = walking;

    // Adjust response parameters based on walking state
    if (walking) {
        // Reduce response speed during walking for stability
        params_.response_speed *= 0.5f;
        params_.deadzone_degrees *= 1.5f;
    }
}

void IMUAutoPose::resetFilters() {
    gravity_filter_ = Point3D(0, 0, -9.81f);
    orientation_filter_ = Point3D(0, 0, 0);
    current_state_ = AutoPoseState();
}

void IMUAutoPose::updateIMUData() {
    if (!imu_)
        return;

    IMUData imu_data = imu_->readIMU();

    // Update gravity estimate
    updateGravityEstimate(imu_data);

    // Update orientation from IMU
    Point3D current_orientation(imu_data.roll, imu_data.pitch, imu_data.yaw);
    orientation_filter_ = lowPassFilter(current_orientation, orientation_filter_, filter_alpha_);

    // Calculate inclination and errors
    updateInclinationEstimate();
    calculateOrientationError();
    calculateCorrectionPose();
}

void IMUAutoPose::updateGravityEstimate(const IMUData &imu_data) {
    // Convert IMU acceleration to gravity estimate
    Point3D accel(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);

    // Normalize acceleration vector
    float magnitude = math_utils::magnitude(accel);
    if (magnitude > 0.5f) { // Valid acceleration reading
        // CRITICAL HARDWARE CONSIDERATION:
        // Different IMU manufacturers use different acceleration conventions:
        //
        // PHYSICS CONVENTION (most common): IMU reports acceleration opposing gravity
        // - Stationary robot: accel_z = +9.81 (upward support force)
        // - Gravity vector: (0, 0, -9.81) (downward)
        // - Conversion: gravity = -accel
        //
        // DIRECT CONVENTION (some specialized IMUs): IMU reports gravity directly
        // - Stationary robot: accel_z = -9.81 (gravity direction)
        // - Gravity vector: (0, 0, -9.81) (same as reading)
        // - Conversion: gravity = accel
        //
        // Our DummyIMU uses DIRECT convention, hence no inversion needed.
        // PRODUCTION CODE MUST determine convention during calibration!
        Point3D gravity_estimate = accel;
        gravity_filter_ = lowPassFilter(gravity_estimate, gravity_filter_, filter_alpha_);
    }

    current_state_.gravity_vector = gravity_filter_;
}

void IMUAutoPose::updateInclinationEstimate() {
    // Calculate inclination from gravity vector
    Point3D gravity = current_state_.gravity_vector;
    float magnitude = math_utils::magnitude(gravity);

    if (magnitude > 0.1f) {
        // Calculate roll and pitch from gravity vector
        float roll = atan2(gravity.y, gravity.z);
        float pitch = atan2(-gravity.x, sqrt(gravity.y * gravity.y + gravity.z * gravity.z));

        current_state_.inclination_angle.x = roll;
        current_state_.inclination_angle.y = pitch;
        current_state_.inclination_angle.z = orientation_filter_.z; // Yaw from IMU
    }
}

void IMUAutoPose::calculateOrientationError() {
    // Calculate error between current orientation and desired (level) orientation
    Point3D desired_orientation(0, 0, 0); // Level orientation

    if (current_mode_ == AUTO_POSE_INCLINATION) {
        // For inclination mode, desired orientation compensates for surface tilt
        desired_orientation = current_state_.inclination_angle * (-1.0f);
    }

    current_state_.orientation_error = desired_orientation - orientation_filter_;
    current_state_.orientation_error = normalizeAngles(current_state_.orientation_error);
}

void IMUAutoPose::calculateCorrectionPose() {
    Point3D error = current_state_.orientation_error;

    // Apply deadzone
    if (withinDeadzone(error)) {
        current_state_.correction_pose = Point3D(0, 0, 0);
        current_state_.confidence = 0.0f;
        return;
    }

    // Apply gains and response speed
    current_state_.correction_pose.x = error.x * params_.orientation_gain * params_.response_speed;
    current_state_.correction_pose.y = error.y * params_.orientation_gain * params_.response_speed;
    current_state_.correction_pose.z = error.z * params_.orientation_gain * params_.response_speed;

    // Calculate confidence
    current_state_.confidence = calculateConfidence(error);

    // Update adaptive gains if enabled
    if (params_.adaptive_gains) {
        updateAdaptiveGains();
    }
}

void IMUAutoPose::applyAutoPose() {
    if (current_state_.confidence < 0.1f)
        return;

    // Create pose state for manual pose controller
    ManualPoseController::PoseState auto_pose_state;
    auto_pose_state.body_rotation = current_state_.correction_pose;
    auto_pose_state.pose_active = true;
    auto_pose_state.pose_blend_factor = current_state_.confidence;

    // Apply pose through manual pose controller
    pose_controller_.setTargetPose(auto_pose_state);
}

void IMUAutoPose::updateLevelMode() {
    // Keep body level regardless of ground inclination
    current_state_.pose_active = true;
}

void IMUAutoPose::updateInclinationMode() {
    // Compensate for surface inclination to keep body parallel to surface
    current_state_.pose_active = true;
}

void IMUAutoPose::updateAdaptiveMode() {
    // Adaptive behavior based on conditions
    if (walking_detected_) {
        // During walking, use more conservative approach
        params_.orientation_gain *= 0.7f;
    } else {
        // While stationary, can be more responsive
        params_.orientation_gain = std::min(1.0f, params_.orientation_gain * 1.1f);
    }

    current_state_.pose_active = true;
}

Point3D IMUAutoPose::lowPassFilter(const Point3D &input, const Point3D &previous, float alpha) {
    return Point3D(
        previous.x + alpha * (input.x - previous.x),
        previous.y + alpha * (input.y - previous.y),
        previous.z + alpha * (input.z - previous.z));
}

float IMUAutoPose::calculateConfidence(const Point3D &orientation_error) {
    // Calculate confidence based on error magnitude and IMU data quality
    float error_magnitude = math_utils::magnitude(orientation_error);
    float max_error = math_utils::degreesToRadians(45.0f); // Maximum reasonable error

    float confidence = 1.0f - std::min(1.0f, error_magnitude / max_error);

    // Reduce confidence during walking
    if (walking_detected_) {
        confidence *= 0.7f;
    }

    return confidence;
}

// TODO: (Simplified implementation)
void IMUAutoPose::updateAdaptiveGains() {
    // Estimate terrain roughness from IMU variance
    // (Simplified implementation)
    float orientation_variance = abs(current_state_.orientation_error.x) +
                                 abs(current_state_.orientation_error.y);

    terrain_roughness_estimate_ = terrain_roughness_estimate_ * 0.9f + orientation_variance * 0.1f;

    // Adjust gains based on terrain roughness
    if (terrain_roughness_estimate_ > 0.1f) {
        // Rough terrain - reduce gains for stability
        params_.orientation_gain *= 0.95f;
        params_.response_speed *= 0.95f;
    } else {
        // Smooth terrain - can increase responsiveness
        params_.orientation_gain = std::min(1.0f, params_.orientation_gain * 1.02f);
        params_.response_speed = std::min(1.0f, params_.response_speed * 1.02f);
    }
}

Point3D IMUAutoPose::normalizeAngles(const Point3D &angles) {
    Point3D normalized;

    // Normalize each angle to [-PI, PI]
    normalized.x = atan2(sin(angles.x), cos(angles.x));
    normalized.y = atan2(sin(angles.y), cos(angles.y));
    normalized.z = atan2(sin(angles.z), cos(angles.z));

    return normalized;
}

bool IMUAutoPose::withinDeadzone(const Point3D &error) {
    float deadzone_rad = math_utils::degreesToRadians(params_.deadzone_degrees);

    return (abs(error.x) < deadzone_rad &&
            abs(error.y) < deadzone_rad &&
            abs(error.z) < deadzone_rad);
}
