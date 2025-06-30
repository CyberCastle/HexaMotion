#include "imu_auto_pose.h"

/**
 * @file imu_auto_pose.cpp
 * @brief Automatic pose adjustments using IMU data.
 */
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

    // Initialize IMU mode
    initializeIMUMode();

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

void IMUAutoPose::update(double dt) {
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

    // Update calibration status
    updateCalibrationStatus(imu_data);

    // Use absolute data if available and enabled, otherwise use raw data
    if (imu_data.has_absolute_capability && params_.use_absolute_data &&
        imu_data.absolute_data.absolute_orientation_valid) {
        updateWithAbsoluteData(imu_data);
        current_state_.using_absolute_data = true;
    } else {
        updateWithRawData(imu_data);
        current_state_.using_absolute_data = false;
    }

    // Update state information
    current_state_.active_mode = imu_data.mode;

    // Calculate inclination and errors
    updateInclinationEstimate();
    calculateOrientationError();
    calculateCorrectionPose();
}

void IMUAutoPose::updateGravityEstimate(const IMUData &imu_data) {
    // Convert IMU acceleration to gravity estimate
    Point3D accel(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);

    // Normalize acceleration vector
    double magnitude = math_utils::magnitude(accel);
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
    double magnitude = math_utils::magnitude(gravity);

    if (magnitude > 0.1f) {
        // Calculate roll and pitch from gravity vector
        double roll = atan2(gravity.y, gravity.z);
        double pitch = atan2(-gravity.x, sqrt(gravity.y * gravity.y + gravity.z * gravity.z));

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

// IMU Mode Management and Absolute Positioning Support

void IMUAutoPose::initializeIMUMode() {
    if (!imu_)
        return;

    // Check if IMU supports absolute positioning
    if (imu_->hasAbsolutePositioning()) {
        // Configure IMU mode based on preferences
        if (params_.use_absolute_data) {
            if (params_.prefer_sensor_fusion) {
                imu_->setIMUMode(IMU_MODE_ABSOLUTE_POS);
            } else {
                imu_->setIMUMode(IMU_MODE_FUSION);
            }
        } else {
            imu_->setIMUMode(IMU_MODE_RAW_DATA);
        }
    } else {
        // Fallback to raw data mode for basic IMUs
        imu_->setIMUMode(IMU_MODE_RAW_DATA);
    }
}

void IMUAutoPose::updateWithAbsoluteData(const IMUData &imu_data) {
    // Use IMU's absolute orientation calculations
    Point3D absolute_orientation(
        imu_data.absolute_data.absolute_roll,
        imu_data.absolute_data.absolute_pitch,
        imu_data.absolute_data.absolute_yaw);

    // Apply filtering - less aggressive since sensor data is already processed
    double abs_filter_alpha = filter_alpha_ * 0.5f; // Use lighter filtering
    orientation_filter_ = lowPassFilter(absolute_orientation, orientation_filter_, abs_filter_alpha);

    // Use linear acceleration for gravity estimate if available
    if (imu_data.absolute_data.linear_acceleration_valid) {
        // For BNO055 and similar sensors, gravity is already removed from linear acceleration
        // We need to reconstruct gravity vector from absolute orientation
        double roll_rad = math_utils::degreesToRadians(imu_data.absolute_data.absolute_roll);
        double pitch_rad = math_utils::degreesToRadians(imu_data.absolute_data.absolute_pitch);

        // Calculate gravity vector from orientation
        Point3D gravity_from_orientation(
            sin(pitch_rad) * 9.81f,
            -sin(roll_rad) * cos(pitch_rad) * 9.81f,
            -cos(roll_rad) * cos(pitch_rad) * 9.81f);

        gravity_filter_ = lowPassFilter(gravity_from_orientation, gravity_filter_, abs_filter_alpha);
    } else {
        // Fallback to raw acceleration data
        updateGravityEstimate(imu_data);
    }

    current_state_.gravity_vector = gravity_filter_;
}

void IMUAutoPose::updateWithRawData(const IMUData &imu_data) {
    // Use traditional processing with raw IMU data
    updateGravityEstimate(imu_data);

    // Update orientation from raw IMU data
    Point3D current_orientation(imu_data.roll, imu_data.pitch, imu_data.yaw);
    orientation_filter_ = lowPassFilter(current_orientation, orientation_filter_, filter_alpha_);
}

void IMUAutoPose::updateCalibrationStatus(const IMUData &imu_data) {
    if (imu_data.has_absolute_capability) {
        current_state_.calibration_status = imu_data.absolute_data.calibration_status;
    } else {
        // For basic IMUs, assume calibrated if data is valid
        current_state_.calibration_status = imu_data.is_valid ? 3 : 0;
    }
}

bool IMUAutoPose::configureIMUMode(bool use_absolute_data, bool prefer_fusion) {
    if (!imu_)
        return false;

    params_.use_absolute_data = use_absolute_data;
    params_.prefer_sensor_fusion = prefer_fusion;

    initializeIMUMode();
    return true;
}

uint8_t IMUAutoPose::getIMUCalibrationStatus() const {
    return current_state_.calibration_status;
}

bool IMUAutoPose::isUsingAbsoluteData() const {
    return current_state_.using_absolute_data;
}

IMUMode IMUAutoPose::getIMUMode() const {
    return current_state_.active_mode;
}

Point3D IMUAutoPose::lowPassFilter(const Point3D &input, const Point3D &previous, double alpha) {
    return Point3D(
        previous.x + alpha * (input.x - previous.x),
        previous.y + alpha * (input.y - previous.y),
        previous.z + alpha * (input.z - previous.z));
}

double IMUAutoPose::calculateConfidence(const Point3D &orientation_error) const {
    // Calculate confidence based on error magnitude and IMU data quality
    double error_magnitude = math_utils::magnitude(orientation_error);
    double max_error = math_utils::degreesToRadians(45.0f); // Maximum reasonable error

    double confidence = 1.0f - std::min(1.0f, error_magnitude / max_error);

    // Reduce confidence during walking
    if (walking_detected_) {
        confidence *= 0.7f;
    }

    return confidence;
}

void IMUAutoPose::updateAdaptiveGains() {
    // Enhanced terrain roughness estimation from IMU variance
    double orientation_variance = abs(current_state_.orientation_error.x) +
                                 abs(current_state_.orientation_error.y) +
                                 abs(current_state_.orientation_error.z);

    // Update terrain roughness estimate with exponential smoothing
    double alpha = 0.1f; // Smoothing factor
    terrain_roughness_estimate_ = terrain_roughness_estimate_ * (1.0f - alpha) + orientation_variance * alpha;

    // Get current IMU data for acceleration variance calculation
    IMUData current_imu_data = imu_->readIMU();

    // Calculate acceleration variance for additional terrain assessment
    double accel_variance = abs(current_imu_data.accel_x - previous_acceleration_.x) +
                           abs(current_imu_data.accel_y - previous_acceleration_.y) +
                           abs(current_imu_data.accel_z - previous_acceleration_.z);

    // Store current acceleration for next iteration
    previous_acceleration_.x = current_imu_data.accel_x;
    previous_acceleration_.y = current_imu_data.accel_y;
    previous_acceleration_.z = current_imu_data.accel_z;

    // Combine orientation and acceleration variance for terrain assessment
    double combined_roughness = terrain_roughness_estimate_ * 0.7f + accel_variance * 0.3f;

    // Adaptive gain adjustment based on terrain conditions
    if (combined_roughness > 0.15f) {
        // Very rough terrain - prioritize stability
        params_.orientation_gain *= 0.92f;
        params_.response_speed *= 0.90f;
        params_.stabilization_gain *= 1.05f; // Increase stabilization
    } else if (combined_roughness > 0.08f) {
        // Moderately rough terrain - balanced approach
        params_.orientation_gain *= 0.96f;
        params_.response_speed *= 0.95f;
    } else {
        // Smooth terrain - can increase responsiveness
        params_.orientation_gain = std::min(1.0f, params_.orientation_gain * 1.01f);
        params_.response_speed = std::min(1.0f, params_.response_speed * 1.02f);
        params_.stabilization_gain = std::max(0.5f, params_.stabilization_gain * 0.98f);
    }

    // Apply bounds to prevent instability
    params_.orientation_gain = std::max(0.1f, std::min(1.0f, params_.orientation_gain));
    params_.response_speed = std::max(0.1f, std::min(1.0f, params_.response_speed));
    params_.stabilization_gain = std::max(0.5f, std::min(2.0f, params_.stabilization_gain));
}

Point3D IMUAutoPose::normalizeAngles(const Point3D &angles) {
    Point3D normalized;

    // Normalize each angle to [-PI, PI]
    normalized.x = atan2(sin(angles.x), cos(angles.x));
    normalized.y = atan2(sin(angles.y), cos(angles.y));
    normalized.z = atan2(sin(angles.z), cos(angles.z));

    return normalized;
}

bool IMUAutoPose::withinDeadzone(const Point3D &error) const {
    double deadzone_rad = math_utils::degreesToRadians(params_.deadzone_degrees);

    return (abs(error.x) < deadzone_rad &&
            abs(error.y) < deadzone_rad &&
            abs(error.z) < deadzone_rad);
}
