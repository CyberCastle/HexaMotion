#include "body_pose.h"
#include <cmath>

BodyPose::BodyPose()
    : current_mode_(BODY_POSE_MANUAL), reset_mode_(BODY_POSE_RESET_NONE), auto_posing_state_(BODY_POSING_COMPLETE),
      interpolation_speed_(0.1), smooth_transitions_(true), trajectory_in_progress_(false),
      trajectory_progress_(0.0), trajectory_step_count_(0), pose_phase_(0), pose_frequency_(0.0),
      pose_phase_length_(0), normaliser_(1), rotation_absement_error_(Eigen::Vector3d::Zero()),
      rotation_position_error_(Eigen::Vector3d::Zero()), rotation_velocity_error_(Eigen::Vector3d::Zero()),
      translation_velocity_input_(Eigen::Vector3d::Zero()), rotation_velocity_input_(Eigen::Vector3d::Zero()) {
    initialize();
}

BodyPose::BodyPose(const BodyPoseConfig& config)
    : pose_config_(config), current_mode_(BODY_POSE_MANUAL), reset_mode_(BODY_POSE_RESET_NONE),
      auto_posing_state_(BODY_POSING_COMPLETE), interpolation_speed_(0.1), smooth_transitions_(true),
      trajectory_in_progress_(false), trajectory_progress_(0.0), trajectory_step_count_(0),
      pose_phase_(0), pose_frequency_(0.0), pose_phase_length_(0), normaliser_(1),
      rotation_absement_error_(Eigen::Vector3d::Zero()), rotation_position_error_(Eigen::Vector3d::Zero()),
      rotation_velocity_error_(Eigen::Vector3d::Zero()), translation_velocity_input_(Eigen::Vector3d::Zero()),
      rotation_velocity_input_(Eigen::Vector3d::Zero()) {
    initialize();
}

void BodyPose::initialize() {
    initializePoseLimits();
    resetAllPosing();
}

void BodyPose::setCurrentPose(const BodyPoseState& pose) {
    if (validatePoseState(pose)) {
        current_pose_ = pose;
    }
}

void BodyPose::setTargetPose(const BodyPoseState& pose) {
    if (validatePoseState(pose)) {
        target_pose_ = pose;
        if (smooth_transitions_) {
            trajectory_in_progress_ = true;
            trajectory_progress_ = 0.0;
            trajectory_step_count_ = 0;
        }
    }
}

void BodyPose::setPose(const Point3D& position, const Eigen::Vector3d& euler_angles) {
    BodyPoseState new_pose;
    new_pose.position = position;
    new_pose.euler_angles = euler_angles;
    new_pose.use_quaternion = false;

    // Convert Euler angles to quaternion
    Eigen::Quaterniond quat = Eigen::AngleAxisd(euler_angles.z(), Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(euler_angles.y(), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX());
    new_pose.quaternion = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());

    setCurrentPose(new_pose);
}

void BodyPose::setPoseQuaternion(const Point3D& position, const Eigen::Vector4d& quaternion) {
    BodyPoseState new_pose;
    new_pose.position = position;
    new_pose.quaternion = quaternion;
    new_pose.use_quaternion = true;

    // Convert quaternion to Euler angles
    Eigen::Quaterniond quat(quaternion(0), quaternion(1), quaternion(2), quaternion(3));
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    new_pose.euler_angles = euler;

    setCurrentPose(new_pose);
}

void BodyPose::setPoseMode(BodyPoseMode mode) {
    current_mode_ = mode;
}

void BodyPose::setResetMode(BodyPoseResetMode mode) {
    reset_mode_ = mode;
}

void BodyPose::setManualPoseInput(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation) {
    translation_velocity_input_ = translation;
    rotation_velocity_input_ = rotation;
}

void BodyPose::getManualPoseInput(Eigen::Vector3d& translation, Eigen::Vector3d& rotation) const {
    translation = translation_velocity_input_;
    rotation = rotation_velocity_input_;
}

void BodyPose::resetAllPosing() {
    current_pose_ = BodyPoseState();
    target_pose_ = BodyPoseState();
    translation_velocity_input_ = Eigen::Vector3d::Zero();
    rotation_velocity_input_ = Eigen::Vector3d::Zero();
    trajectory_in_progress_ = false;
    trajectory_progress_ = 0.0;
    trajectory_step_count_ = 0;
}

void BodyPose::updatePoseInterpolation(double dt) {
    if (!trajectory_in_progress_ || !smooth_transitions_) {
        return;
    }

    trajectory_progress_ += interpolation_speed_ * dt;
    trajectory_step_count_++;

    if (trajectory_progress_ >= 1.0) {
        current_pose_ = target_pose_;
        trajectory_in_progress_ = false;
        trajectory_progress_ = 1.0;
    } else {
        // Interpolate position
        current_pose_.position = current_pose_.position * (1.0 - trajectory_progress_) +
                                target_pose_.position * trajectory_progress_;

        // Interpolate orientation using quaternion slerp
        Eigen::Quaterniond start_quat(current_pose_.quaternion(0), current_pose_.quaternion(1),
                                     current_pose_.quaternion(2), current_pose_.quaternion(3));
        Eigen::Quaterniond end_quat(target_pose_.quaternion(0), target_pose_.quaternion(1),
                                   target_pose_.quaternion(2), target_pose_.quaternion(3));

        Eigen::Quaterniond interpolated_quat = start_quat.slerp(trajectory_progress_, end_quat);
        current_pose_.quaternion = Eigen::Vector4d(interpolated_quat.w(), interpolated_quat.x(),
                                                  interpolated_quat.y(), interpolated_quat.z());

        // Update Euler angles from interpolated quaternion
        Eigen::Vector3d euler = interpolated_quat.toRotationMatrix().eulerAngles(0, 1, 2);
        current_pose_.euler_angles = euler;
    }
}

void BodyPose::resetTrajectory() {
    trajectory_in_progress_ = false;
    trajectory_progress_ = 0.0;
    trajectory_step_count_ = 0;
}

void BodyPose::setSmoothTransitions(bool enable, double speed) {
    smooth_transitions_ = enable;
    interpolation_speed_ = std::max(0.01, std::min(1.0, speed));
}

void BodyPose::setPoseConfig(const BodyPoseConfig& config) {
    pose_config_ = config;
}

void BodyPose::setPoseLimits(const BodyPoseLimits& limits) {
    pose_limits_ = limits;
}

Pose BodyPose::toPose() const {
    Eigen::Quaterniond quat(current_pose_.quaternion(0), current_pose_.quaternion(1),
                           current_pose_.quaternion(2), current_pose_.quaternion(3));
    return Pose(current_pose_.position, quat);
}

BodyPose BodyPose::fromPose(const Pose& pose) {
    BodyPose body_pose;
    BodyPoseState state;
    state.position = pose.position;
    state.quaternion = Eigen::Vector4d(pose.rotation.w(), pose.rotation.x(),
                                      pose.rotation.y(), pose.rotation.z());
    state.use_quaternion = true;

    Eigen::Vector3d euler = pose.rotation.toRotationMatrix().eulerAngles(0, 1, 2);
    state.euler_angles = euler;

    body_pose.setCurrentPose(state);
    return body_pose;
}

bool BodyPose::checkPoseLimits(const BodyPoseState& pose) const {
    // Check translation limits
    if (std::abs(pose.position.x) > pose_limits_.translation_limits.x ||
        std::abs(pose.position.y) > pose_limits_.translation_limits.y ||
        std::abs(pose.position.z) > pose_limits_.translation_limits.z) {
        return false;
    }

    // Check rotation limits
    if (std::abs(pose.euler_angles.x()) > pose_limits_.rotation_limits.x() ||
        std::abs(pose.euler_angles.y()) > pose_limits_.rotation_limits.y() ||
        std::abs(pose.euler_angles.z()) > pose_limits_.rotation_limits.z()) {
        return false;
    }

    // Check height limits
    if (pose.height < pose_limits_.height_min || pose.height > pose_limits_.height_max) {
        return false;
    }

    return true;
}

void BodyPose::constrainPose(BodyPoseState& pose) const {
    // Constrain translation
    pose.position.x = std::max(-pose_limits_.translation_limits.x,
                               std::min(pose_limits_.translation_limits.x, pose.position.x));
    pose.position.y = std::max(-pose_limits_.translation_limits.y,
                               std::min(pose_limits_.translation_limits.y, pose.position.y));
    pose.position.z = std::max(-pose_limits_.translation_limits.z,
                               std::min(pose_limits_.translation_limits.z, pose.position.z));

    // Constrain rotation
    pose.euler_angles.x() = std::max(-pose_limits_.rotation_limits.x(),
                                    std::min(pose_limits_.rotation_limits.x(), pose.euler_angles.x()));
    pose.euler_angles.y() = std::max(-pose_limits_.rotation_limits.y(),
                                    std::min(pose_limits_.rotation_limits.y(), pose.euler_angles.y()));
    pose.euler_angles.z() = std::max(-pose_limits_.rotation_limits.z(),
                                    std::min(pose_limits_.rotation_limits.z(), pose.euler_angles.z()));

    // Constrain height
    pose.height = std::max(pose_limits_.height_min,
                          std::min(pose_limits_.height_max, pose.height));
}

void BodyPose::setAutoPosingState(BodyPosingState state) {
    auto_posing_state_ = state;
}

void BodyPose::setPosePhase(int phase) {
    pose_phase_ = phase;
}

void BodyPose::setPoseFrequency(double frequency) {
    pose_frequency_ = frequency;
}

void BodyPose::setPosePhaseLength(int length) {
    pose_phase_length_ = length;
}

void BodyPose::setNormaliser(int normaliser) {
    normaliser_ = normaliser;
}

void BodyPose::setRotationAbsementError(const Eigen::Vector3d& error) {
    rotation_absement_error_ = error;
}

void BodyPose::setRotationPositionError(const Eigen::Vector3d& error) {
    rotation_position_error_ = error;
}

void BodyPose::setRotationVelocityError(const Eigen::Vector3d& error) {
    rotation_velocity_error_ = error;
}

void BodyPose::initializePoseLimits() {
    // Default limits based on robot characteristics
    pose_limits_.translation_limits = Point3D(0.1, 0.1, 0.05);  // ±10cm X/Y, ±5cm Z
    pose_limits_.rotation_limits = Eigen::Vector3d(0.3, 0.3, 0.5);  // ±17° roll/pitch, ±29° yaw
    pose_limits_.height_min = 80.0;   // 80mm minimum height
    pose_limits_.height_max = 150.0;  // 150mm maximum height
    pose_limits_.max_translation_velocity = 0.1;  // 10cm/s
    pose_limits_.max_rotation_velocity = 0.5;     // 0.5 rad/s
}

bool BodyPose::validatePoseState(const BodyPoseState& pose) const {
    // Check for NaN values
    if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z) ||
        std::isnan(pose.euler_angles.x()) || std::isnan(pose.euler_angles.y()) || std::isnan(pose.euler_angles.z()) ||
        std::isnan(pose.quaternion(0)) || std::isnan(pose.quaternion(1)) ||
        std::isnan(pose.quaternion(2)) || std::isnan(pose.quaternion(3)) ||
        std::isnan(pose.height)) {
        return false;
    }

    // Check quaternion normalization
    double quat_norm = pose.quaternion.norm();
    if (std::abs(quat_norm - 1.0) > 0.01) {
        return false;
    }

    return true;
}