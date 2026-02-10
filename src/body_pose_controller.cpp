#include "body_pose_controller.h"
#include "body_pose_config_factory.h"
#include "gait_types.h"
#include "hexamotion_constants.h"
#include "leg_poser.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * @file body_pose_controller.cpp
 * @brief Implementation of the body pose controller for HexaMotion
 *
 * This implementation is adapted from OpenSHC's PoseController but simplified for HexaMotion architecture.
 * No progress tracking - that's handled by LocomotionSystem.
 */

// Internal implementation class to avoid circular dependencies
class BodyPoseController::LegPoserImpl {
  public:
    LegPoserImpl(int leg_index, Leg &leg, RobotModel &model)
        : poser_(leg_index, leg, model) {}

    LegPoser *get() { return &poser_; }
    const LegPoser *get() const { return &poser_; }

  private:
    LegPoser poser_;
};

// Definition of static constexpr member declared in header
constexpr int BodyPoseController::tripod_leg_groups[2][3];

BodyPoseController::BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config)
    : model(m), body_pose_config(config), auto_pose_enabled(false), current_gait_type_(TRIPOD_GAIT),
      trajectory_in_progress(false), trajectory_progress(0.0), trajectory_step_count(0),
      step_to_new_stance_current_group(0), step_to_new_stance_sequence_generated(false),
      shutdown_sequence_initialized(false),
      // OpenSHC state variables initialization
      executing_transition_(false), transition_step_(0), transition_step_count_(0),
      set_target_(true), proximity_alert_(false), horizontal_transition_complete_(false),
      vertical_transition_complete_(false), first_sequence_execution_(true),
      reset_transition_sequence_(true), legs_completed_step_(0), current_group_(0),
      pack_step_(0) {

    // Initialize leg posers
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_posers_[i] = nullptr;
    }

    // Initialize trajectory arrays
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = Point3D(0, 0, 0);
        trajectory_start_angles[i] = JointAngles(0, 0, 0);
        trajectory_target_positions[i] = Point3D(0, 0, 0);
        trajectory_target_angles[i] = JointAngles(0, 0, 0);
    }

    // Initialize walk plane pose system (OpenSHC equivalent with Bézier curves)
    walk_plane_pose_ = Pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
    walk_plane_pose_enabled = false;
    walk_plane_update_threshold = 1.0; // 1mm threshold

    // Initialize Bézier curve control system
    walk_plane_bezier_in_progress = false;
    walk_plane_bezier_time = 0.0;
    walk_plane_bezier_duration = 1.0; // 1 second transition time

    // Initialize control nodes arrays
    for (int i = 0; i < 5; i++) {
        walk_plane_position_nodes[i] = Point3D(0.0, 0.0, body_pose_config.body_clearance);
        walk_plane_rotation_nodes[i] = Eigen::Quaterniond::Identity();
    }

    // Initialize auto-pose configuration (default: tripod)
    std::string gait_name = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    auto_pose_config = createAutoPoseConfigurationForGait(model.getParams(), gait_name);

    // Default pose reference (identity)
    default_pose_ = Pose::Identity();
}

void BodyPoseController::setManualPoseInput(const Point3D &translation, const Point3D &rotation) {
    Point3D clamped_translation = translation;
    clamped_translation.x = math_utils::clamp(clamped_translation.x, -body_pose_config.max_translation.x, body_pose_config.max_translation.x);
    clamped_translation.y = math_utils::clamp(clamped_translation.y, -body_pose_config.max_translation.y, body_pose_config.max_translation.y);
    clamped_translation.z = math_utils::clamp(clamped_translation.z, -body_pose_config.max_translation.z, body_pose_config.max_translation.z);

    Point3D clamped_rotation = rotation;
    clamped_rotation.x = math_utils::clamp(clamped_rotation.x, -body_pose_config.max_rotation.roll, body_pose_config.max_rotation.roll);
    clamped_rotation.y = math_utils::clamp(clamped_rotation.y, -body_pose_config.max_rotation.pitch, body_pose_config.max_rotation.pitch);
    clamped_rotation.z = math_utils::clamp(clamped_rotation.z, -body_pose_config.max_rotation.yaw, body_pose_config.max_rotation.yaw);

    manual_pose_.position = clamped_translation;
    manual_pose_.rotation = Eigen::Quaterniond(math_utils::eulerPoint3DToQuaternion(clamped_rotation));
}

void BodyPoseController::setIMUData(const IMUData &imu_data) {
    imu_data_ = imu_data;
    imu_data_valid_ = imu_data.is_valid;
}

// -------------------------------------------------------------------------------------------------
// NOTE OpenSHC PoseController::updateCurrentPose adaptation status
// Implemented here:
//  - Walk plane pose estimation (height + normal) with Bézier smoothing and clearance integration.
//  - Global body pose composition currently == walk_plane_pose_ (stored in body_pose_current_).
//  - Global body pose application (rotation + translation) to desired tip positions prior to per‑leg auto pose.
//  - Auto pose per‑leg modulation plus stance‑leg averaged global_auto_pose_ (removal/addition ordering mirrors OpenSHC).
//
// Missing / Deferred relative to full OpenSHC PoseController capabilities:
//  - IMU-based gravity / inclination fusion (imu_pose_) for continuous leveling.
//  - Manual / external body pose input filtering & interpolation inside updateCurrentPose (currently externalised).
//  - Pose reset / recovery and progress tracking sequences (startup/shutdown progress only partially elsewhere).
//  - Dynamic stiffness / compliance modulation tied to pose transitions.
//  - Chained multi-layer pose composition (manual -> imu -> walk plane -> auto -> tip alignment orientation).
//  - Advanced quaternion averaging (current incremental SLERP adequate for small auto pose deltas only).
//  - Per-leg asymmetric stance weighting / adaptive exclusion for slipping or faulted legs.
//  - Velocity & acceleration limiting of body pose changes beyond Bézier walk plane smoothing.
//  - Tip orientation alignment / end-effector rotational modulation (only positional offsets applied now).
//
// Rationale: Non-critical layers (IMU, manual commands, admittance) are centralised in LocomotionSystem or dedicated modules
// to keep this controller focused on geometric walk plane maintenance plus auto pose synthesis. Items above remain TODOs for
// future parity with full OpenSHC if required.
void BodyPoseController::updateCurrentPose(double gait_phase, Leg legs[NUM_LEGS]) {
    // Keep walk plane pose coherent with current stance distribution.
    updateWalkPlanePose(legs);

    // Compose global body pose (OpenSHC order: walk plane -> manual -> inclination -> IMU).
    Pose new_pose = Pose::Identity();
    new_pose = new_pose.addPose(walk_plane_pose_);

    if (manual_pose_enabled_) {
        new_pose = new_pose.addPose(manual_pose_);
    }

    if (inclination_pose_enabled_ && imu_data_valid_) {
        Eigen::Vector3d imu_euler = Eigen::Vector3d::Zero();
        if (imu_data_.absolute_data.absolute_orientation_valid) {
            imu_euler = Eigen::Vector3d(math_utils::degreesToRadians(imu_data_.absolute_data.absolute_roll),
                                        math_utils::degreesToRadians(imu_data_.absolute_data.absolute_pitch),
                                        math_utils::degreesToRadians(imu_data_.absolute_data.absolute_yaw));
        } else {
            imu_euler = Eigen::Vector3d(math_utils::degreesToRadians(imu_data_.roll),
                                        math_utils::degreesToRadians(imu_data_.pitch),
                                        math_utils::degreesToRadians(imu_data_.yaw));
        }

        double longitudinal = -body_pose_config.body_clearance * std::tan(imu_euler.y());
        double lateral = body_pose_config.body_clearance * std::tan(imu_euler.x());
        longitudinal = math_utils::clamp(longitudinal, -body_pose_config.max_translation.x, body_pose_config.max_translation.x);
        lateral = math_utils::clamp(lateral, -body_pose_config.max_translation.y, body_pose_config.max_translation.y);
        inclination_pose_.position = Point3D(longitudinal, lateral, 0.0);
        inclination_pose_.rotation = Eigen::Quaterniond::Identity();
        new_pose = new_pose.addPose(inclination_pose_);
    }

    if (imu_pose_enabled_ && imu_data_valid_) {
        updateIMUPosePID();
        new_pose = new_pose.addPose(imu_pose_);
    }

    // Tip alignment pose (experimental, 3DOF legs only) - OpenSHC equivalent
    if (tip_align_pose_enabled_) {
        updateTipAlignPose(legs);
        new_pose = new_pose.addPose(tip_align_pose_);
    }

    // IK error compensation pose - OpenSHC equivalent
    if (ik_error_pose_enabled_) {
        updateIKErrorPose(legs);
        new_pose = new_pose.addPose(ik_error_pose_);
    }

    // Default (zero-moment) pose for balance during leg manipulation
    if (default_pose_enabled_) {
        calculateDefaultPose(legs);
    }

    body_pose_current_ = new_pose;

    // Update (but do NOT yet apply) auto-pose patterning. We aggregate into global_auto_pose_
    // and per-leg posers; actual spatial effect on desired tip positions happens in
    // applyAutoPoseToDesiredTips() just before IK (mirrors OpenSHC ordering: compose then apply).
    if (auto_pose_enabled && auto_pose_config.enabled) {
        // Run phase update => populates each leg poser auto_pose_ (negated windows) and computes base amplitudes.
        updateAutoPose(gait_phase, legs);
        // Reconstruct a global base auto pose by averaging per‑leg auto poses of stance legs (OpenSHC analogue).
        // Rationale: In OpenSHC, auto_pose_ aggregates multiple AutoPoser outputs before leg-specific negations.
        // Here each LegPoser holds its own per-leg pose; we approximate the shared component by averaging stance legs.
        int count = 0;
        Eigen::Vector3d accum_pos(0, 0, 0);
        std::vector<Eigen::Quaterniond> quats;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!leg_posers_[i])
                continue;
            if (legs[i].getStepPhase() != STANCE_PHASE)
                continue; // only stance legs contribute to stable body estimate
            Pose lp_pose = leg_posers_[i]->get()->getAutoPose();
            accum_pos += Eigen::Vector3d(lp_pose.position.x, lp_pose.position.y, lp_pose.position.z);
            quats.push_back(lp_pose.rotation);
            ++count;
        }
        if (count == 0) { // fallback: use all legs if no stance legs (rare edge during full aerial phase)
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (!leg_posers_[i])
                    continue;
                Pose lp_pose = leg_posers_[i]->get()->getAutoPose();
                accum_pos += Eigen::Vector3d(lp_pose.position.x, lp_pose.position.y, lp_pose.position.z);
                quats.push_back(lp_pose.rotation);
                ++count;
            }
        }
        if (count > 0) {
            Eigen::Vector3d avg_pos = accum_pos / static_cast<double>(count);
            // Quaternion averaging (incremental slerp) – adequate for small pose deviations typical of auto pose.
            Eigen::Quaterniond avg_q = Eigen::Quaterniond::Identity();
            int qi = 0;
            for (const auto &q : quats) {
                if (qi == 0) {
                    avg_q = q;
                } else {
                    double w = 1.0 / static_cast<double>(qi + 1);
                    avg_q = avg_q.slerp(w, q);
                }
                ++qi;
            }
            global_auto_pose_.position = Point3D(avg_pos.x(), avg_pos.y(), avg_pos.z());
            global_auto_pose_.rotation = avg_q.normalized();
        } else {
            global_auto_pose_ = Pose::Identity();
        }
    } else {
        global_auto_pose_ = Pose::Identity();
    }
}

// ================================================================================================
// OpenSHC PID-based IMU posing (PoseController::updateIMUPose equivalent)
// ================================================================================================
void BodyPoseController::updateIMUPosePID() {
    // Obtain current IMU orientation as quaternion
    Eigen::Quaterniond current_rotation = Eigen::Quaterniond::Identity();
    if (imu_data_.absolute_data.absolute_orientation_valid) {
        Eigen::Vector3d euler_rad(math_utils::degreesToRadians(imu_data_.absolute_data.absolute_roll),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_pitch),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_yaw));
        current_rotation = math_utils::eulerToQuaternion(euler_rad);
    } else {
        Eigen::Vector3d euler_rad(math_utils::degreesToRadians(imu_data_.roll),
                                  math_utils::degreesToRadians(imu_data_.pitch),
                                  math_utils::degreesToRadians(imu_data_.yaw));
        current_rotation = math_utils::eulerToQuaternion(euler_rad);
    }
    current_rotation = math_utils::correctRotation(current_rotation, Eigen::Quaterniond::Identity());

    // Target rotation from manual pose component
    Eigen::Quaterniond target_rotation = math_utils::correctRotation(manual_pose_.rotation,
                                                                     Eigen::Quaterniond::Identity());

    // Rotation error: current * target^-1
    Eigen::Quaterniond rotation_error = (current_rotation * target_rotation.inverse()).normalized();

    // PID gains from parameters
    double kp = model.getParams().body_comp.imu_pid_kp;
    double ki = model.getParams().body_comp.imu_pid_ki;
    double kd = model.getParams().body_comp.imu_pid_kd;
    double time_delta = model.getParams().time_delta;

    // Position error (euler angles) - zero yaw compensation
    rotation_position_error_ = math_utils::quaterniondToEulerAngles(rotation_error);
    rotation_position_error_[2] = 0.0;

    // IMU posing deadband (OpenSHC default 0.0 radians)
    constexpr double IMU_POSING_DEADBAND = 0.0;
    if (rotation_position_error_.norm() < IMU_POSING_DEADBAND) {
        return;
    }

    // Integrate position error to absement
    rotation_absement_error_ += rotation_position_error_ * time_delta;

    // Low-pass filter of IMU angular velocity data (OpenSHC smoothing_factor = 0.15)
    constexpr double smoothing_factor = 0.15;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    if (imu_data_.absolute_data.absolute_orientation_valid) {
        // Use gyroscope data directly (deg/s -> rad/s)
        angular_velocity = Eigen::Vector3d(math_utils::degreesToRadians(imu_data_.gyro_x),
                                           math_utils::degreesToRadians(imu_data_.gyro_y),
                                           math_utils::degreesToRadians(imu_data_.gyro_z));
    } else {
        angular_velocity = Eigen::Vector3d(math_utils::degreesToRadians(imu_data_.gyro_x),
                                           math_utils::degreesToRadians(imu_data_.gyro_y),
                                           math_utils::degreesToRadians(imu_data_.gyro_z));
    }
    rotation_velocity_error_ = smoothing_factor * (-angular_velocity) +
                               (1.0 - smoothing_factor) * rotation_velocity_error_;

    // PID correction
    Eigen::Vector3d rotation_correction = -(kd * rotation_velocity_error_ +
                                            kp * rotation_position_error_ +
                                            ki * rotation_absement_error_);

    // Clamp within rotation limits
    double max_roll = body_pose_config.max_rotation.roll;
    double max_pitch = body_pose_config.max_rotation.pitch;
    if (max_roll > 0.0) {
        rotation_correction[0] = math_utils::clamp(rotation_correction[0], -max_roll, max_roll);
    }
    if (max_pitch > 0.0) {
        rotation_correction[1] = math_utils::clamp(rotation_correction[1], -max_pitch, max_pitch);
    }
    // Preserve target yaw (no compensation in yaw)
    Eigen::Vector3d target_euler = math_utils::quaterniondToEulerAngles(target_rotation);
    rotation_correction[2] = target_euler[2];

    // Stability check (OpenSHC STABILITY_THRESHOLD = 100)
    constexpr double STABILITY_THRESHOLD = 100.0;
    if (rotation_correction.norm() > STABILITY_THRESHOLD) {
        // IMU compensation unstable - reset PID state
        rotation_absement_error_ = Eigen::Vector3d::Zero();
        rotation_velocity_error_ = Eigen::Vector3d::Zero();
        return;
    }

    imu_pose_.rotation = math_utils::eulerAnglesToQuaterniond(rotation_correction);
    imu_pose_.rotation = math_utils::correctRotation(imu_pose_.rotation, target_rotation);
    imu_pose_.position = Point3D(0.0, 0.0, 0.0);
}

// ================================================================================================
// OpenSHC calculateDefaultPose equivalent - zero-moment body offset
// ================================================================================================
void BodyPoseController::calculateDefaultPose(Leg legs[NUM_LEGS]) {
    int legs_loaded = 0;
    int legs_transitioning = 0;

    for (int i = 0; i < NUM_LEGS; ++i) {
        LegState state = legs[i].getLegState();
        if (state == LEG_WALKING || state == LEG_MANUAL_TO_WALKING) {
            legs_loaded++;
        }
        if (state == LEG_MANUAL_TO_WALKING || state == LEG_WALKING_TO_MANUAL) {
            legs_transitioning++;
        }
    }

    // Only recalculate when legs are transitioning states
    if (legs_transitioning != 0) {
        if (recalculate_default_pose_) {
            Eigen::Vector3d zero_moment_offset = Eigen::Vector3d::Zero();

            for (int i = 0; i < NUM_LEGS; ++i) {
                LegState state = legs[i].getLegState();
                if (state == LEG_WALKING || state == LEG_MANUAL_TO_WALKING) {
                    // Use default stance positions for zero-moment calculation
                    const auto &stance = body_pose_config.leg_stance_positions[i];
                    zero_moment_offset[0] += stance.x;
                    zero_moment_offset[1] += stance.y;
                }
            }

            if (legs_loaded > 0) {
                zero_moment_offset /= static_cast<double>(legs_loaded);
            }

            if (body_pose_config.max_translation.x > 0.0) {
                zero_moment_offset[0] = math_utils::clamp(zero_moment_offset[0],
                                                          -body_pose_config.max_translation.x, body_pose_config.max_translation.x);
            }
            if (body_pose_config.max_translation.y > 0.0) {
                zero_moment_offset[1] = math_utils::clamp(zero_moment_offset[1],
                                                          -body_pose_config.max_translation.y, body_pose_config.max_translation.y);
            }

            default_pose_.position = Point3D(zero_moment_offset[0], zero_moment_offset[1], 0.0);
            recalculate_default_pose_ = false;
        }
    } else {
        recalculate_default_pose_ = true;
    }
}

// ================================================================================================
// OpenSHC updateTipAlignPose equivalent - align final joint with tip along walk plane normal
// ================================================================================================
void BodyPoseController::updateTipAlignPose(Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i])
            continue;

        // Check if leg is in swing phase with valid progress
        double swing_progress = legs[i].getSwingProgress();
        if (swing_progress < 0.0 || swing_progress > 1.0)
            continue;

        // Walk plane normal (default: vertical)
        Point3D walk_plane_normal_pt = Point3D(0.0, 0.0, 1.0);
        // If walk plane pose is enabled, extract normal from rotation
        if (walk_plane_pose_enabled) {
            Eigen::Vector3d unit_z(0.0, 0.0, 1.0);
            Eigen::Vector3d normal = walk_plane_pose_.rotation * unit_z;
            walk_plane_normal_pt = Point3D(normal.x(), normal.y(), normal.z());
        }
        Eigen::Vector3d walk_plane_normal(walk_plane_normal_pt.x, walk_plane_normal_pt.y, walk_plane_normal_pt.z);
        Eigen::Quaterniond walk_plane_rotation = Eigen::Quaterniond::FromTwoVectors(
            Eigen::Vector3d::UnitZ(), walk_plane_normal);

        // Calculate vector from tip position to last joint position
        // For 3DOF leg: last joint is tibia pivot, tip is foot
        Point3D tip_pos = legs[i].getCurrentTipPositionGlobal();
        // Approximate joint position using FK: tip position + tibia_length * up
        double tibia_length = model.getParams().tibia_length;
        JointAngles angles = legs[i].getJointAngles();
        double base_theta = math_utils::radiansToDegrees(BASE_THETA_OFFSETS[i]);
        // Simplified: compute tibia joint position from tip + tibia vector
        double femur_angle_rad = math_utils::degreesToRadians(angles.femur);
        double tibia_angle_rad = math_utils::degreesToRadians(angles.tibia);
        double leg_angle = math_utils::degreesToRadians(base_theta + angles.coxa);
        double z_component = tibia_length * std::cos(femur_angle_rad + tibia_angle_rad);
        double h_component = tibia_length * std::sin(femur_angle_rad + tibia_angle_rad);
        Eigen::Vector3d tip_to_joint(
            -h_component * std::cos(leg_angle),
            -h_component * std::sin(leg_angle),
            z_component);
        double link_length = tip_to_joint.norm();

        // Body translation to align joint inline with tip along walk plane normal (OpenSHC algorithm)
        Eigen::Vector3d a = walk_plane_rotation._transformVector(tip_to_joint);
        Eigen::Vector3d b = link_length * walk_plane_normal;
        Eigen::Vector3d rejection = a - (a.dot(b) / b.dot(b)) * b;
        Eigen::Vector3d translation_to_alignment = -rejection;

        // Component of current translation aligned with walk plane
        a = Eigen::Vector3d(tip_align_pose_.position.x, tip_align_pose_.position.y, tip_align_pose_.position.z);
        b = walk_plane_normal;
        rejection = a - (a.dot(b) / b.dot(b)) * b;
        Eigen::Vector3d current_walk_plane_aligned = rejection;

        Eigen::Vector3d target_translation = current_walk_plane_aligned + translation_to_alignment;

        // Clamp within limits
        if (body_pose_config.max_translation.x > 0.0) {
            target_translation[0] = math_utils::clamp(target_translation[0],
                                                      -body_pose_config.max_translation.x, body_pose_config.max_translation.x);
        }
        if (body_pose_config.max_translation.y > 0.0) {
            target_translation[1] = math_utils::clamp(target_translation[1],
                                                      -body_pose_config.max_translation.y, body_pose_config.max_translation.y);
        }
        if (body_pose_config.max_translation.z > 0.0) {
            target_translation[2] = math_utils::clamp(target_translation[2],
                                                      -body_pose_config.max_translation.z, body_pose_config.max_translation.z);
        }

        // Interpolate with smoothStep (OpenSHC algorithm: 1st half ramp down, 2nd half ramp up)
        double c = math_utils::smoothStep(swing_progress);
        if (swing_progress < 0.5) {
            double t = math_utils::smoothStep(c * 2.0);
            // Interpolate from origin_tip_align_pose_ to identity
            tip_align_pose_.position = Point3D(
                origin_tip_align_pose_.position.x * (1.0 - t),
                origin_tip_align_pose_.position.y * (1.0 - t),
                origin_tip_align_pose_.position.z * (1.0 - t));
            tip_align_pose_.rotation = Eigen::Quaterniond::Identity().slerp(
                1.0 - t, origin_tip_align_pose_.rotation);
        } else {
            double t = math_utils::smoothStep((c - 0.5) * 2.0);
            // Interpolate from identity to target
            tip_align_pose_.position = Point3D(
                target_translation[0] * t,
                target_translation[1] * t,
                target_translation[2] * t);
            tip_align_pose_.rotation = Eigen::Quaterniond::Identity();
        }

        // Save origin for next swing period interpolation
        if (swing_progress >= 1.0) {
            origin_tip_align_pose_ = tip_align_pose_;
        }
    }
}

// ================================================================================================
// OpenSHC updateIKErrorPose equivalent - compensate IK position errors
// ================================================================================================
void BodyPoseController::updateIKErrorPose(Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        // Only compensate for walking legs
        if (legs[i].getLegState() != LEG_WALKING)
            continue;

        Point3D current_tip = legs[i].getCurrentTipPositionGlobal();
        Point3D desired_tip = legs[i].getDesiredTipPosition();
        Eigen::Vector3d ik_error(current_tip.x - desired_tip.x,
                                 current_tip.y - desired_tip.y,
                                 current_tip.z - desired_tip.z);
        ik_error_pose_.position.x -= ik_error.x();
        ik_error_pose_.position.y -= ik_error.y();
        ik_error_pose_.position.z -= ik_error.z();
    }
    // Decay back to zero (OpenSHC factor: 0.95)
    ik_error_pose_.position.x *= 0.95;
    ik_error_pose_.position.y *= 0.95;
    ik_error_pose_.position.z *= 0.95;
}

void BodyPoseController::applyAutoPoseToDesiredTips(Leg legs[NUM_LEGS]) {
    if (!auto_pose_enabled || !auto_pose_config.enabled)
        return;
    // Remove global pose then add per-leg pose, equivalent to OpenSHC updateStance() logic.
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D raw = legs[i].getDesiredTipPosition();
        Pose raw_pose(raw, Eigen::Quaterniond::Identity());
        Pose posed = raw_pose.removePose(global_auto_pose_);
        if (leg_posers_[i]) {
            posed = posed.addPose(leg_posers_[i]->get()->getAutoPose());
        }
        // Write back only position (orientation ignored by current IK path).
        legs[i].setDesiredTipPosition(posed.position);
    }
}

void BodyPoseController::applyGlobalBodyPoseToDesiredTips(Leg legs[NUM_LEGS]) {
    // Apply translation + rotation of current composed body pose to desired tip positions prior to per‑leg auto pose.
    // Morphology note (AGENTS.md): Default stance has tibia vertical and body reference at z = -tibia_length.
    // Desired tip Z already embeds clearance; to avoid double counting we offset Z by (body_pose_current_.position.z - body_pose_config.body_clearance).
    Eigen::Vector3d translation(body_pose_current_.position.x, body_pose_current_.position.y, body_pose_current_.position.z - body_pose_config.body_clearance);
    Eigen::Quaterniond rotation = body_pose_current_.rotation;
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D p = legs[i].getDesiredTipPosition();
        Eigen::Vector3d v(p.x, p.y, p.z);
        Eigen::Vector3d rotated = rotation * v;
        Eigen::Vector3d transformed = rotated + translation;
        legs[i].setDesiredTipPosition(Point3D(transformed.x(), transformed.y(), transformed.z()));
    }
}

BodyPoseController::~BodyPoseController() {
    // Clean up leg posers
    for (int i = 0; i < NUM_LEGS; i++) {
        delete leg_posers_[i];
        leg_posers_[i] = nullptr;
    }
}

void BodyPoseController::initializeLegPosers(Leg legs[NUM_LEGS]) {
    // Initialize LegPoser instances for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Delete existing poser if any
        delete leg_posers_[i];
        // Create LegPoser with the corresponding leg
        leg_posers_[i] = new LegPoserImpl(i, legs[i], model);
    }
}

LegPoser *BodyPoseController::getLegPoser(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS && leg_posers_[leg_index]) {
        return leg_posers_[leg_index]->get();
    }
    return nullptr;
}

bool BodyPoseController::setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                     Leg legs[NUM_LEGS]) {
    // orientation is already in radians (roll,pitch,yaw)
    if (!checkBodyPoseLimits(position, orientation)) {
        return false;
    }
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation) {
        return setBodyPoseSmooth(position, orientation, legs);
    }
    return setBodyPoseImmediate(position, orientation, legs);
}

bool BodyPoseController::setLegPosition(int leg_index, const Point3D &position, Leg legs[NUM_LEGS]) {
    // Use current joint angles as starting point for IK (OpenSHC approach)
    JointAngles current_angles = legs[leg_index].getJointAngles();
    JointAngles angles = model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    // Update leg with new joint angles and calculated position
    legs[leg_index].setJointAngles(angles);
    Point3D calculated_position = model.forwardKinematicsGlobalCoordinates(leg_index, angles);
    legs[leg_index].setCurrentTipPositionGlobal(calculated_position);

    return true;
}

bool BodyPoseController::calculateBodyPoseFromConfig(double height_offset, Leg legs[NUM_LEGS]) {
    // Update walk plane pose with current leg positions
    updateWalkPlanePose(legs);

    // Calculate target Z position using walk plane pose
    double target_z;
    if (walk_plane_pose_enabled) {
        // Use walk plane pose height (already includes body clearance)
        target_z = walk_plane_pose_.position.z + height_offset;
    } else {
        // Fallback to traditional body clearance calculation
        target_z = -(body_pose_config.body_clearance + height_offset);
    }

    // Use configured stance positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Get stance position from configuration
        double stance_x_mm = body_pose_config.leg_stance_positions[i].x;
        double stance_y_mm = body_pose_config.leg_stance_positions[i].y;

        Point3D target_pos;
        target_pos.x = stance_x_mm;
        target_pos.y = stance_y_mm;
        target_pos.z = target_z;

        // Apply walk plane pose rotation if enabled
        if (walk_plane_pose_enabled) {
            // Transform leg position by walk plane pose rotation
            Eigen::Vector3d leg_vector(target_pos.x, target_pos.y, target_pos.z);
            Eigen::Vector3d rotated_vector = walk_plane_pose_.rotation * leg_vector;
            target_pos.x = rotated_vector.x();
            target_pos.y = rotated_vector.y();
            target_pos.z = rotated_vector.z();
        }

        // Use LegPoser if available for smooth movement
        if (getLegPoser(i)) {
            setLegPosition(i, target_pos, legs);
        } else {
            // Fallback to direct calculation
            JointAngles current_angles = legs[i].getJointAngles();
            JointAngles angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, target_pos);

            if (!model.checkJointLimits(i, angles)) {
                return false;
            }

            legs[i].setCurrentTipPositionGlobal(target_pos);
            legs[i].setJointAngles(angles);
            legs[i].setCurrentTipPositionGlobal(target_pos);
        }
    }

    return true;
}

void BodyPoseController::initializeDefaultPose(Leg legs[NUM_LEGS]) {
    // Initialize LegPosers if not already done
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // Use configuration-based calculation instead of direct geometry
    calculateBodyPoseFromConfig(0.0, legs);
}

bool BodyPoseController::setStandingPose(Leg legs[NUM_LEGS]) {
    // Initialize LegPosers if not already done
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // Apply configured standing pose joint angles for each leg
    for (int i = 0; i < NUM_LEGS; ++i) {
        const auto &standing_joints = body_pose_config.standing_pose_joints[i];

        JointAngles angles;
        angles.coxa = standing_joints.coxa;
        angles.femur = standing_joints.femur;
        angles.tibia = standing_joints.tibia;

        // Update leg with standing joint angles and corresponding tip position
        legs[i].setJointAngles(angles);
        Point3D pos = model.forwardKinematicsGlobalCoordinates(i, angles);
        legs[i].setCurrentTipPositionGlobal(pos);
    }

    return true;
}

StandingPoseJoints BodyPoseController::getStandingPoseJoints(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        return {0.0, 0.0, 0.0};
    }
    return body_pose_config.standing_pose_joints[leg_index];
}

bool BodyPoseController::setBodyPoseQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                               Leg legs[NUM_LEGS]) {
    // Convert quaternion to Euler (radians) directly and pass through
    Point3D euler_rad = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3d orientation(euler_rad.x, euler_rad.y, euler_rad.z);
    if (model.getParams().smooth_trajectory.use_current_servo_positions &&
        model.getParams().smooth_trajectory.enable_pose_interpolation &&
        model.getParams().smooth_trajectory.use_quaternion_slerp) {
        return setBodyPoseSmoothQuaternion(position, quaternion, legs);
    }
    return setBodyPose(position, orientation, legs);
}

bool BodyPoseController::interpolatePose(const Eigen::Vector3d &start_pos, const Eigen::Vector4d &start_quat,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector4d &end_quat,
                                         double t, Leg legs[NUM_LEGS]) {
    // Clamp interpolation parameter
    t = std::max(0.0, std::min(DEFAULT_ANGULAR_SCALING, t));

    // Linear interpolation for position
    Eigen::Vector3d interp_pos = start_pos + t * (end_pos - start_pos);

    // Spherical linear interpolation (SLERP) for quaternion
    Eigen::Vector4d interp_quat = math_utils::quaternionSlerp(start_quat, end_quat, t);

    return setBodyPoseQuaternion(interp_pos, interp_quat, legs);
}

bool BodyPoseController::setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                           Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Check if smooth trajectory is enabled
    if (!model.getParams().smooth_trajectory.use_current_servo_positions) {
        return setBodyPoseImmediate(position, orientation, legs);
    }

    // If not already in progress, initialize trajectory from current servo positions
    if (!trajectory_in_progress) {

        initializeTrajectoryFromCurrent(position, orientation, legs, servos);
        trajectory_in_progress = true;
        trajectory_progress = 0.0;
        trajectory_step_count = 0;
    }

    // Update trajectory step
    return updateTrajectoryStep(legs);
}

bool BodyPoseController::setBodyPoseSmoothQuaternion(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                                     Leg legs[NUM_LEGS]) {
    Point3D euler_rad = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3d orientation(euler_rad.x, euler_rad.y, euler_rad.z);
    return setBodyPoseSmooth(position, orientation, legs);
}

bool BodyPoseController::getCurrentServoPositions(IServoInterface *servos, Leg legs[NUM_LEGS]) {
    if (!servos) {
        return false;
    }

    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles current_angles;
        current_angles.coxa = servos->getJointAngle(i, 0);
        current_angles.femur = servos->getJointAngle(i, 1);
        current_angles.tibia = servos->getJointAngle(i, 2);

        // Update leg object with current servo positions
        legs[i].setJointAngles(current_angles);
        legs[i].setCurrentTipPositionGlobal(model.forwardKinematicsGlobalCoordinates(i, current_angles));
    }

    return true;
}

bool BodyPoseController::setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation,
                                              Leg legs[NUM_LEGS]) {
    // Reset any active trajectory
    resetTrajectory();

    // Calculate leg positions based on body pose
    return calculateBodyPoseFromConfig(0.0, legs);
}

void BodyPoseController::configureSmoothTrajectory(bool use_current_positions, double interpolation_speed, uint8_t max_steps) {
    // Legacy method - parameters are ignored in current implementation
    // Only resets trajectory state for compatibility
    resetTrajectory();
}

bool BodyPoseController::checkBodyPoseLimits(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) {
    // Check translation limits using configured max_translation values
    if (std::abs(position.x()) > body_pose_config.max_translation.x ||
        std::abs(position.y()) > body_pose_config.max_translation.y ||
        std::abs(position.z()) > body_pose_config.max_translation.z) {
        return false;
    }

    // Check rotation limits using configured max_rotation values
    double roll_rad = orientation.x();
    double pitch_rad = orientation.y();
    double yaw_rad = orientation.z();

    if (std::abs(roll_rad) > body_pose_config.max_rotation.roll ||
        std::abs(pitch_rad) > body_pose_config.max_rotation.pitch ||
        std::abs(yaw_rad) > body_pose_config.max_rotation.yaw) {
        return false;
    }

    return true;
}

Eigen::Vector3d BodyPoseController::calculateBodyPosition(Leg legs[NUM_LEGS]) const {

    // Use walk plane pose for body position calculation (OpenSHC equivalent)
    if (walk_plane_pose_enabled) {
        // Update walk plane pose for current leg positions
        const_cast<BodyPoseController *>(this)->updateWalkPlanePose(legs);

        return Eigen::Vector3d(
            walk_plane_pose_.position.x,
            walk_plane_pose_.position.y,
            walk_plane_pose_.position.z);
    }

    // Fallback to legacy calculation if walk plane pose disabled
    double total_z = 0.0;
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D leg_pos = legs[i].getCurrentTipPositionGlobal();
        total_z += leg_pos.z;
    }
    double average_leg_z = total_z / NUM_LEGS;

    return Eigen::Vector3d(0.0, 0.0, average_leg_z);
}

bool BodyPoseController::initializeTrajectoryFromCurrent(const Eigen::Vector3d &target_position,
                                                         const Eigen::Vector3d &target_orientation,
                                                         Leg legs[NUM_LEGS], IServoInterface *servos) {
    // Store current positions as trajectory start
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_start_positions[i] = legs[i].getCurrentTipPositionGlobal();
        trajectory_start_angles[i] = legs[i].getJointAngles();
    }

    // Calculate target positions for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        trajectory_target_positions[i] = trajectory_start_positions[i];
        trajectory_target_positions[i].z = -target_position.z();

        // Calculate target angles using current angles as starting point
        JointAngles current_angles = legs[i].getJointAngles();
        trajectory_target_angles[i] = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, trajectory_target_positions[i]);
    }

    return true;
}

bool BodyPoseController::updateTrajectoryStep(Leg legs[NUM_LEGS]) {
    if (!trajectory_in_progress) {
        return false;
    }

    // Update trajectory progress
    trajectory_progress += model.getParams().smooth_trajectory.interpolation_speed;
    trajectory_step_count++;

    // Check if trajectory is complete
    if (trajectory_progress >= 1.0 || trajectory_step_count >= model.getParams().smooth_trajectory.max_interpolation_steps) {
        // Set final positions
        for (int i = 0; i < NUM_LEGS; i++) {
            legs[i].setCurrentTipPositionGlobal(trajectory_target_positions[i]);
            legs[i].setJointAngles(trajectory_target_angles[i]);
            legs[i].setCurrentTipPositionGlobal(trajectory_target_positions[i]);
        }

        resetTrajectory();
        return true;
    }

    // Interpolate between start and target positions
    for (int i = 0; i < NUM_LEGS; i++) {
        // Linear interpolation for positions
        Point3D interp_pos;
        interp_pos.x = trajectory_start_positions[i].x +
                       trajectory_progress * (trajectory_target_positions[i].x - trajectory_start_positions[i].x);
        interp_pos.y = trajectory_start_positions[i].y +
                       trajectory_progress * (trajectory_target_positions[i].y - trajectory_start_positions[i].y);
        interp_pos.z = trajectory_start_positions[i].z +
                       trajectory_progress * (trajectory_target_positions[i].z - trajectory_start_positions[i].z);

        // Linear interpolation for angles
        JointAngles interp_angles;
        interp_angles.coxa = trajectory_start_angles[i].coxa +
                             trajectory_progress * (trajectory_target_angles[i].coxa - trajectory_start_angles[i].coxa);
        interp_angles.femur = trajectory_start_angles[i].femur +
                              trajectory_progress * (trajectory_target_angles[i].femur - trajectory_start_angles[i].femur);
        interp_angles.tibia = trajectory_start_angles[i].tibia +
                              trajectory_progress * (trajectory_target_angles[i].tibia - trajectory_start_angles[i].tibia);

        // Update leg
        legs[i].setCurrentTipPositionGlobal(interp_pos);
        legs[i].setJointAngles(interp_angles);
        legs[i].setCurrentTipPositionGlobal(interp_pos);
    }

    return true;
}

void BodyPoseController::resetTrajectory() {
    trajectory_in_progress = false;
    trajectory_progress = 0.0;
    trajectory_step_count = 0;
}

bool BodyPoseController::beginInitialStandingPoseTransition(Leg legs[NUM_LEGS]) {
    if (initial_standing_active_) {
        return true; // already active
    }
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // Use OpenSHC direct startup-style transition for standing up.
    initial_standing_use_bezier_ = false;
    initial_standing_progress_ = 0.0;
    initial_standing_active_ = true;
    return true;
}

bool BodyPoseController::stepInitialStandingPoseTransition(Leg legs[NUM_LEGS], double dt,
                                                           double out_pos[NUM_LEGS][DOF_PER_LEG],
                                                           double out_vel[NUM_LEGS][DOF_PER_LEG],
                                                           double out_acc[NUM_LEGS][DOF_PER_LEG]) {
    if (!initial_standing_active_) {
        return false; // nothing to advance
    }

    if (initial_standing_use_bezier_) {
        return false;
    }

    int min_progress = PROGRESS_COMPLETE;
    int completed_legs = 0;
    double time_to_start = body_pose_config.time_to_start;

    for (int l = 0; l < NUM_LEGS; ++l) {
        if (!leg_posers_[l])
            continue;
        LegPoser *lp = leg_posers_[l]->get();
        const StandingPoseJoints &standing = body_pose_config.standing_pose_joints[l];
        JointAngles target_angles;
        target_angles.coxa = standing.coxa;
        target_angles.femur = standing.femur;
        target_angles.tibia = standing.tibia;
        Point3D target_tip = model.forwardKinematicsGlobalCoordinates(l, target_angles);
        Pose target_pose(target_tip, Eigen::Quaterniond::Identity());

        int progress = lp->stepToPosition(target_pose, Pose::Identity(), 0.0, time_to_start, false);
        min_progress = std::min(min_progress, progress);

        Point3D desired_tip = lp->getCurrentPosition();
        legs[l].setCurrentTipPositionGlobal(desired_tip);
        JointAngles current_angles = legs[l].getJointAngles();
        JointAngles next_angles = model.inverseKinematicsCurrentGlobalCoordinates(l, current_angles, desired_tip);
        legs[l].setJointAngles(next_angles);
        legs[l].setCurrentTipPositionGlobal(desired_tip);

        if (progress == PROGRESS_COMPLETE) {
            completed_legs++;
        }
    }

    initial_standing_progress_ = math_utils::clamp(static_cast<double>(min_progress) / 100.0, 0.0, 1.0);

    if (completed_legs == NUM_LEGS) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            const StandingPoseJoints &standing = body_pose_config.standing_pose_joints[l];
            JointAngles ja;
            ja.coxa = standing.coxa;
            ja.femur = standing.femur;
            ja.tibia = standing.tibia;
            legs[l].setJointAngles(ja);
            Point3D pos = model.forwardKinematicsGlobalCoordinates(l, ja);
            legs[l].setCurrentTipPositionGlobal(pos);
            legs[l].setStepPhase(STANCE_PHASE);
            if (leg_posers_[l]) {
                leg_posers_[l]->get()->setTargetPosition(pos);
                leg_posers_[l]->get()->resetStepToPosition();
            }
        }
        initial_standing_active_ = false;
        initial_standing_progress_ = 1.0;
        return true;
    }

    for (int l = 0; l < NUM_LEGS; ++l) {
        JointAngles ja = legs[l].getJointAngles();
        if (out_pos) {
            out_pos[l][0] = ja.coxa;
            out_pos[l][1] = ja.femur;
            out_pos[l][2] = ja.tibia;
        }
        if (out_vel) {
            out_vel[l][0] = 0.0;
            out_vel[l][1] = 0.0;
            out_vel[l][2] = 0.0;
        }
        if (out_acc) {
            out_acc[l][0] = 0.0;
            out_acc[l][1] = 0.0;
            out_acc[l][2] = 0.0;
        }
    }

    return false;
}

// Tripod leg coordination for stance transition (OpenSHC equivalent)
bool BodyPoseController::stepToNewStance(Leg legs[NUM_LEGS], double step_height, double step_time) {
    // OpenSHC stepToNewStance() implementation - Tripod coordination only
    // "The stepping motion is coordinated such that half of the legs execute the step at any one time
    //  (for a hexapod this results in a Tripod stepping coordination)"

    const int leg_count = NUM_LEGS;
    const int legs_per_group = leg_count / 2; // 3 legs per group for hexapod
    bool all_current_group_complete = true;

    // Initialize sequence if not already done
    if (!step_to_new_stance_sequence_generated) {
        // Set target positions using LegStepper's default tip poses (OpenSHC approach)
        // This ensures continuity with gait execution instead of jumping to static standing poses
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg_posers_[i]) {
                // Get default tip pose from leg's stepper (OpenSHC equivalent)
                // This maintains continuity with the planned gait trajectory
                Point3D current_position = legs[i].getCurrentTipPositionGlobal();

                // Use current position as target to maintain stance (OpenSHC behavior)
                // The actual step target will be set by gait execution, not startup sequence
                leg_posers_[i]->get()->setTargetPosition(current_position);
            }
        }
        step_to_new_stance_current_group = 0;
        step_to_new_stance_sequence_generated = true;
    }

    // Process current group legs (OpenSHC approach)
    int completed_legs_in_group = 0;
    for (int idx = 0; idx < legs_per_group; idx++) {
        int leg_index = tripod_leg_groups[step_to_new_stance_current_group][idx];

        if (leg_posers_[leg_index]) {
            LegPoser *leg_poser = leg_posers_[leg_index]->get();

            // Set legs in current group to SWING phase during movement
            legs[leg_index].setStepPhase(SWING_PHASE);

            // Execute step to target position for this leg
            bool leg_complete = leg_poser->stepToPosition(leg_poser->getTargetPosition(), step_height, step_time);

            // Update leg state from poser (OpenSHC pattern)
            Point3D desired_tip = leg_poser->getCurrentPosition();
            legs[leg_index].setCurrentTipPositionGlobal(desired_tip);
            JointAngles current_angles = legs[leg_index].getJointAngles();
            legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, desired_tip));
            legs[leg_index].setCurrentTipPositionGlobal(desired_tip);

            if (leg_complete) {
                // When leg completes movement, set back to STANCE phase
                legs[leg_index].setStepPhase(STANCE_PHASE);
                completed_legs_in_group++;
            } else {
                all_current_group_complete = false;
            }
        }
    }

    // Ensure legs not in current group are in STANCE phase
    for (int i = 0; i < NUM_LEGS; i++) {
        bool in_current_group = false;
        for (int idx = 0; idx < legs_per_group; idx++) {
            if (i == tripod_leg_groups[step_to_new_stance_current_group][idx]) {
                in_current_group = true;
                break;
            }
        }
        if (!in_current_group) {
            legs[i].setStepPhase(STANCE_PHASE);
        }
    }

    // Check if current group is complete and advance to next group or finish
    if (all_current_group_complete && completed_legs_in_group == legs_per_group) {
        if (step_to_new_stance_current_group == 0) {
            // Move to second group (Group B)
            step_to_new_stance_current_group = 1;
            // Reset posers for second group
            for (int idx = 0; idx < legs_per_group; idx++) {
                int leg_index = tripod_leg_groups[step_to_new_stance_current_group][idx];
                if (leg_posers_[leg_index]) {
                    leg_posers_[leg_index]->get()->resetStepToPosition();
                }
            }
        } else {
            // Both groups complete - sequence finished
            step_to_new_stance_sequence_generated = false;
            step_to_new_stance_current_group = 0;
            return true;
        }
    }

    return false;
}

// Execute startup sequence (READY -> RUNNING transition)
bool BodyPoseController::executeStartupSequence(Leg legs[NUM_LEGS]) {
    int progress = executeSequenceInternal("startup", legs);
    if (progress < 0) {
        last_startup_progress_ = 0;
        return false;
    }
    last_startup_progress_ = progress;
    return progress == PROGRESS_COMPLETE;
}

int BodyPoseController::getStartupProgressPercent() const {
    int percent = last_startup_progress_;
    if (percent < 0)
        percent = 0;
    if (percent > PROGRESS_COMPLETE)
        percent = PROGRESS_COMPLETE;
    return percent;
}

// Execute shutdown sequence (RUNNING -> READY transition - OpenSHC equivalent)
bool BodyPoseController::executeShutdownSequence(Leg legs[NUM_LEGS]) {
    int progress = executeSequenceInternal("shutdown", legs);
    if (progress < 0) {
        last_shutdown_progress_ = 0;
        return false;
    }
    last_shutdown_progress_ = progress;
    return progress == PROGRESS_COMPLETE;
}

// Update auto-pose during gait execution (OpenSHC equivalent)
bool BodyPoseController::updateAutoPose(double gait_phase, Leg legs[NUM_LEGS]) {

    if (!auto_pose_enabled || !auto_pose_config.enabled)
        return true; // nothing to do

    // Determine pose cycle length.
    // If pose_frequency == -1.0 we assume sync with gait step cycle and use configured pose_phase_length.
    // Otherwise we still use pose_phase_length as discrete resolution for the pose cycle.
    int base_period = auto_pose_config.pose_phase_length;
    if (base_period <= 0) {
        // Derive fallback from the largest index found in starts/ends (robust for partial configurations)
        int max_idx = 0;
        for (int v : auto_pose_config.pose_phase_starts)
            if (v > max_idx)
                max_idx = v;
        for (int v : auto_pose_config.pose_phase_ends)
            if (v > max_idx)
                max_idx = v;
        base_period = std::max(4, max_idx + 1); // reasonable minimum
    }

    // Convert gait_phase [0,1) to integer phase index in [0, base_period)
    double wrapped = gait_phase - std::floor(gait_phase);
    int current_phase_index = static_cast<int>(wrapped * base_period) % base_period;

    // Per-leg delegation to the LegPoser (OpenSHC parity). Each LegPoser recalculates offsets and applies threshold.
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        if (leg_posers_[leg_index]) {
            leg_posers_[leg_index]->get()->updateAutoPose(current_phase_index, auto_pose_config, body_pose_config);
        }
    }
    return true;
}

// Walk plane pose system implementation (OpenSHC equivalent)
void BodyPoseController::updateWalkPlanePose(Leg legs[NUM_LEGS]) {
    if (!walk_plane_pose_enabled) {
        return;
    }

    // Calculate walk plane normal and height from stance leg positions
    Point3D walk_plane_normal = calculateWalkPlaneNormal(legs);
    double walk_plane_height = calculateWalkPlaneHeight(legs);

    // Create target walk plane pose
    Pose target_walk_plane_pose;
    target_walk_plane_pose.position = Point3D(0.0, 0.0, walk_plane_height + body_pose_config.body_clearance);

    // Set orientation: align body with walk plane normal
    Eigen::Vector3d unit_z(0.0, 0.0, 1.0);
    Eigen::Vector3d walk_normal(walk_plane_normal.x, walk_plane_normal.y, walk_plane_normal.z);
    walk_normal.normalize();
    target_walk_plane_pose.rotation = Eigen::Quaterniond::FromTwoVectors(unit_z, walk_normal);

    // Check if change is significant enough to update
    double position_change = (target_walk_plane_pose.position - walk_plane_pose_.position).norm();
    double rotation_change = target_walk_plane_pose.rotation.angularDistance(walk_plane_pose_.rotation);

    bool need_update = (position_change > walk_plane_update_threshold || rotation_change > 0.01);

    if (!need_update) {
        // Only advance if a Bézier transition is currently active
        if (walk_plane_bezier_in_progress) {
            walk_plane_bezier_time += model.getTimeDelta();
            double t = std::min(1.0, walk_plane_bezier_time / walk_plane_bezier_duration);
            double smooth_t = math_utils::smoothStep(t);
            walk_plane_pose_.position = math_utils::quarticBezier(walk_plane_position_nodes, smooth_t);
            walk_plane_pose_.rotation = walk_plane_rotation_nodes[0].slerp(smooth_t, walk_plane_rotation_nodes[4]);
            if (t >= 1.0) {
                walk_plane_bezier_in_progress = false;
#ifdef TESTING_ENABLED
                std::cout << "[WalkPlane] Bezier transition completed." << std::endl;
#endif
            }
        }
        return;
    }

#ifdef TESTING_ENABLED
    std::cout << "[WalkPlane] target_height=" << target_walk_plane_pose.position.z
              << " current_height=" << walk_plane_pose_.position.z
              << " pos_change=" << position_change << " rot_change=" << rotation_change << std::endl;
#endif

    // Small changes -> direct assignment (skip Bézier smoothing for minor adjustments)
    if (position_change < 200.0 && rotation_change < 0.1) {
        walk_plane_pose_ = target_walk_plane_pose;
        walk_plane_bezier_in_progress = false;
#ifdef TESTING_ENABLED
        std::cout << "[WalkPlane] Direct assignment applied. New height=" << walk_plane_pose_.position.z << std::endl;
#endif
        return;
    }

    // Determine if the target changed significantly (decides whether to (re)build Bézier control nodes)
    bool target_changed = !walk_plane_bezier_in_progress ||
                          (target_walk_plane_pose.position - walk_plane_position_nodes[4]).norm() > 1e-3 ||
                          walk_plane_rotation_nodes[4].angularDistance(target_walk_plane_pose.rotation) > 1e-4;

    if (target_changed) {
        Point3D start_position = walk_plane_pose_.position;
        Point3D end_position = target_walk_plane_pose.position;
        Eigen::Quaterniond start_rotation = walk_plane_pose_.rotation;
        Eigen::Quaterniond end_rotation = target_walk_plane_pose.rotation;

        // Generate quartic Bézier control nodes for position (OpenSHC-inspired smoothing) only when target changes
        walk_plane_position_nodes[0] = start_position;
        walk_plane_position_nodes[1] = start_position + (end_position - start_position) * 0.2;
        walk_plane_position_nodes[2] = start_position + (end_position - start_position) * 0.5;
        walk_plane_position_nodes[3] = start_position + (end_position - start_position) * 0.8;
        walk_plane_position_nodes[4] = end_position;

        // Generate corresponding Slerp-based control nodes for orientation interpolation
        walk_plane_rotation_nodes[0] = start_rotation;
        walk_plane_rotation_nodes[1] = start_rotation.slerp(0.2, end_rotation);
        walk_plane_rotation_nodes[2] = start_rotation.slerp(0.5, end_rotation);
        walk_plane_rotation_nodes[3] = start_rotation.slerp(0.8, end_rotation);
        walk_plane_rotation_nodes[4] = end_rotation;

        walk_plane_bezier_in_progress = true;
        walk_plane_bezier_time = 0.0;

#ifdef TESTING_ENABLED
        std::cout << "[WalkPlane] Bezier transition (re)started." << std::endl;
#endif
    }

    // Advance current Bézier transition
    if (walk_plane_bezier_in_progress) {
        walk_plane_bezier_time += model.getTimeDelta();
        double t = std::min(1.0, walk_plane_bezier_time / walk_plane_bezier_duration);
        double smooth_t = math_utils::smoothStep(t);
        walk_plane_pose_.position = math_utils::quarticBezier(walk_plane_position_nodes, smooth_t);
        walk_plane_pose_.rotation = walk_plane_rotation_nodes[0].slerp(smooth_t, walk_plane_rotation_nodes[4]);
        if (t >= 1.0) {
            walk_plane_bezier_in_progress = false;

#ifdef TESTING_ENABLED
            std::cout << "[WalkPlane] Bezier transition completed." << std::endl;
#endif
        }
    }
}

Point3D BodyPoseController::calculateWalkPlaneNormal(Leg legs[NUM_LEGS]) const {
    // Collect stance leg positions for plane fitting
    std::vector<Point3D> stance_points;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            stance_points.push_back(legs[i].getCurrentTipPositionGlobal());
        }
    }

    // Need at least 3 points to define a plane
    if (stance_points.size() < 3) {
        return Point3D(0.0, 0.0, 1.0); // Default to horizontal plane
    }

    // Prepare data for least squares plane fitting
    std::vector<double> raw_A;
    std::vector<double> raw_B;

    for (const auto &point : stance_points) {
        raw_A.push_back(point.x);
        raw_A.push_back(point.y);
        raw_A.push_back(1.0);
        raw_B.push_back(point.z);
    }

    // Solve for plane equation: ax + by + c = z
    double a, b, c;
    if (math_utils::solveLeastSquaresPlane(raw_A.data(), raw_B.data(), stance_points.size(), a, b, c)) {
        // Convert plane coefficients to normal vector
        // Plane equation: ax + by - z + c = 0
        // Normal vector: (a, b, -1)
        double normal_magnitude = std::sqrt(a * a + b * b + 1.0);
        Point3D normal(-a / normal_magnitude, -b / normal_magnitude, 1.0 / normal_magnitude);

        // Ensure normal points upward (positive Z component)
        if (normal.z < 0) {
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        return normal;
    }

    // Fallback to horizontal plane if calculation fails
    return Point3D(0.0, 0.0, 1.0);
}

double BodyPoseController::calculateWalkPlaneHeight(Leg legs[NUM_LEGS]) const {
    // Calculate average Z position of stance legs
    double total_z = 0.0;
    int stance_count = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            total_z += legs[i].getCurrentTipPositionGlobal().z;
            stance_count++;
        }
    }

    if (stance_count > 0) {
        return total_z / stance_count;
    }

    // Fallback: use current walk plane height minus body clearance
    return walk_plane_pose_.position.z - body_pose_config.body_clearance;
}

Pose BodyPoseController::getWalkPlanePose() const {
    return walk_plane_pose_;
}

void BodyPoseController::setWalkPlanePose(const Pose &pose) {
    walk_plane_pose_ = pose;

    // Reset Bézier transition state
    walk_plane_bezier_in_progress = false;
    walk_plane_bezier_time = 0.0;

    // Initialize all control nodes to current pose
    for (int i = 0; i < 5; i++) {
        walk_plane_position_nodes[i] = pose.position;
        walk_plane_rotation_nodes[i] = pose.rotation;
    }
}

void BodyPoseController::setWalkPlanePoseEnabled(bool enabled) {
    walk_plane_pose_enabled = enabled;
    if (enabled) {
        // Reset to neutral position
        Pose neutral_pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
        walk_plane_pose_ = neutral_pose;

        // Reset Bézier transition state
        walk_plane_bezier_in_progress = false;
        walk_plane_bezier_time = 0.0;

        // Initialize all control nodes to neutral pose
        for (int i = 0; i < 5; i++) {
            walk_plane_position_nodes[i] = neutral_pose.position;
            walk_plane_rotation_nodes[i] = neutral_pose.rotation;
        }
    }
}

bool BodyPoseController::isWalkPlanePoseEnabled() const {
    return walk_plane_pose_enabled;
}

int BodyPoseController::packLegs(Leg legs[NUM_LEGS], double time_to_pack) {
    // OpenSHC packLegs implementation adapted for HexaMotion

    // Get pack height from configuration
    double pack_height = 150.0; // Default pack height in mm

    // Get body position as pack position (use default center position)
    Point3D pack_position;
    pack_position.x = 0.0;         // Center position
    pack_position.y = 0.0;         // Center position
    pack_position.z = pack_height; // Raise to pack height

    int completed_legs = 0;

    // Step all legs to pack position
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        if (leg_posers_[leg_index]) {
            bool success = leg_posers_[leg_index]->get()->stepToPosition(pack_position, 20.0, 1.0); // 20mm step height, 1s time
            if (success) {
                // Update leg state
                legs[leg_index].setCurrentTipPositionGlobal(pack_position);
                JointAngles current_angles = legs[leg_index].getJointAngles();
                legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, pack_position));
                legs[leg_index].setCurrentTipPositionGlobal(pack_position);
                completed_legs++;
            }
        }
    }

    // Update pack step tracking
    pack_step_++;
    legs_completed_step_ = completed_legs;

    // Return progress percentage
    int progress = (completed_legs * 100) / NUM_LEGS;
    return progress;
}

int BodyPoseController::unpackLegs(Leg legs[NUM_LEGS], double time_to_unpack) {
    // OpenSHC unpackLegs implementation adapted for HexaMotion

    // Get default stance height
    double stance_height = 50.0; // Default stance height in mm

    int completed_legs = 0;

    // Calculate unpack positions for each leg
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        // Calculate default standing position for each leg based on robot geometry
        Point3D default_position;

        // Basic hexapod leg positioning (simplified)
        double radius = 120.0;                   // Default radius from center in mm
        double angle = leg_index * (M_PI / 3.0); // 60 degrees between legs

        default_position.x = radius * cos(angle);
        default_position.y = radius * sin(angle);
        default_position.z = stance_height;

        if (leg_posers_[leg_index]) {
            bool success = leg_posers_[leg_index]->get()->stepToPosition(default_position, 20.0, 1.0); // 20mm step height, 1s time
            if (success) {
                // Update leg state
                legs[leg_index].setCurrentTipPositionGlobal(default_position);
                JointAngles current_angles = legs[leg_index].getJointAngles();
                legs[leg_index].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(leg_index, current_angles, default_position));
                legs[leg_index].setCurrentTipPositionGlobal(default_position);
                completed_legs++;
            }
        }
    }

    // Update tracking
    pack_step_--;
    legs_completed_step_ = completed_legs;

    // Return progress percentage
    int progress = (completed_legs * 100) / NUM_LEGS;
    return progress;
}

int BodyPoseController::poseForLegManipulation(Leg legs[NUM_LEGS]) {
    // OpenSHC poseForLegManipulation: simultaneously transition all legs to poses
    // suitable for manual leg manipulation or return to walking.
    // Each leg uses stepToPosition with a step_height and step_time derived from params.

    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    const Parameters &params = model.getParams();
    double step_height = params.standing_height * 0.1; // Modest lift for repositioning
    double step_time = 1.0 / params.step_frequency;

    int min_progress = 100; // Track minimum progress across all legs (OpenSHC UNASSIGNED equivalent)

    for (int i = 0; i < NUM_LEGS; ++i) {
        LegPoser *leg_poser = getLegPoser(i);
        if (!leg_poser)
            continue;

        // Build target tip pose depending on leg state
        Point3D target_tip_position = leg_poser->getDefaultTipPose();

        // For WALKING_TO_MANUAL: lift leg slightly above default to prepare for manipulation
        // For MANUAL_TO_WALKING: return to default walking stance
        // Other legs: hold at default stance (add default_pose offset for load redistribution)

        double effective_step_height = step_height;

        // Use stepToPosition for gradual Bézier-based movement
        Pose target_pose(target_tip_position, Eigen::Quaterniond::Identity());
        int progress = leg_poser->stepToPosition(target_pose, Pose::Identity(),
                                                 effective_step_height, step_time);
        min_progress = std::min(progress, min_progress);

        if (progress != PROGRESS_COMPLETE) {
            // Apply the intermediate pose: set desired tip and run IK
            Point3D tip_pos = leg_poser->getCurrentTipPose().position;
            legs[i].setDesiredTipPosition(tip_pos);
            legs[i].applyIK(tip_pos);
        }
    }

    return min_progress;
}

int BodyPoseController::executeSequenceInternal(const std::string &sequence_type, Leg legs[NUM_LEGS]) {
    const bool is_startup = sequence_type == "startup";
    const bool is_shutdown = sequence_type == "shutdown";
    if (!is_startup && !is_shutdown) {
        return 0;
    }

    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    // Initialise/reset saved transition sequence
    if (reset_transition_sequence_ && is_startup) {
        reset_transition_sequence_ = false;
        first_sequence_execution_ = true;
        transition_step_ = 0;
        set_target_ = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (leg_posers_[i]) {
                LegPoser *lp = leg_posers_[i]->get();
                lp->resetTransitionSequence();
                lp->addTransitionPose(lp->getCurrentTipPose());
            }
        }
    }

    int progress = 0;
    int normalised_progress = 0;
    int total_progress = 0;

    int next_transition_step = transition_step_ + 1;
    int transition_step_target = transition_step_count_;
    bool execute_horizontal_transition = !(transition_step_ % 2);
    bool execute_vertical_transition = transition_step_ % 2;
    if (is_shutdown) {
        execute_horizontal_transition = transition_step_ % 2;
        execute_vertical_transition = !(transition_step_ % 2);
        next_transition_step = transition_step_ - 1;
        transition_step_target = 0;
        total_progress = 100 - transition_step_ * 100 / std::max(transition_step_count_, 1);
    } else {
        total_progress = transition_step_ * 100 / std::max(transition_step_count_, 1);
    }

    bool final_transition = false;
    bool sequence_complete = false;
    if (first_sequence_execution_) {
        final_transition = (horizontal_transition_complete_ || vertical_transition_complete_);
    } else {
        final_transition = (next_transition_step == transition_step_target);
    }

    double safety_factor = (first_sequence_execution_ ? SAFETY_FACTOR / (transition_step_ + 1) : 0.0);

    auto compute_limit_proximity = [&](const JointAngles &angles) {
        double min_proximity = 1.0;
        double joints[3] = {angles.coxa, angles.femur, angles.tibia};
        double limits[3][2] = {
            {model.getParams().coxa_angle_limits[0], model.getParams().coxa_angle_limits[1]},
            {model.getParams().femur_angle_limits[0], model.getParams().femur_angle_limits[1]},
            {model.getParams().tibia_angle_limits[0], model.getParams().tibia_angle_limits[1]}};
        for (int j = 0; j < 3; ++j) {
            double range = limits[j][1] - limits[j][0];
            if (range > 0.0) {
                double half_range = range * 0.5;
                double center = (limits[j][1] + limits[j][0]) * 0.5;
                double distance_from_center = std::abs(joints[j] - center);
                double proximity = math_utils::clamp<double>((half_range - distance_from_center) / half_range, 0.0, 1.0);
                min_proximity = std::min(min_proximity, proximity);
            }
        }
        return min_proximity;
    };

    auto legs_bearing_load = [&]() {
        double body_height_estimate = 0.0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            body_height_estimate += legs[i].getCurrentTipPositionGlobal().z;
        }
        return -(body_height_estimate / NUM_LEGS) > HALF_BODY_DEPTH_MM;
    };

    if (execute_horizontal_transition) {
        if (set_target_) {
            set_target_ = false;
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (!leg_posers_[i])
                    continue;
                LegPoser *lp = leg_posers_[i]->get();
                lp->setLegCompletedStep(false);

                Point3D target_tip_position;
                if (lp->hasTransitionPose(next_transition_step)) {
                    target_tip_position = lp->getTransitionPose(next_transition_step).position;
                } else {
                    const auto &stance = body_pose_config.leg_stance_positions[i];
                    target_tip_position = Point3D(stance.x, stance.y, stance.z);
                }

                target_tip_position.z = legs[i].getCurrentTipPositionGlobal().z;
                lp->setTargetTipPose(Pose(target_tip_position, Eigen::Quaterniond::Identity()));
            }
        }

        bool direct_step = !legs_bearing_load();
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!leg_posers_[i])
                continue;
            LegPoser *lp = leg_posers_[i]->get();
            if (!lp->getLegCompletedStep()) {
                if (i == tripod_leg_groups[current_group_][0] ||
                    i == tripod_leg_groups[current_group_][1] ||
                    i == tripod_leg_groups[current_group_][2] || direct_step) {
                    Pose target_tip_pose = lp->getTargetTipPose();
                    bool apply_delta = (is_startup && final_transition);
                    double step_height = direct_step ? 0.0 : body_pose_config.swing_height;
                    double time_to_step = HORIZONTAL_TRANSITION_TIME / model.getParams().step_frequency;
                    if (first_sequence_execution_) {
                        time_to_step *= 2.0;
                    }
                    progress = lp->stepToPosition(target_tip_pose, Pose::Identity(), step_height, time_to_step, apply_delta);
                    Point3D desired_tip = lp->getCurrentPosition();
                    legs[i].setCurrentTipPositionGlobal(desired_tip);
                    JointAngles current_angles = legs[i].getJointAngles();
                    JointAngles next_angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, desired_tip);
                    legs[i].setJointAngles(next_angles);
                    // Restore authoritative LegPoser position after setJointAngles
                    // (setJointAngles triggers FK which can drift from the bezier output)
                    legs[i].setCurrentTipPositionGlobal(desired_tip);

                    double limit_proximity = compute_limit_proximity(next_angles);
                    bool exceeded_workspace = limit_proximity < safety_factor;
                    if (first_sequence_execution_ && exceeded_workspace) {
                        lp->setTargetTipPose(lp->getCurrentTipPose());
                        lp->resetStepToPosition();
                        progress = PROGRESS_COMPLETE;
                        proximity_alert_ = true;
                    }

                    if (progress == PROGRESS_COMPLETE) {
                        lp->setLegCompletedStep(true);
                        legs_completed_step_++;
                        if (first_sequence_execution_) {
                            bool reached_target = !exceeded_workspace;
                            Pose transition_pose = reached_target ? target_tip_pose : lp->getCurrentTipPose();
                            lp->addTransitionPose(transition_pose);
                        }
                    }
                } else {
                    legs_completed_step_++;
                    lp->setLegCompletedStep(true);
                }
            }
        }

        if (direct_step) {
            normalised_progress = progress / std::max(transition_step_count_, 1);
        } else {
            normalised_progress = (progress / 2 + (current_group_ == 0 ? 0 : 50)) / std::max(transition_step_count_, 1);
        }

        if (legs_completed_step_ == NUM_LEGS) {
            set_target_ = true;
            legs_completed_step_ = 0;
            if (current_group_ == 1 || direct_step) {
                current_group_ = 0;
                transition_step_ = next_transition_step;
                horizontal_transition_complete_ = !proximity_alert_;
                sequence_complete = final_transition;
                proximity_alert_ = false;
            } else {
                current_group_ = 1;
            }
        }
    }

    if (execute_vertical_transition) {
        if (set_target_) {
            set_target_ = false;
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (!leg_posers_[i])
                    continue;
                LegPoser *lp = leg_posers_[i]->get();
                Point3D target_tip_position;
                if (lp->hasTransitionPose(next_transition_step)) {
                    target_tip_position = lp->getTransitionPose(next_transition_step).position;
                } else {
                    const auto &stance = body_pose_config.leg_stance_positions[i];
                    target_tip_position = Point3D(stance.x, stance.y, stance.z);
                }

                Point3D current = legs[i].getCurrentTipPositionGlobal();
                target_tip_position.x = current.x;
                target_tip_position.y = current.y;
                lp->setTargetTipPose(Pose(target_tip_position, Eigen::Quaterniond::Identity()));
            }
        }

        bool all_legs_within_workspace = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!leg_posers_[i])
                continue;
            LegPoser *lp = leg_posers_[i]->get();
            Pose target_tip_pose = lp->getTargetTipPose();
            bool apply_delta = (is_startup && final_transition);
            double time_to_step = VERTICAL_TRANSITION_TIME / model.getParams().step_frequency;
            if (first_sequence_execution_) {
                time_to_step *= 2.0;
            }
            progress = lp->stepToPosition(target_tip_pose, Pose::Identity(), 0.0, time_to_step, apply_delta);
            Point3D desired_tip = lp->getCurrentPosition();
            legs[i].setCurrentTipPositionGlobal(desired_tip);
            JointAngles current_angles = legs[i].getJointAngles();
            JointAngles next_angles = model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, desired_tip);
            legs[i].setJointAngles(next_angles);
            // Restore authoritative LegPoser position after setJointAngles
            // (setJointAngles triggers FK which can drift from the bezier output)
            legs[i].setCurrentTipPositionGlobal(desired_tip);
            double limit_proximity = compute_limit_proximity(next_angles);
            all_legs_within_workspace = all_legs_within_workspace && !(limit_proximity < safety_factor);
        }

        if ((!all_legs_within_workspace && first_sequence_execution_) || progress == PROGRESS_COMPLETE) {
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (!leg_posers_[i])
                    continue;
                LegPoser *lp = leg_posers_[i]->get();
                lp->resetStepToPosition();
                progress = PROGRESS_COMPLETE;
                if (first_sequence_execution_) {
                    bool reached_target = all_legs_within_workspace;
                    Pose transition_pose = reached_target ? lp->getTargetTipPose() : lp->getCurrentTipPose();
                    lp->addTransitionPose(transition_pose);
                }
            }

            vertical_transition_complete_ = all_legs_within_workspace;
            transition_step_ = next_transition_step;
            sequence_complete = final_transition;
            set_target_ = true;
        }

        normalised_progress = progress / std::max(transition_step_count_, 1);
    }

    if (first_sequence_execution_) {
        transition_step_count_ = transition_step_;
        transition_step_target = transition_step_;
    }

    if (transition_step_ > TRANSITION_STEP_THRESHOLD) {
        resetSequenceStates();
        return 0;
    }

    if (sequence_complete) {
        set_target_ = true;
        vertical_transition_complete_ = false;
        horizontal_transition_complete_ = false;
        first_sequence_execution_ = false;
        return PROGRESS_COMPLETE;
    }

    total_progress = std::min(total_progress + normalised_progress, PROGRESS_COMPLETE - 1);
    return (first_sequence_execution_ ? -1 : total_progress);
}

int BodyPoseController::executeSequence(const std::string &sequence_type, Leg legs[NUM_LEGS]) {
    executing_transition_ = true;
    int progress = executeSequenceInternal(sequence_type, legs);
    if (sequence_type == "startup") {
        last_startup_progress_ = (progress < 0 ? 0 : progress);
    } else if (sequence_type == "shutdown") {
        last_shutdown_progress_ = (progress < 0 ? 0 : progress);
    }
    if (progress == PROGRESS_COMPLETE) {
        executing_transition_ = false;
    }
    return progress;
}
