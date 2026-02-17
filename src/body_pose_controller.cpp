#include "body_pose_controller.h"
#include "body_pose_config_factory.h"
#include "gait_types.h"
#include "leg_poser.h"
#include "math_utils.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <vector>

/**
 * @file body_pose_controller.cpp
 * @brief 1:1 port of OpenSHC PoseController for HexaMotion.
 *
 * This file contains ONLY functional equivalents of OpenSHC's PoseController methods.
 * Convenience/utility methods (setStandingPose, setBodyPose, etc.) live in LocomotionSystem.
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController constructor equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BodyPoseController::BodyPoseController(RobotModel &m, const BodyPoseConfiguration &config)
    : model(m), body_pose_config(config), auto_pose_enabled(false), current_gait_type_(TRIPOD_GAIT),
      executing_transition_(false), transition_step_(0), transition_step_count_(0),
      set_target_(true), proximity_alert_(false), horizontal_transition_complete_(false),
      vertical_transition_complete_(false), first_sequence_execution_(true),
      reset_transition_sequence_(true), legs_completed_step_(0), current_group_(0) {

    for (int i = 0; i < NUM_LEGS; i++) {
        leg_posers_[i] = nullptr;
    }

    resetAllPosing();

    rotation_absement_error_ = Eigen::Vector3d::Zero();
    rotation_position_error_ = Eigen::Vector3d::Zero();
    rotation_velocity_error_ = Eigen::Vector3d::Zero();

    translation_velocity_input_ = Eigen::Vector3d::Zero();
    rotation_velocity_input_ = Eigen::Vector3d::Zero();

    // Initialize walk plane pose (OpenSHC: init())
    walk_plane_pose_ = Pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
    origin_walk_plane_pose_ = walk_plane_pose_;

    // Initialize auto-pose configuration
    std::string gait_name = model.getParams().gait_type.empty() ? "tripod_gait" : model.getParams().gait_type;
    auto_pose_config = createAutoPoseConfigurationForGait(model.getParams(), gait_name);
    setAutoPoseParams();

    default_pose_ = Pose::Identity();
    manual_pose_enabled_ = body_pose_config.manual_posing_enabled;
    inclination_pose_enabled_ = body_pose_config.inclination_posing_enabled;
    imu_pose_enabled_ = body_pose_config.imu_posing_enabled;
    auto_pose_enabled = body_pose_config.auto_posing_enabled;
    tip_align_pose_enabled_ = body_pose_config.gravity_aligned_tips_enabled;
}

BodyPoseController::~BodyPoseController() {
    for (int i = 0; i < NUM_LEGS; i++) {
        delete leg_posers_[i];
        leg_posers_[i] = nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::init() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::initializeLegPosers(Leg legs[NUM_LEGS]) {
    legs_ref_ = legs;
    for (int i = 0; i < NUM_LEGS; i++) {
        delete leg_posers_[i];
        leg_posers_[i] = new LegPoserImpl(i, legs[i], model);
    }
    setAutoPoseParams();
    walk_plane_pose_ = Pose(Point3D(0.0, 0.0, body_pose_config.body_clearance), Eigen::Quaterniond::Identity());
    origin_walk_plane_pose_ = walk_plane_pose_;
}

LegPoser *BodyPoseController::getLegPoser(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS && leg_posers_[leg_index]) {
        return leg_posers_[leg_index]->get();
    }
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::setAutoPoseParams() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::setAutoPoseParams() {
    pose_frequency_ = auto_pose_config.pose_frequency;

    double raw_phase_length;
    int base_phase_length;

    // Calculate posing phase length and normalisation values based off gait/posing cycle parameters
    // (OpenSHC parity: 1:1 port of PoseController::setAutoPoseParams)
    if (pose_frequency_ == -1.0) {
        // Use step cycle parameters (OpenSHC: params_.stance_phase + params_.swing_phase)
        base_phase_length = gait_stance_phase_ + gait_swing_phase_;
        if (base_phase_length <= 0) {
            base_phase_length = 1; // Safety fallback
        }
        double swing_ratio = static_cast<double>(gait_swing_phase_) / base_phase_length;
        if (swing_ratio <= 0.0) {
            swing_ratio = 1.0; // Safety fallback
        }
        double step_frequency = std::max(1e-6, model.getParams().step_frequency);
        raw_phase_length = ((1.0 / step_frequency) / model.getTimeDelta()) / swing_ratio;
    } else {
        base_phase_length = auto_pose_config.pose_phase_length;
        if (base_phase_length <= 0) {
            int max_phase = 0;
            for (size_t i = 0; i < auto_pose_config.pose_phase_starts.size(); ++i) {
                max_phase = std::max(max_phase, auto_pose_config.pose_phase_starts[i]);
            }
            for (size_t i = 0; i < auto_pose_config.pose_phase_ends.size(); ++i) {
                max_phase = std::max(max_phase, auto_pose_config.pose_phase_ends[i]);
            }
            base_phase_length = std::max(1, max_phase + 1);
        }
        raw_phase_length = (1.0 / std::max(1e-6, pose_frequency_)) / model.getTimeDelta();
    }

    pose_phase_length_ = math_utils::roundToEvenInt(raw_phase_length / base_phase_length) * base_phase_length;
    if (pose_phase_length_ < base_phase_length) {
        pose_phase_length_ = base_phase_length;
    }
    normaliser_ = pose_phase_length_ / std::max(1, base_phase_length);
    if (normaliser_ < 1) {
        normaliser_ = 1;
    }

    // Set posing negation phase variables per leg (OpenSHC parity)
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (leg_posers_[i]) {
            LegPoser *lp = leg_posers_[i]->get();
            lp->setPoseNegationPhaseStart(auto_pose_config.negation_phase_start[i]);
            lp->setPoseNegationPhaseEnd(auto_pose_config.negation_phase_end[i]);
            lp->setNegationTransitionRatio(auto_pose_config.negation_transition_ratio[i]);
        }
    }

    // Set reference leg for auto posing system (leg with zero phase offset) (OpenSHC parity)
    auto_pose_reference_leg_ = 0;
    for (auto it = body_pose_config.offset_multiplier.begin(); it != body_pose_config.offset_multiplier.end(); ++it) {
        if (it->second == 0) {
            auto_pose_reference_leg_ = it->first;
            break;
        }
    }

    // Clear any old auto-poser objects and re-populate container (OpenSHC parity)
    auto_poser_container_.clear();
    size_t poser_count = std::min(auto_pose_config.pose_phase_starts.size(), auto_pose_config.pose_phase_ends.size());
    for (size_t i = 0; i < poser_count; ++i) {
        std::shared_ptr<AutoPoser> poser = std::make_shared<AutoPoser>(static_cast<int>(i));
        poser->setStartPhase(auto_pose_config.pose_phase_starts[i]);
        poser->setEndPhase(auto_pose_config.pose_phase_ends[i]);
        if (i < auto_pose_config.x_amplitudes.size())
            poser->setXAmplitude(auto_pose_config.x_amplitudes[i]);
        if (i < auto_pose_config.y_amplitudes.size())
            poser->setYAmplitude(auto_pose_config.y_amplitudes[i]);
        if (i < auto_pose_config.z_amplitudes.size())
            poser->setZAmplitude(auto_pose_config.z_amplitudes[i]);
        if (i < auto_pose_config.gravity_amplitudes.size())
            poser->setGravityAmplitude(auto_pose_config.gravity_amplitudes[i]);
        if (i < auto_pose_config.roll_amplitudes.size())
            poser->setRollAmplitude(auto_pose_config.roll_amplitudes[i]);
        if (i < auto_pose_config.pitch_amplitudes.size())
            poser->setPitchAmplitude(auto_pose_config.pitch_amplitudes[i]);
        if (i < auto_pose_config.yaw_amplitudes.size())
            poser->setYawAmplitude(auto_pose_config.yaw_amplitudes[i]);
        poser->resetChecks();
        auto_poser_container_.push_back(poser);
    }

    pose_phase_ = 0;
    auto_posing_state_ = POSING_COMPLETE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::setManualPoseInput() equivalent (simple setter)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::setManualPoseInput(const Eigen::Vector3d &translation_velocity,
                                            const Eigen::Vector3d &rotation_velocity) {
    translation_velocity_input_ = translation_velocity;
    rotation_velocity_input_ = rotation_velocity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateManualPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateManualPose() {
    const double time_delta = model.getParams().time_delta;
    Eigen::Vector3d current_position(manual_pose_.position.x, manual_pose_.position.y, manual_pose_.position.z);
    Eigen::Vector3d current_rotation = math_utils::quaterniondToEulerAngles(manual_pose_.rotation);
    Eigen::Vector3d default_position(default_pose_.position.x, default_pose_.position.y, default_pose_.position.z);
    Eigen::Vector3d default_rotation = math_utils::quaterniondToEulerAngles(default_pose_.rotation);
    Eigen::Vector3d max_position(body_pose_config.max_translation.x,
                                 body_pose_config.max_translation.y,
                                 body_pose_config.max_translation.z);
    Eigen::Vector3d max_rotation(body_pose_config.max_rotation.roll,
                                 body_pose_config.max_rotation.pitch,
                                 body_pose_config.max_rotation.yaw);

    Eigen::Vector3d translation_limit(0.0, 0.0, 0.0);
    Eigen::Vector3d rotation_limit(0.0, 0.0, 0.0);
    Eigen::Vector3d translation_velocity(0.0, 0.0, 0.0);
    Eigen::Vector3d rotation_velocity(0.0, 0.0, 0.0);
    Eigen::Vector3d desired_position(0.0, 0.0, 0.0);
    Eigen::Vector3d desired_rotation(0.0, 0.0, 0.0);

    bool reset_translation[3] = {false, false, false};
    bool reset_rotation[3] = {false, false, false};
    switch (pose_reset_mode_) {
    case Z_AND_YAW_RESET:
        reset_translation[2] = true;
        reset_rotation[2] = true;
        break;
    case X_AND_Y_RESET:
        reset_translation[0] = true;
        reset_translation[1] = true;
        break;
    case PITCH_AND_ROLL_RESET:
        reset_rotation[0] = true;
        reset_rotation[1] = true;
        break;
    case ALL_RESET:
        for (int i = 0; i < 3; ++i) {
            reset_translation[i] = true;
            reset_rotation[i] = true;
        }
        break;
    case IMMEDIATE_ALL_RESET:
        manual_pose_ = default_pose_;
        return;
    default:
        break;
    }

    for (int i = 0; i < 3; ++i) {
        if (reset_translation[i]) {
            double diff = current_position[i] - default_position[i];
            if (diff < 0.0) {
                translation_velocity_input_[i] = 1.0;
            } else if (diff > 0.0) {
                translation_velocity_input_[i] = -1.0;
            }
        }
        if (reset_rotation[i]) {
            double diff = current_rotation[i] - default_rotation[i];
            if (diff < 0.0) {
                rotation_velocity_input_[i] = 1.0;
            } else if (diff > 0.0) {
                rotation_velocity_input_[i] = -1.0;
            }
        }

        translation_velocity[i] = translation_velocity_input_[i] * body_pose_config.max_translation_velocity;
        rotation_velocity[i] = rotation_velocity_input_[i] * body_pose_config.max_rotation_velocity;

        desired_position[i] = current_position[i] + translation_velocity[i] * time_delta;
        desired_rotation[i] = current_rotation[i] + rotation_velocity[i] * time_delta;

        translation_limit[i] = math_utils::sign(translation_velocity[i]) * max_position[i];
        if (reset_translation[i] && default_position[i] < max_position[i] && default_position[i] > -max_position[i]) {
            translation_limit[i] = default_position[i];
        }

        bool positive_translation_velocity = math_utils::sign(translation_velocity[i]) > 0;
        bool exceeds_positive_translation_limit = positive_translation_velocity && desired_position[i] > translation_limit[i];
        bool exceeds_negative_translation_limit = !positive_translation_velocity && desired_position[i] < translation_limit[i];
        if (exceeds_positive_translation_limit || exceeds_negative_translation_limit) {
            translation_velocity[i] = (translation_limit[i] - current_position[i]) / time_delta;
        }

        rotation_limit[i] = math_utils::sign(rotation_velocity[i]) * max_rotation[i];
        if (reset_rotation[i] && default_rotation[i] < max_rotation[i] && default_rotation[i] > -max_rotation[i]) {
            rotation_limit[i] = default_rotation[i];
        }

        bool positive_rotation_velocity = math_utils::sign(rotation_velocity[i]) > 0;
        bool exceeds_positive_rotation_limit = positive_rotation_velocity && desired_rotation[i] > rotation_limit[i];
        bool exceeds_negative_rotation_limit = !positive_rotation_velocity && desired_rotation[i] < rotation_limit[i];
        if (exceeds_positive_rotation_limit || exceeds_negative_rotation_limit) {
            rotation_velocity[i] = (rotation_limit[i] - current_rotation[i]) / time_delta;
        }

        desired_position[i] = current_position[i] + translation_velocity[i] * time_delta;
        desired_rotation[i] = current_rotation[i] + rotation_velocity[i] * time_delta;
    }

    manual_pose_.position = Point3D(desired_position.x(), desired_position.y(), desired_position.z());
    manual_pose_.rotation = math_utils::correctRotation(math_utils::eulerAnglesToQuaterniond(desired_rotation, true),
                                                        Eigen::Quaterniond::Identity());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateInclinationPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateInclinationPose() {
    if (!imu_data_valid_) {
        inclination_pose_ = Pose::Identity();
        return;
    }

    Eigen::Quaterniond imu_orientation = Eigen::Quaterniond::Identity();
    if (imu_data_.absolute_data.absolute_orientation_valid) {
        Eigen::Vector3d imu_euler(math_utils::degreesToRadians(imu_data_.absolute_data.absolute_roll),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_pitch),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_yaw));
        imu_orientation = math_utils::eulerAnglesToQuaterniond(imu_euler);
    } else {
        Eigen::Vector3d imu_euler(math_utils::degreesToRadians(imu_data_.roll),
                                  math_utils::degreesToRadians(imu_data_.pitch),
                                  math_utils::degreesToRadians(imu_data_.yaw));
        imu_orientation = math_utils::eulerAnglesToQuaterniond(imu_euler);
    }

    Eigen::Quaterniond compensation_combined = (manual_pose_.rotation * auto_pose_.rotation).normalized();
    Eigen::Quaterniond compensation_removed = (imu_orientation * compensation_combined.inverse()).normalized();
    Eigen::Vector3d euler = math_utils::quaterniondToEulerAngles(compensation_removed);

    double longitudinal = -body_pose_config.body_clearance * std::tan(euler[1]);
    double lateral = body_pose_config.body_clearance * std::tan(euler[0]);
    longitudinal = math_utils::clamp(longitudinal, -body_pose_config.max_translation.x, body_pose_config.max_translation.x);
    lateral = math_utils::clamp(lateral, -body_pose_config.max_translation.y, body_pose_config.max_translation.y);

    inclination_pose_.position = Point3D(longitudinal, lateral, 0.0);
    inclination_pose_.rotation = Eigen::Quaterniond::Identity();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMU data input (replaces ROS subscription in OpenSHC)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::setIMUData(const IMUData &imu_data) {
    imu_data_ = imu_data;
    imu_data_valid_ = imu_data.is_valid;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateCurrentPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateCurrentPose(int robot_state, Leg legs[NUM_LEGS]) {
    Pose new_pose = Pose::Identity();

    updateWalkPlanePose(legs);
    new_pose = new_pose.addPose(walk_plane_pose_);

    if (manual_pose_enabled_) {
        updateManualPose();
        new_pose = new_pose.addPose(manual_pose_);
    }

    if (inclination_pose_enabled_) {
        updateInclinationPose();
        new_pose = new_pose.addPose(inclination_pose_);
    }

    bool running_state = (robot_state > 1);
    if (imu_pose_enabled_ && imu_data_valid_ && running_state) {
        updateIMUPosePID();
        new_pose = new_pose.addPose(imu_pose_);
    } else if (auto_pose_enabled && auto_pose_config.enabled) {
        updateAutoPose(legs);
        new_pose = new_pose.addPose(auto_pose_);
    }

    if (tip_align_pose_enabled_) {
        updateTipAlignPose(legs);
        new_pose = new_pose.addPose(tip_align_pose_);
    }

    if (ik_error_pose_enabled_) {
        updateIKErrorPose(legs);
        new_pose = new_pose.addPose(ik_error_pose_);
    }

    if (default_pose_enabled_) {
        calculateDefaultPose(legs);
        new_pose = new_pose.addPose(default_pose_);
    }

    body_pose_current_ = new_pose;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateIMUPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateIMUPosePID() {
    Eigen::Quaterniond current_rotation = Eigen::Quaterniond::Identity();
    if (imu_data_.absolute_data.absolute_orientation_valid) {
        Eigen::Vector3d euler_rad(math_utils::degreesToRadians(imu_data_.absolute_data.absolute_roll),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_pitch),
                                  math_utils::degreesToRadians(imu_data_.absolute_data.absolute_yaw));
        current_rotation = math_utils::eulerAnglesToQuaterniond(euler_rad);
    } else {
        Eigen::Vector3d euler_rad(math_utils::degreesToRadians(imu_data_.roll),
                                  math_utils::degreesToRadians(imu_data_.pitch),
                                  math_utils::degreesToRadians(imu_data_.yaw));
        current_rotation = math_utils::eulerAnglesToQuaterniond(euler_rad);
    }
    current_rotation = math_utils::correctRotation(current_rotation, Eigen::Quaterniond::Identity());

    Eigen::Quaterniond target_rotation = math_utils::correctRotation(manual_pose_.rotation,
                                                                     Eigen::Quaterniond::Identity());

    Eigen::Quaterniond rotation_error = (current_rotation * target_rotation.inverse()).normalized();

    double kp = model.getParams().body_comp.imu_pid_kp;
    double ki = model.getParams().body_comp.imu_pid_ki;
    double kd = model.getParams().body_comp.imu_pid_kd;
    double time_delta = model.getParams().time_delta;

    rotation_position_error_ = math_utils::quaterniondToEulerAngles(rotation_error);
    rotation_position_error_[2] = 0.0;

    constexpr double IMU_POSING_DEADBAND = 0.0;
    if (rotation_position_error_.norm() < IMU_POSING_DEADBAND) {
        return;
    }

    rotation_absement_error_ += rotation_position_error_ * time_delta;

    constexpr double smoothing_factor = 0.15;
    Eigen::Vector3d angular_velocity(math_utils::degreesToRadians(imu_data_.gyro_x),
                                     math_utils::degreesToRadians(imu_data_.gyro_y),
                                     math_utils::degreesToRadians(imu_data_.gyro_z));
    rotation_velocity_error_ = smoothing_factor * (-angular_velocity) +
                               (1.0 - smoothing_factor) * rotation_velocity_error_;

    Eigen::Vector3d rotation_correction = -(kd * rotation_velocity_error_ +
                                            kp * rotation_position_error_ +
                                            ki * rotation_absement_error_);

    double max_roll = body_pose_config.max_rotation.roll;
    double max_pitch = body_pose_config.max_rotation.pitch;
    if (max_roll > 0.0) {
        rotation_correction[0] = math_utils::clamp(rotation_correction[0], -max_roll, max_roll);
    }
    if (max_pitch > 0.0) {
        rotation_correction[1] = math_utils::clamp(rotation_correction[1], -max_pitch, max_pitch);
    }
    Eigen::Vector3d target_euler = math_utils::quaterniondToEulerAngles(target_rotation);
    rotation_correction[2] = target_euler[2];

    constexpr double STABILITY_THRESHOLD = 100.0;
    if (rotation_correction.norm() > STABILITY_THRESHOLD) {
        rotation_absement_error_ = Eigen::Vector3d::Zero();
        rotation_velocity_error_ = Eigen::Vector3d::Zero();
        return;
    }

    imu_pose_.rotation = math_utils::eulerAnglesToQuaterniond(rotation_correction);
    imu_pose_.rotation = math_utils::correctRotation(imu_pose_.rotation, target_rotation);
    imu_pose_.position = Point3D(0.0, 0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::calculateDefaultPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    if (legs_transitioning != 0) {
        if (recalculate_default_pose_) {
            Eigen::Vector3d zero_moment_offset = Eigen::Vector3d::Zero();

            for (int i = 0; i < NUM_LEGS; ++i) {
                LegState state = legs[i].getLegState();
                if (state == LEG_WALKING || state == LEG_MANUAL_TO_WALKING) {
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateTipAlignPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateTipAlignPose(Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i])
            continue;

        double swing_progress = legs[i].getSwingProgress();
        if (swing_progress < 0.0 || swing_progress > 1.0) {
            continue;
        }

        Eigen::Vector3d walk_plane_normal = walk_plane_pose_.rotation * Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond walk_plane_rotation = Eigen::Quaterniond::FromTwoVectors(
            Eigen::Vector3d::UnitZ(), walk_plane_normal);

        JointAngles angles = legs[i].getJointAngles();
        Point3D tip_to_joint_point = model.getTipToLastJointVectorGlobal(i, angles);
        Eigen::Vector3d tip_to_joint(tip_to_joint_point.x, tip_to_joint_point.y, tip_to_joint_point.z);
        double link_length = tip_to_joint.norm();

        Eigen::Vector3d a = walk_plane_rotation._transformVector(tip_to_joint);
        Eigen::Vector3d b = link_length * walk_plane_normal;
        Eigen::Vector3d rejection = a - (a.dot(b) / b.dot(b)) * b;
        Eigen::Vector3d translation_to_alignment = -rejection;

        a = Eigen::Vector3d(tip_align_pose_.position.x, tip_align_pose_.position.y, tip_align_pose_.position.z);
        b = walk_plane_normal;
        rejection = a - (a.dot(b) / b.dot(b)) * b;
        Eigen::Vector3d current_walk_plane_aligned = rejection;

        Eigen::Vector3d target_translation = current_walk_plane_aligned + translation_to_alignment;

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

        double c = math_utils::smoothStep(swing_progress);
        if (swing_progress < 0.5) {
            c = math_utils::smoothStep(c * 2.0);
            tip_align_pose_ = origin_tip_align_pose_.interpolate(c, Pose::Identity());
        } else {
            c = math_utils::smoothStep((c - 0.5) * 2.0);
            tip_align_pose_ = Pose::Identity().interpolate(
                c, Pose(Point3D(target_translation[0], target_translation[1], target_translation[2]),
                        Eigen::Quaterniond::Identity()));
        }

        if (swing_progress >= 1.0) {
            origin_tip_align_pose_ = tip_align_pose_;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateIKErrorPose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateIKErrorPose(Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
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
    ik_error_pose_.position.x *= 0.95;
    ik_error_pose_.position.y *= 0.95;
    ik_error_pose_.position.z *= 0.95;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateStance() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateStance(Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i]) {
            continue;
        }
        LegPoser *leg_poser = leg_posers_[i]->get();
        Pose current_pose = body_pose_current_;
        LegState leg_state = legs[i].getLegState();

        if (leg_state == LEG_WALKING || leg_state == LEG_MANUAL_TO_WALKING) {
            current_pose = current_pose.removePose(auto_pose_);
            current_pose = current_pose.addPose(leg_poser->getAutoPose());
            Point3D desired_tip = legs[i].getDesiredTipPosition();
            Point3D posed_tip = current_pose.inverseTransformVector(desired_tip);
            leg_poser->setCurrentTipPose(model, Pose(posed_tip, Eigen::Quaterniond::Identity()));
            legs[i].setDesiredTipPosition(leg_poser->getCurrentTipPose().position);
        } else if (leg_state == LEG_MANUAL || leg_state == LEG_WALKING_TO_MANUAL) {
            Point3D current_tip = legs[i].getCurrentTipPositionGlobal();
            leg_poser->setCurrentTipPose(model, Pose(current_tip, Eigen::Quaterniond::Identity()));
            legs[i].setDesiredTipPosition(leg_poser->getCurrentTipPose().position);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateWalkPlanePose() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BodyPoseController::updateWalkPlanePose(Leg legs[NUM_LEGS]) {
    // Generate control input for transitioning to new walk plane pose using swinging leg as reference (OpenSHC)
    Eigen::Vector3d walk_plane = Eigen::Vector3d::Zero();
    Eigen::Vector3d walk_plane_normal = Eigen::Vector3d::UnitZ();
    double c = 0.0;

    // Calculate swing_progress_scaler (OpenSHC parity: handles overlapping swing periods)
    double swing_progress_scaler = 1.0;
    if (gait_phase_offset_ > 0) {
        swing_progress_scaler = std::max(1.0, static_cast<double>(gait_swing_phase_) / gait_phase_offset_);
    }

    for (int i = 0; i < NUM_LEGS; ++i) {
        double swing_progress = legs[i].getSwingProgress() * swing_progress_scaler;
        if (swing_progress >= 0.0 && swing_progress <= 1.0) {
            c = math_utils::smoothStep(swing_progress);
            // Calculate walk plane from stance legs (HexaMotion: no per-leg LegStepper walk plane data)
            Point3D normal = calculateWalkPlaneNormal(legs);
            walk_plane_normal = Eigen::Vector3d(normal.x, normal.y, normal.z);
            walk_plane[2] = calculateWalkPlaneHeight(legs);
        }
    }

    // Align robot body with walk plane (OpenSHC)
    Pose new_walk_plane_pose;
    new_walk_plane_pose.rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), walk_plane_normal);
    new_walk_plane_pose.rotation = math_utils::correctRotation(new_walk_plane_pose.rotation,
                                                               Eigen::Quaterniond::Identity());

    // Pose robot body along normal of walk plane, offset by body clearance (OpenSHC)
    Eigen::Vector3d body_clearance(0.0, 0.0, body_pose_config.body_clearance);
    Eigen::Vector3d offset = new_walk_plane_pose.rotation._transformVector(body_clearance);
    new_walk_plane_pose.position = Point3D(offset.x(), offset.y(), offset.z() + walk_plane[2]);

    // Interpolate walk plane pose from origin to new (OpenSHC)
    walk_plane_pose_ = origin_walk_plane_pose_.interpolate(c, new_walk_plane_pose);
    if (c == 1.0) {
        origin_walk_plane_pose_ = walk_plane_pose_;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walk plane calculation helpers (HexaMotion adaptation - stance-leg-based since no per-leg LegStepper walk plane)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point3D BodyPoseController::calculateWalkPlaneNormal(Leg legs[NUM_LEGS]) const {
    std::vector<Point3D> stance_points;
    for (int i = 0; i < NUM_LEGS; i++) {
        if (legs[i].getStepPhase() == STANCE_PHASE) {
            stance_points.push_back(legs[i].getCurrentTipPositionGlobal());
        }
    }

    if (stance_points.size() < 3) {
        return Point3D(0.0, 0.0, 1.0);
    }

    std::vector<double> raw_A;
    std::vector<double> raw_B;
    for (const auto &point : stance_points) {
        raw_A.push_back(point.x);
        raw_A.push_back(point.y);
        raw_A.push_back(1.0);
        raw_B.push_back(point.z);
    }

    double a, b, c_val;
    if (math_utils::solveLeastSquaresPlane(raw_A.data(), raw_B.data(), stance_points.size(), a, b, c_val)) {
        double normal_magnitude = std::sqrt(a * a + b * b + 1.0);
        Point3D normal(-a / normal_magnitude, -b / normal_magnitude, 1.0 / normal_magnitude);
        if (normal.z < 0) {
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }
        return normal;
    }

    return Point3D(0.0, 0.0, 1.0);
}

double BodyPoseController::calculateWalkPlaneHeight(Leg legs[NUM_LEGS]) const {
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
    return walk_plane_pose_.position.z - body_pose_config.body_clearance;
}

Pose BodyPoseController::getWalkPlanePose() const {
    return walk_plane_pose_;
}

void BodyPoseController::setWalkPlanePose(const Pose &pose) {
    walk_plane_pose_ = pose;
    origin_walk_plane_pose_ = pose;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::updateAutoPose() equivalent — 1:1 port
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool BodyPoseController::updateAutoPose(Leg legs[NUM_LEGS]) {
    if (!auto_pose_enabled || !auto_pose_config.enabled || auto_poser_container_.empty()) {
        auto_pose_ = Pose::Identity();
        auto_posing_state_ = POSING_COMPLETE;
        return true;
    }

    auto_pose_ = Pose::Identity();

    // Update auto posing state from walk state (OpenSHC parity)
    if (current_walk_state_ == WALK_STARTING || current_walk_state_ == WALK_MOVING) {
        auto_posing_state_ = POSING;
    } else if ((body_velocity_zero_ && current_walk_state_ == WALK_STOPPING) ||
               current_walk_state_ == WALK_STOPPED) {
        auto_posing_state_ = STOP_POSING;
    }

    // Update master phase (OpenSHC parity: sync_with_step_cycle or free-running)
    int master_phase;
    bool sync_with_step_cycle = (pose_frequency_ == -1.0);
    if (sync_with_step_cycle) {
        master_phase = pose_phase_; // Phase already set by caller via updateAutoPose(gait_phase, legs)
    } else {
        master_phase = pose_phase_;
        pose_phase_ = (pose_phase_ + 1) % std::max(1, pose_phase_length_);
    }

    if (pose_phase_length_ <= 0) {
        pose_phase_length_ = std::max(1, auto_pose_config.pose_phase_length);
    }

    // Estimate gravity direction
    Eigen::Vector3d gravity = Eigen::Vector3d::UnitZ();
    if (imu_data_valid_) {
        if (imu_data_.absolute_data.linear_acceleration_valid) {
            gravity = Eigen::Vector3d(-imu_data_.absolute_data.linear_accel_x,
                                      -imu_data_.absolute_data.linear_accel_y,
                                      -imu_data_.absolute_data.linear_accel_z);
        } else {
            gravity = Eigen::Vector3d(-imu_data_.accel_x, -imu_data_.accel_y, -imu_data_.accel_z);
        }
    }
    if (gravity.norm() < 1e-9) {
        gravity = Eigen::Vector3d::UnitZ();
    }
    gravity.normalize();

    // Update auto pose from auto posers (OpenSHC parity — new signature with normaliser/PosingState)
    int auto_posers_complete = 0;
    for (size_t i = 0; i < auto_poser_container_.size(); ++i) {
        Pose component = auto_poser_container_[i]->updatePose(
            master_phase, pose_phase_length_, normaliser_,
            static_cast<int>(auto_posing_state_), pose_frequency_, gravity);
        auto_posers_complete += int(!auto_poser_container_[i]->isPosing());
        auto_pose_ = auto_pose_.addPose(component);
    }

    // All auto posers have completed their required posing cycle (OpenSHC parity)
    if (auto_posers_complete == static_cast<int>(auto_poser_container_.size())) {
        auto_posing_state_ = POSING_COMPLETE;
    }

    // Update leg specific auto pose using leg posers (OpenSHC parity — new signature)
    for (int leg_index = 0; leg_index < NUM_LEGS; ++leg_index) {
        if (leg_posers_[leg_index]) {
            leg_posers_[leg_index]->get()->updateAutoPose(
                master_phase, auto_pose_, normaliser_, pose_phase_length_);
        }
    }

    (void)legs;
    return true;
}

bool BodyPoseController::updateAutoPose(int gait_phase, Leg legs[NUM_LEGS]) {
    pose_phase_ = gait_phase;
    return updateAutoPose(legs);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::stepToNewStance() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::stepToNewStance() {
    if (!legs_ref_) {
        return 0;
    }
    Leg *legs = legs_ref_;
    int progress = 0;
    int leg_count = NUM_LEGS;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (i == tripod_leg_groups[current_group_][0] ||
            i == tripod_leg_groups[current_group_][1] ||
            i == tripod_leg_groups[current_group_][2]) {
            if (!leg_posers_[i]) {
                continue;
            }
            Pose target_pose(Point3D(body_pose_config.leg_stance_positions[i].x,
                                     body_pose_config.leg_stance_positions[i].y,
                                     body_pose_config.leg_stance_positions[i].z),
                             Eigen::Quaterniond::Identity());
            double step_height = body_pose_config.swing_height;
            double step_time = 1.0 / std::max(0.001, model.getParams().step_frequency);
            progress = leg_posers_[i]->get()->stepToPosition(target_pose, Pose::Identity(), step_height, step_time, true);
            Point3D p = leg_posers_[i]->get()->getCurrentPosition();
            legs[i].setDesiredTipPosition(p);
            legs[i].applyIK(p);
            legs_completed_step_ += int(progress == PROGRESS_COMPLETE);
        }
    }

    progress = progress / 2 + current_group_ * 50;
    current_group_ = legs_completed_step_ / (leg_count / 2);
    if (legs_completed_step_ >= leg_count) {
        legs_completed_step_ = 0;
        current_group_ = 0;
    }
    reset_transition_sequence_ = true;
    return progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::poseForLegManipulation() equivalent — 1:1 port
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::poseForLegManipulation() {
    if (!legs_ref_) {
        return 0;
    }
    Leg *legs = legs_ref_;
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

    const Parameters &params = model.getParams();
    double step_height = body_pose_config.swing_height;
    double step_time = 1.0 / std::max(0.001, params.step_frequency);

    Pose target_pose;
    int min_progress = 100;

    for (int i = 0; i < NUM_LEGS; ++i) {
        LegPoser *leg_poser = getLegPoser(i);
        if (!leg_poser)
            continue;

        // Set up target pose for legs depending on state (OpenSHC parity)
        if (legs[i].getLegState() == LEG_WALKING_TO_MANUAL) {
            target_pose = Pose::Identity();
            target_pose.position = target_pose.position + inclination_pose_.position;
            target_pose.position.z -= step_height; // Pose leg at step height to begin manipulation
        } else {
            // Use current composed pose minus manual plus default (OpenSHC parity)
            target_pose = body_pose_current_;
            target_pose.position.x -= manual_pose_.position.x;
            target_pose.position.y -= manual_pose_.position.y;
            target_pose.position.z -= manual_pose_.position.z;
            target_pose.position.x += default_pose_.position.x;
            target_pose.position.y += default_pose_.position.y;
            target_pose.position.z += default_pose_.position.z;
        }

        Point3D default_tip = leg_poser->getDefaultTipPose();
        Point3D target_tip_position = target_pose.inverseTransformVector(default_tip);

        Pose target_tip_pose(target_tip_position, Eigen::Quaterniond::Identity());

        double effective_step_height = step_height;

        // Set walker tip position for use in manual or walking mode (OpenSHC parity)
        if (legs[i].getLegState() == LEG_WALKING_TO_MANUAL) {
            leg_poser->setCurrentTipPose(model, target_tip_pose);
            effective_step_height = 0.0; // Zero step height in transition from WALKING to MANUAL
        } else if (legs[i].getLegState() == LEG_MANUAL_TO_WALKING) {
            Pose default_pose(default_tip, Eigen::Quaterniond::Identity());
            leg_poser->setCurrentTipPose(model, default_pose);
        }

        int progress = leg_poser->stepToPosition(target_tip_pose, Pose::Identity(),
                                                 effective_step_height, step_time);
        min_progress = std::min(progress, min_progress);

        if (progress != PROGRESS_COMPLETE) {
            Point3D tip_pos = leg_poser->getCurrentTipPose().position;
            legs[i].setDesiredTipPosition(tip_pos);
            legs[i].applyIK(tip_pos);
        }
    }

    return min_progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::directStartup() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::directStartup(Leg legs[NUM_LEGS]) {
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }
    int progress = 0;
    double time_to_start = body_pose_config.time_to_start;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i]) {
            continue;
        }
        const StandingPoseJoints &standing = body_pose_config.standing_pose_joints[i];
        leg_posers_[i]->get()->setDesiredConfiguration(standing.coxa, standing.femur, standing.tibia);
        progress = leg_posers_[i]->get()->transitionConfiguration(time_to_start);
    }
    executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
    return progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::packLegs() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::packLegs(double time_to_pack, Leg legs[NUM_LEGS]) {
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }
    int progress = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i]) {
            continue;
        }
        const JointPoseAngles &packed = model.getParams().packed_pose_joints[i];
        leg_posers_[i]->get()->setDesiredConfiguration(packed.coxa, packed.femur, packed.tibia);
        progress = leg_posers_[i]->get()->transitionConfiguration(time_to_pack);
    }
    executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
    if (progress == PROGRESS_COMPLETE) {
        pack_step_ = std::min(pack_step_ + 1, 1);
    }
    return progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::unpackLegs() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::unpackLegs(double time_to_unpack, Leg legs[NUM_LEGS]) {
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }
    int progress = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i]) {
            continue;
        }
        const JointPoseAngles &unpacked = model.getParams().unpacked_pose_joints[i];
        leg_posers_[i]->get()->setDesiredConfiguration(unpacked.coxa, unpacked.femur, unpacked.tibia);
        progress = leg_posers_[i]->get()->transitionConfiguration(time_to_unpack);
    }
    executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
    if (progress == PROGRESS_COMPLETE) {
        pack_step_ = std::max(pack_step_ - 1, 0);
    }
    return progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::transitionConfiguration() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::transitionConfiguration(double transition_time, Leg legs[NUM_LEGS]) {
    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }
    int min_progress = INT_MAX;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!leg_posers_[i]) {
            continue;
        }
        int progress = leg_posers_[i]->get()->transitionConfiguration(transition_time);
        min_progress = std::min(min_progress, progress);
    }
    if (min_progress == INT_MAX) {
        min_progress = PROGRESS_COMPLETE;
    }
    executing_transition_ = (min_progress != 0 && min_progress != PROGRESS_COMPLETE);
    return min_progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::transitionStance() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::transitionStance(Leg legs[NUM_LEGS], double transition_time) {
    int min_progress = INT_MAX;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (!leg_posers_[i])
            continue;

        LegPoser *leg_poser = leg_posers_[i]->get();
        ExternalTarget target = leg_poser->getExternalTarget();
        Pose target_tip_pose = Pose::Identity();
        double swing_clearance = 0.0;

        if (target.defined) {
            target_tip_pose.position = target.transform.position + target.pose.position;
            target_tip_pose.rotation = target.transform.rotation * target.pose.rotation;
            swing_clearance = target.swing_clearance;
        }

        int progress = leg_poser->stepToPosition(target_tip_pose, body_pose_current_,
                                                 swing_clearance, transition_time, true);

        Point3D desired_tip = leg_poser->getCurrentPosition();
        legs[i].setCurrentTipPositionGlobal(desired_tip);
        JointAngles current_angles = legs[i].getJointAngles();
        legs[i].setJointAngles(model.inverseKinematicsCurrentGlobalCoordinates(i, current_angles, desired_tip));
        legs[i].setCurrentTipPositionGlobal(desired_tip);

        min_progress = std::min(progress, min_progress);

        if (target.defined && progress == PROGRESS_COMPLETE) {
            target.defined = false;
            leg_poser->setExternalTarget(target);
        }
    }

    executing_transition_ = (min_progress != 0 && min_progress != PROGRESS_COMPLETE);
    return min_progress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC PoseController::executeSequence() equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int BodyPoseController::executeSequence(const SequenceSelection &sequence, Leg legs[NUM_LEGS]) {
    executing_transition_ = true;
    int progress = executeSequenceInternal(sequence, legs);
    if (progress == PROGRESS_COMPLETE) {
        executing_transition_ = false;
    }
    return progress;
}

int BodyPoseController::executeSequenceInternal(const SequenceSelection &sequence, Leg legs[NUM_LEGS]) {
    const bool is_startup = sequence == START_UP;

    if (!getLegPoser(0)) {
        initializeLegPosers(legs);
    }

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
    if (sequence == SHUT_DOWN) {
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
            {model.getCoxaAngleLimitRad(0), model.getCoxaAngleLimitRad(1)},
            {model.getFemurAngleLimitRad(0), model.getFemurAngleLimitRad(1)},
            {model.getTibiaAngleLimitRad(0), model.getTibiaAngleLimitRad(1)}};
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

        bool direct_step = !legsBearingLoad(legs);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenSHC Model::legsBearingLoad() equivalent (used by executeSequenceInternal)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool BodyPoseController::legsBearingLoad(const Leg legs[NUM_LEGS]) const {
    double body_height_estimate = 0.0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        body_height_estimate += legs[i].getCurrentTipPositionGlobal().z;
    }
    return -(body_height_estimate / NUM_LEGS) > HALF_BODY_DEPTH_MM;
}
