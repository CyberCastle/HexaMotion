#ifndef AUTO_POSER_H
#define AUTO_POSER_H

#include "math_utils.h"
#include "robot_model.h"

// Forward-declare PosingState so AutoPoser can use it without pulling in state_controller.h.
// The canonical definition lives in locomotion_types.h (included via robot_model.h).
// If PosingState is not yet visible, define a compatible fallback.
#ifndef POSING_STATE_DEFINED
// Will be resolved at link time – locomotion_types.h is the authoritative source.
#endif

/**
 * @brief OpenSHC-style automatic pose component generator (1:1 port of OpenSHC AutoPoser).
 *
 * Each AutoPoser contributes one phased quartic Bézier pose component.
 * Multiple AutoPoser instances are aggregated by BodyPoseController.
 *
 * Normaliser scaling, mutually exclusive gravity amplitude, and PosingState-aware
 * lifecycle coordination match OpenSHC's AutoPoser::updatePose() exactly.
 */
class AutoPoser {
  public:
    /**
     * @brief Construct a pose component with a stable identifier.
     * @param id Unique component identifier.
     */
    explicit AutoPoser(int id)
        : id_number_(id), start_phase_(0), end_phase_(0), start_check_(false),
          end_check_(false, false), allow_posing_(false),
          x_amplitude_(0.0), y_amplitude_(0.0), z_amplitude_(0.0), gravity_amplitude_(0.0),
          roll_amplitude_(0.0), pitch_amplitude_(0.0), yaw_amplitude_(0.0) {}

    /** @brief Get component identifier. */
    int getIDNumber() const { return id_number_; }

    /** @brief Set phase window start (inclusive). */
    void setStartPhase(int start_phase) { start_phase_ = start_phase; }
    /** @brief Set phase window end (exclusive, cyclic). */
    void setEndPhase(int end_phase) { end_phase_ = end_phase; }

    /** @brief Set translation amplitudes in mm. */
    void setXAmplitude(double x) { x_amplitude_ = x; }
    void setYAmplitude(double y) { y_amplitude_ = y; }
    void setZAmplitude(double z) { z_amplitude_ = z; }
    /** @brief Set gravity-aligned translation amplitude in mm. */
    void setGravityAmplitude(double gravity) { gravity_amplitude_ = gravity; }

    /** @brief Set rotation amplitudes in radians. */
    void setRollAmplitude(double roll) { roll_amplitude_ = roll; }
    void setPitchAmplitude(double pitch) { pitch_amplitude_ = pitch; }
    void setYawAmplitude(double yaw) { yaw_amplitude_ = yaw; }

    /** @brief Reset start/end checks used by phase edge detection (OpenSHC parity). */
    void resetChecks() {
        start_check_ = false;
        end_check_ = std::pair<bool, bool>(false, false);
        allow_posing_ = false;
    }

    /** @brief Return whether component is actively posing. */
    bool isPosing() const { return allow_posing_; }

    /**
     * @brief Update phased component pose (1:1 port of OpenSHC AutoPoser::updatePose).
     *
     * Uses normaliser-scaled phase windows, mutually exclusive gravity/xyz amplitudes,
     * and PosingState-aware lifecycle (posing only ends after a FULL cycle completes
     * while in STOP_POSING state).
     *
     * @param phase Current master phase index.
     * @param phase_length Total normalised phase length (pose_phase_length_).
     * @param normaliser Normaliser value for scaling base phases.
     * @param posing_state Current PosingState from BodyPoseController.
     * @param pose_frequency Pose frequency (-1.0 = gait-synced).
     * @param gravity_direction Estimated gravity direction in body frame.
     * @return Pose component for current phase.
     */
    Pose updatePose(int phase, int phase_length, int normaliser, int posing_state,
                    double pose_frequency, const Eigen::Vector3d &gravity_direction) {
        Pose return_pose = Pose::Identity();

        // Scale base phases by normaliser (OpenSHC: start_phase_ * poser_->getNormaliser())
        int start_phase = start_phase_ * normaliser;
        int end_phase = end_phase_ * normaliser;

        // Handle phase overlapping master phase start/end (OpenSHC parity)
        if (start_phase > end_phase) {
            end_phase += phase_length;
            if (phase < start_phase) {
                phase += phase_length;
            }
        }

        bool sync_with_step_cycle = (pose_frequency == -1.0);

        // Coordinate starting/stopping of posing period (OpenSHC parity):
        // Posing only ends once a FULL posing cycle completes whilst in STOP_POSING state.
        // PosingState values: POSING=0, STOP_POSING=1, POSING_COMPLETE=2
        start_check_ = !sync_with_step_cycle || (!start_check_ && posing_state == 0 && phase == start_phase);
        end_check_.first = (end_check_.first || (posing_state == 1 && phase == start_phase));
        end_check_.second = (end_check_.second || (posing_state == 1 && phase == end_phase && end_check_.first));
        if (!allow_posing_ && start_check_) {
            allow_posing_ = true;
            end_check_ = std::pair<bool, bool>(false, false);
        } else if (allow_posing_ && sync_with_step_cycle && end_check_.first && end_check_.second) {
            allow_posing_ = false;
            start_check_ = false;
        }

        // Pose if in correct phase (OpenSHC parity)
        if (phase >= start_phase && phase < end_phase && allow_posing_) {
            int iteration = phase - start_phase + 1;
            int num_iterations = end_phase - start_phase;

            Eigen::Vector3d zero(0.0, 0.0, 0.0);
            Eigen::Vector3d position_control_nodes[5] = {zero, zero, zero, zero, zero};
            Eigen::Vector3d rotation_control_nodes[5] = {zero, zero, zero, zero, zero};

            bool first_half = iteration <= num_iterations / 2;
            Eigen::Vector3d gravity = gravity_direction;
            if (gravity.norm() < 1e-9) {
                gravity = Eigen::Vector3d::UnitZ();
            }
            gravity.normalize();

            // Mutually exclusive gravity/xyz amplitudes (OpenSHC parity):
            // If gravity_amplitude_ != 0, use gravity-aligned; otherwise use xyz.
            if (first_half) {
                rotation_control_nodes[3] = Eigen::Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
                rotation_control_nodes[4] = Eigen::Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
                if (gravity_amplitude_ != 0.0) {
                    position_control_nodes[3] = gravity * gravity_amplitude_;
                    position_control_nodes[4] = gravity * gravity_amplitude_;
                } else {
                    position_control_nodes[3] = Eigen::Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
                    position_control_nodes[4] = Eigen::Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
                }
            } else {
                rotation_control_nodes[0] = Eigen::Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
                rotation_control_nodes[1] = Eigen::Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
                if (gravity_amplitude_ != 0.0) {
                    position_control_nodes[0] = gravity * gravity_amplitude_;
                    position_control_nodes[1] = gravity * gravity_amplitude_;
                } else {
                    position_control_nodes[0] = Eigen::Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
                    position_control_nodes[1] = Eigen::Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
                }
            }

            double delta_t = 1.0 / (num_iterations / 2.0);
            int offset = static_cast<int>((first_half ? 0 : num_iterations / 2.0));
            double time_input = (iteration - offset) * delta_t;

            // Use Eigen Vector3d Bézier directly (OpenSHC uses quarticBezier with Vector3d)
            Point3D p_nodes[5], r_nodes[5];
            for (int i = 0; i < 5; ++i) {
                p_nodes[i] = Point3D(position_control_nodes[i].x(), position_control_nodes[i].y(),
                                     position_control_nodes[i].z());
                r_nodes[i] = Point3D(rotation_control_nodes[i].x(), rotation_control_nodes[i].y(),
                                     rotation_control_nodes[i].z());
            }
            Point3D p = math_utils::quarticBezier(p_nodes, time_input);
            Point3D r = math_utils::quarticBezier(r_nodes, time_input);
            Eigen::Vector3d euler_rad(r.x, r.y, r.z);
            return_pose = Pose(p, math_utils::eulerAnglesToQuaterniond(euler_rad, true));
        }

        return return_pose;
    }

  private:
    int id_number_;
    int start_phase_;
    int end_phase_;

    bool start_check_;
    std::pair<bool, bool> end_check_; /**< OpenSHC end_check_ pair for lifecycle coordination. */
    bool allow_posing_;

    double x_amplitude_;
    double y_amplitude_;
    double z_amplitude_;
    double gravity_amplitude_;
    double roll_amplitude_;
    double pitch_amplitude_;
    double yaw_amplitude_;
};

#endif // AUTO_POSER_H
