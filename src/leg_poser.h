#ifndef LEG_POSER_H
#define LEG_POSER_H

#include "body_pose_config.h" // AutoPoseConfiguration & BodyPoseConfiguration
#include "hexamotion_constants.h"
#include "leg.h"
#include "math_utils.h" // For clamp utility used in defensive admittance validation
#include "robot_model.h"
#include <memory>
#include <vector>

/**
 * @brief LegPoser class for HexaMotion
 *
 * This class handles the lower-level mechanisms for moving a leg to defined target tip positions.
 * Adapted from OpenSHC's LegPoser class but simplified for HexaMotion architecture.
 * No progress tracking - that's handled by LocomotionSystem.
 */
class LegPoser {
  public:
    /**
     * @brief Constructor
     * @param leg_index Index of the leg this poser controls
     * @param leg Reference to the leg object
     * @param robot_model Reference to the robot model for parameter access
     */
    LegPoser(int leg_index, Leg &leg, RobotModel &robot_model);

    /**
     * @brief Copy constructor
     * @param leg_poser Pointer to the Leg Poser object to be copied from
     */
    LegPoser(const LegPoser *leg_poser);

    // Accessors
    inline int getLegIndex() const { return leg_index_; }
    /**
     * @brief Get the current tip pose (OpenSHC parity: returns stored current_tip_pose_).
     * The stored value is set by setCurrentTipPose() from BodyPoseController::updateStance().
     */
    inline const Pose &getCurrentTipPose() const { return current_tip_pose_; }
    inline Pose getTargetTipPose() const { return target_tip_pose_; }
    inline Pose getAutoPose() const { return auto_pose_; }
    inline bool getLegCompletedStep() const { return leg_completed_step_; }
    /**
     * @brief Get the default (stance) tip position for this leg.
     * @return Default tip position from the associated Leg object
     */
    inline Point3D getDefaultTipPose() const { return leg_.getDefaultTipPosition(); }

    // Transition sequence helpers (OpenSHC parity)
    /** Reset stored transition poses for sequence generation. */
    void resetTransitionSequence() { transition_poses_.clear(); }
    /** Append a transition pose in sequence order. */
    void addTransitionPose(const Pose &pose) { transition_poses_.push_back(pose); }
    /** Check if a transition pose exists for the given index. */
    bool hasTransitionPose(int index) const {
        return index >= 0 && static_cast<size_t>(index) < transition_poses_.size();
    }
    /** Get transition pose by index (identity if missing). */
    Pose getTransitionPose(int index) const {
        if (!hasTransitionPose(index)) {
            return Pose::Identity();
        }
        return transition_poses_[index];
    }

    // Progress (0.0 - 1.0) of current stepping maneuver
    double getCurrentStepProgress() const {
        if (current_num_iterations_ <= 1) {
            return leg_completed_step_ ? 1.0 : 0.0;
        }
        double prog = static_cast<double>(master_iteration_count_ - 1) / static_cast<double>(current_num_iterations_);
        if (prog < 0.0)
            prog = 0.0;
        if (prog > 1.0)
            prog = 1.0;
        return prog;
    }

    // Modifiers
    // OpenSHC parity: only store in LegPoser's current_tip_pose_, do NOT modify
    // the Leg's global tip position. The Leg's tip is updated exclusively via FK
    // after IK (setJointAngles → updateTipPosition). Writing it here would make
    // current == desired in the IK pipeline, producing zero movement.
    inline void setCurrentTipPose(const RobotModel &model, const Pose &current) {
        current_tip_pose_ = current;
    }
    inline void setTargetTipPose(const Pose &target) { target_tip_pose_ = target; }
    inline void setAutoPose(const Pose &auto_pose) { auto_pose_ = auto_pose; }
    inline void setLegCompletedStep(bool complete) { leg_completed_step_ = complete; }

    // Negation parameter setters (OpenSHC parity: set from setAutoPoseParams)
    inline void setPoseNegationPhaseStart(int start) { pose_negation_phase_start_ = start; }
    inline void setPoseNegationPhaseEnd(int end) { pose_negation_phase_end_ = end; }
    inline void setNegationTransitionRatio(double ratio) { negation_transition_ratio_ = ratio; }
    // Admittance delta (external compliance offset) setter
    inline void setAdmittanceDelta(const Point3D &delta) {
        // Defensive validation: sanitize NaN/Inf and clamp to safe engineering bounds (class constant)
        auto sanitize = [&](double v) {
            if (!std::isfinite(v))
                return 0.0; // Replace NaN/Inf with 0
            return math_utils::clamp(v, -ADMITTANCE_MAX_ABS_DELTA_MM, ADMITTANCE_MAX_ABS_DELTA_MM);
        };
        admittance_delta_.x = sanitize(delta.x);
        admittance_delta_.y = sanitize(delta.y);
        admittance_delta_.z = sanitize(delta.z);
        // Optional micro-noise deadband (eliminate jitter below 0.01 mm)
        auto deadband = [](double v) { return (std::fabs(v) < 0.01) ? 0.0 : v; };
        admittance_delta_.x = deadband(admittance_delta_.x);
        admittance_delta_.y = deadband(admittance_delta_.y);
        admittance_delta_.z = deadband(admittance_delta_.z);
    }
    inline Point3D getAdmittanceDelta() const { return admittance_delta_; }

    /**
     * @brief Reset the key variables of stepToPosition() ready for new stepping maneuver
     */
    inline void resetStepToPosition() {
        first_iteration_ = true;
        master_iteration_count_ = 0;
    }

    /**
     * @brief Set the desired joint configuration for transitionConfiguration.
     * @param coxa Target coxa angle (radians)
     * @param femur Target femur angle (radians)
     * @param tibia Target tibia angle (radians)
     */
    inline void setDesiredConfiguration(double coxa, double femur, double tibia) {
        desired_config_coxa_ = coxa;
        desired_config_femur_ = femur;
        desired_config_tibia_ = tibia;
        desired_config_set_ = true;
    }

    /**
     * @brief Uses a cubic bezier curve to smoothly transition joint positions from current
     * to target configuration (OpenSHC LegPoser::transitionConfiguration equivalent).
     * @param transition_time The time period in which to complete this transition (seconds)
     * @return Progress percentage (0-100), 100 indicates completion
     */
    int transitionConfiguration(double transition_time);

    /**
     * @brief Uses bezier curves to smoothly update the desired tip position of the leg
     * @param target_tip_pose The target tip pose in reference to the body centre frame
     * @param target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
     * @param lift_height The height which the stepping leg trajectory should reach at its peak
     * @param time_to_step The time period to complete this maneuver
     * @param apply_delta A bool defining if a position offset value should be applied to the target tip position
     * @return true if step is complete, false if still in progress
     */
    int stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
                       double lift_height, double time_to_step, bool apply_delta = true);

    /**
     * @brief Update leg-specific auto pose (1:1 port of OpenSHC LegPoser::updateAutoPose).
     *
     * Takes the global auto_pose_ from BodyPoseController and applies per-leg negation
     * using iteration-based first_half/smoothStep logic, matching OpenSHC exactly.
     *
     * @param phase Current master phase index.
     * @param global_auto_pose Global auto pose from BodyPoseController aggregation.
     * @param normaliser Normaliser for scaling negation phase windows.
     * @param phase_length Total normalised phase length (pose_phase_length_).
     */
    void updateAutoPose(int phase, const Pose &global_auto_pose, int normaliser, int phase_length);

    /**
     * @brief Set target position for leg movement
     * @param target_position Target position in world coordinates
     */
    inline void setTargetPosition(const Point3D &target_position) {
        target_tip_pose_ = Pose(target_position, Eigen::Vector3d(0, 0, 0));
    }

    /**
     * @brief Get target position for leg movement
     * @return Target position in world coordinates
     */
    Point3D getTargetPosition() const {
        return target_tip_pose_.position;
    }

    /**
     * @brief Get current position of the leg
     * @return Current position in world coordinates
     */
    Point3D getCurrentPosition() const {
        return current_tip_pose_.position;
    }

    /**
     * @brief Simplified stepToPosition method for stance transitions
     * @param target_position Target position in world coordinates
     * @param step_height Height for leg lifting during transition
     * @param step_time Time for the step transition
     * @return true if step is complete, false if still in progress
     */
    bool stepToPosition(const Point3D &target_position, double step_height, double step_time) {
        Pose target_pose(target_position, Eigen::Vector3d(0, 0, 0));
        return stepToPosition(target_pose, Pose::Identity(), step_height, step_time, false) == PROGRESS_COMPLETE;
    }

  private:
    int leg_index_;           //< Index of the leg this poser controls
    Leg &leg_;                //< Reference to the Leg object this poser controls
    RobotModel &robot_model_; //< Reference to the robot model for parameter access

    Pose auto_pose_;                         //< Leg specific auto pose (post-negation)
    int pose_negation_phase_start_ = 0;      //< Base phase start for negation window (pre-normaliser)
    int pose_negation_phase_end_ = 0;        //< Base phase end for negation window (pre-normaliser)
    double negation_transition_ratio_ = 0.0; //< Ratio of window used for ramp in/out
    bool negate_auto_pose_ = false;          //< Flag if currently negating
    bool first_iteration_ = true;            //< Flag denoting if an iterating function is on it's first iteration
    int master_iteration_count_ = 0;         //< Master iteration count used in generating time input for bezier curves
    int current_num_iterations_ = 0;         //< Total iterations for current step (for progress reporting)

    Pose origin_tip_pose_;            //< Origin tip pose used in bezier curve equations
    Pose current_tip_pose_;           //< Current tip pose
    Pose target_tip_pose_;            //< Target tip pose used in bezier curve equations
    bool leg_completed_step_ = false; //< Flag denoting if leg has completed its required step in a sequence

    // OpenSHC-style transition sequence poses
    std::vector<Pose> transition_poses_;

    double physical_reference_height_;  //< Physical reference height (z = getDefaultHeightOffset() when all angles are 0°)
    Point3D admittance_delta_{0, 0, 0}; //< Latest admittance (compliance) delta applied when apply_delta=true

    // transitionConfiguration state (OpenSHC LegPoser parity)
    bool desired_config_set_ = false;    //< Flag if desired configuration has been set
    bool config_first_iteration_ = true; //< First iteration flag for configuration transition
    int config_iteration_count_ = 0;     //< Iteration counter for configuration transition
    double desired_config_coxa_ = 0.0;   //< Target coxa angle (radians)
    double desired_config_femur_ = 0.0;  //< Target femur angle (radians)
    double desired_config_tibia_ = 0.0;  //< Target tibia angle (radians)
    double origin_config_coxa_ = 0.0;    //< Origin coxa angle (radians)
    double origin_config_femur_ = 0.0;   //< Origin femur angle (radians)
    double origin_config_tibia_ = 0.0;   //< Origin tibia angle (radians)
};

#endif // LEG_POSER_H