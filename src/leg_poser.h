#ifndef LEG_POSER_H
#define LEG_POSER_H

#include "robot_model.h"
#include "body_pose_controller.h"
#include "leg.h"
#include <vector>
#include <memory>

// Forward declarations
class BodyPoseController;
class LocomotionSystem;

/**
 * @brief External target structure for leg positioning
 * Equivalent to OpenSHC's ExternalTarget
 */
struct ExternalTarget {
    Pose pose;              //< The target tip pose
    double swing_clearance; //< The height of the swing trajectory clearance
    std::string frame_id;   //< The target tip pose reference frame id
    unsigned long time;     //< The time of the request for the target tip pose (milliseconds)
    Pose transform;         //< The transform between reference frames
    bool defined = false;   //< Flag denoting if external target object has been defined
};

/**
 * @brief Joint configuration structure for HexaMotion
 * Simplified version of sensor_msgs::JointState
 */
struct JointConfiguration {
    std::vector<std::string> names;
    std::vector<double> positions;

    void clear() {
        names.clear();
        positions.clear();
    }

    size_t size() const {
        return names.size();
    }
};

/**
 * @brief LegPoser class for HexaMotion
 *
 * This class handles the lower-level mechanisms for moving a leg to defined target tip or joint positions
 * and stores several leg specific variables including the current tip position of the leg and saved
 * transition target tip positions for generated robot motion sequences.
 *
 * Adapted from OpenSHC's LegPoser class for HexaMotion architecture.
 */
class LegPoser {
public:
    /**
     * @brief Constructor
     * @param body_pose_controller Pointer to the Body Pose Controller object
     * @param locomotion_system Pointer to the locomotion system
     * @param leg_index Index of the leg this poser controls
     * @param leg Reference to the Leg object this poser controls
     */
    LegPoser(BodyPoseController* body_pose_controller, LocomotionSystem* locomotion_system, int leg_index, Leg& leg);

    /**
     * @brief Copy constructor
     * @param leg_poser Pointer to the Leg Poser object to be copied from
     */
    LegPoser(const LegPoser* leg_poser);

    // Accessors
    inline int getLegIndex() const { return leg_index_; }
    inline Pose getCurrentTipPose() const {
        Point3D tip_pos = leg_.getCurrentTipPositionGlobal();
        return Pose(tip_pos, Eigen::Vector3d(0, 0, 0));
    }
    inline Pose getTargetTipPose() const { return target_tip_pose_; }
    inline ExternalTarget getExternalTarget() const { return external_target_; }
    inline Pose getAutoPose() const { return auto_pose_; }
    inline int getPoseNegationPhaseStart() const { return pose_negation_phase_start_; }
    inline int getPoseNegationPhaseEnd() const { return pose_negation_phase_end_; }
    inline bool getLegCompletedStep() const { return leg_completed_step_; }

    // Leg access
    inline Leg& getLeg() { return leg_; }
    inline const Leg& getLeg() const { return leg_; }

    // Modifiers
    inline void setCurrentTipPose(const Pose& current) {
        leg_.setCurrentTipPositionGlobal(current.position);
        current_tip_pose_ = current;
    }
    inline void setTargetTipPose(const Pose& target) { target_tip_pose_ = target; }
    inline void setExternalTarget(const ExternalTarget& target) { external_target_ = target; }
    inline void setAutoPose(const Pose& auto_pose) { auto_pose_ = auto_pose; }
    inline void setPoseNegationPhaseStart(int start) { pose_negation_phase_start_ = start; }
    inline void setPoseNegationPhaseEnd(int end) { pose_negation_phase_end_ = end; }
    inline void setNegationTransitionRatio(double ratio) { negation_transition_ratio_ = ratio; }
    inline void setLegCompletedStep(bool complete) { leg_completed_step_ = complete; }
    inline void setDesiredConfiguration(const JointConfiguration& config) { desired_configuration_ = config; }

    // Transition pose management
    inline Pose getTransitionPose(int index) const {
        return (index < static_cast<int>(transition_poses_.size())) ? transition_poses_[index] : Pose();
    }
    inline bool hasTransitionPose(int index) const { return static_cast<int>(transition_poses_.size()) > index; }
    inline void addTransitionPose(const Pose& transition) { transition_poses_.push_back(transition); }
    inline void resetTransitionSequence() { transition_poses_.clear(); }

    /**
     * @brief Reset the key variables of stepToPosition() ready for new stepping maneuver
     * @return 100 if resetting is successful
     */
    inline int resetStepToPosition() {
        first_iteration_ = true;
        return PROGRESS_COMPLETE;
    }

    /**
     * @brief Uses a bezier curve to smoothly update the desired joint position of each joint in the leg
     * @param transition_time The time period in which to complete this transition
     * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
     */
    int transitionConfiguration(double transition_time);

    /**
     * @brief Uses bezier curves to smoothly update the desired tip position of the leg
     * @param target_tip_pose The target tip pose in reference to the body centre frame
     * @param target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
     * @param lift_height The height which the stepping leg trajectory should reach at its peak
     * @param time_to_step The time period to complete this maneuver
     * @param apply_delta A bool defining if a position offset value should be applied to the target tip position
     * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
     */
    int stepToPosition(const Pose& target_tip_pose, const Pose& target_pose,
                       double lift_height, double time_to_step, bool apply_delta = true);

    /**
     * @brief Sets the leg specific auto pose from the default auto pose
     * @param phase The phase is the input value which is used to determine the progression along the bezier curves
     */
    void updateAutoPose(int phase);

    /**
     * @brief Set target position for leg movement
     * @param target_position Target position in world coordinates
     */
    void setTargetPosition(const Point3D& target_position) {
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
     * @return Progress percentage (0-100)
     */
    int stepToPosition(const Point3D& target_position, double step_height, double step_time) {
        Pose target_pose(target_position, Eigen::Vector3d(0, 0, 0));
        return stepToPosition(target_pose, Pose::Identity(), step_height, step_time, false);
    }

private:
    BodyPoseController* body_pose_controller_; //< Pointer to body pose controller object
    LocomotionSystem* locomotion_system_;    //< Pointer to locomotion system
    int leg_index_;                          //< Index of the leg this poser controls
    Leg& leg_;                               //< Reference to the Leg object this poser controls

    Pose auto_pose_;                         //< Leg specific auto pose (from default auto pose - negated if required)
    bool negate_auto_pose_ = false;          //< Flag denoting if negation of auto pose is to occur
    int pose_negation_phase_start_ = 0;      //< Phase start of auto pose negation cycle
    int pose_negation_phase_end_ = 0;        //< Phase end of auto pose negation cycle
    double negation_transition_ratio_ = 0.0; //< The ratio of the negation period used to transition to total negation

    bool first_iteration_ = true;            //< Flag denoting if an iterating function is on it's first iteration
    int master_iteration_count_ = 0;         //< Master iteration count used in generating time input for bezier curves

    JointConfiguration desired_configuration_; //< Configuration target for transitionConfiguration function
    JointConfiguration origin_configuration_;  //< Configuration origin for transitionConfiguration function

    Pose origin_tip_pose_;                   //< Origin tip pose used in bezier curve equations
    Pose current_tip_pose_;                  //< Current tip pose
    Pose target_tip_pose_;                   //< Target tip pose used in bezier curve equations
    ExternalTarget external_target_;         //< Externally set target tip pose object

    std::vector<Pose> transition_poses_;     //< Vector of transition target tip poses

    bool leg_completed_step_ = false;        //< Flag denoting if leg has completed its required step in a sequence

    // Constants
    static constexpr double JOINT_TOLERANCE = 0.01;  // rad
    static constexpr double TIP_TOLERANCE = 0.01;    // m
    static constexpr double UNDEFINED_VALUE = 1e6;
};

#endif // LEG_POSER_H