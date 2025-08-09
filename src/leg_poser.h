#ifndef LEG_POSER_H
#define LEG_POSER_H

#include "leg.h"
#include "robot_model.h"
#include <memory>
#include <vector>

// Use existing Pose from robot_model.h

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
    inline Pose getCurrentTipPose() const {
        Point3D tip_pos = leg_.getCurrentTipPositionGlobal();
        return Pose(tip_pos, Eigen::Vector3d(0, 0, 0));
    }
    inline Pose getTargetTipPose() const { return target_tip_pose_; }
    inline ExternalTarget getExternalTarget() const { return external_target_; }
    inline Pose getAutoPose() const { return auto_pose_; }
    inline bool getLegCompletedStep() const { return leg_completed_step_; }

    // Modifiers
    inline void setCurrentTipPose(const RobotModel &model, const Pose &current) {
        leg_.setCurrentTipPositionGlobal(current.position);
        current_tip_pose_ = current;
    }
    inline void setTargetTipPose(const Pose &target) { target_tip_pose_ = target; }
    inline void setExternalTarget(const ExternalTarget &target) { external_target_ = target; }
    inline void setAutoPose(const Pose &auto_pose) { auto_pose_ = auto_pose; }
    inline void setLegCompletedStep(bool complete) { leg_completed_step_ = complete; }

    /**
     * @brief Reset the key variables of stepToPosition() ready for new stepping maneuver
     */
    inline void resetStepToPosition() {
        first_iteration_ = true;
        master_iteration_count_ = 0;
    }

    /**
     * @brief Uses bezier curves to smoothly update the desired tip position of the leg
     * @param target_tip_pose The target tip pose in reference to the body centre frame
     * @param target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
     * @param lift_height The height which the stepping leg trajectory should reach at its peak
     * @param time_to_step The time period to complete this maneuver
     * @param apply_delta A bool defining if a position offset value should be applied to the target tip position
     * @return true if step is complete, false if still in progress
     */
    bool stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
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
        return stepToPosition(target_pose, Pose::Identity(), step_height, step_time, false);
    }

  private:
    int leg_index_;           //< Index of the leg this poser controls
    Leg &leg_;                //< Reference to the Leg object this poser controls
    RobotModel &robot_model_; //< Reference to the robot model for parameter access

    Pose auto_pose_;                 //< Leg specific auto pose
    bool first_iteration_ = true;    //< Flag denoting if an iterating function is on it's first iteration
    int master_iteration_count_ = 0; //< Master iteration count used in generating time input for bezier curves

    Pose origin_tip_pose_;           //< Origin tip pose used in bezier curve equations
    Pose current_tip_pose_;          //< Current tip pose
    Pose target_tip_pose_;           //< Target tip pose used in bezier curve equations
    ExternalTarget external_target_; //< Externally set target tip pose object

    bool leg_completed_step_ = false; //< Flag denoting if leg has completed its required step in a sequence

    double physical_reference_height_; //< Physical reference height (z = getDefaultHeightOffset() when all angles are 0°)

    // Constants
    static constexpr double JOINT_TOLERANCE = 0.01; // rad
    static constexpr double TIP_TOLERANCE = 0.01;   // m
};

#endif // LEG_POSER_H