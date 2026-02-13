#ifndef STATE_CONTROLLER_CONTEXT_H
#define STATE_CONTROLLER_CONTEXT_H

#include "body_pose_controller.h"
#include "gait_config.h"
#include "hexamotion_constants.h"
#include "robot_model.h"
#include "walk_controller.h"

/**
 * @brief Context interface used by StateController to access locomotion services.
 *
 * This abstraction keeps StateController decoupled from LocomotionSystem
 * concrete implementation while still allowing orchestrated interactions.
 */
class StateControllerContext {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~StateControllerContext() {}

    virtual bool isSystemEnabled() const = 0;
    virtual RobotModel &getRobotModel() = 0;
    virtual const Parameters &getParams() const = 0;
    virtual bool setGaitConfiguration(const GaitConfiguration &gait_config) = 0;
    virtual WalkController *getWalkController() = 0;
    virtual BodyPoseController *getBodyPoseController() = 0;
    virtual Leg *getLegsArray() = 0;
    virtual bool planGaitSequence(double velocity_x, double velocity_y, double angular_velocity) = 0;
    virtual SystemState getSystemState() const = 0;
    virtual bool setManualBodyPoseInput(const Eigen::Vector3d &position,
                                        const Eigen::Vector3d &orientation) = 0;
    /**
     * @brief Apply manual leg control inputs for up to two selected legs.
     *
     * @param primary_leg_index Primary selected leg index (-1 if none)
     * @param primary_tip_velocity Primary tip velocity input
     * @param secondary_leg_index Secondary selected leg index (-1 if none)
     * @param secondary_tip_velocity Secondary tip velocity input
     * @param primary_pose_valid True when primary tip pose input is valid
     * @param primary_tip_pose Primary tip pose input
     * @param secondary_pose_valid True when secondary tip pose input is valid
     * @param secondary_tip_pose Secondary tip pose input
     * @return True if inputs were accepted
     */
    virtual bool applyManualLegInputs(int primary_leg_index,
                                      const Eigen::Vector3d &primary_tip_velocity,
                                      int secondary_leg_index,
                                      const Eigen::Vector3d &secondary_tip_velocity,
                                      bool primary_pose_valid,
                                      const Point3D &primary_tip_pose,
                                      bool secondary_pose_valid,
                                      const Point3D &secondary_tip_pose) = 0;
    virtual bool executeStartupSequence() = 0;
    virtual int getStartupProgressPercent() const = 0;
    virtual bool executeShutdownSequence() = 0;
    virtual bool setStandingPose() = 0;
    virtual JointAngles getJointAngles(int leg_index) const = 0;
    virtual bool establishInitialStandingPose() = 0;
    virtual bool isInitialStandingPoseActive() const = 0;
    virtual Eigen::Vector3d getBodyPosition() const = 0;
    virtual Eigen::Vector3d getBodyOrientation() const = 0;
    /**
     * @brief Execute one locomotion control pipeline iteration.
     *
     * This runs sensing, gait/walk updates, IK and servo publication
     * according to the current system state.
     *
     * @return True if the pipeline step completed successfully.
     */
    virtual bool runControlPipelineStep() = 0;

    /**
     * @brief Update admittance stiffness for a single leg during state transition.
     *
     * Called by StateController during WALKING_TO_MANUAL (scale 0->1) and
     * MANUAL_TO_WALKING (scale 1->0) transitions. Delegates to
     * AdmittanceController::updateStiffness(legs, leg_index, scale_reference).
     *
     * OpenSHC equivalent: admittance_->updateStiffness(leg, scale_reference)
     * in StateController::legStateToggle().
     *
     * @param leg_index       Index of the transitioning leg
     * @param scale_reference Transition progress (0.0 to 1.0)
     */
    virtual void updateAdmittanceStiffness(int leg_index, double scale_reference) = 0;
};

#endif /**< STATE_CONTROLLER_CONTEXT_H */
