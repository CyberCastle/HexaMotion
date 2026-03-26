#ifndef TEST_POSE_HELPERS_H
#define TEST_POSE_HELPERS_H

/**
 * @file test_pose_helpers.h
 * @brief Standalone helper functions for tests that previously called
 *        non-OpenSHC methods on BodyPoseController.
 *
 * These functions replicate the removed BPC utility methods so tests
 * can continue to set up standing poses and body positions without
 * depending on non-OpenSHC BPC API surface.
 */

#include "body_pose_controller.h"
#include "math_utils.h"
#include "robot_model.h"
#include <ArduinoEigen.h>

/**
 * @brief Set standing pose for all legs using configured joint angles.
 * Equivalent to the removed BodyPoseController::setStandingPose().
 */
inline bool testSetStandingPose(BodyPoseController &pc, const RobotModel &model, Leg legs[NUM_LEGS]) {
    if (!pc.getLegPoser(0)) {
        pc.initializeLegPosers(legs);
    }
    const auto &config = pc.getBodyPoseConfig();
    for (int i = 0; i < NUM_LEGS; ++i) {
        const auto &standing_joints = config.standing_pose_joints[i];
        JointAngles angles;
        angles.coxa = standing_joints.coxa;
        angles.femur = standing_joints.femur;
        angles.tibia = standing_joints.tibia;
        legs[i].setJointAngles(angles);
        Point3D pos = model.forwardKinematicsGlobalCoordinates(i, angles);
        legs[i].setCurrentTipPositionGlobal(pos);
    }
    return true;
}

/**
 * @brief Calculate body position from average leg tip height.
 * Equivalent to the removed BodyPoseController::calculateBodyPosition().
 */
inline Eigen::Vector3d testCalculateBodyPosition(Leg legs[NUM_LEGS]) {
    double total_z = 0.0;
    for (int i = 0; i < NUM_LEGS; i++) {
        total_z += legs[i].getCurrentTipPositionGlobal().z;
    }
    return Eigen::Vector3d(0.0, 0.0, total_z / NUM_LEGS);
}

/**
 * @brief Apply body pose via IK to all legs.
 * Equivalent to the removed BodyPoseController::setBodyPose().
 * @param position Translation in mm
 * @param orientation Euler angles in radians (roll, pitch, yaw)
 */
inline bool testSetBodyPose(RobotModel &model, const Eigen::Vector3d &position,
                            const Eigen::Vector3d &orientation, Leg legs[NUM_LEGS]) {
    Eigen::Quaterniond body_rotation = math_utils::eulerAnglesToQuaterniond(orientation);
    Pose body_pose(Point3D(position.x(), position.y(), position.z()), body_rotation);
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D default_tip = legs[i].getCurrentTipPositionGlobal();
        Point3D posed_tip = body_pose.inverseTransformVector(default_tip);
        if (!legs[i].applyIK(posed_tip)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if body pose is within configured limits.
 * Equivalent to the removed BodyPoseController::checkBodyPoseLimits().
 */
inline bool testCheckBodyPoseLimits(const BodyPoseConfiguration &config,
                                    const Eigen::Vector3d &position,
                                    const Eigen::Vector3d &orientation) {
    if (config.max_translation.x > 0.0 && std::abs(position.x()) > config.max_translation.x)
        return false;
    if (config.max_translation.y > 0.0 && std::abs(position.y()) > config.max_translation.y)
        return false;
    if (config.max_translation.z > 0.0 && std::abs(position.z()) > config.max_translation.z)
        return false;
    if (config.max_rotation.roll > 0.0 && std::abs(orientation.x()) > config.max_rotation.roll)
        return false;
    if (config.max_rotation.pitch > 0.0 && std::abs(orientation.y()) > config.max_rotation.pitch)
        return false;
    if (config.max_rotation.yaw > 0.0 && std::abs(orientation.z()) > config.max_rotation.yaw)
        return false;
    return true;
}

/**
 * @brief Initialize default pose for all legs.
 * Equivalent to the removed BodyPoseController::initializeDefaultPose().
 */
inline void testInitializeDefaultPose(RobotModel &model, Leg legs[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; i++) {
        JointAngles zero_angles;
        zero_angles.coxa = 0.0;
        zero_angles.femur = 0.0;
        zero_angles.tibia = 0.0;
        legs[i].setJointAngles(zero_angles);
        Point3D pos = model.forwardKinematicsGlobalCoordinates(i, zero_angles);
        legs[i].setCurrentTipPositionGlobal(pos);
    }
}

#endif // TEST_POSE_HELPERS_H
