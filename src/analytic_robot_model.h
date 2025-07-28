#ifndef ANALYTIC_ROBOT_MODEL_H
#define ANALYTIC_ROBOT_MODEL_H

#include "hexamotion_constants.h"
#include "math_utils.h"
#include "robot_model.h"
#include <ArduinoEigen.h>
#include <vector>

extern const double BASE_THETA_OFFSETS[NUM_LEGS];

/**
 * @class AnalyticRobotModel
 * @brief Implements analytic kinematics for the hexapod robot described in Agents.md.
 *
 * This class provides analytic forward kinematics, Jacobian, and base position calculations
 * for the specific mechanical configuration described in Agents.md. It is not intended for
 * general hexapod robots, but only for the described mechanical structure.
 */
class AnalyticRobotModel {
  public:
    /**
     * @brief Construct an AnalyticRobotModel with the given parameters.
     * @param params Robot kinematic parameters
     */
    explicit AnalyticRobotModel(const Parameters &params);

    /**
     * @brief Get the analytic position of the leg base (without joint transformations).
     * @param leg_index Index of the leg (0-5)
     * @return Base position in robot coordinates
     */
    Point3D getAnalyticLegBasePosition(int leg_index) const;

    /**
     * @brief Compute analytic forward kinematics for a leg (global coordinates).
     * @param leg_index Index of the leg (0-5)
     * @param angles Joint angles
     * @return Tip position in global robot coordinates
     */
    Point3D forwardKinematicsGlobalCoordinatesAnalytic(int leg_index, const JointAngles &angles) const;

    /**
     * @brief Compute the analytic homogeneous transform for a leg.
     * @param leg_index Index of the leg (0-5)
     * @param q Joint angles
     * @return Homogeneous transformation matrix
     */
    Eigen::Matrix4d legTransformAnalytic(int leg_index, const JointAngles &q) const;

    /**
     * @brief Compute the analytic Jacobian for a leg.
     * @param leg Index of the leg (0-5)
     * @param q Joint angles
     * @param tip Unused (for interface compatibility)
     * @return 3x3 Jacobian matrix
     */
    Eigen::Matrix3d calculateJacobianAnalytic(int leg, const JointAngles &q, const Point3D &tip = Point3D()) const;

    /**
     * @brief Build analytic transforms for each joint in the leg.
     * @param leg Index of the leg (0-5)
     * @param q Joint angles
     * @return Vector of homogeneous transformation matrices
     */
    std::vector<Eigen::Matrix4d> buildDHTransformsAnalytic(int leg, const JointAngles &q) const;

  private:
    Parameters params;
};

#endif // ANALYTIC_ROBOT_MODEL_H