#include "pose_controller.h"

/**
 * @file pose_controller.cpp
 * @brief Implementation of the kinematic pose controller.
 */

PoseController::PoseController(RobotModel &m, IServoInterface *s)
    : model(m), servos(s) {}

bool PoseController::setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation,
                                 Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    for (int i = 0; i < NUM_LEGS; ++i) {
        Point3D old_leg_body(leg_pos[i].x - position[0], leg_pos[i].y - position[1], leg_pos[i].z - position[2]);
        Point3D new_leg_body = math_utils::rotatePoint(old_leg_body, orientation);
        Point3D new_leg_world(position[0] + new_leg_body.x,
                              position[1] + new_leg_body.y,
                              position[2] + new_leg_body.z);
        JointAngles angles = model.inverseKinematics(i, new_leg_body);
        if (!model.checkJointLimits(i, angles))
            return false;
        joint_q[i] = angles;
        leg_pos[i] = new_leg_world;
        if (servos)
            for (int j = 0; j < DOF_PER_LEG; ++j)
                servos->setJointAngle(i, j, j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));
    }
    return true;
}

bool PoseController::setLegPosition(int leg_index, const Point3D &position,
                                    Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS]) {
    JointAngles angles = model.inverseKinematics(leg_index, position);

    angles.coxa = model.constrainAngle(angles.coxa, model.getParams().coxa_angle_limits[0],
                                       model.getParams().coxa_angle_limits[1]);
    angles.femur = model.constrainAngle(angles.femur, model.getParams().femur_angle_limits[0],
                                        model.getParams().femur_angle_limits[1]);
    angles.tibia = model.constrainAngle(angles.tibia, model.getParams().tibia_angle_limits[0],
                                        model.getParams().tibia_angle_limits[1]);

    leg_pos[leg_index] = model.forwardKinematics(leg_index, angles);
    joint_q[leg_index] = angles;

    if (servos)
        for (int j = 0; j < DOF_PER_LEG; ++j)
            servos->setJointAngle(leg_index, j,
                                  j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));

    return true;
}

void PoseController::initializeDefaultPose(Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS],
                                           float hex_radius, float robot_height) {
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle = i * 60.0f;
        leg_pos[i].x = hex_radius * cos(math_utils::degreesToRadians(angle)) + 50.0f;
        leg_pos[i].y = hex_radius * sin(math_utils::degreesToRadians(angle));
        leg_pos[i].z = -robot_height;
        joint_q[i] = JointAngles(0, 45, -90);
    }
}

bool PoseController::setStandingPose(Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS], float robot_height) {
    Eigen::Vector3f position(0.0f, 0.0f, robot_height);
    Eigen::Vector3f orientation(0.0f, 0.0f, 0.0f);
    return setBodyPose(position, orientation, leg_pos, joint_q);
}

bool PoseController::setCrouchPose(Point3D leg_pos[NUM_LEGS], JointAngles joint_q[NUM_LEGS], float robot_height) {
    Eigen::Vector3f position(0.0f, 0.0f, robot_height * 0.6f);
    Eigen::Vector3f orientation(0.0f, 0.0f, 0.0f);
    return setBodyPose(position, orientation, leg_pos, joint_q);
}

bool PoseController::setBodyPoseQuaternion(const Eigen::Vector3f &position, const Eigen::Vector4f &quaternion,
                                           Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Convert quaternion to Euler angles and use existing function
    Point3D euler_degrees = math_utils::quaternionToEulerPoint3D(quaternion);
    Eigen::Vector3f orientation = math_utils::point3DToVector3f(euler_degrees);
    return setBodyPose(position, orientation, leg_positions, joint_angles);
}

bool PoseController::interpolatePose(const Eigen::Vector3f &start_pos, const Eigen::Vector4f &start_quat,
                                     const Eigen::Vector3f &end_pos, const Eigen::Vector4f &end_quat,
                                     float t, Point3D leg_positions[NUM_LEGS], JointAngles joint_angles[NUM_LEGS]) {
    // Clamp interpolation parameter
    t = std::max(0.0f, std::min(1.0f, t));

    // Linear interpolation for position
    Eigen::Vector3f interp_pos = start_pos + t * (end_pos - start_pos);

    // Spherical linear interpolation (SLERP) for quaternion
    Eigen::Vector4f interp_quat = quaternionSlerp(start_quat, end_quat, t);

    return setBodyPoseQuaternion(interp_pos, interp_quat, leg_positions, joint_angles);
}

// Quaternion spherical linear interpolation (SLERP)
Eigen::Vector4f PoseController::quaternionSlerp(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2, float t) {
    // Compute dot product
    float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // If dot product is negative, take the shorter path by negating one quaternion
    Eigen::Vector4f q2_adj = q2;
    if (dot < 0.0f) {
        q2_adj = -q2;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation to avoid numerical issues
    if (dot > 0.9995f) {
        Eigen::Vector4f result = q1 + t * (q2_adj - q1);
        float norm = sqrt(result[0] * result[0] + result[1] * result[1] +
                          result[2] * result[2] + result[3] * result[3]);
        if (norm > 0.0f) {
            result = result / norm;
        }
        return result;
    }

    // Calculate angle and perform SLERP
    float theta_0 = acos(std::abs(dot));
    float sin_theta_0 = sin(theta_0);

    float theta = theta_0 * t;
    float sin_theta = sin(theta);

    float s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    return s0 * q1 + s1 * q2_adj;
}
