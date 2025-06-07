#include "pose_controller.h"

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
    if (!model.checkJointLimits(leg_index, angles))
        return false;
    leg_pos[leg_index] = position;
    joint_q[leg_index] = angles;
    if (servos)
        for (int j = 0; j < DOF_PER_LEG; ++j)
            servos->setJointAngle(leg_index, j, j == 0 ? angles.coxa : (j == 1 ? angles.femur : angles.tibia));
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
