#include "model.h"
#include <math.h>

RobotModel::RobotModel(const Parameters &p) : params(p) {
    initializeDH();
}

void RobotModel::initializeDH() {
    bool custom = false;
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            for (int k = 0; k < 4; ++k) {
                dh[l][j][k] = params.dh_parameters[l][j][k];
                if (params.dh_parameters[l][j][k] != 0.0f)
                    custom = true;
            }
        }
    }

    if (!custom) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            // Joint 1 (coxa): vertical axis
            dh[l][0][0] = 0.0f;      // a0
            dh[l][0][1] = -90.0f;    // alpha0 rotate to femur axis
            dh[l][0][2] = 0.0f;      // d1
            dh[l][0][3] = 0.0f;      // theta1 offset

            // Joint 2 (femur): pitch axis
            dh[l][1][0] = params.coxa_length; // translation along rotated x
            dh[l][1][1] = 0.0f;      // alpha1
            dh[l][1][2] = 0.0f;      // d2
            dh[l][1][3] = 0.0f;      // theta2 offset

            // Joint 3 (tibia): pitch axis
            dh[l][2][0] = params.femur_length; // translation along femur
            dh[l][2][1] = 0.0f;      // alpha2
            dh[l][2][2] = 0.0f;      // d3
            dh[l][2][3] = 0.0f;      // theta3 offset
        }
    }
}

JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) {
    JointAngles q{};
    q = JointAngles(0, 0, 0);
    const uint8_t N = params.ik.max_iterations;
    const float eps = params.ik.pos_threshold_mm;

    for (uint8_t it = 0; it < N; ++it) {
        Point3D p_now = forwardKinematics(leg, q);
        Eigen::Vector3f e(p_target.x - p_now.x,
                          p_target.y - p_now.y,
                          p_target.z - p_now.z);

        if (e.norm() < eps)
            return q;

        Eigen::Matrix3f J = analyticJacobian(leg, q);
        const float lambda = params.ik.use_damping ? params.ik.damping_lambda : 0.0f;
        Eigen::Matrix3f Z = (J * J.transpose() + lambda * lambda * Eigen::Matrix3f::Identity()).inverse();
        Eigen::Vector3f dq = J.transpose() * Z * e;

        q.coxa += math_utils::radiansToDegrees(dq[0]);
        q.femur += math_utils::radiansToDegrees(dq[1]);
        q.tibia += math_utils::radiansToDegrees(dq[2]);

        if (params.ik.clamp_joints) {
            q.coxa = constrainAngle(q.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
            q.femur = constrainAngle(q.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
            q.tibia = constrainAngle(q.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
        }
    }
    return q;
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) {
    Eigen::Matrix4f transform = legTransform(leg_index, angles);
    Point3D position;
    position.x = transform(0, 3);
    position.y = transform(1, 3);
    position.z = transform(2, 3);
    return position;
}

Eigen::Matrix4f RobotModel::legTransform(int leg_index, const JointAngles &q) {
    const float base_angle_deg = leg_index * 60.0f;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    T(1, 3) = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));
    T.block<3, 3>(0, 0) = math_utils::rotationMatrixZ(base_angle_deg);

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 0; j < DOF_PER_LEG; ++j) {
        float a = dh[leg_index][j][0];
        float alpha = dh[leg_index][j][1];
        float d = dh[leg_index][j][2];
        float theta0 = dh[leg_index][j][3];
        float theta = theta0 + joint_deg[j];
        T *= math_utils::dhTransform(a, alpha, d, theta);
    }
    T *= math_utils::dhTransform(params.tibia_length, 0.0f, 0.0f, 0.0f);
    return T;
}

Eigen::Matrix3f RobotModel::analyticJacobian(int leg, const JointAngles &q) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Vector3f o[DOF_PER_LEG + 1];
    Eigen::Vector3f z[DOF_PER_LEG];

    const float base_angle = leg * 60.0f;
    T(0, 3) = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
    T(1, 3) = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));
    T.block<3, 3>(0, 0) = math_utils::rotationMatrixZ(base_angle);

    o[0] = T.block<3, 1>(0, 3);

    const float q_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};
    for (int j = 0; j < DOF_PER_LEG; ++j) {
        float a = dh[leg][j][0];
        float alpha = dh[leg][j][1];
        float d = dh[leg][j][2];
        float theta0 = dh[leg][j][3];
        float theta = theta0 + q_deg[j];
        T *= math_utils::dhTransform(a, alpha, d, theta);
        z[j] = T.block<3, 1>(0, 2);
        o[j + 1] = T.block<3, 1>(0, 3);
    }

    T *= math_utils::dhTransform(params.tibia_length, 0.0f, 0.0f, 0.0f);
    Eigen::Vector3f p_end = T.block<3, 1>(0, 3);
    Eigen::Matrix3f J;
    for (int j = 0; j < DOF_PER_LEG; ++j)
        J.col(j) = z[j].cross(p_end - o[j]);
    return J;
}

bool RobotModel::checkJointLimits(int leg_index, const JointAngles &angles) const {
    return (angles.coxa >= params.coxa_angle_limits[0] && angles.coxa <= params.coxa_angle_limits[1] &&
            angles.femur >= params.femur_angle_limits[0] && angles.femur <= params.femur_angle_limits[1] &&
            angles.tibia >= params.tibia_angle_limits[0] && angles.tibia <= params.tibia_angle_limits[1]);
}

float RobotModel::constrainAngle(float angle, float min_angle, float max_angle) const {
    return std::max(min_angle, std::min(max_angle, angle));
}

bool RobotModel::validate() const {
    return (params.hexagon_radius > 0 && params.coxa_length > 0 && params.femur_length > 0 && params.tibia_length > 0 &&
            params.robot_height > 0 && params.control_frequency > 0);
}

