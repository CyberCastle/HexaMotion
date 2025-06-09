#include "model.h"
#include <limits>
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
            dh[l][0][0] = 0.0f;   // a0
            dh[l][0][1] = -90.0f; // alpha0 rotate to femur axis
            dh[l][0][2] = 0.0f;   // d1
            dh[l][0][3] = 0.0f;   // theta1 offset

            // Joint 2 (femur): pitch axis
            dh[l][1][0] = params.coxa_length; // translation along rotated x
            dh[l][1][1] = 0.0f;               // alpha1
            dh[l][1][2] = 0.0f;               // d2
            dh[l][1][3] = 0.0f;               // theta2 offset

            // Joint 3 (tibia): pitch axis
            dh[l][2][0] = params.femur_length; // translation along femur
            dh[l][2][1] = 0.0f;                // alpha2
            dh[l][2][2] = 0.0f;                // d3
            dh[l][2][3] = 0.0f;                // theta3 offset
        }
    }
}

JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) {
    // Transform target to leg coordinate frame
    float base_angle = leg * 60.0f;
    float cos_b = cos(math_utils::degreesToRadians(base_angle));
    float sin_b = sin(math_utils::degreesToRadians(base_angle));

    float x = p_target.x - params.hexagon_radius * cos_b;
    float y = p_target.y - params.hexagon_radius * sin_b;
    float z = p_target.z;

    float xr = cos_b * x + sin_b * y;
    float yr = -sin_b * x + cos_b * y;

    JointAngles q{};
    q.coxa = math_utils::radiansToDegrees(atan2(yr, xr));

    float dx = sqrt(xr * xr + yr * yr) - params.coxa_length;
    float dz = z;

    float L1 = params.femur_length;
    float L2 = params.tibia_length;

    float d = sqrt(dx * dx + dz * dz);
    float min_d = fabs(L1 - L2) + 1e-3f;
    float max_d = L1 + L2 - 1e-3f;
    if (d < min_d) {
        float scale = min_d / d;
        dx *= scale;
        dz *= scale;
        d = min_d;
    } else if (d > max_d) {
        float scale = max_d / d;
        dx *= scale;
        dz *= scale;
        d = max_d;
    }

    float cos_tibia = (dx * dx + dz * dz - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cos_tibia = std::max(-1.0f, std::min(1.0f, cos_tibia));
    float tibia = atan2(-sqrt(1.0f - cos_tibia * cos_tibia), cos_tibia);

    float k1 = L1 + L2 * cos(tibia);
    float k2 = L2 * sin(tibia);
    float femur = atan2(dz, dx) - atan2(k2, k1);

    q.femur = math_utils::radiansToDegrees(femur);
    q.tibia = math_utils::radiansToDegrees(tibia);

    if (params.ik.clamp_joints) {
        q.coxa = constrainAngle(q.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        q.femur = constrainAngle(q.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        q.tibia = constrainAngle(q.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }
    return q;
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) const {
    Eigen::Matrix4f transform = legTransform(leg_index, angles);
    Point3D position;
    position.x = transform(0, 3);
    position.y = transform(1, 3);
    position.z = transform(2, 3);
    return position;
}

Eigen::Matrix4f RobotModel::legTransform(int leg_index, const JointAngles &q) const {
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

Eigen::Matrix3f RobotModel::analyticJacobian(int leg, const JointAngles &q) const {
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

std::pair<float, float> RobotModel::calculateHeightRange() const {
    float min_h = std::numeric_limits<float>::max();
    float max_h = -std::numeric_limits<float>::max();

    // Workspace analysis: discretize the joint configuration space
    // Based on "Introduction to Robotics" - Craig and "Robotics: Modelling, Planning and Control" - Siciliano
    const int resolution = 10; // discretization resolution

    const float coxa_step = (params.coxa_angle_limits[1] - params.coxa_angle_limits[0]) / resolution;
    const float femur_step = (params.femur_angle_limits[1] - params.femur_angle_limits[0]) / resolution;
    const float tibia_step = (params.tibia_angle_limits[1] - params.tibia_angle_limits[0]) / resolution;

    // Evaluate the entire workspace of valid joint configurations
    for (int i = 0; i <= resolution; i++) {
        for (int j = 0; j <= resolution; j++) {
            for (int k = 0; k <= resolution; k++) {
                float coxa = params.coxa_angle_limits[0] + i * coxa_step;
                float femur = params.femur_angle_limits[0] + j * femur_step;
                float tibia = params.tibia_angle_limits[0] + k * tibia_step;
                JointAngles q(coxa, femur, tibia);

                // Check that angles are within limits
                if (!checkJointLimits(0, q))
                    continue;

                Point3D pos = forwardKinematics(0, q);

                // Calculate body height considering the robot's physical offset
                // pos.z is negative when the leg is below the body
                float height = -pos.z + params.height_offset;

                // Only consider physically valid heights (positive)
                if (height > 0) {
                    min_h = std::min(min_h, height);
                    max_h = std::max(max_h, height);
                }
            }
        }
    }

    // If no valid configurations were found, indicates parameter error
    if (min_h == std::numeric_limits<float>::max()) {
        // Return values indicating error - inconsistent robot parameters
        return {-1.0f, -1.0f};
    }

    return {min_h, max_h};
}
