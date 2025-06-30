#include "HexaModel.h"
#include "hexamotion_constants.h"

/**
 * @file robot_model.cpp
 * @brief Implementation of the kinematic robot model.
 */
#include <limits>
#include <math.h>
#include <vector>

// Per-leg base orientation offsets (degrees)
static const float BASE_THETA_OFFSETS[NUM_LEGS] = {0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};

RobotModel::RobotModel(const Parameters &p) : params(p) {
    initializeDH();
}

void RobotModel::initializeDH() {

    // Initialize default DH parameters if custom parameters are not used
    if (!params.use_custom_dh_parameters) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            // Base transform from body center to leg mount
            dh_transforms[l][0][0] = params.hexagon_radius; // a
            dh_transforms[l][0][1] = 0.0f;                  // alpha
            dh_transforms[l][0][2] = 0.0f;                  // d
            dh_transforms[l][0][3] = BASE_THETA_OFFSETS[l]; // theta

            // Coxa link (horizontal rotation)
            dh_transforms[l][1][0] = params.coxa_length; // a
            dh_transforms[l][1][1] = 0.0f;              // alpha (no twist)
            dh_transforms[l][1][2] = 0.0f;               // d
            dh_transforms[l][1][3] = 0.0f;               // theta offset

            // Femur link (vertical rotation)
            dh_transforms[l][2][0] = params.femur_length; // a
            dh_transforms[l][2][1] = 90.0f;               // alpha (90° twist to vertical)
            dh_transforms[l][2][2] = 0.0f;                // d
            dh_transforms[l][2][3] = 0.0f;                // theta offset

            // Tibia link (vertical rotation)
            dh_transforms[l][3][0] = params.tibia_length; // a
            dh_transforms[l][3][1] = 0.0f;                // alpha (no twist)
            dh_transforms[l][3][2] = 0.0f;                // d
            dh_transforms[l][3][3] = -90.0f;              // theta offset (vertical tibia)
        }
    }
}

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::solveIK(int leg, const Point3D &local_target, JointAngles current_angles) const {
    const float tolerance = 0.001f;
    const float dls_coefficient = 0.05f;

    for (int iter = 0; iter < params.ik.max_iterations; ++iter) {
        const float joint_deg[DOF_PER_LEG] = {current_angles.coxa, current_angles.femur, current_angles.tibia};
        Eigen::Matrix4f current_tf = Eigen::Matrix4f::Identity();
        for (int j = 1; j <= DOF_PER_LEG; ++j) {
            float a = dh_transforms[leg][j][0];
            float alpha = dh_transforms[leg][j][1];
            float d = dh_transforms[leg][j][2];
            float theta_off = dh_transforms[leg][j][3];
            float theta = theta_off + joint_deg[j - 1];
            float alpha_rad = math_utils::degreesToRadians(alpha);
            float theta_rad = math_utils::degreesToRadians(theta);
            current_tf *= math_utils::dhTransform(a, alpha_rad, d, theta_rad);
        }
        Point3D current_pos{current_tf(0, 3), current_tf(1, 3), current_tf(2, 3)};

        Eigen::Vector3f position_error3;
        position_error3 << (local_target.x - current_pos.x),
            (local_target.y - current_pos.y),
            (local_target.z - current_pos.z);

        if (position_error3.norm() < tolerance)
            break;

        std::vector<Eigen::Matrix4f> transforms(DOF_PER_LEG + 1);
        transforms[0] = Eigen::Matrix4f::Identity();
        for (int j = 1; j <= DOF_PER_LEG; ++j) {
            float a = dh_transforms[leg][j][0];
            float alpha = dh_transforms[leg][j][1];
            float d = dh_transforms[leg][j][2];
            float theta_off = dh_transforms[leg][j][3];
            float theta = theta_off + joint_deg[j - 1];
            float alpha_rad = math_utils::degreesToRadians(alpha);
            float theta_rad = math_utils::degreesToRadians(theta);
            transforms[j] = transforms[j - 1] * math_utils::dhTransform(a, alpha_rad, d, theta_rad);
        }

        Eigen::Matrix3f jacobian_pos;
        Eigen::Vector3f pe = transforms[DOF_PER_LEG].block<3, 1>(0, 3);
        Eigen::Vector3f z0(0, 0, 1);
        Eigen::Vector3f p0 = transforms[0].block<3, 1>(0, 3);
        jacobian_pos.col(0) = z0.cross(pe - p0);
        for (int j = 1; j < DOF_PER_LEG; ++j) {
            Eigen::Vector3f zj = transforms[j].block<3, 1>(0, 2);
            Eigen::Vector3f pj = transforms[j].block<3, 1>(0, 3);
            jacobian_pos.col(j) = zj.cross(pe - pj);
        }

        Eigen::Matrix3f JJT3 = jacobian_pos * jacobian_pos.transpose();
        Eigen::Matrix3f identity3 = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f damped_inv3 = (JJT3 + dls_coefficient * dls_coefficient * identity3).inverse();
        Eigen::Matrix3f jacobian_inverse3 = jacobian_pos.transpose() * damped_inv3;

        Eigen::Vector3f angle_delta = jacobian_inverse3 * position_error3;

        float step_scale = 1.0f;
        float max_angle_change = 5.0f;
        float max_delta = std::max({std::abs(angle_delta(0)), std::abs(angle_delta(1)), std::abs(angle_delta(2))});
        if (max_delta > math_utils::degreesToRadians(max_angle_change)) {
            step_scale = math_utils::degreesToRadians(max_angle_change) / max_delta;
        }

        current_angles.coxa += angle_delta(0) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.femur += angle_delta(1) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.tibia += angle_delta(2) * RADIANS_TO_DEGREES_FACTOR * step_scale;

        current_angles.coxa = normalizeAngle(current_angles.coxa);
        current_angles.femur = normalizeAngle(current_angles.femur);
        current_angles.tibia = normalizeAngle(current_angles.tibia);

        if (params.ik.clamp_joints) {
            current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
            current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
            current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
        }
    }

    return current_angles;
}

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) const {
    // Inverse kinematics: Damped Least Squares solver using DH parameters

    // Transform target to leg coordinate system
    const float base_angle_deg = BASE_THETA_OFFSETS[leg];
    float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    float dx = p_target.x - base_x;
    float dy = p_target.y - base_y;
    float angle_rad = math_utils::degreesToRadians(-base_angle_deg);
    local_target.x = cos(angle_rad) * dx - sin(angle_rad) * dy;
    local_target.y = sin(angle_rad) * dx + cos(angle_rad) * dy;
    local_target.z = p_target.z;

    // Basic workspace validation only (detailed validation done by WorkspaceValidator)
    float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float min_reach = std::abs(params.femur_length - params.tibia_length);
    float distance = sqrt(local_target.x * local_target.x +
                          local_target.y * local_target.y +
                          local_target.z * local_target.z);

    if (distance > max_reach * 0.98f || distance < min_reach * 1.02f) {
        // Target outside workspace - return safe default angles
        float coxa_angle = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
        coxa_angle = constrainAngle(coxa_angle, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        return JointAngles(coxa_angle, -45.0f, 60.0f);
    }

    // Initial guess based on target direction and realistic kinematics
    float coxa_start = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
    coxa_start = constrainAngle(coxa_start, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

    // Initial estimates for femur and tibia based only on DH model
    float femur_estimate = 0.0f;
    float tibia_estimate = 0.0f;

    // Clamp initial estimates to joint limits
    femur_estimate = constrainAngle(femur_estimate, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    tibia_estimate = constrainAngle(tibia_estimate, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    JointAngles current_angles(coxa_start, femur_estimate, tibia_estimate);

    // Clamp to joint limits

    if (params.ik.clamp_joints) {
        current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }

    return solveIK(leg, local_target, current_angles);
}

JointAngles RobotModel::inverseKinematicsCurrent(int leg, const JointAngles &current_angles,
                                                 const Point3D &p_target) const {
    const float base_angle_deg = BASE_THETA_OFFSETS[leg];
    float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    float dx = p_target.x - base_x;
    float dy = p_target.y - base_y;
    float angle_rad = math_utils::degreesToRadians(-base_angle_deg);
    local_target.x = cos(angle_rad) * dx - sin(angle_rad) * dy;
    local_target.y = sin(angle_rad) * dx + cos(angle_rad) * dy;
    local_target.z = p_target.z;

    JointAngles start = current_angles;
    if (params.ik.clamp_joints) {
        start.coxa = constrainAngle(start.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        start.femur = constrainAngle(start.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        start.tibia = constrainAngle(start.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }

    return solveIK(leg, local_target, start);
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) const {
    // Forward kinematics using double precision for stability
    Eigen::Matrix4d transform = legTransform(leg_index, angles);
    return Point3D{static_cast<float>(transform(0, 3)),
                   static_cast<float>(transform(1, 3)),
                   static_cast<float>(transform(2, 3))};
}

Point3D RobotModel::getLegBasePosition(int leg_index) const {
    // Get only the base transform (without joint angles)
    Eigen::Matrix4f base_transform = math_utils::dhTransform(
        dh_transforms[leg_index][0][0],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][1]),
        dh_transforms[leg_index][0][2],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][3]));

    return Point3D{base_transform(0, 3), base_transform(1, 3), base_transform(2, 3)};
}

Eigen::Matrix4d RobotModel::legTransform(int leg_index, const JointAngles &q) const {
    // Compute transform chain using double precision for accuracy
    Eigen::Matrix4d T = math_utils::dhTransform<double>(
        static_cast<double>(dh_transforms[leg_index][0][0]),
        math_utils::degreesToRadians(static_cast<double>(dh_transforms[leg_index][0][1])),
        static_cast<double>(dh_transforms[leg_index][0][2]),
        math_utils::degreesToRadians(static_cast<double>(dh_transforms[leg_index][0][3])));

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        double a = static_cast<double>(dh_transforms[leg_index][j][0]);
        double alpha = math_utils::degreesToRadians(static_cast<double>(dh_transforms[leg_index][j][1]));
        double d = static_cast<double>(dh_transforms[leg_index][j][2]);
        double theta0 = static_cast<double>(dh_transforms[leg_index][j][3]);
        double theta = theta0 + static_cast<double>(joint_deg[j - 1]);
        T *= math_utils::dhTransform<double>(a, alpha, d, math_utils::degreesToRadians(theta));
    }

    return T;
}

Eigen::Matrix3f RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &) const {
    const float delta = JACOBIAN_DELTA;
    const float delta_deg = math_utils::radiansToDegrees(delta);

    Eigen::Matrix3f jacobian;
    for (int j = 0; j < DOF_PER_LEG; ++j) {
        JointAngles plus = q;
        JointAngles minus = q;
        switch (j) {
            case 0:
                plus.coxa += delta_deg * 0.5f;
                minus.coxa -= delta_deg * 0.5f;
                break;
            case 1:
                plus.femur += delta_deg * 0.5f;
                minus.femur -= delta_deg * 0.5f;
                break;
            case 2:
                plus.tibia += delta_deg * 0.5f;
                minus.tibia -= delta_deg * 0.5f;
                break;
        }

        Point3D p_plus = forwardKinematics(leg, plus);
        Point3D p_minus = forwardKinematics(leg, minus);

        jacobian(0, j) = (p_plus.x - p_minus.x) / delta;
        jacobian(1, j) = (p_plus.y - p_minus.y) / delta;
        jacobian(2, j) = (p_plus.z - p_minus.z) / delta;
    }

    return jacobian;
}

bool RobotModel::checkJointLimits(int leg_index, const JointAngles &angles) const {
    return (angles.coxa >= params.coxa_angle_limits[0] && angles.coxa <= params.coxa_angle_limits[1] &&
            angles.femur >= params.femur_angle_limits[0] && angles.femur <= params.femur_angle_limits[1] &&
            angles.tibia >= params.tibia_angle_limits[0] && angles.tibia <= params.tibia_angle_limits[1]);
}

float RobotModel::constrainAngle(float angle, float min_angle, float max_angle) const {
    // First normalize angle to [-180, 180] range to handle wraparound
    float normalized_angle = normalizeAngle(angle);

    // Then clamp to the specified joint limits
    return std::max(min_angle, std::min(max_angle, normalized_angle));
}

float RobotModel::normalizeAngle(float angle_deg) const {
    // Normalize angle to [-180, 180] range following syropod implementation
    // This handles angle wraparound issues that can occur during IK iteration

    // Convert to [-PI, PI] range first
    float angle_rad = angle_deg * M_PI / 180.0f;

    // Normalize to [-PI, PI] using atan2 trick
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));

    // Convert back to degrees
    return angle_rad * RADIANS_TO_DEGREES_FACTOR;
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
    const int resolution = WORKSPACE_RESOLUTION; // discretization resolution

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

Pose RobotModel::getPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &leg_frame_pose) const {
    // Get the full transform from robot frame to leg frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);
    Eigen::Matrix4f tf = transform.cast<float>();

    // Transform the leg frame pose to robot frame
    return leg_frame_pose.transform(tf);
}

Pose RobotModel::getPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose) const {
    // Get the full transform from robot frame to leg frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);
    Eigen::Matrix4f tf = transform.cast<float>();

    // Transform the robot frame pose to leg frame (inverse transform)
    return robot_frame_pose.transform(tf.inverse());
}

Pose RobotModel::getTipPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &tip_frame_pose) const {
    // Get the full transform from robot frame to tip frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);
    Eigen::Matrix4f tf = transform.cast<float>();

    // Transform the tip frame pose to robot frame
    return tip_frame_pose.transform(tf);
}

Pose RobotModel::getTipPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose) const {
    // Get the full transform from robot frame to tip frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);
    Eigen::Matrix4f tf = transform.cast<float>();

    // Transform the robot frame pose to tip frame (inverse transform)
    return robot_frame_pose.transform(tf.inverse());
}
