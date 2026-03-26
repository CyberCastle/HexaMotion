#include "analytic_robot_model.h"
#include <cmath>

AnalyticRobotModel::AnalyticRobotModel(const Parameters &params) : params(params) {}

Point3D AnalyticRobotModel::getAnalyticLegBasePosition(int leg_index) const {
    // Compute base position using nominal leg offset angle
    const double angle_rad = BASE_THETA_OFFSETS[leg_index];
    double x = params.hexagon_radius * cos(angle_rad);
    double y = params.hexagon_radius * sin(angle_rad);
    return Point3D{x, y, 0.0f};
}

Point3D AnalyticRobotModel::forwardKinematicsGlobalCoordinatesAnalytic(int leg_index, const JointAngles &angles) const {
    // Forward kinematics using analytic leg model
    Eigen::Matrix4d transform = legTransformAnalytic(leg_index, angles);
    return Point3D{transform(0, 3), transform(1, 3), transform(2, 3)};
}

Eigen::Matrix4d AnalyticRobotModel::legTransformAnalytic(int leg_index, const JointAngles &q) const {
    const double base_angle = BASE_THETA_OFFSETS[leg_index];
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisd(base_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T(0, 3) = params.hexagon_radius * cos(base_angle);
    T(1, 3) = params.hexagon_radius * sin(base_angle);
    Eigen::Matrix4d R_coxa = Eigen::Matrix4d::Identity();
    R_coxa.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.coxa, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d T_coxa = Eigen::Matrix4d::Identity();
    T_coxa(0, 3) = params.coxa_length;
    Eigen::Matrix4d R_femur = Eigen::Matrix4d::Identity();
    R_femur.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.femur, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix4d T_femur = Eigen::Matrix4d::Identity();
    T_femur(0, 3) = params.femur_length;
    Eigen::Matrix4d R_tibia = Eigen::Matrix4d::Identity();
    R_tibia.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.tibia, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix4d T_tibia = Eigen::Matrix4d::Identity();
    T_tibia(2, 3) = -params.tibia_length;
    T = T * R_coxa * T_coxa * R_femur * T_femur * R_tibia * T_tibia;
    return T;
}

Eigen::Matrix3d AnalyticRobotModel::calculateJacobianAnalytic(int leg, const JointAngles &q, const Point3D &) const {
    /** Numerical Jacobian via central differences for O(h²) accuracy. */
    const double delta = JACOBIAN_DELTA;
    const double half = delta * 0.5;
    Eigen::Matrix3d jacobian;
    for (int joint = 0; joint < DOF_PER_LEG; ++joint) {
        JointAngles plus = q, minus = q;
        if (joint == 0) {
            plus.coxa += half;
            minus.coxa -= half;
        } else if (joint == 1) {
            plus.femur += half;
            minus.femur -= half;
        } else {
            plus.tibia += half;
            minus.tibia -= half;
        }
        Point3D pp = forwardKinematicsGlobalCoordinatesAnalytic(leg, plus);
        Point3D pm = forwardKinematicsGlobalCoordinatesAnalytic(leg, minus);
        jacobian(0, joint) = (pp.x - pm.x) / delta;
        jacobian(1, joint) = (pp.y - pm.y) / delta;
        jacobian(2, joint) = (pp.z - pm.z) / delta;
    }
    return jacobian;
}

std::vector<Eigen::Matrix4d> AnalyticRobotModel::buildDHTransformsAnalytic(int leg, const JointAngles &q) const {
    std::vector<Eigen::Matrix4d> transforms(DOF_PER_LEG + 1);
    const double base_angle = BASE_THETA_OFFSETS[leg];
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisd(base_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T(0, 3) = params.hexagon_radius * cos(base_angle);
    T(1, 3) = params.hexagon_radius * sin(base_angle);
    transforms[0] = T;
    Eigen::Matrix4d R_coxa = Eigen::Matrix4d::Identity();
    R_coxa.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.coxa, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d T_coxa = Eigen::Matrix4d::Identity();
    T_coxa(0, 3) = params.coxa_length;
    T = T * R_coxa * T_coxa;
    transforms[1] = T;
    Eigen::Matrix4d R_femur = Eigen::Matrix4d::Identity();
    R_femur.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.femur, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix4d T_femur = Eigen::Matrix4d::Identity();
    T_femur(0, 3) = params.femur_length;
    T = T * R_femur * T_femur;
    transforms[2] = T;
    Eigen::Matrix4d R_tibia = Eigen::Matrix4d::Identity();
    R_tibia.block<3, 3>(0, 0) = Eigen::AngleAxisd(q.tibia, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix4d T_tibia = Eigen::Matrix4d::Identity();
    T_tibia(2, 3) = -params.tibia_length;
    T = T * R_tibia * T_tibia;
    transforms[3] = T;
    return transforms;
}