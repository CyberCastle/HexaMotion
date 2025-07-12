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
    // Numerical Jacobian computation using forward kinematics
    const double delta = JACOBIAN_DELTA;
    Point3D base = forwardKinematicsGlobalCoordinatesAnalytic(leg, q);
    JointAngles qd = q;
    qd.coxa += delta;
    Point3D p_dx = forwardKinematicsGlobalCoordinatesAnalytic(leg, qd);
    qd = q;
    qd.femur += delta;
    Point3D p_dy = forwardKinematicsGlobalCoordinatesAnalytic(leg, qd);
    qd = q;
    qd.tibia += delta;
    Point3D p_dz = forwardKinematicsGlobalCoordinatesAnalytic(leg, qd);
    Eigen::Matrix3d jacobian;
    jacobian.col(0) = Eigen::Vector3d((p_dx.x - base.x) / delta,
                                      (p_dx.y - base.y) / delta,
                                      (p_dx.z - base.z) / delta);
    jacobian.col(1) = Eigen::Vector3d((p_dy.x - base.x) / delta,
                                      (p_dy.y - base.y) / delta,
                                      (p_dy.z - base.z) / delta);
    jacobian.col(2) = Eigen::Vector3d((p_dz.x - base.x) / delta,
                                      (p_dz.y - base.y) / delta,
                                      (p_dz.z - base.z) / delta);
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