#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <ArduinoEigen.h>

struct Point3D; // forward declaration

namespace math_utils {
float degreesToRadians(float degrees);
float radiansToDegrees(float radians);
float normalizeAngle(float angle);
Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation);
float distance3D(const Point3D &p1, const Point3D &p2);
bool isPointReachable(const Point3D &point, float max_reach);

Eigen::Matrix3f rotationMatrixX(float angle);
Eigen::Matrix3f rotationMatrixY(float angle);
Eigen::Matrix3f rotationMatrixZ(float angle);

Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion);
Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler);
Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2);
Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q);
Eigen::Matrix4f dhTransform(float a, float alpha, float d, float theta);
} // namespace math_utils

#endif // MATH_UTILS_H
