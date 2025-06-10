#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <ArduinoEigen.h>

struct Point3D; // forward declaration

namespace math_utils {
/** Convert degrees to radians. */
float degreesToRadians(float degrees);
/** Convert radians to degrees. */
float radiansToDegrees(float radians);
/** Normalize an angle to the [-180,180] range. */
float normalizeAngle(float angle);
/** Rotate a 3D point by roll/pitch/yaw angles. */
Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation);
/** Euclidean distance between two points. */
float distance3D(const Point3D &p1, const Point3D &p2);
/** Check if a point is within a spherical reach. */
bool isPointReachable(const Point3D &point, float max_reach);

/** Rotation matrix about X axis. */
Eigen::Matrix3f rotationMatrixX(float angle);
/** Rotation matrix about Y axis. */
Eigen::Matrix3f rotationMatrixY(float angle);
/** Rotation matrix about Z axis. */
Eigen::Matrix3f rotationMatrixZ(float angle);

/** Convert quaternion to Euler angles (degrees). */
Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion);
/** Convert Euler angles (degrees) to quaternion. */
Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler);
/** Multiply two quaternions. */
Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2);
/** Invert a quaternion. */
Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q);
/** Denavit-Hartenberg transform matrix helper. */
Eigen::Matrix4f dhTransform(float a, float alpha, float d, float theta);

/**
 * @brief Evaluate a quadratic Bezier curve.
 * @tparam T Vector type supporting basic arithmetic.
 * @param points Array of 3 control points.
 * @param t Curve parameter between 0 and 1.
 */
template <class T>
inline T quadraticBezier(const T *points, double t) {
    double s = 1.0 - t;
    return points[0] * (s * s) + points[1] * (2.0 * t * s) + points[2] * (t * t);
}

/**
 * @brief Evaluate a cubic Bezier curve.
 * @tparam T Vector type supporting basic arithmetic.
 * @param points Array of 4 control points.
 * @param t Curve parameter between 0 and 1.
 */
template <class T>
inline T cubicBezier(const T *points, double t) {
    double s = 1.0 - t;
    return points[0] * (s * s * s) +
           points[1] * (3.0 * t * s * s) +
           points[2] * (3.0 * t * t * s) +
           points[3] * (t * t * t);
}

/**
 * @brief Derivative of a cubic Bezier curve.
 */
template <class T>
inline T cubicBezierDot(const T *points, double t) {
    double s = 1.0 - t;
    return 3.0 * s * s * (points[1] - points[0]) +
           6.0 * s * t * (points[2] - points[1]) +
           3.0 * t * t * (points[3] - points[2]);
}

/**
 * @brief Evaluate a quartic Bezier curve.
 */
template <class T>
inline T quarticBezier(const T *points, double t) {
    double s = 1.0 - t;
    return points[0] * (s * s * s * s) +
           points[1] * (4.0 * t * s * s * s) +
           points[2] * (6.0 * t * t * s * s) +
           points[3] * (4.0 * t * t * t * s) +
           points[4] * (t * t * t * t);
}

/**
 * @brief Derivative of a quartic Bezier curve.
 */
template <class T>
inline T quarticBezierDot(const T *points, double t) {
    double s = 1.0 - t;
    return 4.0 * s * s * s * (points[1] - points[0]) +
           12.0 * s * s * t * (points[2] - points[1]) +
           12.0 * s * t * t * (points[3] - points[2]) +
           4.0 * t * t * t * (points[4] - points[3]);
}
} // namespace math_utils

#endif // MATH_UTILS_H
