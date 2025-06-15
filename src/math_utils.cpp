#include "math_utils.h"
#include "HexaModel.h"
#include <math.h>

// Utility function implementations
namespace math_utils {
float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float radiansToDegrees(float radians) {
    return radians * 180.0f / M_PI;
}

float normalizeAngle(float angle) {
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation) {
    Eigen::Matrix3f Rx = rotationMatrixX(rotation[0]);
    Eigen::Matrix3f Ry = rotationMatrixY(rotation[1]);
    Eigen::Matrix3f Rz = rotationMatrixZ(rotation[2]);

    Eigen::Matrix3f R = Rz * Ry * Rx;
    Eigen::Vector3f p(point.x, point.y, point.z);
    Eigen::Vector3f rotated = R * p;

    return Point3D(rotated[0], rotated[1], rotated[2]);
}

float distance3D(const Point3D &p1, const Point3D &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

float magnitude(const Point3D &point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

float distance(const Point3D &p1, const Point3D &p2) {
    return distance3D(p1, p2);
}

bool isPointReachable(const Point3D &point, float max_reach) {
    float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return distance <= max_reach;
}

bool isPointReachable(const Point3D &point, float min_reach, float max_reach) {
    float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return distance >= min_reach && distance <= max_reach;
}

Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2) {
    Eigen::Vector4f result;
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    return result;
}

Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q) {
    float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (norm_sq > 0) {
        return Eigen::Vector4f(q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq);
    }
    return q; // Return original if degenerate
}

Eigen::Matrix4f dhTransform(float a, float alpha, float d, float theta) {
    float cos_theta = cos(math_utils::degreesToRadians(theta));
    float sin_theta = sin(math_utils::degreesToRadians(theta));
    float cos_alpha = cos(math_utils::degreesToRadians(alpha));
    float sin_alpha = sin(math_utils::degreesToRadians(alpha));

    Eigen::Matrix4f transform;
    transform << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta,
        sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
        0, sin_alpha, cos_alpha, d,
        0, 0, 0, 1;
    return transform;
}

Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion) {
    Eigen::Vector3f euler;
    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        euler[1] = copysign(M_PI / 2, sinp);
    else
        euler[1] = asin(sinp);

    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);

    euler[0] = math_utils::radiansToDegrees(euler[0]);
    euler[1] = math_utils::radiansToDegrees(euler[1]);
    euler[2] = math_utils::radiansToDegrees(euler[2]);

    return euler;
}

Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler) {
    float roll = math_utils::degreesToRadians(euler[0]);
    float pitch = math_utils::degreesToRadians(euler[1]);
    float yaw = math_utils::degreesToRadians(euler[2]);

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Eigen::Vector4f q;
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    return q;
}

Eigen::Matrix3f rotationMatrixX(float angle) {
    float rad = math_utils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << 1, 0, 0,
        0, cos(rad), -sin(rad),
        0, sin(rad), cos(rad);
    return R;
}

Eigen::Matrix3f rotationMatrixY(float angle) {
    float rad = math_utils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << cos(rad), 0, sin(rad),
        0, 1, 0,
        -sin(rad), 0, cos(rad);
    return R;
}

Eigen::Matrix3f rotationMatrixZ(float angle) {
    float rad = math_utils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << cos(rad), -sin(rad), 0,
        sin(rad), cos(rad), 0,
        0, 0, 1;
    return R;
}

// Pose system quaternion utilities
Eigen::Vector3f point3DToVector3f(const Point3D &point) {
    return Eigen::Vector3f(point.x, point.y, point.z);
}

Point3D vector3fToPoint3D(const Eigen::Vector3f &vec) {
    return Point3D(vec[0], vec[1], vec[2]);
}

Eigen::Vector4f eulerPoint3DToQuaternion(const Point3D &euler_degrees) {
    Eigen::Vector3f euler_vec = point3DToVector3f(euler_degrees);
    return eulerToQuaternion(euler_vec);
}

Point3D quaternionToEulerPoint3D(const Eigen::Vector4f &quaternion) {
    Eigen::Vector3f euler_vec = quaternionToEuler(quaternion);
    return vector3fToPoint3D(euler_vec);
}

} // namespace math_utils
