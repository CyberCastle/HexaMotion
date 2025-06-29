#include "math_utils.h"
#include "HexaModel.h"
#include "hexamotion_constants.h"
#include <math.h>

// Utility function implementations
namespace math_utils {
float degreesToRadians(float degrees) {
    return degrees * DEGREES_TO_RADIANS_FACTOR;
}

float radiansToDegrees(float radians) {
    return radians * RADIANS_TO_DEGREES_FACTOR;
}

float normalizeAngle(float angle) {
    while (angle > HALF_ROTATION_DEGREES)
        angle -= FULL_ROTATION_DEGREES;
    while (angle < -HALF_ROTATION_DEGREES)
        angle += FULL_ROTATION_DEGREES;
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

float distance2D(const Point3D &p1, const Point3D &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y));
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
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);

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

    return euler;
}

Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler) {
    float roll = euler[0];
    float pitch = euler[1];
    float yaw = euler[2];

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
    Eigen::Matrix3f R;
    R << 1, 0, 0,
        0, cos(angle), -sin(angle),
        0, sin(angle), cos(angle);
    return R;
}

Eigen::Matrix3f rotationMatrixY(float angle) {
    Eigen::Matrix3f R;
    R << cos(angle), 0, sin(angle),
        0, 1, 0,
        -sin(angle), 0, cos(angle);
    return R;
}

Eigen::Matrix3f rotationMatrixZ(float angle) {
    Eigen::Matrix3f R;
    R << cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
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

Eigen::Vector4f eulerPoint3DToQuaternion(const Point3D &euler) {
    Eigen::Vector3f euler_vec = point3DToVector3f(euler);
    return eulerToQuaternion(euler_vec);
}

Point3D quaternionToEulerPoint3D(const Eigen::Vector4f &quaternion) {
    Eigen::Vector3f euler_vec = quaternionToEuler(quaternion);
    return vector3fToPoint3D(euler_vec);
}

float pointToLineDistance(const Point3D &point, const Point3D &line_start, const Point3D &line_end) {
    // Calculate the vector from line_start to line_end
    Point3D line_vec = Point3D(line_end.x - line_start.x, line_end.y - line_start.y, line_end.z - line_start.z);

    // Calculate the vector from line_start to the point
    Point3D point_vec = Point3D(point.x - line_start.x, point.y - line_start.y, point.z - line_start.z);

    // Calculate the length squared of the line vector
    float line_length_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y + line_vec.z * line_vec.z;

    // Handle degenerate case where line_start == line_end
    if (line_length_sq < 1e-6f) {
        return distance(point, line_start);
    }

    // Calculate the projection of point_vec onto line_vec
    float dot_product = point_vec.x * line_vec.x + point_vec.y * line_vec.y + point_vec.z * line_vec.z;
    float t = dot_product / line_length_sq;

    // Clamp t to [0, 1] to ensure we're on the line segment
    t = std::max(0.0f, std::min(DEFAULT_ANGULAR_SCALING, t));

    // Calculate the closest point on the line segment
    Point3D closest_point = Point3D(
        line_start.x + t * line_vec.x,
        line_start.y + t * line_vec.y,
        line_start.z + t * line_vec.z);

    // Return the distance from the point to the closest point on the line
    return distance(point, closest_point);
}

} // namespace math_utils
