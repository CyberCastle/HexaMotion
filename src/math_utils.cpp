#include "math_utils.h"
#include "hexamotion_constants.h"
#include "robot_model.h"
#include <algorithm>
#include <cmath>

// Utility function implementations
namespace math_utils {
double degreesToRadians(double degrees) {
    return degrees * DEGREES_TO_RADIANS_FACTOR;
}

double radiansToDegrees(double radians) {
    return radians * RADIANS_TO_DEGREES_FACTOR;
}

double normalizeAngle(double angle) {
    while (angle > HALF_ROTATION_DEGREES)
        angle -= FULL_ROTATION_DEGREES;
    while (angle < -HALF_ROTATION_DEGREES)
        angle += FULL_ROTATION_DEGREES;
    return angle;
}

Point3D rotatePoint(const Point3D &point, const Eigen::Vector3d &rotation) {
    Eigen::Matrix3d Rx = rotationMatrixX(rotation[0]);
    Eigen::Matrix3d Ry = rotationMatrixY(rotation[1]);
    Eigen::Matrix3d Rz = rotationMatrixZ(rotation[2]);

    Eigen::Matrix3d R = Rz * Ry * Rx;
    Eigen::Vector3d p(point.x, point.y, point.z);
    Eigen::Vector3d rotated = R * p;

    return Point3D(rotated[0], rotated[1], rotated[2]);
}

double distance3D(const Point3D &p1, const Point3D &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

double distance2D(const Point3D &p1, const Point3D &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y));
}

double magnitude(const Point3D &point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

double distance(const Point3D &p1, const Point3D &p2) {
    return distance3D(p1, p2);
}

bool isPointReachable(const Point3D &point, double max_reach) {
    double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return distance <= max_reach;
}

bool isPointReachable(const Point3D &point, double min_reach, double max_reach) {
    double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return distance >= min_reach && distance <= max_reach;
}

Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) {
    Eigen::Vector4d result;
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    return result;
}

Eigen::Vector4d quaternionInverse(const Eigen::Vector4d &q) {
    double norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (norm_sq > 0) {
        return Eigen::Vector4d(q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq);
    }
    return q; // Return original if degenerate
}

template <typename T>
Eigen::Matrix<T, 4, 4> dhTransform(T a, T alpha, T d, T theta) {
    T cos_theta = std::cos(theta);
    T sin_theta = std::sin(theta);
    T cos_alpha = std::cos(alpha);
    T sin_alpha = std::sin(alpha);

    Eigen::Matrix<T, 4, 4> transform;
    transform << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta,
        sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
        T(0), sin_alpha, cos_alpha, d,
        T(0), T(0), T(0), T(1);
    return transform;
}

template <typename T>
Eigen::Matrix<T, 4, 4> dhTransformY(T a, T alpha, T d, T theta) {

    // Similar to dhTransform but rotates around the Y axis. This matches
    // the analytic model where femur and tibia joints pitch about Y.
    Eigen::Matrix<T, 4, 4> R_y = Eigen::Matrix<T, 4, 4>::Identity();
    R_y.block(0, 0, 3, 3) =
        Eigen::AngleAxis<T>(theta, Eigen::Matrix<T, 3, 1>::UnitY()).toRotationMatrix();

    Eigen::Matrix<T, 4, 4> T_z = Eigen::Matrix<T, 4, 4>::Identity();
    T_z(2, 3) = d;

    Eigen::Matrix<T, 4, 4> T_x = Eigen::Matrix<T, 4, 4>::Identity();
    T_x(0, 3) = a;

    Eigen::Matrix<T, 4, 4> R_x = Eigen::Matrix<T, 4, 4>::Identity();
    R_x.block(0, 0, 3, 3) =
        Eigen::AngleAxis<T>(alpha, Eigen::Matrix<T, 3, 1>::UnitX()).toRotationMatrix();

    return R_y * T_z * T_x * R_x;
}

// Explicit instantiations for common precisions
template Eigen::Matrix4d dhTransform<double>(double, double, double, double);
template Eigen::Matrix4d dhTransformY<double>(double, double, double, double);

Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta) {
    return dhTransform<double>(a, alpha, d, theta);
}

Eigen::Matrix4d dhTransformY(double a, double alpha, double d, double theta) {
    return dhTransformY<double>(a, alpha, d, theta);
}

Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d &quaternion) {
    Eigen::Vector3d euler;
    double w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        euler[1] = copysign(M_PI / 2, sinp);
    else
        euler[1] = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);

    return euler;
}

Eigen::Vector4d eulerToQuaternion(const Eigen::Vector3d &euler) {
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Vector4d q;
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    return q;
}

Eigen::Matrix3d rotationMatrixX(double angle) {
    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, cos(angle), -sin(angle),
        0, sin(angle), cos(angle);
    return R;
}

Eigen::Matrix3d rotationMatrixY(double angle) {
    Eigen::Matrix3d R;
    R << cos(angle), 0, sin(angle),
        0, 1, 0,
        -sin(angle), 0, cos(angle);
    return R;
}

Eigen::Matrix3d rotationMatrixZ(double angle) {
    Eigen::Matrix3d R;
    R << cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1;
    return R;
}

// Pose system quaternion utilities
Eigen::Vector3d point3DToVector3d(const Point3D &point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Point3D vector3fToPoint3D(const Eigen::Vector3d &vec) {
    return Point3D(vec[0], vec[1], vec[2]);
}

Eigen::Vector4d eulerPoint3DToQuaternion(const Point3D &euler) {
    Eigen::Vector3d euler_vec = point3DToVector3d(euler);
    return eulerToQuaternion(euler_vec);
}

Point3D quaternionToEulerPoint3D(const Eigen::Vector4d &quaternion) {
    Eigen::Vector3d euler_vec = quaternionToEuler(quaternion);
    return vector3fToPoint3D(euler_vec);
}

Eigen::Vector4d quaternionSlerp(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, double t) {
    // Compute dot product
    double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // If dot product is negative, take the shorter path by negating one quaternion
    Eigen::Vector4d q2_adj = q2;
    if (dot < 0.0) {
        q2_adj = -q2;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation to avoid numerical issues
    if (dot > 0.9995) {
        Eigen::Vector4d result = q1 + t * (q2_adj - q1);
        double norm = sqrt(result[0] * result[0] + result[1] * result[1] +
                           result[2] * result[2] + result[3] * result[3]);
        if (norm > 0.0) {
            result = result / norm;
        }
        return result;
    }

    // Calculate angle and perform SLERP
    double theta_0 = acos(std::abs(dot));
    double sin_theta_0 = sin(theta_0);

    double theta = theta_0 * t;
    double sin_theta = sin(theta);

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    return s0 * q1 + s1 * q2_adj;
}

double pointToLineDistance(const Point3D &point, const Point3D &line_start, const Point3D &line_end) {
    // Calculate the vector from line_start to line_end
    Point3D line_vec = Point3D(line_end.x - line_start.x, line_end.y - line_start.y, line_end.z - line_start.z);

    // Calculate the vector from line_start to the point
    Point3D point_vec = Point3D(point.x - line_start.x, point.y - line_start.y, point.z - line_start.z);

    // Calculate the length squared of the line vector
    double line_length_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y + line_vec.z * line_vec.z;

    // Handle degenerate case where line_start == line_end
    if (line_length_sq < 1e-6) {
        return distance(point, line_start);
    }

    // Calculate the projection parameter t
    double t = (point_vec.x * line_vec.x + point_vec.y * line_vec.y + point_vec.z * line_vec.z) / line_length_sq;

    // Clamp t to [0, 1] to get the closest point on the line segment
    t = std::max(0.0, std::min(1.0, t));

    // Calculate the closest point on the line segment
    Point3D closest_point = Point3D(line_start.x + t * line_vec.x,
                                    line_start.y + t * line_vec.y,
                                    line_start.z + t * line_vec.z);

    // Return the distance from the point to the closest point on the line segment
    return distance(point, closest_point);
}

Point3D crossProduct(const Point3D &a, const Point3D &b) {
    return Point3D(a.y * b.z - a.z * b.y,
                   a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x);
}

Point3D projectVector(const Point3D &vector, const Point3D &onto) {
    double onto_magnitude_sq = onto.x * onto.x + onto.y * onto.y + onto.z * onto.z;

    // Handle degenerate case where onto vector is zero
    if (onto_magnitude_sq < 1e-6) {
        return Point3D(0, 0, 0);
    }

    double dot_product = vector.x * onto.x + vector.y * onto.y + vector.z * onto.z;
    double scale = dot_product / onto_magnitude_sq;

    return Point3D(onto.x * scale, onto.y * scale, onto.z * scale);
}

bool solveLeastSquaresPlane(const double *raw_A, const double *raw_B, int num_points, double &a, double &b, double &c) {

    // Build normal equations: A^T * A * x = A^T * b
    double ATA[3][3] = {{0}};
    double ATb[3] = {0};

    for (int i = 0; i < num_points; i++) {
        double x = raw_A[i * 3];
        double y = raw_A[i * 3 + 1];
        double z = raw_B[i];

        // A^T * A
        ATA[0][0] += x * x;
        ATA[0][1] += x * y;
        ATA[0][2] += x;
        ATA[1][1] += y * y;
        ATA[1][2] += y;
        ATA[2][2] += 1;

        // A^T * b
        ATb[0] += x * z;
        ATb[1] += y * z;
        ATb[2] += z;
    }

    // Symmetric matrix
    ATA[1][0] = ATA[0][1];
    ATA[2][0] = ATA[0][2];
    ATA[2][1] = ATA[1][2];

    // Solve using Cramer's rule for 3x3 system
    double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
                 ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
                 ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

    if (abs(det) > 1e-6) { // Check for singular matrix
        a = (ATb[0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
             ATA[0][1] * (ATb[1] * ATA[2][2] - ATA[1][2] * ATb[2]) +
             ATA[0][2] * (ATb[1] * ATA[2][1] - ATA[1][1] * ATb[2])) /
            det;
        b = (ATA[0][0] * (ATb[1] * ATA[2][2] - ATA[1][2] * ATb[2]) -
             ATb[0] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
             ATA[0][2] * (ATA[1][0] * ATb[2] - ATb[1] * ATA[2][0])) /
            det;
        c = (ATA[0][0] * (ATA[1][1] * ATb[2] - ATA[1][2] * ATb[1]) -
             ATA[0][1] * (ATA[1][0] * ATb[2] - ATA[1][2] * ATb[0]) +
             ATb[0] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0])) /
            det;
        return true;
    }

    return false; // Matrix is singular
}

} // namespace math_utils
