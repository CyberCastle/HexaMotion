#ifndef POSE_H
#define POSE_H

#include <ArduinoEigen.h>
#include <cmath>
#include <limits>

/**
 * @brief Sentinel value used for undefined pose components.
 */
constexpr double POSE_UNDEFINED_COMPONENT = static_cast<double>(std::numeric_limits<int>::max());

/**
 * @brief Numeric tolerance used for approximate Point3D comparisons.
 */
constexpr double POSE_POINT_APPROX_EPSILON = 1e-9;

/**
 * @brief 3D coordinate in millimeters.
 */
struct Point3D {
    double x, y, z;
    explicit Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Point3D operator+(const Point3D &other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D &operator+=(const Point3D &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Point3D operator-(const Point3D &other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    Point3D operator/(double scalar) const {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    bool operator==(const Point3D &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }

    bool operator!=(const Point3D &other) const {
        return !(*this == other);
    }

    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    /**
     * @brief Approximate comparison using Euclidean norm tolerance.
     * @param other Point to compare against.
     * @param epsilon Comparison tolerance.
     * @return True when points are approximately equal.
     */
    bool isApprox(const Point3D &other, double epsilon = POSE_POINT_APPROX_EPSILON) const;

    Point3D normalized() const {
        double n = norm();
        if (n > 0) {
            return Point3D(x / n, y / n, z / n);
        }
        return Point3D(0, 0, 0);
    }
};

/**
 * @brief Pose representation with translation and quaternion rotation.
 *
 * Equivalent to OpenSHC's Pose API, excluding ROS message conversions.
 */
struct Pose {
    Point3D position;
    Eigen::Quaterniond rotation;

    /**
     * @brief Construct a pose from position and quaternion.
     */
    explicit Pose(const Point3D &pos = Point3D(), const Eigen::Quaterniond &rot = Eigen::Quaterniond::Identity());

    /**
     * @brief Construct a pose from Eigen vector and quaternion.
     */
    explicit Pose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot);

    /**
     * @brief Construct a pose from position and Euler angles in degrees.
     */
    explicit Pose(const Point3D &pos, const Eigen::Vector3d &euler_angles_deg);

    /**
     * @brief Get pose position as Eigen::Vector3d.
     */
    Eigen::Vector3d positionVector() const;

    /**
     * @brief Return the identity pose.
     */
    static Pose Identity();

    /**
     * @brief Return an undefined sentinel pose.
     */
    static Pose Undefined();

    /**
     * @brief Check if pose is finite and not undefined.
     */
    bool isValid() const;

    /**
     * @brief Check whether this pose equals the Undefined sentinel.
     */
    bool isUndefined() const;

    /**
     * @brief Check whether only the rotation component is undefined.
     *
     * OpenSHC equivalent: rotation_.isApprox(UNDEFINED_ROTATION).
     */
    bool isRotationUndefined() const;

    /**
     * @brief Check whether only the position component is undefined.
     */
    bool isPositionUndefined() const;

    bool operator==(const Pose &other) const;

    bool operator!=(const Pose &other) const;

    /**
     * @brief Return the inverse pose (OpenSHC operator~ equivalent).
     */
    Pose operator~() const;

    /**
     * @brief Transform this pose by a homogeneous transform matrix.
     */
    Pose transform(const Eigen::Matrix4d &transform_matrix) const;

    /**
     * @brief Transform a vector into this pose reference frame.
     */
    Point3D transformVector(const Point3D &vec) const;

    /**
     * @brief Transform an Eigen vector into this pose reference frame.
     */
    Eigen::Vector3d transformVector(const Eigen::Vector3d &vec) const;

    /**
     * @brief Transform a vector from this pose reference frame.
     */
    Point3D inverseTransformVector(const Point3D &vec) const;

    /**
     * @brief Transform an Eigen vector from this pose reference frame.
     */
    Eigen::Vector3d inverseTransformVector(const Eigen::Vector3d &vec) const;

    /**
     * @brief Return the inverse pose.
     */
    Pose inverse() const;

    /**
     * @brief Compose this pose with another pose.
     */
    Pose addPose(const Pose &other) const;

    /**
     * @brief Remove another pose from this pose.
     */
    Pose removePose(const Pose &other) const;

    /**
     * @brief Interpolate this pose toward a target pose.
     */
    Pose interpolate(double control_input, const Pose &target_pose) const;
};

inline bool Point3D::isApprox(const Point3D &other, double epsilon) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= epsilon;
}

inline Pose::Pose(const Point3D &pos, const Eigen::Quaterniond &rot)
    : position(pos), rotation(rot) {}

inline Pose::Pose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
    : position(Point3D(pos.x(), pos.y(), pos.z())), rotation(rot) {}

inline Pose::Pose(const Point3D &pos, const Eigen::Vector3d &euler_angles_deg)
    : position(pos) {
    const double deg_to_rad = 3.14159265358979323846 / 180.0;
    Eigen::Vector3d euler_rad = euler_angles_deg * deg_to_rad;
    rotation = (Eigen::AngleAxisd(euler_rad.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(euler_rad.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler_rad.x(), Eigen::Vector3d::UnitX()));
}

inline Eigen::Vector3d Pose::positionVector() const {
    return Eigen::Vector3d(position.x, position.y, position.z);
}

inline Pose Pose::Identity() {
    return Pose(Point3D(), Eigen::Quaterniond::Identity());
}

inline Pose Pose::Undefined() {
    return Pose(Point3D(POSE_UNDEFINED_COMPONENT, POSE_UNDEFINED_COMPONENT, POSE_UNDEFINED_COMPONENT),
                Eigen::Quaterniond(POSE_UNDEFINED_COMPONENT,
                                   POSE_UNDEFINED_COMPONENT,
                                   POSE_UNDEFINED_COMPONENT,
                                   POSE_UNDEFINED_COMPONENT));
}

inline bool Pose::isValid() const {
    const bool valid_position = std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
                                (std::abs(position.x) < POSE_UNDEFINED_COMPONENT) &&
                                (std::abs(position.y) < POSE_UNDEFINED_COMPONENT) &&
                                (std::abs(position.z) < POSE_UNDEFINED_COMPONENT);
    const bool valid_rotation = std::isfinite(rotation.w()) && std::isfinite(rotation.x()) &&
                                std::isfinite(rotation.y()) && std::isfinite(rotation.z()) &&
                                (std::abs(rotation.w()) < POSE_UNDEFINED_COMPONENT) &&
                                (std::abs(rotation.x()) < POSE_UNDEFINED_COMPONENT) &&
                                (std::abs(rotation.y()) < POSE_UNDEFINED_COMPONENT) &&
                                (std::abs(rotation.z()) < POSE_UNDEFINED_COMPONENT);
    return valid_position && valid_rotation;
}

inline bool Pose::isUndefined() const {
    return (*this) == Pose::Undefined();
}

inline bool Pose::isRotationUndefined() const {
    return std::abs(rotation.w()) >= POSE_UNDEFINED_COMPONENT ||
           std::abs(rotation.x()) >= POSE_UNDEFINED_COMPONENT ||
           std::abs(rotation.y()) >= POSE_UNDEFINED_COMPONENT ||
           std::abs(rotation.z()) >= POSE_UNDEFINED_COMPONENT;
}

inline bool Pose::isPositionUndefined() const {
    return std::abs(position.x) >= POSE_UNDEFINED_COMPONENT ||
           std::abs(position.y) >= POSE_UNDEFINED_COMPONENT ||
           std::abs(position.z) >= POSE_UNDEFINED_COMPONENT;
}

inline bool Pose::operator==(const Pose &other) const {
    return position.isApprox(other.position) && rotation.isApprox(other.rotation);
}

inline bool Pose::operator!=(const Pose &other) const {
    return !(*this == other);
}

inline Pose Pose::operator~() const {
    return inverse();
}

inline Pose Pose::transform(const Eigen::Matrix4d &transform_matrix) const {
    Eigen::Vector4d pos_homogeneous(position.x, position.y, position.z, 1.0);
    Eigen::Vector4d transformed_pos = transform_matrix * pos_homogeneous;

    Eigen::Matrix3d rot_matrix = transform_matrix.block<3, 3>(0, 0);
    Eigen::Quaterniond transformed_rot(rot_matrix);
    Eigen::Quaterniond result_rot = (transformed_rot * rotation).normalized();

    return Pose(Point3D(transformed_pos.x(), transformed_pos.y(), transformed_pos.z()), result_rot);
}

inline Point3D Pose::transformVector(const Point3D &vec) const {
    Eigen::Vector3d transformed = transformVector(Eigen::Vector3d(vec.x, vec.y, vec.z));
    return Point3D(transformed.x(), transformed.y(), transformed.z());
}

inline Eigen::Vector3d Pose::transformVector(const Eigen::Vector3d &vec) const {
    return positionVector() + rotation._transformVector(vec);
}

inline Point3D Pose::inverseTransformVector(const Point3D &vec) const {
    Eigen::Vector3d transformed = inverseTransformVector(Eigen::Vector3d(vec.x, vec.y, vec.z));
    return Point3D(transformed.x(), transformed.y(), transformed.z());
}

inline Eigen::Vector3d Pose::inverseTransformVector(const Eigen::Vector3d &vec) const {
    return (~(*this)).transformVector(vec);
}

inline Pose Pose::inverse() const {
    Eigen::Quaterniond inv_rotation = rotation.conjugate();
    Eigen::Vector3d inv_position = inv_rotation._transformVector(-positionVector());
    return Pose(Point3D(inv_position.x(), inv_position.y(), inv_position.z()), inv_rotation);
}

inline Pose Pose::addPose(const Pose &other) const {
    Point3D new_position = transformVector(other.position);
    Eigen::Quaterniond new_rotation = rotation * other.rotation;
    return Pose(new_position, new_rotation);
}

inline Pose Pose::removePose(const Pose &other) const {
    Point3D new_position = transformVector(Point3D(-other.position.x, -other.position.y, -other.position.z));
    Eigen::Quaterniond new_rotation = rotation * other.rotation.inverse();
    return Pose(new_position, new_rotation);
}

inline Pose Pose::interpolate(double control_input, const Pose &target_pose) const {
    Point3D new_position = position * (1.0 - control_input) + target_pose.position * control_input;
    Eigen::Quaterniond new_rotation = rotation.slerp(control_input, target_pose.rotation);
    return Pose(new_position, new_rotation);
}

#endif // POSE_H