#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <ArduinoEigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point3D; // forward declaration

namespace math_utils {

// Standard gravity acceleration (mm/s^2) - BIPM definition (9.80665 m/s^2)
constexpr double GRAVITY_ACCELERATION = 9806.65;

/** Convert degrees to radians. */
constexpr inline double degreesToRadians(double degrees) { return degrees * (M_PI / 180.0); }

/** Convert radians to degrees. */
constexpr inline double radiansToDegrees(double radians) { return radians * (180.0 / M_PI); }

/** Normalize an angle to the [-180,180] range. */
double normalizeAngle(double angle);

/**
 * Rotate a 3D point by roll/pitch/yaw angles (radians).
 */
Point3D rotatePoint(const Point3D &point, const Eigen::Vector3d &rotation);
/** Euclidean distance between two points. */
double distance3D(const Point3D &p1, const Point3D &p2);
/** 2D distance between two points (ignoring Z coordinate). */
double distance2D(const Point3D &p1, const Point3D &p2);
/** Check if a point is within a spherical reach (simple max reach check). */
bool isPointReachable(const Point3D &point, double max_reach);
/** Check if a point is within spherical reach bounds (min and max reach). */
bool isPointReachable(const Point3D &point, double min_reach, double max_reach);
/** Magnitude of a 3D vector. */
double magnitude(const Point3D &point);
/** Distance between two 3D points (alias for distance3D). */
double distance(const Point3D &p1, const Point3D &p2);
/** Calculate perpendicular distance from point to line segment. */
double pointToLineDistance(const Point3D &point, const Point3D &line_start, const Point3D &line_end);

/** Cross product of two 3D vectors. */
Point3D crossProduct(const Point3D &a, const Point3D &b);
/**
 * Project a vector onto another vector.
 * Reference: https://en.wikipedia.org/wiki/Vector_projection
 */
Point3D projectVector(const Point3D &vector, const Point3D &onto);

/**
 * Vector rejection of a onto b, defined as a - proj_b(a).
 * Reference: https://en.wikipedia.org/wiki/Vector_projection#Vector_rejection
 */
Point3D rejectVector(const Point3D &vector, const Point3D &onto);

/** Rotation matrix about X axis (angle in radians). */
Eigen::Matrix3d rotationMatrixX(double angle);
/** Rotation matrix about Y axis (angle in radians). */
Eigen::Matrix3d rotationMatrixY(double angle);
/** Rotation matrix about Z axis (angle in radians). */
Eigen::Matrix3d rotationMatrixZ(double angle);

/** Convert quaternion to Euler angles (radians). */
Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d &quaternion);
/** Convert Euler angles (radians) to quaternion. */
Eigen::Vector4d eulerToQuaternion(const Eigen::Vector3d &euler);

/**
 * @brief Correct quaternion hemisphere to ensure shortest rotation path.
 * Equivalent to OpenSHC's correctRotation() — ensures the quaternion
 * lies in the same hemisphere as the reference to prevent unwanted
 * 360° flips during interpolation.
 * @param rotation The quaternion to correct
 * @param reference The reference quaternion for hemisphere comparison
 * @return Corrected quaternion in the same hemisphere as reference
 */
inline Eigen::Quaterniond correctRotation(const Eigen::Quaterniond &rotation,
                                          const Eigen::Quaterniond &reference) {
    if (rotation.dot(reference) < 0.0) {
        return Eigen::Quaterniond(-rotation.w(), -rotation.x(), -rotation.y(), -rotation.z());
    }
    return rotation;
}

/**
 * @brief Convert Eigen::Quaterniond to Euler angles using intrinsic ZYX convention.
 * Equivalent to OpenSHC's quaternionToEulerAngles for Quaterniond types.
 * @param q Input quaternion
 * @param extrinsic If true, use extrinsic convention (OpenSHC default)
 * @return Euler angles as Vector3d (roll, pitch, yaw) in radians
 */
inline Eigen::Vector3d quaterniondToEulerAngles(const Eigen::Quaterniond &q, bool extrinsic = false) {
    Eigen::Quaterniond qn = q.normalized();
    if (extrinsic) {
        // Extrinsic XYZ = Intrinsic ZYX
        Eigen::Vector3d euler = qn.toRotationMatrix().eulerAngles(2, 1, 0);
        return Eigen::Vector3d(euler[2], euler[1], euler[0]); // roll, pitch, yaw
    }
    Eigen::Vector3d euler = qn.toRotationMatrix().eulerAngles(2, 1, 0);
    return Eigen::Vector3d(euler[2], euler[1], euler[0]);
}

/**
 * @brief Convert Euler angles to Eigen::Quaterniond using intrinsic ZYX convention.
 * Equivalent to OpenSHC's eulerAnglesToQuaternion for Quaterniond types.
 * @param euler Euler angles as Vector3d (roll, pitch, yaw) in radians
 * @param extrinsic If true, use extrinsic convention (OpenSHC default)
 * @return Quaternion
 */
inline Eigen::Quaterniond eulerAnglesToQuaterniond(const Eigen::Vector3d &euler, bool extrinsic = false) {
    if (extrinsic) {
        return Eigen::Quaterniond(
            Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}

/**
 * @brief Clamp a vector's magnitude without changing direction.
 * Equivalent to OpenSHC's clamped() for Vector3d.
 * @param v Vector to clamp
 * @param max_magnitude Maximum allowed magnitude
 * @return Clamped vector
 */
inline Eigen::Vector3d clampedVector(const Eigen::Vector3d &v, double max_magnitude) {
    double mag = v.norm();
    if (mag > max_magnitude && mag > 0.0) {
        return v * (max_magnitude / mag);
    }
    return v;
}

/**
 * @brief Clamp a 2D vector's magnitude.
 * @param v Vector to clamp
 * @param max_magnitude Maximum allowed magnitude
 * @return Clamped vector
 */
inline Eigen::Vector2d clampedVector2d(const Eigen::Vector2d &v, double max_magnitude) {
    double mag = v.norm();
    if (mag > max_magnitude && mag > 0.0) {
        return v * (max_magnitude / mag);
    }
    return v;
}

/**
 * @brief Modulo function that handles negative values correctly.
 * Equivalent to OpenSHC's mod() utility.
 * @param a Value
 * @param b Divisor
 * @return Positive modulo result
 */
inline int mod(int a, int b) {
    int r = a % b;
    return (r < 0) ? r + b : r;
}

/**
 * @brief Round to nearest integer
 * @param x The value to round
 * @return Rounded integer
 */
inline int roundToInt(double x) {
    return (x >= 0) ? static_cast<int>(x + 0.5) : -static_cast<int>(0.5 - x);
}

/**
 * @brief Round to nearest even integer.
 * Equivalent to OpenSHC's roundToEvenInt().
 * @param x Value to round
 * @return Nearest even integer
 */
inline int roundToEvenInt(double x) {
    int rounded = roundToInt(x);
    return (rounded % 2 == 0) ? rounded : (rounded + 1);
}

/**
 * @brief Sign function returning -1, 0, or 1.
 * @param x Value
 * @return -1 if x<0, 0 if x==0, 1 if x>0
 */
inline double sign(double x) {
    if (x > 0.0)
        return 1.0;
    if (x < 0.0)
        return -1.0;
    return 0.0;
}

/**
 * @brief Round a floating point value to a fixed number of decimal places.
 *
 * Direct extraction of the helper previously local to LegStepper (_ls_setPrecision),
 * kept inline for zero‑overhead usage in tight kinematic loops.
 *
 * @param value     Input floating point value.
 * @param precision Number of decimal digits to keep (>=0).
 * @return Rounded value with the requested precision.
 */
inline double setPrecision(double value, int precision) {
    if (precision <= 0) {
        return std::round(value); // fast path for integer rounding
    }
    double scale = std::pow(10.0, precision);
    return std::round(value * scale) / scale;
}

/**
 * @brief Set precision of a 3D Eigen vector.
 * @param v Input vector
 * @param precision Number of decimal places
 * @return Vector with truncated precision
 */
inline Eigen::Vector3d setPrecisionVec(const Eigen::Vector3d &v, int precision) {
    return Eigen::Vector3d(setPrecision(v[0], precision),
                           setPrecision(v[1], precision),
                           setPrecision(v[2], precision));
}
/** Multiply two quaternions. */
Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
/** Invert a quaternion. */
Eigen::Vector4d quaternionInverse(const Eigen::Vector4d &q);

/** Convert Point3D to Eigen::Vector3d for quaternion operations. */
Eigen::Vector3d point3DToVector3d(const Point3D &point);
/** Convert Eigen::Vector3d to Point3D for pose operations. */
Point3D vector3fToPoint3D(const Eigen::Vector3d &vec);
/** Convert Point3D Euler angles (radians) to quaternion. */
Eigen::Vector4d eulerPoint3DToQuaternion(const Point3D &euler);
/** Convert quaternion to Point3D Euler angles (radians). */
Point3D quaternionToEulerPoint3D(const Eigen::Vector4d &quaternion);
/** Spherical linear interpolation (SLERP) between two quaternions. */
Eigen::Vector4d quaternionSlerp(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, double t);

/** Generic Denavit-Hartenberg transform (angles in radians). */
template <typename T>
Eigen::Matrix<T, 4, 4> dhTransform(T a, T alpha, T d, T theta);

/**
 * @brief Generic DH transform rotating about the Y axis.
 *
 * This variant is used for joints that pitch instead of yaw so the
 * kinematic chain matches the analytic leg model.
 * @tparam T Floating point type.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> dhTransformY(T a, T alpha, T d, T theta);

/** Float-specialized DH transform for backwards compatibility. */
Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta);

/** Float-specialized DH transform with Y-axis rotation. */
Eigen::Matrix4d dhTransformY(double a, double alpha, double d, double theta);

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
    return (points[1] - points[0]) * (3.0 * s * s) +
           (points[2] - points[1]) * (6.0 * s * t) +
           (points[3] - points[2]) * (3.0 * t * t);
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
    return (points[1] - points[0]) * (4.0 * s * s * s) +
           (points[2] - points[1]) * (12.0 * s * s * t) +
           (points[3] - points[2]) * (12.0 * s * t * t) +
           (points[4] - points[3]) * (4.0 * t * t * t);
}

/**
 * @brief Compute the Bézier control point that makes a quadratic curve pass through points[1] at parameter t.
 *
 * Given three points where points[0] is the start, points[2] is the end, and points[1] is
 * the desired pass-through point at parameter t, this function returns the actual quadratic
 * Bézier control point that achieves exact interpolation through points[1].
 *
 * OpenSHC equivalent: quadraticBezierCurveThroughControlPoint
 *
 * @tparam T Vector type supporting basic arithmetic (e.g. Eigen::Vector3d, Point3D).
 * @param points Array of 3 points: [start, pass-through, end].
 * @param t Curve parameter (0.0 to 1.0) at which the curve should pass through points[1].
 * @return The control point to use in the standard quadratic Bézier formula.
 */
template <class T>
inline T quadraticBezierCurveThroughControlPoint(const T *points, double t) {
    double s = 1.0 - t;
    double denom = 2.0 * t * s;
    if (std::fabs(denom) < 1e-12) {
        return points[1]; // Degenerate: t≈0 or t≈1, control point is the target itself
    }
    return (points[1] - points[0] * (s * s) - points[2] * (t * t)) / denom;
}

/**
 * @brief Compute the Bézier control point that makes a cubic curve pass through a given point.
 *
 * Given four points defining the cubic Bézier, this solves for the actual control point at
 * index control_point_index (1 or 2) such that the curve passes exactly through
 * points[control_point_index] at parameter t.
 *
 * OpenSHC equivalent: cubicBezierCurveThroughControlPoint
 *
 * @tparam T Vector type supporting basic arithmetic.
 * @param points Array of 4 points.
 * @param t Curve parameter (0.0 to 1.0).
 * @param control_point_index Which control point to solve for (1 or 2).
 * @return The adjusted control point for the standard cubic Bézier formula.
 */
template <class T>
inline T cubicBezierCurveThroughControlPoint(const T *points, double t, unsigned int control_point_index) {
    double s = 1.0 - t;
    if (control_point_index == 1) {
        double denom = 3.0 * t * s * s;
        if (std::fabs(denom) < 1e-12) {
            return points[1];
        }
        return (points[1] - points[0] * (s * s * s) - points[2] * (3.0 * t * t * s) -
                points[3] * (t * t * t)) /
               denom;
    } else if (control_point_index == 2) {
        double denom = 3.0 * t * t * s;
        if (std::fabs(denom) < 1e-12) {
            return points[2];
        }
        return (points[2] - points[0] * (s * s * s) - points[1] * (3.0 * t * s * s) -
                points[3] * (t * t * t)) /
               denom;
    }
    // Invalid index: return zero vector
    return T();
}

/**
 * @brief Compute the Bézier control point that makes a quartic curve pass through a given point.
 *
 * Given five points defining the quartic Bézier, this solves for the actual control point at
 * index control_point_index (1, 2, or 3) such that the curve passes exactly (or approximately,
 * for complex target curves) through points[control_point_index] at parameter t.
 *
 * OpenSHC equivalent: quarticBezierCurveThroughControlPoint
 *
 * @tparam T Vector type supporting basic arithmetic.
 * @param points Array of 5 points.
 * @param t Curve parameter (0.0 to 1.0).
 * @param control_point_index Which control point to solve for (1, 2, or 3).
 * @return The adjusted control point for the standard quartic Bézier formula.
 */
template <class T>
inline T quarticBezierCurveThroughControlPoint(const T *points, double t, unsigned int control_point_index) {
    double s = 1.0 - t;
    if (control_point_index == 1) {
        double denom = 4.0 * t * s * s * s;
        if (std::fabs(denom) < 1e-12) {
            return points[1];
        }
        return (points[1] - points[0] * (s * s * s * s) - points[2] * (6.0 * t * t * s * s) -
                points[3] * (4.0 * t * t * t * s) - points[4] * (t * t * t * t)) /
               denom;
    } else if (control_point_index == 2) {
        double denom = 6.0 * t * t * s * s;
        if (std::fabs(denom) < 1e-12) {
            return points[2];
        }
        return (points[2] - points[0] * (s * s * s * s) - points[1] * (4.0 * t * s * s * s) -
                points[3] * (4.0 * t * t * t * s) - points[4] * (t * t * t * t)) /
               denom;
    } else if (control_point_index == 3) {
        double denom = 4.0 * t * t * t * s;
        if (std::fabs(denom) < 1e-12) {
            return points[3];
        }
        return (points[3] - points[0] * (s * s * s * s) - points[1] * (4.0 * t * s * s * s) -
                points[2] * (6.0 * t * t * s * s) - points[4] * (t * t * t * t)) /
               denom;
    }
    // Invalid index: return zero vector
    return T();
}

/**
 * @brief Smooth step function for interpolation (OpenSHC equivalent)
 * Provides smooth interpolation with zero derivatives at endpoints
 * @param control_input The linear control input from 0.0 to 1.0
 * @return The output of the control input run through a smoothStep function
 */
inline double smoothStep(double control_input) {
    return (6.0 * pow(control_input, 5) - 15.0 * pow(control_input, 4) + 10.0 * pow(control_input, 3));
}

/**
 * @brief Linear interpolation between two values
 * @param origin The origin value
 * @param target The target value
 * @param control_input The interpolation factor (0.0 to 1.0)
 * @return The interpolated value
 */
template <class T>
inline T interpolate(const T &origin, const T &target, double control_input) {
    return (1.0 - control_input) * origin + control_input * target;
}

/**
 * @brief Clamp a value between min and max
 * @param value The value to clamp
 * @param min_value Minimum value
 * @param max_value Maximum value
 * @return Clamped value
 */
template <class T>
inline T clamped(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(max_value, value));
}

/**
 * @brief State vector for numerical integration
 */
template <typename T>
struct StateVector {
    T position; //< Position component
    T velocity; //< Velocity component

    StateVector() = default;
    StateVector(const T &pos, const T &vel) : position(pos), velocity(vel) {}

    StateVector operator+(const StateVector &other) const {
        return StateVector(position + other.position, velocity + other.velocity);
    }

    StateVector operator*(double scalar) const {
        return StateVector(position * scalar, velocity * scalar);
    }
};

/**
 * @brief Function type for differential equation derivatives
 * @tparam T Vector type (Point3D, Eigen::Vector3d, etc.)
 * @param state Current state vector [position, velocity]
 * @param t Current time
 * @return Derivative of the state vector
 */
template <typename T>
using DerivativeFunction = StateVector<T> (*)(const StateVector<T> &state, double t, void *params);

/**
 * @brief Solve least squares for plane equation z = ax + by + c
 * @param raw_A Array of x,y coordinates [x1,y1,x2,y2,...]
 * @param raw_B Array of z coordinates [z1,z2,...]
 * @param num_points Number of points (length of raw_B array)
 * @param a Output coefficient for x
 * @param b Output coefficient for y
 * @param c Output constant term
 * @return true if solution found, false if matrix is singular
 */
bool solveLeastSquaresPlane(const double *raw_A, const double *raw_B, int num_points, double &a, double &b, double &c);

/**
 * @brief Runge-Kutta 4th order integration for differential equations
 * @tparam T Vector type supporting arithmetic operations
 * @param derivative_func Function computing derivatives
 * @param initial_state Initial state [position, velocity]
 * @param t0 Initial time
 * @param dt Time step
 * @param params User-defined parameters for derivative function
 * @return New state after time step
 */
template <typename T>
StateVector<T> rungeKutta4(DerivativeFunction<T> derivative_func,
                           const StateVector<T> &initial_state,
                           double t0, double dt, void *params) {
    // k1 = f(t, y)
    StateVector<T> k1 = derivative_func(initial_state, t0, params) * dt;

    // k2 = f(t + dt/2, y + k1/2)
    StateVector<T> state_k2 = initial_state + k1 * 0.5;
    StateVector<T> k2 = derivative_func(state_k2, t0 + dt / 2.0, params) * dt;

    // k3 = f(t + dt/2, y + k2/2)
    StateVector<T> state_k3 = initial_state + k2 * 0.5;
    StateVector<T> k3 = derivative_func(state_k3, t0 + dt / 2.0, params) * dt;

    // k4 = f(t + dt, y + k3)
    StateVector<T> state_k4 = initial_state + k3;
    StateVector<T> k4 = derivative_func(state_k4, t0 + dt, params) * dt;

    // y_{n+1} = y_n + (dt/6)(k1 + 2k2 + 2k3 + k4)
    StateVector<T> result = initial_state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (1.0 / 6.0);

    return result;
}

/**
 * @brief Runge-Kutta 2nd order (midpoint method) integration
 * @tparam T Vector type supporting arithmetic operations
 * @param derivative_func Function computing derivatives
 * @param initial_state Initial state [position, velocity]
 * @param t0 Initial time
 * @param dt Time step
 * @param params User-defined parameters
 * @return New state after time step
 */
template <typename T>
StateVector<T> rungeKutta2(DerivativeFunction<T> derivative_func,
                           const StateVector<T> &initial_state,
                           double t0, double dt, void *params) {
    // k1 = f(t, y)
    StateVector<T> k1 = derivative_func(initial_state, t0, params) * dt;

    // k2 = f(t + dt/2, y + k1/2)
    StateVector<T> state_k2 = initial_state + k1 * 0.5;
    StateVector<T> k2 = derivative_func(state_k2, t0 + dt / 2.0, params) * dt;

    // y_{n+1} = y_n + k2
    return initial_state + k2;
}

/**
 * @brief Forward Euler integration (first-order)
 * @tparam T Vector type supporting arithmetic operations
 * @param derivative_func Function computing derivatives
 * @param initial_state Initial state [position, velocity]
 * @param t0 Initial time
 * @param dt Time step
 * @param params User-defined parameters
 * @return New state after time step
 */
template <typename T>
StateVector<T> forwardEuler(DerivativeFunction<T> derivative_func,
                            const StateVector<T> &initial_state,
                            double t0, double dt, void *params) {
    StateVector<T> derivatives = derivative_func(initial_state, t0, params);
    return initial_state + derivatives * dt;
}

/**
 * @brief Compatibility clamp function for pre-C++17 compilers
 * @param value The value to clamp
 * @param min_val The minimum allowed value
 * @param max_val The maximum allowed value
 * @return The clamped value
 */
template <typename T>
inline constexpr T clamp(const T &value, const T &min_val, const T &max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val
                                                           : value;
}

} // namespace math_utils

#endif // MATH_UTILS_H
