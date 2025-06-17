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
/** Check if a point is within a spherical reach (simple max reach check). */
bool isPointReachable(const Point3D &point, float max_reach);
/** Check if a point is within spherical reach bounds (min and max reach). */
bool isPointReachable(const Point3D &point, float min_reach, float max_reach);
/** Magnitude of a 3D vector. */
float magnitude(const Point3D &point);
/** Distance between two 3D points (alias for distance3D). */
float distance(const Point3D &p1, const Point3D &p2);
/** Calculate perpendicular distance from point to line segment. */
float pointToLineDistance(const Point3D &point, const Point3D &line_start, const Point3D &line_end);

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

/** Convert Point3D to Eigen::Vector3f for quaternion operations. */
Eigen::Vector3f point3DToVector3f(const Point3D &point);
/** Convert Eigen::Vector3f to Point3D for pose operations. */
Point3D vector3fToPoint3D(const Eigen::Vector3f &vec);
/** Convert Point3D Euler angles to quaternion using centralized functions. */
Eigen::Vector4f eulerPoint3DToQuaternion(const Point3D &euler_degrees);
/** Convert quaternion to Point3D Euler angles using centralized functions. */
Point3D quaternionToEulerPoint3D(const Eigen::Vector4f &quaternion);

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

/**
 * @brief State vector for numerical integration
 */
template <typename T>
struct StateVector {
    T position; ///< Position component
    T velocity; ///< Velocity component

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
 * @tparam T Vector type (Point3D, Eigen::Vector3f, etc.)
 * @param state Current state vector [position, velocity]
 * @param t Current time
 * @param params User-defined parameters
 * @return Derivative vector [velocity, acceleration]
 */
template <typename T>
using DerivativeFunction = StateVector<T> (*)(const StateVector<T> &state, double t, void *params);

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

} // namespace math_utils

#endif // MATH_UTILS_H
