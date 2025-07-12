#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <ArduinoEigen.h>

struct Point3D; // forward declaration

namespace math_utils {
/** Convert degrees to radians. */
double degreesToRadians(double degrees);
/** Convert radians to degrees. */
double radiansToDegrees(double radians);
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
/** Project a vector onto another vector. */
Point3D projectVector(const Point3D &vector, const Point3D &onto);

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
inline T interpolate(const T& origin, const T& target, double control_input) {
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
 * @brief Round to nearest integer
 * @param x The value to round
 * @return Rounded integer
 */
inline int roundToInt(double x) {
    return (x >= 0) ? static_cast<int>(x + 0.5) : -static_cast<int>(0.5 - x);
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
bool solveLeastSquaresPlane(const double* raw_A, const double* raw_B, int num_points, double& a, double& b, double& c);

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
