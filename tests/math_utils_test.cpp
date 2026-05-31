/**
 * @file math_utils_test.cpp
 * @brief Coverage for math_utils helpers (TODO_TEST_COVERAGE [E]).
 *
 * Exercises the previously-uncovered pure utility functions: angle/normalisation,
 * rounding, sign, smoothStep, 2D/line distances, cross product, rotations, DH-Y
 * transform, quaternion <-> Euler conversions, SLERP, reachability and the
 * Point3D <-> Eigen conversion helpers.
 */

#include "../src/math_utils.h"
#include "../src/pose.h"
#include <cassert>
#include <cmath>
#include <iostream>

static bool close(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }
static bool closeP(const Point3D &a, const Point3D &b, double eps = 1e-4) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) && close(a.z, b.z, eps);
}

int main() {
    using namespace math_utils;

    // --- Degrees/radians round-trip ---
    double rad = degreesToRadians(90.0);
    assert(static_cast<int>(radiansToDegrees(rad)) == 90);

    auto R = rotationMatrixZ(degreesToRadians(45.0));
    assert((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-5);

    // --- normalizeAngle: wraps into [-180, 180] ---
    assert(close(normalizeAngle(190.0), -170.0, 1e-6));
    assert(close(normalizeAngle(-190.0), 170.0, 1e-6));
    assert(close(normalizeAngle(45.0), 45.0, 1e-6));

    // --- mod: positive result for negative input ---
    assert(mod(-1, 4) == 3);
    assert(mod(5, 4) == 1);

    // --- sign ---
    assert(close(sign(-3.0), -1.0));
    assert(close(sign(2.0), 1.0));
    assert(close(sign(0.0), 0.0));

    // --- rounding helpers ---
    assert(roundToInt(2.4) == 2);
    assert(roundToInt(2.6) == 3);
    assert(roundToInt(-2.6) == -3);
    assert(roundToEvenInt(3.0) == 4); // odd rounds up to even
    assert(roundToEvenInt(4.0) == 4);

    // --- smoothStep: 5th-order smootherstep, endpoints 0/1, midpoint 0.5, monotonic ---
    assert(close(smoothStep(0.0), 0.0));
    assert(close(smoothStep(1.0), 1.0));
    assert(close(smoothStep(0.5), 0.5));
    assert(smoothStep(0.25) < smoothStep(0.75)); // monotonically increasing in range

    // --- setPrecision ---
    assert(close(setPrecision(1.23456, 2), 1.23, 1e-9));
    assert(close(setPrecision(1.6, 0), 2.0, 1e-9));

    // --- distance helpers ---
    Point3D a(0, 0, 0), b(3, 4, 0), c(3, 4, 12);
    assert(close(distance2D(a, b), 5.0, 1e-6));
    assert(close(distance3D(a, c), 13.0, 1e-6));

    // --- pointToLineDistance: point (0,5,0) to X-axis segment => 5 ---
    Point3D p(0, 5, 0), ls(-10, 0, 0), le(10, 0, 0);
    assert(close(pointToLineDistance(p, ls, le), 5.0, 1e-6));

    // --- crossProduct: X x Y = Z ---
    Point3D cx = crossProduct(Point3D(1, 0, 0), Point3D(0, 1, 0));
    assert(closeP(cx, Point3D(0, 0, 1)));

    // --- rotatePoint: rotate (1,0,0) by 90deg yaw -> (0,1,0) ---
    Point3D rotated = rotatePoint(Point3D(1, 0, 0), Eigen::Vector3d(0, 0, degreesToRadians(90.0)));
    assert(closeP(rotated, Point3D(0, 1, 0)));

    // --- rotationMatrixX/Y are orthonormal ---
    Eigen::Matrix3d rx = rotationMatrixX(degreesToRadians(30.0));
    Eigen::Matrix3d ry = rotationMatrixY(degreesToRadians(30.0));
    assert((rx * rx.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-6);
    assert((ry * ry.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-6);

    // --- dhTransformY produces a valid homogeneous transform ---
    Eigen::Matrix4d dh = dhTransformY(50.0, 0.0, 0.0, 0.0);
    assert(close(dh(3, 3), 1.0));

    // --- isPointReachable ---
    assert(isPointReachable(Point3D(10, 0, 0), 20.0));
    assert(!isPointReachable(Point3D(30, 0, 0), 20.0));
    assert(isPointReachable(Point3D(15, 0, 0), 10.0, 20.0));
    assert(!isPointReachable(Point3D(5, 0, 0), 10.0, 20.0));

    // --- Point3D <-> Eigen conversions ---
    Eigen::Vector3d v = point3DToVector3d(Point3D(1, 2, 3));
    assert(close(v[0], 1) && close(v[1], 2) && close(v[2], 3));
    Point3D back = vector3fToPoint3D(Eigen::Vector3d(4, 5, 6));
    assert(closeP(back, Point3D(4, 5, 6)));

    // --- Euler <-> quaternion (Point3D form) round-trip for a small rotation ---
    Point3D euler(degreesToRadians(10.0), degreesToRadians(0.0), degreesToRadians(20.0));
    Eigen::Vector4d q = eulerPoint3DToQuaternion(euler);
    Point3D euler_back = quaternionToEulerPoint3D(q);
    assert(closeP(euler_back, euler, 1e-3));

    // --- quaternionSlerp: t=0 returns q1, t=1 returns q2 ---
    Eigen::Vector4d q1(1, 0, 0, 0);
    Eigen::Vector4d q2 = eulerPoint3DToQuaternion(Point3D(0, 0, degreesToRadians(90.0)));
    Eigen::Vector4d s0 = quaternionSlerp(q1, q2, 0.0);
    Eigen::Vector4d s1 = quaternionSlerp(q1, q2, 1.0);
    assert((s0 - q1).norm() < 1e-6);
    assert((s1 - q2).norm() < 1e-6 || (s1 + q2).norm() < 1e-6); // sign ambiguity allowed

    std::cout << "math_utils_test executed successfully" << std::endl;
    return 0;
}
