#include <cassert>
#include <iostream>
#include "../src/math_utils.h"

int main() {
    using namespace math_utils;
    float deg = 90.0f;
    float rad = degreesToRadians(deg);
    assert(static_cast<int>(radiansToDegrees(rad)) == 90);

    auto R = rotationMatrixZ(45.0f);
    Eigen::Matrix3f I = R * R.transpose();
    assert((I - Eigen::Matrix3f::Identity()).norm() < 1e-5);
    std::cout << "math_utils_test executed successfully" << std::endl;
    return 0;
}
