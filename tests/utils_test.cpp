#include <cassert>
#include "../include/hexapod_locomotion_system.h"

int main() {
    using namespace HexapodUtils;
    // degreesToRadians and radiansToDegrees round trip
    float deg = 90.0f;
    float rad = degreesToRadians(deg);
    assert(static_cast<int>(radiansToDegrees(rad)) == 90);

    // rotation matrices orthogonality test
    auto R = rotationMatrixZ(45.0f);
    Eigen::Matrix3f I = R * R.transpose();
    assert((I - Eigen::Matrix3f::Identity()).norm() < 1e-5);
    return 0;
}
