#include "math_utils.h"
#include <cmath>
#include <iostream>

int main() {
    std::cout << "=== Test quaternion functions in math_utils ===" << std::endl;

    // Test eulerToQuaternion and quaternionToEuler
    Eigen::Vector3f euler_input(30.0f, 45.0f, 60.0f); // degrees
    Eigen::Vector4f quat = math_utils::eulerToQuaternion(euler_input);
    Eigen::Vector3f euler_output = math_utils::quaternionToEuler(quat);

    std::cout << "Input Euler (deg): " << euler_input.transpose() << std::endl;
    std::cout << "Quaternion: " << quat.transpose() << std::endl;
    std::cout << "Output Euler (deg): " << euler_output.transpose() << std::endl;

    // Check if conversion is consistent
    float tolerance = 0.01f;
    bool conversion_ok = (euler_input - euler_output).norm() < tolerance;
    std::cout << "Euler<->Quaternion conversion: " << (conversion_ok ? "PASS" : "FAIL") << std::endl;

    // Test quaternionMultiply
    Eigen::Vector4f q1(0.7071f, 0.7071f, 0.0f, 0.0f); // 90° around X
    Eigen::Vector4f q2(0.7071f, 0.0f, 0.7071f, 0.0f); // 90° around Y
    Eigen::Vector4f q_mult = math_utils::quaternionMultiply(q1, q2);
    std::cout << "Q1 * Q2: " << q_mult.transpose() << std::endl;

    // Test quaternionInverse
    Eigen::Vector4f q_inv = math_utils::quaternionInverse(quat);
    Eigen::Vector4f identity = math_utils::quaternionMultiply(quat, q_inv);
    std::cout << "Q * Q_inv (should be ~[1,0,0,0]): " << identity.transpose() << std::endl;

    bool inverse_ok = std::abs(identity[0] - 1.0f) < tolerance &&
                      std::abs(identity[1]) < tolerance &&
                      std::abs(identity[2]) < tolerance &&
                      std::abs(identity[3]) < tolerance;
    std::cout << "Quaternion inverse: " << (inverse_ok ? "PASS" : "FAIL") << std::endl;

    std::cout << "\n=== Test completed ===" << std::endl;
    return 0;
}
