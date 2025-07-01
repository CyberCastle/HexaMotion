#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

constexpr double A_COXA = 50.0;   // mm (unused but kept for reference)
constexpr double B_FEMUR = 101.0; // mm
constexpr double C_TIBIA = 208.0; // mm

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

struct Angles {
    double theta1; // femur servo (deg)
    double theta2; // tibia servo (deg)
    bool valid;
};

Angles calcLegAngles(double H_mm) {
    Angles result{0, 0, false};
    double ratio = (H_mm - C_TIBIA) / B_FEMUR;
    if (ratio < -1.0 || ratio > 1.0)
        return result;
    double theta = std::asin(ratio);
    double femur_deg = theta * RAD2DEG;
    double tibia_deg = -femur_deg;
    if (femur_deg < -75.0 || femur_deg > 75.0)
        return result;
    if (tibia_deg < -45.0 || tibia_deg > 45.0)
        return result;
    result.theta1 = femur_deg;
    result.theta2 = tibia_deg;
    result.valid = true;
    return result;
}

double calcHeight(double t1, double t2, bool &valid) {
    valid = false;
    if (t1 < -75.0 || t1 > 75.0)
        return 0.0;
    if (t2 < -45.0 || t2 > 45.0)
        return 0.0;
    if (std::fabs(t1 + t2) > 1e-6)
        return 0.0; // tibia not vertical
    valid = true;
    return C_TIBIA + B_FEMUR * std::sin(t1 * DEG2RAD);
}

int main() {
    // Alturas dentro del rango alcanzable respetando los límites de los servos
    std::vector<double> heights{140, 160, 180, 200, 220, 240};
    int passed = 0;

    std::cout << "Height | theta1  theta2  | calcH   | status" << std::endl;
    std::cout << std::string(46, '-') << std::endl;

    for (double h : heights) {
        Angles sol = calcLegAngles(h);
        if (!sol.valid) {
            std::cout << std::setw(6) << h << " | INVALID" << std::endl;
            continue;
        }
        bool ok;
        double H = calcHeight(sol.theta1, sol.theta2, ok);
        bool match = ok && std::fabs(H - h) < 0.01 && std::fabs(sol.theta1 + sol.theta2) < 1e-6;
        if (match) passed++;
        std::cout << std::setw(6) << h << " | "
                  << std::setw(6) << sol.theta1 << " "
                  << std::setw(6) << sol.theta2 << " | "
                  << std::setw(7) << H << " | "
                  << (match ? "PASS" : "FAIL") << std::endl;
    }

    std::cout << std::string(46, '-') << std::endl;
    std::cout << "Passed " << passed << "/" << heights.size() << std::endl;
    return passed == heights.size() ? 0 : 1;
}
