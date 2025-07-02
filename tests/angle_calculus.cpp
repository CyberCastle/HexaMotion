#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

constexpr double A_COXA = 50.0;   // mm
constexpr double B_FEMUR = 101.0; // mm
constexpr double C_TIBIA = 208.0; // mm

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

//------------------------------------------------------------------
//   Helper structures
//------------------------------------------------------------------
struct Angles {
    double theta1; // °  coxa-femur
    double theta2; // °  femur-tibia
    bool valid;    // solution within limits
};

//------------------------------------------------------------------
//   Analytic inverse kinematics  (height → angles)
//------------------------------------------------------------------
Angles calcLegAngles(double H_mm) {
    Angles best{0.0, 0.0, false};
    double bestErr = std::numeric_limits<double>::infinity();

    // Ranges in radians
    const double alphaMin = -75.0 * DEG2RAD;
    const double alphaMax = 75.0 * DEG2RAD;
    const double betaMin = -45.0 * DEG2RAD;
    const double betaMax = 45.0 * DEG2RAD;
    const double dBeta = 0.1 * DEG2RAD; // sweep step

    const double A = A_COXA;
    const double B = B_FEMUR;
    const double C = C_TIBIA;

    // Sweep beta to find a solution with a vertical tibia
    for (double beta = betaMin; beta <= betaMax; beta += dBeta) {
        double sum = A + B * std::cos(beta);
        double discriminant = sum * sum - (H_mm * H_mm - C * C);
        if (discriminant < 0.0)
            continue;

        double sqrtD = std::sqrt(discriminant);
        // Two possible roots for alpha
        for (int sign : {-1, 1}) {
            double t = (-sum + sign * sqrtD) / (H_mm + C);
            double alpha = 2.0 * std::atan(t);
            if (alpha < alphaMin || alpha > alphaMax)
                continue;

            double err = std::fabs(alpha + beta);
            if (err >= bestErr)
                continue;

            double theta1 = (alpha - beta) * RAD2DEG; // θ₁ = α - β
            double theta2 = -beta * RAD2DEG;          // θ₂ = −β

            if (theta1 < -75.0 || theta1 > 75.0)
                continue;
            if (theta2 < -45.0 || theta2 > 45.0)
                continue;

            bestErr = err;
            best = {theta1, theta2, true};
        }
    }

    return best;
}

//------------------------------------------------------------------
//   Forward kinematics (angles → height)
//   Returns the achieved height H_mm and marks "valid" if the angles
//   respect all mechanical limits.
//------------------------------------------------------------------
double calcHeight(double theta1_deg,
                  double theta2_deg,
                  bool &valid) {
    valid = false;

    // Check relative angle limits
    if (theta1_deg < -75.0 || theta1_deg > 75.0)
        return 0.0;
    if (theta2_deg < -45.0 || theta2_deg > 45.0)
        return 0.0;

    // Convert to radians
    double theta1 = theta1_deg * DEG2RAD;
    double theta2 = theta2_deg * DEG2RAD;

    // DH model geometric relations
    double beta = -theta2;        // β = −θ₂
    double alpha = theta1 + beta; // α = θ₁ + β

    if (alpha < -75.0 * DEG2RAD || alpha > 75.0 * DEG2RAD)
        return 0.0;
    if (beta < -45.0 * DEG2RAD || beta > 45.0 * DEG2RAD)
        return 0.0;

    // Height computed using the DH chain
    double H_mm = C_TIBIA * std::cos(alpha) -
                  A_COXA * std::sin(alpha) -
                  B_FEMUR * std::sin(alpha) * std::cos(beta);

    valid = true;
    return H_mm;
}

//------------------------------------------------------------------
//   Demonstration program
//------------------------------------------------------------------
int main() {
    std::cout << "1) Compute angles from height\n"
                 "2) Compute height from angles\n"
                 "Option: ";
    int opt;
    std::cin >> opt;

    if (opt == 1) {
        double H;
        std::cout << "Desired height (mm): ";
        std::cin >> H;
        Angles sol = calcLegAngles(H);

        if (sol.valid)
            std::cout << std::fixed << std::setprecision(2)
                      << "θ₁ (coxa-femur)  = " << sol.theta1 << "°\n"
                      << "θ₂ (femur-tibia) = " << sol.theta2 << "°\n";
        else
            std::cout << "Requested height is out of reach.\n";
    } else if (opt == 2) {
        double t1, t2;
        std::cout << "θ₁ (coxa-femur)  in degrees: ";
        std::cin >> t1;
        std::cout << "θ₂ (femur-tibia) in degrees: ";
        std::cin >> t2;

        bool ok;
        double H = calcHeight(t1, t2, ok);

        if (ok)
            std::cout << std::fixed << std::setprecision(2)
                      << "Resulting height = " << H << " mm\n";
        else
            std::cout << "Input angles violate mechanical limits.\n";
    } else
        std::cout << "Invalid option.\n";
    return 0;
}
