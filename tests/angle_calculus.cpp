#include <cmath>
#include <iomanip>
#include <iostream>

constexpr double A_COXA = 50.0;   // mm
constexpr double B_FEMUR = 101.0; // mm
constexpr double C_TIBIA = 208.0; // mm

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

//------------------------------------------------------------------
//   Estructuras auxiliares
//------------------------------------------------------------------
struct Angles {
    double theta1; // °  coxa-fémur
    double theta2; // °  fémur-tibia
    bool valid;    // solución dentro de límites
};

//------------------------------------------------------------------
//   Cinemática inversa analítica  (altura  →  ángulos)
//------------------------------------------------------------------
Angles calcLegAngles(double H_mm) {
    const double alphaMin = -75.0 * DEG2RAD;
    const double alphaMax = 75.0 * DEG2RAD;
    const double betaMin = -45.0 * DEG2RAD;
    const double betaMax = 45.0 * DEG2RAD;

    Angles best{0, 0, false};
    double bestScore = 1e9;

    for (double beta = betaMin; beta <= betaMax; beta += 0.5 * DEG2RAD) {
        double sum = A_COXA + B_FEMUR * std::cos(beta);
        double discriminant =
            sum * sum - (H_mm * H_mm - C_TIBIA * C_TIBIA);
        if (discriminant < 0.0)
            continue;

        double sqrt_disc = std::sqrt(discriminant);
        for (int sign = -1; sign <= 1; sign += 2) {
            double t = (-sum + sign * sqrt_disc) / (H_mm + C_TIBIA);
            double alpha = 2.0 * std::atan(t);

            if (alpha < alphaMin || alpha > alphaMax)
                continue;

            double score = std::fabs(alpha) + std::fabs(beta);
            if (score < bestScore) {
                best.theta1 = (beta - alpha) * RAD2DEG; // coxa-fémur
                best.theta2 = -beta * RAD2DEG;          // fémur-tibia
                best.valid = (best.theta1 >= -75.0 && best.theta1 <= 75.0 &&
                             best.theta2 >= -45.0 && best.theta2 <= 45.0);
                if (best.valid)
                    bestScore = score;
            }
        }
    }
    return best;
}

//------------------------------------------------------------------
//   Cinemática directa  (ángulos  →  altura)
//   Devuelve la altura alcanzada H_mm y marca "valid" si los ángulos
//   respetan todos los límites mecánicos.
//------------------------------------------------------------------
double calcHeight(double theta1_deg,
                  double theta2_deg,
                  bool &valid) {
    valid = false;

    // Comprobar límites de los ángulos relativos
    if (theta1_deg < -75.0 || theta1_deg > 75.0)
        return 0.0;
    if (theta2_deg < -45.0 || theta2_deg > 45.0)
        return 0.0;

    // Convertir a radianes
    double theta1 = theta1_deg * DEG2RAD;
    double theta2 = theta2_deg * DEG2RAD;

    // Relaciones geométricas del modelo DH
    double beta = -theta2;        // β = −θ₂
    double alpha = beta - theta1; // α = β − θ₁

    if (alpha < -75.0 * DEG2RAD || alpha > 75.0 * DEG2RAD)
        return 0.0;
    if (beta < -45.0 * DEG2RAD || beta > 45.0 * DEG2RAD)
        return 0.0;

    // Altura alcanzada según la cadena DH
    double H_mm = C_TIBIA * std::cos(alpha) -
                  A_COXA * std::sin(alpha) -
                  B_FEMUR * std::sin(alpha) * std::cos(beta);

    valid = true;
    return H_mm;
}

//------------------------------------------------------------------
//   Programa de demostración
//------------------------------------------------------------------
int main() {
    std::cout << "1) Calcular angulos a partir de la altura\n"
                 "2) Calcular altura a partir de los angulos\n"
                 "Opcion: ";
    int opt;
    std::cin >> opt;

    if (opt == 1) {
        double H;
        std::cout << "Altura deseada (mm): ";
        std::cin >> H;
        Angles sol = calcLegAngles(H);

        if (sol.valid)
            std::cout << std::fixed << std::setprecision(2)
                      << "θ₁ (coxa-fémur)  = " << sol.theta1 << "°\n"
                      << "θ₂ (fémur-tibia) = " << sol.theta2 << "°\n";
        else
            std::cout << "La altura solicitada esta fuera del rango alcanzable.\n";
    } else if (opt == 2) {
        double t1, t2;
        std::cout << "θ₁ (coxa-fémur)  en grados: ";
        std::cin >> t1;
        std::cout << "θ₂ (fémur-tibia) en grados: ";
        std::cin >> t2;

        bool ok;
        double H = calcHeight(t1, t2, ok);

        if (ok)
            std::cout << std::fixed << std::setprecision(2)
                      << "Altura resultante = " << H << " mm\n";
        else
            std::cout << "Los angulos introducidos violan los limites mecánicos.\n";
    } else
        std::cout << "Opcion no valida.\n";
    return 0;
}
