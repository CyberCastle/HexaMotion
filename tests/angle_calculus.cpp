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
//   Cinemática inversa  (altura  →  ángulos)
//------------------------------------------------------------------
Angles calcLegAngles(double H_mm) {
    const double alphaMin = -75.0 * DEG2RAD;
    const double alphaMax = 75.0 * DEG2RAD;
    const double betaMin = -45.0 * DEG2RAD;
    const double betaMax = 45.0 * DEG2RAD;

    const double Ytarget = H_mm - C_TIBIA;

    Angles best{0, 0, false};
    double bestScore = 1e9;

    for (double beta = betaMin; beta <= betaMax; beta += 0.5 * DEG2RAD) {
        double yRem = Ytarget - B_FEMUR * std::sin(beta);
        double s = yRem / A_COXA; // argumento de asin

        if (s < -1.0 || s > 1.0)
            continue;

        double alpha = std::asin(s);

        if (alpha < alphaMin || alpha > alphaMax)
            continue;

        double theta1 = (beta - alpha) * RAD2DEG; // coxa-fémur
        if (theta1 < -75.0 || theta1 > 75.0)
            continue;

        double theta2 = -beta * RAD2DEG; // fémur-tibia

        double score = std::fabs(alpha) + std::fabs(beta);
        if (score < bestScore) {
            best = {theta1, theta2, true};
            bestScore = score;
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

    // Relaciones geométricas empleadas en la inversa:
    //   θ₂ = −β     ⇒  β = −θ₂
    //   θ₁ = β − α  ⇒  α = β − θ₁
    double beta = -theta2;
    double alpha = beta - theta1;

    // Chequeo opcional de los límites absolutos de α y β
    if (alpha < -75.0 * DEG2RAD || alpha > 75.0 * DEG2RAD)
        return 0.0;
    if (beta < -45.0 * DEG2RAD || beta > 45.0 * DEG2RAD)
        return 0.0;

    // Altura alcanzada (positivo hacia abajo)
    double H_mm = A_COXA * std::sin(alpha) + B_FEMUR * std::sin(beta) + C_TIBIA;

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
