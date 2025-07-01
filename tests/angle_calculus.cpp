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
    Angles out{0.0, 0.0, false};

    const double B = B_FEMUR;
    const double C = C_TIBIA;

    constexpr int MAX_ITERS = 50;
    constexpr double STEP_MM = 1.0;

    double H = H_mm;

    for (int i = 0; i < MAX_ITERS; i++) {
        if (H < C || H > B + C)
            break;

        double s = (H - C) / B;
        if (s < -1.0 || s > 1.0)
            break;
        double alpha = std::asin(s);
        double beta = M_PI_2 - alpha;

        double femurDeg = alpha * RAD2DEG;
        double tibiaDeg = beta * RAD2DEG;

        bool fem_ok = femurDeg >= -75.0 && femurDeg <= 75.0;
        bool tib_ok = tibiaDeg >= -45.0 && tibiaDeg <= 45.0;

        if (fem_ok && tib_ok) {
            out.theta1 = femurDeg;
            out.theta2 = tibiaDeg;
            out.valid = true;
            break;
        }

        if (femurDeg > 75.0 || tibiaDeg < -45.0)
            H -= STEP_MM;
        else if (femurDeg < -75.0 || tibiaDeg > 45.0)
            H += STEP_MM;
        else
            break;
    }

    return out;
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
    double alpha = theta1 + beta; // α = θ₁ + β

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
