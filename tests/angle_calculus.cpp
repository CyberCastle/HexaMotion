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
//   Estructuras auxiliares
//------------------------------------------------------------------
struct Angles {
    double theta1; // °  coxa-fémur
    double theta2; // °  fémur-tibia
    bool valid;    // solución dentro de límites
};

//------------------------------------------------------------------
//   Cinemática inversa analítica con tibia vertical (altura → ángulos)
//   Se busca el par de ángulos que produzca la altura deseada manteniendo
//   la tibia perpendicular al suelo (θ₁ + θ₂ = 0).
//------------------------------------------------------------------
Angles calcLegAngles(double H_mm) {
    Angles best{0, 0, false};
    double best_err = std::numeric_limits<double>::max();

    // Recorrer todo el rango permitido del fémur.
    for (double femur = -75.0; femur <= 75.0; femur += 0.5) {
        double tibia = -femur; // Mantener la tibia vertical

        if (tibia < -45.0 || tibia > 45.0)
            continue; // Violación de límites del servo

        double theta = femur * DEG2RAD;
        double height = C_TIBIA + B_FEMUR * std::sin(theta);
        double err = std::fabs(height - H_mm);

        if (err < best_err) {
            best = {femur, tibia, err < 1.0}; // 1 mm de tolerancia
            best_err = err;
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

    if (theta1_deg < -75.0 || theta1_deg > 75.0)
        return 0.0;
    if (theta2_deg < -45.0 || theta2_deg > 45.0)
        return 0.0;

    if (std::fabs(theta1_deg + theta2_deg) > 1e-6)
        return 0.0; // Tibia not vertical

    double theta = theta1_deg * DEG2RAD;

    valid = true;
    return C_TIBIA + B_FEMUR * std::sin(theta);
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
