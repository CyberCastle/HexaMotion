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
//   Cinemática inversa analítica  (altura  →  ángulos)
//------------------------------------------------------------------
Angles calcLegAngles(double H_mm) {
    Angles result{0, 0, false};

    // Mantener la tibia vertical implica que el ángulo relativo rodilla–fémur
    // sea opuesto al ángulo absoluto del fémur. Así, la rodilla compensa la
    // rotación del fémur y la tibia queda perpendicular al suelo.
    //
    // 1) Calcular el ángulo del fémur a partir de la altura deseada.
    //    La geometría resulta en:
    //       H = C_TIBIA + B_FEMUR * sin(theta_femur)
    //    de donde theta_femur = asin((H - C_TIBIA) / B_FEMUR).
    double ratio = (H_mm - C_TIBIA) / B_FEMUR;
    if (ratio < -1.0 || ratio > 1.0)
        return result; // la altura no es alcanzable

    double theta = std::asin(ratio);
    double femur_deg = theta * RAD2DEG;
    double tibia_deg = -femur_deg; // compensación para mantener la tibia vertical

    // 2) Verificar que ambos ángulos respeten los límites mecánicos.
    if (femur_deg < -75.0 || femur_deg > 75.0)
        return result;
    if (tibia_deg < -45.0 || tibia_deg > 45.0)
        return result;

    result.theta1 = femur_deg;
    result.theta2 = tibia_deg;
    result.valid = true;
    return result;
}

//------------------------------------------------------------------
//   Cinemática directa  (ángulos  →  altura)
//   Devuelve la altura alcanzada H_mm y marca "valid" si los ángulos
//   respetan todos los límites mecánicos.
//------------------------------------------------------------------
double calcHeight(double theta1_deg, double theta2_deg, bool &valid) {
    valid = false;

    // Comprobar que las órdenes de servo estén dentro de sus márgenes.
    if (theta1_deg < -75.0 || theta1_deg > 75.0)
        return 0.0;
    if (theta2_deg < -45.0 || theta2_deg > 45.0)
        return 0.0;

    // La tibia debe permanecer vertical. En nuestra convención eso implica que
    // los ángulos de servo sean opuestos: θ1 + θ2 ≈ 0.
    if (std::fabs(theta1_deg + theta2_deg) > 1e-6)
        return 0.0;

    // Con la tibia vertical, la altura depende únicamente del ángulo absoluto
    // del fémur.
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
