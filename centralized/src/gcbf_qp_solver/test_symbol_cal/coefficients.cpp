// Auto-generated C++ code from SymPy
// Computes the coefficients a0, a1, a2 for given r, R, A

#include <cmath>

void compute_coefficients(double r, double R, double A,
                          double& a0, double& a1, double& a2) {
    a0 = (A*std::pow(R, 3) - 3*A*R*std::pow(r, 2) + 2*A*std::pow(r, 3) - 3*R*std::pow(r, 2) + 2*std::pow(r, 3))/(std::pow(R, 3)*std::pow(r, 2)*(std::pow(R, 2) - 2*R*r + std::pow(r, 2)));
    a1 = 2*(-A*std::pow(R, 4) + 2*A*std::pow(R, 2)*std::pow(r, 2) - A*std::pow(r, 4) + 2*std::pow(R, 2)*std::pow(r, 2) - std::pow(r, 4))/(std::pow(R, 3)*std::pow(r, 2)*(std::pow(R, 2) - 2*R*r + std::pow(r, 2)));
    a2 = (A*std::pow(R, 4) - 4*A*R*std::pow(r, 3) + 3*A*std::pow(r, 4) - 4*R*std::pow(r, 3) + 3*std::pow(r, 4))/(std::pow(R, 2)*std::pow(r, 2)*(std::pow(R, 2) - 2*R*r + std::pow(r, 2)));
}
