import sympy as sp

# 定义符号变量
a0, a1, a2 = sp.symbols('a0 a1 a2')
r, R, A = sp.symbols('r R A')

# 方程 1: a0*r^4 + a1*r^3 + a2*r^2 = A

eq1 = sp.Eq(a0*r**4 + a1*r**3 + a2*r**2, A)


# 方程 2: a0*R^4 + a1*R^3 + a2*R^2 = A + 1

eq2 = sp.Eq(a0*R**4 + a1*R**3 + a2*R**2, A + 1)


# 方程 3: 4*a0*R^3 + 3*a1*R^2 + 2*a2*R = 0

eq3 = sp.Eq(4*a0*R**3 + 3*a1*R**2 + 2*a2*R, 0)


# 求解方程组

solution = sp.solve((eq1, eq2, eq3), (a0, a1, a2))

# Simplify the expressions
a0_expr = sp.simplify(solution[a0])
a1_expr = sp.simplify(solution[a1])
a2_expr = sp.simplify(solution[a2])

# Generate C++ code for the three expressions
cpp_code = f"""// Auto-generated C++ code from SymPy
// Computes the coefficients a0, a1, a2 for given r, R, A

#include <cmath>

void compute_coefficients(double r, double R, double A,
                          double& a0, double& a1, double& a2) {{
    a0 = {sp.cxxcode(a0_expr)};
    a1 = {sp.cxxcode(a1_expr)};
    a2 = {sp.cxxcode(a2_expr)};
}}
"""

# Write to a .cpp file
with open("coefficients.cpp", "w") as f:
    f.write(cpp_code)

print("C++ code written to coefficients.cpp")