import sympy as sp

# ========================= 符号变量定义 =========================
p1, p2, p3 = sp.symbols('p1 p2 p3')
v1, v2, v3 = sp.symbols('v1 v2 v3')
phi, theta, psi = sp.symbols('phi theta psi')
w1, w2, w3 = sp.symbols('w1 w2 w3')
c = sp.symbols('c')
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')
g = sp.symbols('g')
m = sp.symbols('m')

J11, J12, J13, J22, J23, J33 = sp.symbols('J11 J12 J13 J22 J23 J33')
J = sp.Matrix([[J11, J12, J13],
               [J12, J22, J23],
               [J13, J23, J33]])

# ========================= 辅助函数 =========================
def rot_x(angle):
    return sp.Matrix([[1, 0, 0],
                      [0, sp.cos(angle), -sp.sin(angle)],
                      [0, sp.sin(angle),  sp.cos(angle)]])

def rot_y(angle):
    return sp.Matrix([[ sp.cos(angle), 0, sp.sin(angle)],
                      [0, 1, 0],
                      [-sp.sin(angle), 0, sp.cos(angle)]])

def rot_z(angle):
    return sp.Matrix([[sp.cos(angle), -sp.sin(angle), 0],
                      [sp.sin(angle),  sp.cos(angle), 0],
                      [0, 0, 1]])

# ========================= 动力学方程 =========================
z_W = sp.Matrix([0, 0, 1])
R = rot_z(psi) * rot_y(theta) * rot_x(phi)
z_B = R[:, 2]

W = sp.Matrix([[1, 0, sp.sin(theta)],
               [0, sp.cos(phi), -sp.cos(theta) * sp.sin(phi)],
               [0, sp.sin(phi),  sp.cos(theta) * sp.cos(phi)]])
W_inv = W.inv()
omega = sp.Matrix([w1, w2, w3])
dot_Theta = W_inv * omega

tau = sp.Matrix([tau1, tau2, tau3])
dot_omega = J.inv() * (tau - sp.Matrix.cross(omega, J * omega))

dot_p = sp.Matrix([v1, v2, v3])
dot_v = (-g * z_W + c * z_B) / m

# 状态向量共12维: [p1,p2,p3, v1,v2,v3, phi,theta,psi, w1,w2,w3]
f = sp.Matrix.vstack(dot_p, dot_v, dot_Theta, dot_omega)

# ========================= 雅可比矩阵 =========================
x = sp.Matrix([p1, p2, p3, v1, v2, v3, phi, theta, psi, w1, w2, w3])
u = sp.Matrix([c, tau1, tau2, tau3])

A = f.jacobian(x)   # 12x12
B = f.jacobian(u)   # 12x4

# ========================= 悬停平衡点 =========================
eq_point = {
    v1: 0, v2: 0, v3: 0,
    w1: 0, w2: 0, w3: 0,
    phi: 0, theta: 0, psi: 0,
    c: g,
    tau1: 0, tau2: 0, tau3: 0
}

A_eq = sp.simplify(A.subs(eq_point))
B_eq = sp.simplify(B.subs(eq_point))

# ========================= 输出 LaTeX 文件 =========================
with open("linearized_matrices.tex", "w") as f:
    f.write("\\documentclass{article}\n")
    f.write("\\usepackage{amsmath}\n")
    f.write("\\begin{document}\n\n")
    f.write("\\section*{Linearized Dynamics of a Multirotor (Euler Angles)}\n\n")
    f.write("\\subsection*{General State Matrix $A$ (before linearization)}\n")
    f.write("\\[\n" + sp.latex(A, mat_delim="[", mat_str="matrix") + "\n\\]\n\n")
    f.write("\\subsection*{General Input Matrix $B$ (before linearization)}\n")
    f.write("\\[\n" + sp.latex(B, mat_delim="[", mat_str="matrix") + "\n\\]\n\n")
    f.write("\\subsection*{Linearized State Matrix at Hover: $A_{\\text{eq}}$}\n")
    f.write("\\[\n" + sp.latex(A_eq, mat_delim="[", mat_str="matrix") + "\n\\]\n\n")
    f.write("\\subsection*{Linearized Input Matrix at Hover: $B_{\\text{eq}}$}\n")
    f.write("\\[\n" + sp.latex(B_eq, mat_delim="[", mat_str="matrix") + "\n\\]\n\n")
    f.write("\\end{document}\n")

print("LaTeX 文件已生成: linearized_matrices.tex")

# ========================= 生成 C++ 函数文件 =========================
# 公共子表达式消除
exprs = [A_eq, B_eq]
sub_expr, reduced = sp.cse(exprs)
reduced_A, reduced_B = reduced[0], reduced[1]

with open("linearized_dynamics_v2.cpp", "w") as f:
    f.write("#include <array>\n\n")
    f.write("// 线性化动力学模型 (悬停点，基于欧拉角)\n")
    f.write("// 状态维度: 12, 控制维度: 4\n")
    f.write("// 输入: g (重力), J (惯性矩阵元素)\n")
    f.write("// 输出: A[12][12] (状态矩阵), B[12][4] (输入矩阵)\n")
    f.write("void linearized_dynamics(\n")
    f.write("    double mass, double g, double J11, double J12, double J13,\n")
    f.write("    double J22, double J23, double J33,\n")
    f.write("    double A[12][12], double B[12][4]) {\n\n")

    # 写入公共子表达式
    for name, expr in sub_expr:
        f.write(f"    double {sp.cxxcode(name)} = {sp.cxxcode(expr)};\n")
    f.write("\n")

    # 写入 A 矩阵非零元素 (12x12)
    for i in range(12):
        for j in range(12):
            val = reduced_A[i, j]
            if val != 0:
                f.write(f"    A[{i}][{j}] = {sp.cxxcode(val)};\n")
    f.write("\n")

    # 写入 B 矩阵非零元素 (12x4)
    for i in range(12):
        for j in range(4):
            val = reduced_B[i, j]
            if val != 0:
                f.write(f"    B[{i}][{j}] = {sp.cxxcode(val)};\n")
    f.write("}\n")

print("C++ 函数文件已生成: linearized_dynamics_v2.cpp")