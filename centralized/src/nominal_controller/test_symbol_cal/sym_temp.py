import sympy as sp

# ========================= 符号变量定义 =========================
p1, p2, p3 = sp.symbols('p1 p2 p3')
v1, v2, v3 = sp.symbols('v1 v2 v3')
phi, theta, psi = sp.symbols('phi theta psi')
w1, w2, w3 = sp.symbols('w1 w2 w3')
c = sp.symbols('c')
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')
g = sp.symbols('g')

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

print(R)

# ========================= 输出 LaTeX 文件 =========================
with open("temp.tex", "w") as f:
    f.write("\\documentclass{article}\n")
    f.write("\\usepackage{amsmath}\n")
    f.write("\\begin{document}\n\n")
    f.write("\\[\n" + sp.latex(R, mat_delim="[", mat_str="matrix") + "\n\\]\n\n")
    f.write("\\end{document}\n")

print("LaTeX 文件已生成: temp.tex")