import sympy as sp

# 定义符号变量
p1, p2, p3 = sp.symbols('p1 p2 p3')          # 位置
v1, v2, v3 = sp.symbols('v1 v2 v3')          # 速度
q0, q1, q2, q3 = sp.symbols('q0 q1 q2 q3')   # 四元数 (实部在前)
w1, w2, w3 = sp.symbols('w1 w2 w3')          # 角速度
c = sp.symbols('c')                          # 推力 (标量)
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')  # 力矩
g = sp.symbols('g')                          # 重力加速度

# 惯性矩阵 J (对称，取一般形式)
J11, J12, J13, J22, J23, J33 = sp.symbols('J11 J12 J13 J22 J23 J33')
J = sp.Matrix([[J11, J12, J13],
               [J12, J22, J23],
               [J13, J23, J33]])

# 世界坐标系 z 轴
z_W = sp.Matrix([0, 0, 1])

# 由四元数计算的机体 z 轴 (旋转矩阵第三列)
z_B = sp.Matrix([
    2*(q1*q3 + q0*q2),
    2*(q2*q3 - q0*q1),
    1 - 2*(q1**2 + q2**2)
])

# 四元数乘法：q ⊗ ω (ω 作为纯四元数)
def quat_mult(q, w):
    # q = [q0, q1, q2, q3], w = [0, w1, w2, w3]
    q0, q1, q2, q3 = q
    w0, w1, w2, w3 = w
    return sp.Matrix([
        q0*w0 - q1*w1 - q2*w2 - q3*w3,
        q0*w1 + q1*w0 + q2*w3 - q3*w2,
        q0*w2 - q1*w3 + q2*w0 + q3*w1,
        q0*w3 + q1*w2 - q2*w1 + q3*w0
    ])

# 角速度作为纯四元数
w_pure = sp.Matrix([0, w1, w2, w3])

# 状态导数
dot_p = sp.Matrix([v1, v2, v3])
dot_v = -g * z_W + c * z_B
dot_q = sp.Rational(1,2) * quat_mult(sp.Matrix([q0, q1, q2, q3]), w_pure)
dot_w = J.inv() * (sp.Matrix([tau1, tau2, tau3]) - sp.Matrix.cross(sp.Matrix([w1, w2, w3]), J * sp.Matrix([w1, w2, w3])))

# 构造 f
f = sp.Matrix.vstack(dot_p, dot_v, dot_q, dot_w)

# 状态向量 x 和控制向量 u
x = sp.Matrix([p1, p2, p3, v1, v2, v3, q0, q1, q2, q3, w1, w2, w3])
u = sp.Matrix([c, tau1, tau2, tau3])

# 计算雅可比矩阵
A = f.jacobian(x)   # 13x13 矩阵
B = f.jacobian(u)   # 13x4 矩阵

# print("线性化结果: A:")
# sp.pprint(A)
# print("线性化结果: B:")
# sp.pprint(B)
# print()

# 平衡点: v=0, ω=0, q=[1,0,0,0], c=g, τ=0
eq_point = {
    v1: 0, v2: 0, v3: 0,
    w1: 0, w2: 0, w3: 0,
    q0: 1, q1: 0, q2: 0, q3: 0,
    c: g,
    tau1: 0, tau2: 0, tau3: 0
}

# 代入平衡点并简化
A_eq = sp.simplify(A.subs(eq_point))
B_eq = sp.simplify(B.subs(eq_point))

# 输出结果
# print("线性化后的 A 矩阵 (在平衡点):")
# sp.pprint(A_eq)
# print("\n线性化后的 B 矩阵 (在平衡点):")
# sp.pprint(B_eq)

# 将 A_eq 和 B_eq 转为 LaTeX 并保存到文件
with open("linearized_matrices.tex", "w") as f:
    f.write("\\documentclass{article}\n")
    f.write("\\usepackage{amsmath}\n")
    f.write("\\begin{document}\n\n")
    
    f.write("\\section*{Linearized Dynamics at Hover}\n\n")

    f.write("\\subsection*{State Matrix $A$}\n")
    f.write("\\[\n")
    f.write(sp.latex(A, mat_delim="[", mat_str="matrix"))
    f.write("\n\\]\n\n")
    
    f.write("\\subsection*{Input Matrix $B$}\n")
    f.write("\\[\n")
    f.write(sp.latex(B, mat_delim="[", mat_str="matrix"))
    f.write("\n\\]\n\n")
    
    f.write("\\subsection*{State Matrix $A_{eq}$}\n")
    f.write("\\[\n")
    f.write(sp.latex(A_eq, mat_delim="[", mat_str="matrix"))
    f.write("\n\\]\n\n")
    
    f.write("\\subsection*{Input Matrix $B_{eq}$}\n")
    f.write("\\[\n")
    f.write(sp.latex(B_eq, mat_delim="[", mat_str="matrix"))
    f.write("\n\\]\n\n")
    
    f.write("\\end{document}\n")


'''
# 假设已得到 A_eq, B_eq 符号矩阵（见前文）
# 为简化表达式，先进行公共子表达式消除
exprs = [A_eq, B_eq]
sub_expr, reduced = sp.cse(exprs)
reduced_A, reduced_B = reduced[0], reduced[1]

# 打开文件写入 C++ 函数
with open("linearized_dynamics.cpp", "w") as f:
    f.write("#include <array>\n\n")
    f.write("// 线性化动力学模型 (悬停点)\n")
    f.write("// 输入: g (重力), J (惯性矩阵元素), 输出: A[13][13], B[13][4]\n")
    f.write("void linearized_dynamics(\n")
    f.write("    double g, double J11, double J12, double J13,\n")
    f.write("    double J22, double J23, double J33,\n")
    f.write("    double A[13][13], double B[13][4]) {\n\n")

    # 写入公共子表达式
    for name, expr in sub_expr:
        f.write(f"    double {sp.cxxcode(name)} = {sp.cxxcode(expr)};\n")
    f.write("\n")

    # 写入 A 矩阵元素
    for i in range(13):
        for j in range(13):
            val = reduced_A[i, j]
            if val != 0:
                f.write(f"    A[{i}][{j}] = {sp.cxxcode(val)};\n")
    f.write("\n")

    # 写入 B 矩阵元素
    for i in range(13):
        for j in range(4):
            val = reduced_B[i, j]
            if val != 0:
                f.write(f"    B[{i}][{j}] = {sp.cxxcode(val)};\n")
    f.write("}\n")
'''
