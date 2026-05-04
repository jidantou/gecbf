import sympy as sp

# 定义符号变量
p1, p2, p3 = sp.symbols('p1 p2 p3')          # 位置
v1, v2, v3 = sp.symbols('v1 v2 v3')          # 速度
q0, q1, q2, q3 = sp.symbols('q0 q1 q2 q3')   # 四元数 (实部在前)
w1, w2, w3 = sp.symbols('w1 w2 w3')          # 角速度
c = sp.symbols('c')                          # 推力 (标量)
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')  # 力矩
g = sp.symbols('g')                          # 重力加速度
m = sp.symbols('mass')

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
dot_v = -g * z_W + c / m * z_B
dot_q = sp.Rational(1,2) * quat_mult(sp.Matrix([q0, q1, q2, q3]), w_pure)
dot_w = J.inv() * (sp.Matrix([tau1, tau2, tau3]) - sp.Matrix.cross(sp.Matrix([w1, w2, w3]), J * sp.Matrix([w1, w2, w3])))

# 构造 f
f = sp.Matrix.vstack(dot_p, dot_v, dot_q, dot_w)

# 状态向量 x 和控制向量 u
x = sp.Matrix([p1, p2, p3, v1, v2, v3, q0, q1, q2, q3, w1, w2, w3])
u = sp.Matrix([c, tau1, tau2, tau3])


# 分离 f(x) 和 g(x)
# 不含控制输入的部分
dot_v_no_u = -g * z_W
# dot_w_no_u = -J.inv() * sp.Matrix.cross(sp.Matrix([w1, w2, w3]), J * sp.Matrix([w1, w2, w3]))


# 预计算惯性矩阵的逆（符号形式）
J_inv = J.inv()
# 提取逆矩阵的元素
J_inv_11, J_inv_12, J_inv_13 = J_inv[0,0], J_inv[0,1], J_inv[0,2]
J_inv_21, J_inv_22, J_inv_23 = J_inv[1,0], J_inv[1,1], J_inv[1,2]
J_inv_31, J_inv_32, J_inv_33 = J_inv[2,0], J_inv[2,1], J_inv[2,2]

# 重写 dot_w_no_u，显式使用 J_inv 元素，避免在生成的代码中求逆
w_cross = sp.Matrix.cross(sp.Matrix([w1, w2, w3]), J * sp.Matrix([w1, w2, w3]))
dot_w_no_u = -J_inv * w_cross

# 重新构造 f_vec 和 g_mat（使用显式的 J_inv 元素）
f_vec = sp.Matrix.vstack(dot_p, dot_v_no_u, dot_q, dot_w_no_u)

# 构造 g_mat
g_mat = sp.zeros(13, 4)
# v 部分
for i in range(3):
    g_mat[3 + i, 0] = z_B[i]
# w 部分，使用 J_inv 元素
for i in range(3):
    for j in range(3):
        g_mat[10 + i, 1 + j] = J_inv[i, j]

# 打印 f 和 g 到屏幕
print("f(x) =")
sp.pprint(f_vec)
print("\ng(x) =")
sp.pprint(g_mat)

# 生成 C++ 代码（使用 Eigen 库）
cpp_code = """
#include <Eigen/Dense>

// 状态向量顺序: [p1, p2, p3, v1, v2, v3, q0, q1, q2, q3, w1, w2, w3]
// 控制向量顺序: [c, tau1, tau2, tau3]

void compute_f(const Eigen::Matrix<double, 13, 1>& x, Eigen::Matrix<double, 13, 1>& f) {
    // 提取状态变量
    double p1 = x(0), p2 = x(1), p3 = x(2);
    double v1 = x(3), v2 = x(4), v3 = x(5);
    double q0 = x(6), q1 = x(7), q2 = x(8), q3 = x(9);
    double w1 = x(10), w2 = x(11), w3 = x(12);
    
    // 常量
    const double g = """ + str(g) + """;
    // 惯性矩阵元素（常量，需根据实际系统设置）
    const double J11 = """ + str(J11) + """, J12 = """ + str(J12) + """, J13 = """ + str(J13) + """;
    const double J22 = """ + str(J22) + """, J23 = """ + str(J23) + """, J33 = """ + str(J33) + """;
    // 惯性矩阵逆的元素（预计算）
    const double J_inv11 = """ + sp.cxxcode(J_inv_11) + """;
    const double J_inv12 = """ + sp.cxxcode(J_inv_12) + """;
    const double J_inv13 = """ + sp.cxxcode(J_inv_13) + """;
    const double J_inv21 = """ + sp.cxxcode(J_inv_21) + """;
    const double J_inv22 = """ + sp.cxxcode(J_inv_22) + """;
    const double J_inv23 = """ + sp.cxxcode(J_inv_23) + """;
    const double J_inv31 = """ + sp.cxxcode(J_inv_31) + """;
    const double J_inv32 = """ + sp.cxxcode(J_inv_32) + """;
    const double J_inv33 = """ + sp.cxxcode(J_inv_33) + """;
    
    // 计算 f(x) 各分量
"""

# 添加 f 的每个分量
for i, expr in enumerate(f_vec):
    # 使用 cxxcode 生成表达式，赋值给 f(i)
    c_expr = sp.cxxcode(expr, assign_to=f"f({i})")
    cpp_code += "    " + c_expr + "\n"

cpp_code += """
}

void compute_g(const Eigen::Matrix<double, 13, 1>& x, Eigen::Matrix<double, 13, 4>& g) {
    // 提取状态变量
    double p1 = x(0), p2 = x(1), p3 = x(2);
    double v1 = x(3), v2 = x(4), v3 = x(5);
    double q0 = x(6), q1 = x(7), q2 = x(8), q3 = x(9);
    double w1 = x(10), w2 = x(11), w3 = x(12);
    
    // 常量
    const double g_const = """ + str(g) + """;
    // 惯性矩阵元素
    const double J11 = """ + str(J11) + """, J12 = """ + str(J12) + """, J13 = """ + str(J13) + """;
    const double J22 = """ + str(J22) + """, J23 = """ + str(J23) + """, J33 = """ + str(J33) + """;
    // 惯性矩阵逆的元素（与 f 中相同）
    const double J_inv11 = """ + sp.cxxcode(J_inv_11) + """;
    const double J_inv12 = """ + sp.cxxcode(J_inv_12) + """;
    const double J_inv13 = """ + sp.cxxcode(J_inv_13) + """;
    const double J_inv21 = """ + sp.cxxcode(J_inv_21) + """;
    const double J_inv22 = """ + sp.cxxcode(J_inv_22) + """;
    const double J_inv23 = """ + sp.cxxcode(J_inv_23) + """;
    const double J_inv31 = """ + sp.cxxcode(J_inv_31) + """;
    const double J_inv32 = """ + sp.cxxcode(J_inv_32) + """;
    const double J_inv33 = """ + sp.cxxcode(J_inv_33) + """;
    
    // 计算 g(x) 矩阵，按行优先存储（13行4列）
"""

# 添加 g 的每个元素
for i in range(13):
    for j in range(4):
        expr = g_mat[i, j]
        if expr != 0:
            c_expr = sp.cxxcode(expr, assign_to=f"g({i},{j})")
        else:
            c_expr = f"g({i},{j}) = 0.0;"
        cpp_code += "    " + c_expr + "\n"

cpp_code += "}\n"

# 写入文件
with open("quadrotor_dynamics_eigen.cpp", "w") as f:
    f.write(cpp_code)

print("\nC++ 代码已写入 quadrotor_dynamics_eigen.cpp（使用 Eigen 库）")