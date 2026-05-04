import sympy as sp

# 定义符号变量
phi, theta, psi = sp.symbols('phi theta psi')

# ============================================
# 1. 旋转矩阵 R (惯性系 → 机体系)
# ============================================
R = sp.Matrix([
    [ sp.cos(theta)*sp.cos(psi),  sp.cos(theta)*sp.sin(psi), -sp.sin(theta) ],
    [ sp.sin(phi)*sp.sin(theta)*sp.cos(psi) - sp.cos(phi)*sp.sin(psi),
      sp.sin(phi)*sp.sin(theta)*sp.sin(psi) + sp.cos(phi)*sp.cos(psi),
      sp.sin(phi)*sp.cos(theta) ],
    [ sp.cos(phi)*sp.sin(theta)*sp.cos(psi) + sp.sin(phi)*sp.sin(psi),
      sp.cos(phi)*sp.sin(theta)*sp.sin(psi) - sp.sin(phi)*sp.cos(psi),
      sp.cos(phi)*sp.cos(theta) ]
])

# ============================================
# 2. 四元数 q = [q0, q1, q2, q3] 从欧拉角计算
# ============================================
q0 = sp.cos(phi/2)*sp.cos(theta/2)*sp.cos(psi/2) - sp.sin(phi/2)*sp.sin(theta/2)*sp.sin(psi/2)
q1 = sp.cos(phi/2)*sp.sin(theta/2)*sp.sin(psi/2) + sp.sin(phi/2)*sp.cos(theta/2)*sp.cos(psi/2)
q2 = sp.cos(phi/2)*sp.sin(theta/2)*sp.cos(psi/2) + sp.sin(phi/2)*sp.cos(theta/2)*sp.sin(psi/2)
q3 = sp.cos(phi/2)*sp.cos(theta/2)*sp.sin(psi/2) - sp.sin(phi/2)*sp.sin(theta/2)*sp.cos(psi/2)

q = sp.Matrix([q0, q1, q2, q3])  # 四元数向量形式

# ============================================
# 3. 从四元数求欧拉角 (假设已知 q0,q1,q2,q3 为符号)
# ============================================
# 若需要从四元数反解，可以定义新的符号变量
q0_sym, q1_sym, q2_sym, q3_sym = sp.symbols('q0 q1 q2 q3')

phi_from_q = sp.atan2(2*(q2_sym*q3_sym + q0_sym*q1_sym), 1 - 2*(q1_sym**2 + q2_sym**2))
theta_from_q = -sp.asin(2*(q1_sym*q3_sym - q0_sym*q2_sym))
psi_from_q = sp.atan2(2*(q1_sym*q2_sym + q0_sym*q3_sym), 1 - 2*(q2_sym**2 + q3_sym**2))

def quat_to_rot_Matrix_manual(q, normalize=False):
    """
    根据四元数旋转公式手动计算旋转矩阵
    q = w + x*i + y*j + z*k
    公式：
        R = [[1-2(y^2+z^2),   2(xy - zw),     2(xz + yw)],
             [2(xy + zw),     1-2(x^2+z^2),   2(yz - xw)],
             [2(xz - yw),     2(yz + xw),     1-2(x^2+y^2)]]
    
    参数:
        q : 四元数，Quaternion 对象 或 [w, x, y, z] 序列
        normalize : bool, 是否归一化（默认 False）
    
    返回:
        sympy.sp.Matrix : 3x3 旋转矩阵
    """
    # 提取分量
    if isinstance(q, sp.Quaternion):
        w, x, y, z = q.a, q.b, q.c, q.d
    else:
        if len(q) != 4:
            raise ValueError("输入四元数必须包含4个分量: [w, x, y, z]")
        w, x, y, z = q[0], q[1], q[2], q[3]
    
    # 可选归一化（确保单位四元数）
    if normalize:
        norm2 = w**2 + x**2 + y**2 + z**2
        w = w / sp.sqrt(norm2)
        x = x / sp.sqrt(norm2)
        y = y / sp.sqrt(norm2)
        z = z / sp.sqrt(norm2)
    
    # 计算矩阵元素
    R = sp.Matrix([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])
    # print("debug: R: ", R)
    return R

R_from_q = quat_to_rot_Matrix_manual(q)
# 可选：显示表达式
print("R =", R)
print("R from q: ", R_from_q)
print("is equal: ", R.equals(R_from_q))
# print("q0 =", q0)
# print("phi =", phi_from_q)