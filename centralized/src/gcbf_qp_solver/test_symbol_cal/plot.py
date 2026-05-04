import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

# Symbols
a10, a11 = sp.symbols('a10 a11')
x = sp.symbols('x')
r, R, A = sp.symbols('r R A', positive=True)

def h1(x):
    return a10*x**2 + a11*x - A

def h1_dot(x):
    return 2*a10*x + a11

# Constraints
eq1 = sp.Eq(h1(r), 0)
eq2 = sp.Eq(h1_dot(r), 2 / (R - r))

# Solve for a0, a1, a2
sol = sp.solve((eq1, eq2), (a10, a11))
a10_expr = sp.simplify(sol[a10])
a11_expr = sp.simplify(sol[a11])

print(f'a10: {a10_expr}')
print(f'a11: {a11_expr}')

# Choose numerical values (change as you like)
r_val = 0.1
R_val = 3.0
A_val = 5.0

# Convert to numeric functions
a10_func = sp.lambdify((r, R, A), a10_expr, 'numpy')
a11_func = sp.lambdify((r, R, A), a11_expr, 'numpy')

a10_num = a10_func(r_val, R_val, A_val)
a11_num = a11_func(r_val, R_val, A_val)

print(f'a10_num = {a10_num}')
print(f'a11_num = {a11_num}')

# Polynomial for plotting
def h1_num(x):
    return a10_num*x**2 + a11_num*x - A_val

def h1_num_dot(x):
    return 2*a10_num*x + a11_num

def h2_num(x):
    return - (R_val - x)**2 / (R_val - r_val)**2 + 1

def h2_num_dot(x):
    return 2 * (R_val - x) / (R_val - r_val)**2

# 绘图区间
x1_vals = np.linspace(0, r_val, 100)
y1_vals = h1_num(x1_vals)
y1_prime_vals = h1_num_dot(x1_vals)

x2_vals = np.linspace(r_val, R_val, 200)
y2_vals = h2_num(x2_vals)
y2_prime_vals = h2_num_dot(x2_vals)

# 图1：原多项式 p(x)
plt.figure(figsize=(8, 5))
plt.plot(x1_vals, y1_vals, 'b-', linewidth=2)
plt.plot(x2_vals, y2_vals, 'b-', linewidth=2)
plt.xlabel('x')
plt.ylabel('p(x)')
plt.title(f'Polynomial p(x)\n(r={r_val}, R={R_val}, A={A_val})')
plt.grid(True, linestyle=':')
plt.show()   # 先显示第一张图

# 图2：导数 p'(x)
plt.figure(figsize=(8, 5))
plt.plot(x1_vals, y1_prime_vals, 'r-', linewidth=2)
plt.plot(x2_vals, y2_prime_vals, 'r-', linewidth=2)
# 标注 p'(R)=0 点
plt.plot(R_val, h2_num_dot(R_val), 'ko', markersize=8)
# plt.annotate(f"p'({R_val}) = 0", xy=(R_val, 0), xytext=(R_val+0.2, 0.2),
#              arrowprops=dict(arrowstyle='->', color='gray'))
plt.xlabel('x')
plt.ylabel("p'(x)")
plt.title(f"Derivative of p(x)\n(r={r_val}, R={R_val}, A={A_val})")
plt.grid(True, linestyle=':')
plt.show()