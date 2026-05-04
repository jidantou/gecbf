import sympy as sp

# 定义符号变量
x, R, gamma = sp.symbols('x R gamma', real=True)
a, b, c, d = sp.symbols('a b c d', real=True)

# 三次函数及其导数
g = a * x**3 + b * x**2 + c * x + d
g_prime = sp.diff(g, x)

# 四个条件
conditions = [
    sp.Eq(g.subs(x, R), 0),           # g(R) = 0
    sp.Eq(g.subs(x, R - gamma), 1),   # g(R-γ) = 1
    sp.Eq(g_prime.subs(x, R), 0),     # g'(R) = 0
    sp.Eq(g_prime.subs(x, R - gamma), 0)  # g'(R-γ) = 0
]

# 求解系数
solution = sp.solve(conditions, (a, b, c, d))

# 输出结果
print("解得的系数 (用 R 和 γ 表示):")
for coeff, expr in solution.items():
    sp.pprint(expr, use_unicode=True)
    print()

# 为了验证，构造具体的函数并检查条件（可选）
print("\n验证：")
g_solved = g.subs(solution)
print("g(x) =", sp.simplify(g_solved))
print("g(R) =", sp.simplify(g_solved.subs(x, R)))
print("g(R-γ) =", sp.simplify(g_solved.subs(x, R - gamma)))
print("g'(R) =", sp.simplify(sp.diff(g_solved, x).subs(x, R)))
print("g'(R-γ) =", sp.simplify(sp.diff(g_solved, x).subs(x, R - gamma)))