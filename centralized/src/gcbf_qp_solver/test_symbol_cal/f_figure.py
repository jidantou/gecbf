import numpy as np
import matplotlib.pyplot as plt

def f(x, n, R=1.0):
    """函数 f(x) = x + x^n (R-x) / R^n"""
    return x + (x**n * (R - x)) / (R**n)

# 参数设置
R = 5.0                     # 定义域右端点
x = np.linspace(0, R, 500)  # 绘图点

# 不同的 n 值（n 越大，函数越贴近 y=x）
n_values = [1, 2, 5, 10, 50, 100]

# 绘图
plt.figure(figsize=(8, 6))
for n in n_values:
    y = f(x, n, R)
    plt.plot(x, y, label=f'n = {n}')

# 参考直线 y = x
plt.plot(x, x, 'k--', linewidth=2, label='y = x')

# 图形修饰
plt.xlabel('x')
plt.ylabel('f(x)')
plt.title(f'$f(x) = x + \\frac{{x^{{n}} (R-x)}}{{R^{{n}}}}$,  $R={R}$')
plt.legend()
plt.grid(True)
# 保存图片（新增的一行）
plt.savefig('f_plot.png', dpi=300, bbox_inches='tight')
plt.show()