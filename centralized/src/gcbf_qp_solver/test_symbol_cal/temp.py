import sympy as sp

a, s = sp.symbols('a s')
x = sp.symbols('x')
r, R, A = sp.symbols('r R A')

def h(x):
    return -a/3*x**3 + a/2*(R+s)*x**2 - a*R*s*x - A

equ1 = sp.Eq(h(r), 0)
equ2 = sp.Eq(h(R), 1)

sol = sp.solve((equ1, equ2), (a, s))
a_expr = sp.simplify(sol[0][0])
s_expr = sp.simplify(sol[0][1])

print(f'a: {a_expr}')
print(f's: {s_expr}')