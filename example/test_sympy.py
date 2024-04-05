import sympy as sp

x, y = sp.symbols("x, y")
H = sp.Matrix([sp.sqrt(x**2 + y**2), sp.atan(y / x)])
print(H.jacobian([x, y]))
