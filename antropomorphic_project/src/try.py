#!/usr/bin/env python3

from sympy import symbols, solve, cos, sin

# Symbolic variables for the angles
t1, t2, t3 = symbols('t1 t2 t3')

# Given values for px, py, pz
px_val =  0.5
py_val =  0.6
pz_val =  0.7

# Expressions for A, B, C, D, E, F
A = cos(t1) * cos(t2)
B = cos(t1) * cos(t2 + t3)
C = sin(t1) * cos(t2)
D = sin(t1) * cos(t2 + t3)
E = sin(t2)
F = sin(t2 + t3)

# Equations based on the given expressions
eq1 = A + B - px_val
eq2 = C + D - py_val
eq3 = E + F - pz_val

# Solve the system of equations for t1, t2, and t3
solutions = solve((eq1, eq2, eq3), (t1, t2, t3))

# Print the solutions
print("Solutions:")
for solution in solutions:
    print(f"t1 = {solution[0]}, t2 = {solution[1]}, t3 = {solution[2]}")