#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing
import math
import numpy as np


# To make display prety
printing.init_printing(use_latex = True)

theta_i = Symbol("theta_i")
alpha_i = Symbol("alpha_i")
r_i = Symbol("r_i")
d_i = Symbol("d_i")

DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                            [0, sin(alpha_i), cos(alpha_i), d_i],
                            [0,0,0,1]])

result_simpl = simplify(DH_Matric_Generic)

from sympy import preview


# Now create A01, A12, A23

theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")


theta_1 = input("please enter theta1 : ")
theta_2 = input("please enter theta2 : ")
theta_3 = input("please enter theta3 : ")

alpha_planar = 0.0

alpha_1 = math.pi/2.0
alpha_2 = 0.0
alpha_3 = 0.0

r_1 = Symbol("r_1")
r_2 = Symbol("r_2")
r_3 = Symbol("r_3")

r_1 = 0.0
r_2 = 1.0
r_3 = 1.0

d_planar = 0.0
d_1 = d_planar
d_2 = d_planar
d_3 = d_planar

A01 = DH_Matric_Generic.subs(r_i,r_1).subs(alpha_i,alpha_1).subs(d_i,d_1).subs(theta_i, theta_1)
A12 = DH_Matric_Generic.subs(r_i,r_2).subs(alpha_i,alpha_2).subs(d_i,d_2).subs(theta_i, theta_2)
A23 = DH_Matric_Generic.subs(r_i,r_3).subs(alpha_i,alpha_3).subs(d_i,d_3).subs(theta_i, theta_3)

A01_simplify = trigsimp(A01)


A03 = A01 * A12 * A23
A13 = A12 * A23

A03_simplify = trigsimp(A03)
A13_simplify = trigsimp(A13)

# We save

position = A03[0:3, 3]
orientation = A03[0:3, 0:3]

print("position =", position)
print("Orientation =", orientation)


