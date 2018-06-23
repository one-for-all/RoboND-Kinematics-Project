# Find equation for inverse kinematics of last three angles (i.e orientation)

from sympy import *
from IK_server import dh_transform


if __name__ == '__main__':
    a3, d4, dg = symbols("a3 d4 dg")
    q4, q5, q6 = symbols("q4:7")

    T3_4 = dh_transform(-pi / 2, a3, d4, q4)
    T4_5 = dh_transform(pi / 2, 0, 0, q5)
    T5_6 = dh_transform(-pi / 2, 0, 0, q6)
    T6_g = dh_transform(0, 0, dg, 0)

    R3_4 = T3_4[0:3, 0:3]
    R4_5 = T4_5[0:3, 0:3]
    R5_6 = T5_6[0:3, 0:3]
    R6_g = T6_g[0:3, 0:3]

    R3_g = R3_4*R4_5*R5_6*R6_g
    R3_g = simplify(R3_g)
    print(R3_g)