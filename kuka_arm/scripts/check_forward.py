from IK_server import forward_dh_transform, rot_z, rot_y
from sympy import *


if __name__ == "__main__":
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    dh_transforms, _ = forward_dh_transform(q1, q2, q3, q4, q5, q6)
    T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_g = dh_transforms

    angles = [-0.79,-0.11,-2.33,1.94,1.14,-3.68]
    params = {q1: angles[0],
              q2: angles[1],
              q3: angles[2],
              q4: angles[3],
              q5: angles[4],
              q6: angles[5]}

    T0_4 = T0_1 * T1_2 * T2_3 * T3_4
    T0_4_eval = T0_4.evalf(subs=params)
    pwc = T0_4_eval[0:3, 3]
    print("Wrist center position: {}".format(pwc))

    R_corr = rot_z(pi) * rot_y(-pi / 2)
    print(R_corr.inv())
    T_corr = R_corr.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))

    T0_g = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_g * T_corr



    T0_g_eval = T0_g.evalf(subs=params)
    pee = T0_g_eval[0:3, 3]
    R_XYZ = T0_g_eval[0:3, 0:3]
    r21 = R_XYZ[1, 0]
    r11 = R_XYZ[0, 0]
    r31 = R_XYZ[2, 0]
    r32 = R_XYZ[2, 1]
    r33 = R_XYZ[2, 2]

    alpha = atan2(r21, r11)  # rotation about Z-axis
    beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21))  # rotation about Y-axis
    gamma = atan2(r32, r33)  # rotation about X-axis
    euler_angles = Matrix([gamma, beta, alpha])

    print("End effector position: {}".format(pee))
    print("End effector euler angles: {}".format(euler_angles))
