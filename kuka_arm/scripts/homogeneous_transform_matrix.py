# Find homogeneous transform matrix from base to gripper

from sympy import *
from IK_server import dh_transform, rot_z, rot_y, rot_x

if __name__ == '__main__':
    roll, pitch, yaw = symbols("roll pitch yaw")
    x, y, z = symbols("x y z")

    R = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
    R_corr = rot_z(pi) * rot_y(-pi / 2)
    Rrpy = R * R_corr.transpose()

    pee = Matrix([[x], [y], [z]])

    T = Rrpy.row_join(pee).col_join(Matrix([[0, 0, 0, 1]]))

    T = simplify(T)
    print(T)
