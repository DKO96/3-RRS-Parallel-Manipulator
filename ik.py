import numpy as np
from plot import plot_robot

p = 55.0 # radius of platform []
b = 45.0 # base radius [mm]
L1 = 56.0 # length of link 1 [mm] 
L2 = 84.0 # length of link 2 [mm]
alpha = np.array([0, np.deg2rad(120), np.deg2rad(240)])


def inverse_kinematics(n=np.array([0.0, 0.0, 1.0]), h=100.0):
    # Determine platform rotation
    psi_y = np.arcsin(n[0])
    psi_x = np.arcsin(-n[1] / np.cos(psi_y))
    psi_z = np.arctan2((-np.sin(psi_x) * np.sin(psi_y)) , (np.cos(psi_x) + np.cos(psi_y)))

    sx, cx = np.sin(psi_x), np.cos(psi_x)
    sy, cy = np.sin(psi_y), np.cos(psi_y)
    sz, cz = np.sin(psi_z), np.cos(psi_z)

    R = np.array([
        [           cy*cz,           -cy*sz,     sy],
        [cx*sz + sx*sy*cz, cx*cz - sx*sy*sz, -sx*cy],
        [sx*sz - cx*sy*cz, sx*cz + cx*sy*sz,  cx*cy],
        ])

    # Determine platform rotation
    vx, vy = R[0][1], R[1][1]
    ux, uy = R[0][0], vx

    O = np.array([
        -uy*p,
        (p*(ux - vy)) / 2,
        h
    ])

    theta = np.array([0.0, 0.0, 0.0])

    for i in range(3):
        sa, ca = np.sin(alpha[i]), np.cos(alpha[i])
        Rz = np.array([
            [ca, -sa, 0],
            [sa,  ca, 0],
            [ 0,   0, 1]
        ])

        # Determine spherical joint position 
        O_j = O + (R @ Rz @ np.array([p, 0, 0]).T)

        # Determine theta
        A = 2*L1*ca*(-O_j[0] + b * ca)
        B = 2*L1*O_j[2]*ca*ca
        C = O_j[0]*O_j[0] - 2*b*O_j[0]*ca + ca*ca*(b*b + L1*L1 - L2*L2 + O_j[2]*O_j[2])

        t = (-B + np.sqrt(A*A + B*B - C*C)) / (C - A)

        theta[i] = 2 * np.arctan(t)

    return theta


if __name__ == "__main__":
    inverse_kinematics()