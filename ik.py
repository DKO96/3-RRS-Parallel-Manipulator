import numpy as np
from plot import plot_robot

r = 70.0 # radius of platform [mm]
h = 60.0 # height of the platform [mm]
n = np.array([0.3, 0.0, 1.0]) # orientation of the platform 
b = 45.0 # base radius [mm]
L1 = 55.0 # length of link 1 [mm] 
L2 = 82.5 # length of link 2 [mm]
P1 = 0  # plane of arm 1
P2 = np.deg2rad(120) # plane of arm 2
P3 = np.deg2rad(240) # plane of arm 3

def plane_normal(theta):
    return np.array([-np.sin(theta), np.cos(theta), 0.0])

def joint_intersection(pt1, r1, pt2, r2, normal):
    d = np.linalg.norm(pt2 - pt1)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        print(f"Joint intersection error: {d=}")
        return None
    
    a = (r1**2 - r2**2 + d**2) / (2*d)
    h = np.sqrt(r1**2 - a**2)
    u = (pt2 - pt1) / d
    v = np.cross(normal, u)
    v /= np.linalg.norm(v)
    m = pt1 + a * u

    return m + h * v

def check_link_distance(pt1, pt2, expected):
    d = np.linalg.norm(pt2 - pt1)
    if not np.isclose(d, expected):
        print(f"Invalid geometry: {d:.4f} != {expected}")
    return  

def inverse_kinematics(n=np.array([0.0, 0.0, 0.0]), h=80, ax=None):
    # Find position of spherical joints
    # Arm 1 
    sj1 = np.array([0.0, 0.0, 0.0])
    sj1[2] = h - ((n[0] * r) / np.sqrt(n[2]**2 + n[0]**2))
    sj1[1] = 0
    sj1[0] = np.sqrt(r**2 - (h - sj1[2])**2)

    # Arm 2
    sj2 = np.array([0.0, 0.0, 0.0])
    denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*np.sqrt(3)*n[0]*n[1])
    sj2[2] = h + (r * (n[0] - np.sqrt(3) * n[1]) / denom)
    sj2[1] = (np.sqrt(3) * r * n[2]) / denom
    sj2[0] = (-r * n[2]) / denom

    # Arm 3
    sj3 = np.array([0.0, 0.0, 0.0])
    denom = np.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*np.sqrt(3)*n[0]*n[1])
    sj3[2] = h + (r * (n[0] + np.sqrt(3) * n[1]) / denom)
    sj3[1] = (-np.sqrt(3) * r * n[2]) / denom
    sj3[0] = (-r * n[2]) / denom

    # print(f"{sj1=}")
    # print(f"{sj2=}")
    # print(f"{sj3=}")

    # Find position of the pin joints
    # Arm 1 
    n1 = plane_normal(P1)
    rj1 = np.array([b, 0.0, 0.0])
    pj1 = joint_intersection(rj1, L1, sj1, L2, n1)

    # Arm 2 
    rj2 = np.array([b*np.cos(P2), b*np.sin(P2), 0.0])
    n2 = plane_normal(P2)
    pj2 = joint_intersection(rj2, L1, sj2, L2, n2)

    # Arm 1 
    rj3 = np.array([b*np.cos(P3), b*np.sin(P3), 0.0])
    n3 = plane_normal(P3)
    pj3 = joint_intersection(rj3, L1, sj3, L2, n3)

    # print(f"{pj1=}")
    # print(f"{pj2=}")
    # print(f"{pj3=}")

    # Check geometry
    check_link_distance(rj1, pj1, L1)
    check_link_distance(rj2, pj2, L1)
    check_link_distance(rj3, pj3, L1)
    check_link_distance(pj1, sj1, L2)
    check_link_distance(pj2, sj2, L2)
    check_link_distance(pj3, sj3, L2)

    # Find angle of driven revolute joints
    # Arm 1
    t1 = np.pi/2 - np.arctan2(pj1[0]-r, pj1[2])

    # Arm 2
    radial2 = np.sqrt(pj2[0]**2 + pj2[1]**2)
    t2 = np.pi/2 - np.arctan2(radial2 - r, pj2[2])

    # Arm 3
    radial3 = np.sqrt(pj3[0]**2 + pj3[1]**2)
    t3 = np.pi/2 - np.arctan2(radial3 - r, pj3[2])

    # print(f"{np.rad2deg(t1)=}")
    # print(f"{np.rad2deg(t2)=}")
    # print(f"{np.rad2deg(t3)=}")

    # plot_robot(ax, rj1, rj2, rj3, sj1, sj2, sj3, pj1, pj2, pj3, r, h, n)

    return np.array([t1, t2, t3])


if __name__ == "__main__":
    inverse_kinematics()
