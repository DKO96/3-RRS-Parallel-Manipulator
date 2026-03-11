import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import art3d

def get_circle_points(center, normal, radius, num_points=100):
    """Generate 3D points for a circle at a given center, oriented to a normal."""
    normal = np.array(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)  # normalize

    # Find two vectors perpendicular to the normal
    # Pick an arbitrary vector not parallel to normal
    if abs(normal[0]) < 0.9:
        arbitrary = np.array([1, 0, 0])
    else:
        arbitrary = np.array([0, 1, 0])

    # Gram-Schmidt to get two orthonormal vectors in the plane
    u = np.cross(normal, arbitrary)
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)

    theta = np.linspace(0, 2 * np.pi, num_points)
    circle = (center[:, np.newaxis]
              + radius * np.cos(theta) * u[:, np.newaxis]
              + radius * np.sin(theta) * v[:, np.newaxis])
    return circle  # shape (3, num_points)

def plot_robot(ax, rj1, rj2, rj3, sj1, sj2, sj3, pj1, pj2, pj3, r, h, n):
    ax.cla()

    ax.set_xlim([-80, 80])
    ax.set_ylim([-80, 80])
    ax.set_zlim([0, 120])

    ax.scatter(rj1[0], rj1[1], rj1[2], c='b')
    ax.scatter(rj2[0], rj2[1], rj2[2], c='b')
    ax.scatter(rj3[0], rj3[1], rj3[2], c='b')

    ax.scatter(sj1[0], sj1[1], sj1[2], c='r')
    ax.scatter(sj2[0], sj2[1], sj2[2], c='r')
    ax.scatter(sj3[0], sj3[1], sj3[2], c='r')

    ax.scatter(pj1[0], pj1[1], pj1[2], c='g')
    ax.scatter(pj2[0], pj2[1], pj2[2], c='g')
    ax.scatter(pj3[0], pj3[1], pj3[2], c='g')

    ax.plot([rj1[0], pj1[0], sj1[0]],
            [rj1[1], pj1[1], sj1[1]],
            [rj1[2], pj1[2], sj1[2]], c='g')

    ax.plot([rj2[0], pj2[0], sj2[0]],
            [rj2[1], pj2[1], sj2[1]],
            [rj2[2], pj2[2], sj2[2]], c='g')

    ax.plot([rj3[0], pj3[0], sj3[0]],
            [rj3[1], pj3[1], sj3[1]],
            [rj3[2], pj3[2], sj3[2]], c='g')

    circle1 = mpatches.Circle((0, 0), radius=r, color='blue', alpha=0.4)
    ax.add_patch(circle1)
    art3d.pathpatch_2d_to_3d(circle1, z=0, zdir='z')

    # Tilted platform circle
    center = np.array([0, 0, h])  # or wherever the platform center is
    pts = get_circle_points(center, n, r)
    ax.plot(pts[0], pts[1], pts[2], c='red')

    # If you want a filled disk:
    from matplotlib.tri import Triangulation
    # Add center point for triangulation
    xs = np.append(pts[0], center[0])
    ys = np.append(pts[1], center[1])
    zs = np.append(pts[2], center[2])
    ax.plot_trisurf(xs, ys, zs, color='red', alpha=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # plt.draw()
    # plt.pause(0.1)