import numpy as np
import matplotlib.pyplot as plt
from ik import inverse_kinematics

def generate_trajectory(pose1, pose2, num_steps=50):
    trajectory = []

    for i in range(num_steps):
        t = i / (num_steps - 1)

        n = (1 - t) * pose1[:3] + t * pose2[:3]
        n = n / np.linalg.norm(n)

        h = (1 - t) * pose1[3] + t * pose2[3]

        trajectory.append((n, h))
    
    return trajectory

def theta_to_steps(theta, stepsize=0.003926991):
    theta_1 = theta[:, 0]
    theta_2 = theta[:, 1]
    theta_3 = theta[:, 2]

    q = np.array([theta_1[0], theta_2[0], theta_3[0]])
    result = [q.copy()]

    for i in range(1, len(theta)):
        target = np.array([theta_1[i], theta_2[i], theta_3[i]])

        # How many steps each motor needs for this segment
        diff = target - q
        steps_needed = np.round(np.abs(diff) / stepsize).astype(int)
        direction = np.sign(diff)

        # The motor that needs the most steps sets the pace
        n = np.max(steps_needed)
        if n == 0:
            continue

        # Bresenham-style error accumulators
        error = np.zeros(3)

        for _ in range(n):
            error += steps_needed
            step = np.zeros(3)
            for axis in range(3):
                if error[axis] >= n:
                    step[axis] = direction[axis] * stepsize
                    error[axis] -= n
            q = q + step
            result.append(q.copy())

    return np.array(result)

def steps_to_motor(steps):
    lookup = np.array([0x01, 0x00, 0x10], dtype=np.uint8)
    deltas = np.sign(np.diff(steps, axis=0)).astype(int)

    command_1 = lookup[deltas[:, 0] + 1]
    command_2 = lookup[deltas[:, 1] + 1]
    command_3 = lookup[deltas[:, 2] + 1]

    return np.column_stack((command_1, command_2, command_3))

def motion_planning():


    pose1 = np.array([0.3, 0.2, 1.0, 40.0])
    pose2 = np.array([-0.3, -0.2, 1.0, 65.0])
    theta = []

    trajectory = generate_trajectory(pose1, pose2, 50)

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for t in trajectory:
        theta.append(inverse_kinematics(t[0], t[1], ax))
    
    steps = theta_to_steps(np.array(theta))

    command = steps_to_motor(steps)

    return command
    
    # plt.ioff()
    # plt.show()


if __name__ == "__main__":
    motion_planning()
