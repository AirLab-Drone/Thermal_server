#!/usr/bin/env python3


import numpy as np
import rclpy


def camera_to_world(camera_points, world_points):
    # Check if the number of points matches
    if len(camera_points) != len(world_points):
        raise ValueError("Number of camera points and world points must match")

    num_points = len(camera_points)

    # Construct the coefficient matrix A
    A = np.zeros((3*num_points, 12))
    for i in range(num_points):
        x, y = camera_points[i]
        X, Y, Z = world_points[i]
        A[3*i] = [x, y, 1, 0, 0, 0, 0, 0, 0, -X*x, -X*y, -X]
        A[3*i+1] = [0, 0, 0, x, y, 1, 0, 0, 0, -Y*x, -Y*y, -Y]
        A[3*i+2] = [0, 0, 0, 0, 0, 0, x, y, 1, -Z*x, -Z*y, -Z]

    # Solve the linear system using least squares
    _, _, V = np.linalg.svd(A)
    M = V[-1].reshape(3, 4)

    return M

# Example camera points and corresponding world points
camera_points = [(100, 100), (200, 100), (200, 200), (100, 200)]
world_points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]

# Perform camera to world coordinate transformation
M = camera_to_world(camera_points, world_points)
print("Transformation matrix M:")
print(M)

if __name__ == '__main__':
    pass
