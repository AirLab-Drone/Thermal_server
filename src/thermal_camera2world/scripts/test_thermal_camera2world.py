#!/usr/bin/env python3

'''
This script is used to test the thermal_camera2world node.

When run this script, you need to caibrate the camera 4 coner points and their corresponding world points.
'''
import numpy as np
import yaml
 

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray


class Thermal_camera_to_world(Node):
    def __init__(self):
        super().__init__('thermal_camera_to_world')

        # image
        # self.sub_thermal_image = self.create_subscription(
        #     Image, 
        #     '/thermal_image', 
        #     self.thermal_image_callback, 10
        # )
        # self.sub_thermal_image  # prevent unused variable warning

        # self.cv_bridge = CvBridge()

        # hot spot temperature
        self.sub_hot_spot_temperature = self.create_subscription(
            Float32,
            '/hot_spot_temperature',
            self.hot_spot_temperature_callback, 10
        )
        self.sub_hot_spot_temperature   # prevent unused variable warning

        # hot spot pixel
        self.sub_hot_spot_pixel = self.create_subscription(
            Int32MultiArray,
            '/hot_spot_temperature_pos',
            self.hot_spot_pixel_callback, 10
        )
        self.sub_hot_spot_pixel   # prevent unused variable warning
        
        self.camera_points = [(0, 0, 1), (512, 0, 1), (512, 384, 1), (0, 384, 1)]
        self.world_points = [(1.19, 3.35, -1.50), (3.61, 2.05, -1.40), (1.50, 0.62, -1.70), (0.40, 1.50, -1.35)]

        self.Transform_matrix = self.camera_to_world(self.camera_points, self.world_points)

    def hot_spot_pixel_callback(self, msg):
        data = msg.data
        pixel_coord = (data[0], data[1], 1)
        M = self.Transform_matrix
        # Convert pixel coordinate to world coordinate
        homogeneous_coord = np.array([pixel_coord[0], pixel_coord[1], pixel_coord[2], 1])  # Add an extra element for homogeneous coordinate
        homogeneous_coord = homogeneous_coord.reshape(-1, 1)  # Reshape to a column vector
        world_coord = np.dot(M, homogeneous_coord)
        print(f'x:{world_coord[0]}, y:{world_coord[1]}, z:{world_coord[2]}')


            
    def hot_spot_temperature_callback(self, msg):
        pass
        # print(msg.data)


    def camera_to_world(self, camera_points, world_points):
        # Check if the number of points matches
        if len(camera_points) != len(world_points):
            raise ValueError("Number of camera points and world points must match")

        num_points = len(camera_points)

        # Construct the coefficient matrix A
        A = np.zeros((3*num_points, 12))
        for i in range(num_points):
            x, y, z = camera_points[i]
            X, Y, Z = world_points[i]
            A[3*i] = [x, y, z, 1, 0, 0, 0, 0, 0, -X*x, -X*y, -X*z]
            A[3*i+1] = [0, 0, 0, 0, x, y, z, 1, 0, -Y*x, -Y*y, -Y*z]
            A[3*i+2] = [0, 0, 0, 0, 0, 0, 0, 0, x, -Z*x, -Z*y, -Z*z]

        # Solve the linear system using least squares
        _, _, V = np.linalg.svd(A)
        M = V[-1].reshape(3, 4)

        return M

def main(args=None):
    rclpy.init(args=args)
    thermal_camera2world = Thermal_camera_to_world()
    rclpy.spin(thermal_camera2world)
    thermal_camera2world.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

