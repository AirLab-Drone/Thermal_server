#!/usr/bin/env python3

"""
This script is used to test the thermal_camera2world node.

When run this script, you need to caibrate the camera 4 coner points and their corresponding world points.
"""

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray


RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
ORANGE = (0, 165, 255)
PURPLE = (255, 0, 255)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class Thermal_camera_to_world(Node):
    def __init__(self):
        super().__init__("thermal_camera_to_world")

        # -------------------------------- Parameters -------------------------------- #
        # This parameter is used to set the four coner points of the world coordinate
        self.declare_parameter('World_UpperLeft', [0.0, 21.0])
        self.declare_parameter('World_UpperRight', [29.7, 21.0])
        self.declare_parameter('World_LowerRight', [29.7, 0.0])
        self.declare_parameter('World_LowerLeft', [0.0, 0.0])


        # ------------------------------- Thermal image ------------------------------ #
        self.sub_thermal_image = self.create_subscription(
            Image, "/thermal_image", self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.thermal_image = None
        self.thermal_image_debug = None


        # --------------------------- Hot spot temperature --------------------------- #
        self.sub_hot_spot_temperature = self.create_subscription(
            Float32, "/hot_spot_temperature", self.hot_spot_temperature_callback, 10
        )
        self.sub_hot_spot_temperature  # prevent unused variable warning
        self.hot_spot_temperature = 0.0


        # ------------------------------ Hot spot pixel ------------------------------ #
        self.sub_hot_spot_pixel = self.create_subscription(
            Int32MultiArray,
            "/hot_spot_temperature_pos",
            self.hot_spot_pixel_callback,
            10,
        )
        self.sub_hot_spot_pixel  # prevent unused variable warning
        self.hot_spot_pixel = [0, 0]


        # ---------------------------------- opencv windows ---------------------------------- #
        self.thermal_origin_image_window_name = "Thermal Image"
        self.thermal_debug_image_window_name = "Thermal Image Debug"

        # ---------------------------- add mouse listener ---------------------------- #
        cv2.namedWindow(self.thermal_debug_image_window_name)
        cv2.setMouseCallback(self.thermal_debug_image_window_name, self.SelectFourConerCallback)
        self.four_coner_points = []


    def SelectFourConerCallback(self, event, x, y, flags, param) -> None:
        # add the point to the list if you click left button
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.four_coner_points.__len__() >= 4:
                print("You have already selected all the coner points.")
                print(self.four_coner_points)
                return
            print("Clicked at pixel coordinates (x={}, y={})".format(x, y))
            self.four_coner_points.append((x, y))

        # delete the last point if you click right button
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.four_coner_points:
                self.four_coner_points.pop()


    def hot_spot_pixel_callback(self, msg) -> None:
        self.hot_spot_pixel = [msg.data[0], msg.data[1]]


    def hot_spot_temperature_callback(self, msg) -> None:
        self.hot_spot_temperature = msg.data


    def DrawPoints(self, image:cv2.Mat, points:list, color:tuple, radius:int=-1) -> None:
        for point in points:
            cv2.circle(image, point, 3, color, radius)

    def DrawLine(self, image:cv2.Mat, points:list, color:tuple, width:int=2) -> None:
        if len(points) < 2:
            return
        for i in range(len(points)-1):
            cv2.line(image, points[i], points[i+1], color, width)
        if len(points) == 4:
            cv2.line(image, points[0], points[3], color, width)


    def thermal_image_callback(self, msg) -> None:

        # ------------------------- get four corner world coodinate ------------------------- #
        # type is double array [x, y]
        world_upper_left = self.get_parameter('World_UpperLeft').get_parameter_value().double_array_value
        world_upper_right = self.get_parameter('World_UpperRight').get_parameter_value().double_array_value
        world_lower_right = self.get_parameter('World_LowerRight').get_parameter_value().double_array_value
        world_lower_left = self.get_parameter('World_LowerLeft').get_parameter_value().double_array_value
        # print(upper_left, upper_right, lower_right, lower_left)



        # ----------------------- ros2 image topic to cv2 image ---------------------- #
        self.thermal_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        self.thermal_image_debug = self.thermal_image.copy()

        self.DrawPoints(self.thermal_image_debug, [self.hot_spot_pixel], BLUE)


        if self.four_coner_points:
            self.DrawPoints(self.thermal_image_debug, self.four_coner_points, RED)
            self.DrawLine(self.thermal_image_debug, self.four_coner_points, ORANGE, 1)

        cv2.imshow(self.thermal_debug_image_window_name, self.thermal_image_debug)

        if self.four_coner_points.__len__() == 4:
            target_area = np.array([world_upper_left, world_upper_right, world_lower_right, world_lower_left], dtype=np.float32)
            selected_area = np.array(self.four_coner_points, dtype=np.float32)

        
        

            # print(target_area)

            h, _ = cv2.findHomography(selected_area, target_area)   # numpy array

        cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    thermal_camera2world = Thermal_camera_to_world()
    rclpy.spin(thermal_camera2world)

    thermal_camera2world.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
