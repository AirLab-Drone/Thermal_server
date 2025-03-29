#!/usr/bin/env python3

"""
This script is used to test the thermal_camera2world node.

When run this script, you need to caibrate the camera 4 coner points and their corresponding world points.
"""

import cv2
import numpy as np
import os

import rclpy
from rclpy.parameter import Parameter
from rclpy.clock import Clock
from rclpy.time import Duration
from rclpy.time import Time
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

import rclpy.time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

from thermal_msgs.msg import ThermalAlert


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
        self.current_namespace = self.get_namespace()  # 設為類別屬性
        self.get_logger().info(f"Current namespace: {self.current_namespace}")

        # 儲存一次性開關（建議在 __init__ 中宣告）
        if not hasattr(self, "saved_points"):
            self.saved_points = False


        # -------------------------------- Parameters -------------------------------- #
        # 世界座標四個角點
        self.declare_parameter("World_UpperLeft", [0.0, 0.0])
        self.declare_parameter("World_UpperRight", [1.0, 5.0])
        self.declare_parameter("World_LowerRight", [45.0, 0.0])
        self.declare_parameter("World_LowerLeft", [0.0, 6.0])

        # 像素座標四個角點
        self.declare_parameter("Thermal_UpperLeft", [0, 0])
        self.declare_parameter("Thermal_UpperRight", [0, 0])
        self.declare_parameter("Thermal_LowerRight", [0, 0])
        self.declare_parameter("Thermal_LowerLeft", [0, 0])

        self.declare_parameter("Threshold_Temperature", 80.0)
        self.declare_parameter("Alert_Waiting_Time", 5)



        # 讀取參數
        world_upper_left = (
            self.get_parameter("World_UpperLeft")
            .get_parameter_value()
            .double_array_value
        )
        world_upper_right = (
            self.get_parameter("World_UpperRight")
            .get_parameter_value()
            .double_array_value
        )
        world_lower_right = (
            self.get_parameter("World_LowerRight")
            .get_parameter_value()
            .double_array_value
        )
        world_lower_left = (
            self.get_parameter("World_LowerLeft")
            .get_parameter_value()
            .double_array_value
        )
        thermal_upper_left = (
            self.get_parameter("Thermal_UpperLeft")
            .get_parameter_value()
            .integer_array_value
        )
        thermal_upper_right = (
            self.get_parameter("Thermal_UpperRight")
            .get_parameter_value()
            .integer_array_value
        )
        thermal_lower_right = (
            self.get_parameter("Thermal_LowerRight")
            .get_parameter_value()
            .integer_array_value
        )
        thermal_lower_left = (
            self.get_parameter("Thermal_LowerLeft")
            .get_parameter_value()
            .integer_array_value
        )

        target_area = np.array(
            [world_upper_left, world_upper_right, world_lower_right, world_lower_left],
            dtype=np.float32,
        )
        selected_area = np.array(
            [
                thermal_upper_left,
                thermal_upper_right,
                thermal_lower_right,
                thermal_lower_left,
            ],
            dtype=np.float32,
        )
        # self.get_logger().info(f"{target_area}, {selected_area}")


        self.homography_matrix, _ = cv2.findHomography(selected_area, target_area)  # numpy array

        

        # ------------------------------- Thermal image ------------------------------ #
        self.sub_thermal_image = self.create_subscription(
            Image, "thermal_image", self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.thermal_image = None
        self.thermal_image_debug = None


        # ---------------------------------- opencv windows -------------------------- #
        self.thermal_debug_image_window_name = "Thermal Image Debug"

        cv2.namedWindow(self.thermal_debug_image_window_name, cv2.WINDOW_NORMAL)




    def thermal_image_callback(self, msg) -> None:

        self.thermal_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        points_row = 6
        points_column = 4

        x_border = 50   #pixel
        y_border = 50

        
        height, width = self.thermal_image.shape[:2]

        self.thermal_image_debug = self.thermal_image.copy()

        points_board = []

        x_offset = (width - (2 * x_border)) / (points_row-1)
        y_offset = (height - (2 * y_border)) / (points_column-1)

            
        for col in range(points_column):
            for row in range(points_row):
                x = int(x_border + row * x_offset)
                y = int(y_border + col * y_offset)
                points_board.append((x, y))
                cv2.circle(self.thermal_image_debug, (x, y), radius=3, color=RED, thickness=-1)  # 紅色實心圓

        
        # self.get_logger().info(f"{str(points_board)}")
        
        

        # 將熱點像素座標轉換到世界座標

        pixel_points_np = np.array(points_board, dtype=np.float32).reshape(-1, 1, 2)

        if self.homography_matrix is not None:
            transformed_points = cv2.perspectiveTransform(pixel_points_np, self.homography_matrix)

        # 拆成 list，並存下或印出來
        world_points = [tuple(pt[0]) for pt in transformed_points]
        # self.get_logger().info(f"世界座標點: {str(world_points)}")


        if not self.saved_points:
            # 轉為 numpy 陣列
            pixel_np = np.array(points_board, dtype=np.float32)  # shape (N, 2)
            world_np = np.array(world_points, dtype=np.float32)  # shape (N, 2)

            # 設定儲存路徑
            base_path = os.path.expanduser("~")  # 儲存在使用者家目錄
            pixel_path = os.path.join(base_path, f"{str(self.current_namespace[1:])}_pixel_points.npy")
            world_path = os.path.join(base_path, f"{str(self.current_namespace[1:])}_world_points.npy")

            # 儲存
            np.save(pixel_path, pixel_np)
            np.save(world_path, world_np)

            self.get_logger().info(f"已儲存像素點到：{pixel_path}")
            self.get_logger().info(f"已儲存世界座標點到：{world_path}")

            self.saved_points = True


        cv2.imshow(self.thermal_debug_image_window_name, self.thermal_image_debug)

        cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    thermal_camera2world = Thermal_camera_to_world()
    rclpy.spin(thermal_camera2world)

    thermal_camera2world.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()