#!/usr/bin/env python3

"""
This script is used to test the thermal_camera2world node.

When run this script, you need to caibrate the camera 4 coner points and their corresponding world points.
"""

import cv2
import numpy as np

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

        # ------------- Republish World Coordinate and Hot Spot Temperature ------------ #
        self.pub_thermal_alert = self.create_publisher(
            ThermalAlert, "thermal_alert", 10
        )
        self.world_coordinate_x = 0.0
        self.world_coordinate_y = 0.0
        self.thermal_alert = ThermalAlert()  # create a ThermalAlert message

        # ------------------------------- Thermal image ------------------------------ #
        self.sub_thermal_image = self.create_subscription(
            Image, "thermal_image", self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.thermal_image = None
        self.thermal_image_debug = None

        # --------------------------- Hot spot temperature --------------------------- #
        self.sub_hot_spot_temperature = self.create_subscription(
            Float32, "hot_spot_temperature", self.hot_spot_temperature_callback, 10
        )
        self.sub_hot_spot_temperature  # prevent unused variable warning
        self.hot_spot_temperature = 0.0

        # ------------------------------ Hot spot pixel ------------------------------ #
        self.sub_hot_spot_pixel = self.create_subscription(
            Int32MultiArray,
            "hot_spot_temperature_pos",
            self.hot_spot_pixel_callback,
            10,
        )
        self.sub_hot_spot_pixel  # prevent unused variable warning
        self.hot_spot_pixel = [0, 0]

        # ---------------------------------- opencv windows -------------------------- #
        self.thermal_origin_image_window_name = "Thermal Image"
        self.thermal_debug_image_window_name = "Thermal Image Debug"

        cv2.namedWindow(self.thermal_debug_image_window_name, cv2.WINDOW_NORMAL)

        self.detcet_fire_time = None


    def hot_spot_pixel_callback(self, msg) -> None:
        self.hot_spot_pixel = [msg.data[0], msg.data[1]]

    def hot_spot_temperature_callback(self, msg) -> None:
        threshold_temperature = (
            self.get_parameter("Threshold_Temperature")
            .get_parameter_value()
            .double_value
        )

        alert_waiting_time = (
            self.get_parameter("Alert_Waiting_Time")
            .get_parameter_value()
            .integer_value
        )

        self.hot_spot_temperature = msg.data

        self.thermal_alert.temperature = self.hot_spot_temperature
        self.thermal_alert.x = float(self.world_coordinate_x)
        self.thermal_alert.y = float(self.world_coordinate_y)

        if self.hot_spot_temperature > threshold_temperature:
            if not self.detcet_fire_time:
                self.detcet_fire_time = Clock().now()
                print(f"[Info] Didn't Detect Fire...")
            else:
                delta_time = Clock().now() - self.detcet_fire_time
                self.get_logger().info(
                    f"[Info] Detect Fire! in {delta_time.nanoseconds/1e9:.2f} seconds"
                )

                if delta_time > Duration(seconds=alert_waiting_time):
                    self.pub_thermal_alert.publish(self.thermal_alert)
                    print(f"[Info] Published thermal alert!")
                    print(f"[Info] World coordinate: [x: {self.world_coordinate_x:.2f}, y: {self.world_coordinate_y:.2f}]")
                    print(f"[Info] Temperature: {self.hot_spot_temperature:.2f}")
        else:
            self.detcet_fire_time = None

    def thermal_image_callback(self, msg) -> None:

        self.thermal_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        self.thermal_image_debug = self.thermal_image.copy()


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

        h, _ = cv2.findHomography(selected_area, target_area)  # numpy array

        # 將熱點像素座標轉換到世界座標
        point = np.array(self.hot_spot_pixel, dtype=np.float32)
        corrected_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), h)

        self.world_coordinate_x, self.world_coordinate_y = corrected_point[0][0]

        self.DrawPoints(self.thermal_image_debug, [self.hot_spot_pixel], BLUE)
        self.DrawPoints(self.thermal_image_debug, selected_area.astype(np.int32).tolist(), RED)
        self.DrawLine(self.thermal_image_debug, selected_area.astype(np.int32).tolist(), ORANGE, 1)


        self.get_logger().info(f'{self.world_coordinate_x:.2f}, {self.world_coordinate_y:.2f}')


        cv2.imshow(self.thermal_debug_image_window_name, self.thermal_image_debug)

        cv2.waitKey(1)

    def DrawPoints(
        self, image: cv2.Mat, points: list, color: tuple, radius: int = -1
    ) -> None:
        for point in points:
            cv2.circle(image, point, 3, color, radius)


    def DrawLine(
        self, image: cv2.Mat, points: list, color: tuple, width: int = 2
    ) -> None:
        if len(points) < 2:
            return
        for i in range(len(points) - 1):
            cv2.line(image, points[i], points[i + 1], color, width)
        if len(points) == 4:
            cv2.line(image, points[0], points[3], color, width)

def main(args=None):
    rclpy.init(args=args)
    thermal_camera2world = Thermal_camera_to_world()
    rclpy.spin(thermal_camera2world)

    thermal_camera2world.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()