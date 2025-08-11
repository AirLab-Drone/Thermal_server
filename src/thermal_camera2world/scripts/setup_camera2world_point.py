#!/usr/bin/env python3

import cv2
import numpy as np
import tkinter as tk
from tkinter import simpledialog, messagebox
import threading
import yaml
import os

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from sensor_msgs.msg import Image


RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
ORANGE = (0, 165, 255)
PURPLE = (255, 0, 255)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class SetupCamera2WorldPoint(Node):
    def __init__(self):
        super().__init__("setup_camera2world_point")
        self.current_namespace = self.get_namespace()  # 設為類別屬性
        self.get_logger().info(f"Current namespace: {self.current_namespace}")

        # ------------------------------- Thermal image ------------------------------ #
        self.sub_thermal_image = self.create_subscription(
            Image, "thermal_image", self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.thermal_image = None
        self.thermal_image_debug = None

        # ---------------------------------- opencv windows -------------------------- #
        self.thermal_origin_image_window_name = "Thermal Image"
        self.thermal_debug_image_window_name = "Thermal Image Debug"

        self.pixel_and_world_coordinate = []

        self.declare_parameter(
            "config_file",
            os.getcwd() + "/src/thermal_camera2world/config/camera2world.yaml",
        )
        self.config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        # TODO: 讀取robot laser back 座標
        self.declare_parameter(
            "isRobot",
            False,
        )
        self.isRobot = (
            self.get_parameter("isRobot").get_parameter_value().bool_value
        )

        if self.isRobot:
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)

        # Setup mouse callback
        cv2.namedWindow(self.thermal_debug_image_window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(
            self.thermal_debug_image_window_name, self.SelectFourConerCallback
        )

    def thermal_image_callback(self, msg) -> None:

        self.thermal_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        self.thermal_image_debug = self.thermal_image.copy()

        if self.pixel_and_world_coordinate:
            # 提取所有像素座標
            pixel_points = [pair[0] for pair in self.pixel_and_world_coordinate]

            # 繪製點與線
            self.DrawPoints(self.thermal_image_debug, pixel_points, RED)
            self.DrawLine(self.thermal_image_debug, pixel_points, ORANGE, 1)

        cv2.imshow(self.thermal_debug_image_window_name, self.thermal_image_debug)
        cv2.waitKey(1)

    def SelectFourConerCallback(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.pixel_and_world_coordinate.__len__() >= 4:
                self.get_logger().warn("Already selected 4 points.")
                self.update_config_file()
                return
            # 點擊後紀錄像素座標
            pixel_point = (x, y)
            self.get_logger().info(f"Pixel point clicked: {pixel_point}")

            # TODO 讀取robot laser back 座標

            # 啟動子執行緒處理世界座標輸入
            threading.Thread(target=self.ask_for_world_point, args=(pixel_point,)).start()

        elif event == cv2.EVENT_RBUTTONDOWN:
            # 右鍵刪除最近一個點
            if self.pixel_and_world_coordinate:
                removed_point = self.pixel_and_world_coordinate.pop()
                self.get_logger().warn(f"Removed point: Pixel point: {removed_point[0]}, World point: {removed_point[1]}")


    def update_config_file(self):
        try:
            # 準備 YAML 資料
            world_points = [pair[1] for pair in self.pixel_and_world_coordinate]
            pixel_points = [pair[0] for pair in self.pixel_and_world_coordinate]

            # 讀取原始 YAML 檔案
            if os.path.exists(self.config_file):
                with open(self.config_file, "r") as file:
                    config_data = yaml.safe_load(file)
            else:
                config_data = {}  # 如果檔案不存在，初始化為空字典

            # 構建新的 namespace 資料
            namespace_key = f'{self.current_namespace}/thermal_camera_to_world'
            config_data[namespace_key] = {
                "ros__parameters": {
                    "World_UpperLeft": world_points[0],
                    "World_UpperRight": world_points[1],
                    "World_LowerRight": world_points[2],
                    "World_LowerLeft": world_points[3],
                    "Thermal_UpperLeft": list(pixel_points[0]),
                    "Thermal_UpperRight": list(pixel_points[1]),
                    "Thermal_LowerRight": list(pixel_points[2]),
                    "Thermal_LowerLeft": list(pixel_points[3]),
                    "Threshold_Temperature": 50.0,
                    "Alert_Waiting_Time": 5,
                }
            }

            # 寫回 YAML 檔案
            with open(self.config_file, "w") as file:
                yaml.dump(config_data, file, default_flow_style=False, allow_unicode=True)

            self.get_logger().info(f"Config file updated for namespace: {namespace_key}")
        except Exception as e:
            self.get_logger().error(f"Failed to update config file: {e}")



    def ask_for_world_point(self, pixel_point):
        # 在主執行緒中呼叫 Tkinter 輸入框
        root = tk.Tk()
        root.withdraw()

        def get_point():
            world_point = self.get_world_point()
            if world_point:
                self.pixel_and_world_coordinate.append([pixel_point, world_point])
                # self.get_logger().info(f"Pixel point: {pixel_point}, World point: {world_point}")
                self.get_logger().info(f"{self.pixel_and_world_coordinate}")
            root.destroy()
        root.after(0, get_point)
        root.mainloop()

    def get_world_point(self):
        user_input = simpledialog.askstring(
            "輸入世界座標", "請輸入世界座標 (格式: x,y):"
        )
        if user_input:
            try:
                x, y = map(float, user_input.split(","))
                return [x, y]
            except ValueError:
                messagebox.showerror("輸入錯誤", "請輸入有效的座標格式，如 3.5,4.2")
        return None

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
    setup_camera2world_point = SetupCamera2WorldPoint()
    rclpy.spin(setup_camera2world_point)

    setup_camera2world_point.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
