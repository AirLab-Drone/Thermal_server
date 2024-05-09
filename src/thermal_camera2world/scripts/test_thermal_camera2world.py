#!/usr/bin/env python3

"""
This script is used to test the thermal_camera2world node.

When run this script, you need to caibrate the camera 4 coner points and their corresponding world points.
"""

import cv2
import numpy as np
import yaml


import rclpy
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

with open("src/thermal_camera2world/config/camera2world.yml", "r") as f:
    calibration = yaml.safe_load(f)

upper_left = calibration["Test"]["UpperLeft"][0]
upper_right = calibration["Test"]["UpperRight"][0]
lower_right = calibration["Test"]["LowerRight"][0]
lower_left = calibration["Test"]["LowerLeft"][0]


class Thermal_camera_to_world(Node):
    def __init__(self):
        super().__init__("thermal_camera_to_world")

        # ------------------------------- Thermal image ------------------------------ #
        self.sub_thermal_image = self.create_subscription(
            Image, "/thermal_image", self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.cv_thermal_image = None
        self.cv_thermal_image_copy = None

        # --------------------------- Hot spot temperature --------------------------- #
        self.sub_hot_spot_temperature = self.create_subscription(
            Float32, "/hot_spot_temperature", self.hot_spot_temperature_callback, 10
        )
        self.sub_hot_spot_temperature  # prevent unused variable warning

        # ------------------------------ Hot spot pixel ------------------------------ #
        self.sub_hot_spot_pixel = self.create_subscription(
            Int32MultiArray,
            "/hot_spot_temperature_pos",
            self.hot_spot_pixel_callback,
            10,
        )
        self.sub_hot_spot_pixel  # prevent unused variable warning
        self.hot_spot_pixel_x = 0
        self.hot_spot_pixel_y = 0

        # ---------------------------------- 影像視窗名稱 ---------------------------------- #
        self.window_name = "Thermal Image"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.coner_points = []

    def mouse_callback(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.coner_points.__len__() >= 4:
                print("You have already selected all the coner points.")
                print(self.coner_points)
                return
            print("Clicked at pixel coordinates (x={}, y={})".format(x, y))
            self.draw_point(x, y)
            self.coner_points.append((x, y))


    def draw_point(self, x, y) -> None:
        # 在指定位置畫一個紅色點
        if self.coner_points:
            # 先畫出所有的點
            for point in self.coner_points:
                cv2.circle(self.cv_thermal_image_copy, point, 2, ORANGE, -1)
            # 再連接點與點
            for i in range(len(self.coner_points) - 1):
                cv2.line(self.cv_thermal_image_copy, self.coner_points[i], self.coner_points[i+1], RED, 1)
            # 如果已經有四個點了，則將第一個點與最後一個點連接起來，形成一個封閉的多邊形
            if len(self.coner_points) == 4:
                cv2.line(self.cv_thermal_image_copy, self.coner_points[0], self.coner_points[-1], RED, 1)

        print(f"draw point at ({x}, {y})")


    def hot_spot_pixel_callback(self, msg) -> None:
        data = msg.data
        self.hot_spot_pixel_x = data[0]
        self.hot_spot_pixel_y = data[1]

    def hot_spot_temperature_callback(self, msg) -> None:
        pass
        # print(msg.data)

    def thermal_image_callback(self, msg) -> None:
        try:
            # 將ROS圖像訊息轉換為OpenCV影像
            self.cv_thermal_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.cv_thermal_image_copy = self.cv_thermal_image.copy()
            copy_ = cv2.copyTo(self.cv_thermal_image, None)
            cv2.circle(self.cv_thermal_image, upper_left, 2, (0, 0, 255), -1)
            cv2.circle(self.cv_thermal_image, upper_right, 2, (0, 0, 255), -1)
            cv2.circle(self.cv_thermal_image, lower_right, 2, (0, 0, 255), -1)
            cv2.circle(self.cv_thermal_image, lower_left, 2, (0, 0, 255), -1)
            cv2.line(self.cv_thermal_image, upper_left, upper_right, BLUE, 1)
            cv2.line(self.cv_thermal_image, upper_left, lower_left, BLUE, 1)
            cv2.line(self.cv_thermal_image, lower_right, upper_right, BLUE, 1)
            cv2.line(self.cv_thermal_image, lower_right, lower_left, BLUE, 1)

            cv2.circle(self.cv_thermal_image, (self.hot_spot_pixel_x, self.hot_spot_pixel_y), 5, ORANGE, -1)
            cv2.imshow(self.window_name, self.cv_thermal_image)
            cv2.imshow("Copy", self.cv_thermal_image_copy)

            selected_area = np.array(
                [upper_left, upper_right, lower_right, lower_left], dtype=np.float32
            )

            x = 29.7 *6
            y = 21.0 *6
            o = 300
            target_area = np.array(
                [[o, o], [o+x, o], [o+x, o+y], [o, o+y]], dtype=np.float32
            )
            h, _ = cv2.findHomography(selected_area, target_area)
            corrected_area = cv2.warpPerspective(self.cv_thermal_image, h, (1000, 800))
            
            point = np.array(
                [self.hot_spot_pixel_x, self.hot_spot_pixel_y], dtype=np.float32
            )
            # 將點進行轉換
            corrected_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), h)

            # 提取轉換後的座標
            corrected_x, corrected_y = corrected_point[0][0]

            # print(f"Corrected point: ({corrected_x:.2f}, {corrected_y:.2f})")
            cv2.circle(
                corrected_area,
                (int(corrected_x), int(corrected_y)),
                5,
                PURPLE,
                -1,
            )
            cv2.putText(
                corrected_area,
                f"Corrected point: ({(corrected_x-o)*2:.2f}, {(corrected_y-o)*2:.2f})",
                (int(corrected_x + 5), int(corrected_y - 5)),
                cv2.FONT_HERSHEY_DUPLEX,
                0.5,
                PURPLE,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                corrected_area,
                f"Origin",
                (int(o + 8), int(o - 8)),
                cv2.FONT_HERSHEY_DUPLEX,
                0.6,
                BLUE,
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Corrected Area", corrected_area)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error("Error processing the image: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    thermal_camera2world = Thermal_camera_to_world()
    rclpy.spin(thermal_camera2world)

    thermal_camera2world.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
