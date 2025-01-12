#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import requests
import cv2
import base64
from datetime import datetime, timezone, time


from check_status_py.error_code import *
from check_status_py.tools import (
    post_to_server,
    ros2_time_to_taiwan_timezone,
    print_json,
)


"""
ros2 bag play ../Desktop/rosbag2_2024_12_13-21_49_32/ --loop --read-ahead-queue-size 5000
"""



# TODO: RTSP 串流
class check_thermal_camera(Node):
    def __init__(self):
        super().__init__("check_thermal_camera")

        # ---------------------------------- IPT430M --------------------------------- #
        self.ipt430m_thermal_img_subscription = self.create_subscription(
            Image,
            "/thermal_IPT430M/thermal_image",
            self.ipt430m_thermal_img_callback,
            10,
        )

        self.ipt430m_hot_spot_temp_subscription = self.create_subscription(
            Float32,
            "/thermal_IPT430M/hot_spot_temperature",
            self.ipt430m_hot_spot_callback,
            10,
        )

        self.ipt430m_thermal_img = None
        self.ipt430m_thermal_hot_spot_temp = None

        # --------------------------------- DS4025FT --------------------------------- #
        self.ds4025ft_thermal_img_subscription = self.create_subscription(
            Image,
            "/thermal_DS4025FT/thermal_image",
            self.ds4025ft_thermal_img_callback,
            10,
        )

        self.ds4025ft_hot_spot_temp_subscription = self.create_subscription(
            Float32,
            "/thermal_DS4025FT/hot_spot_temperature",
            self.ds4025ft_hot_spot_callback,
            10,
        )

        self.ds4025ft_thermal_img = None
        self.ds4025ft_thermal_hot_spot_temp = None

        # todo:用RTSP發送及時畫面


        # -------------------------------- send server ------------------------------- #
        # 設定時間間隔，檢查是否接收到新的數據
        self.data_timeout_sec = 5.0  # 設置超時時間
        self.last_ipt430m_update = self.get_clock().now()
        self.last_ds4025ft_update = self.get_clock().now()

        self.bridge = CvBridge()

        self.ipt430m_status_dict = {
            "upload_time": None,
            "source": "ipt430m",
            "hot_spot_temp": None,
            "thermal_img": None,
            "error_code": set(),
        }

        self.ds4025ft_status_dict = {
            "upload_time": None,
            "source": "ds4025ft",
            "hot_spot_temp": None,
            "thermal_img": None,
            "error_code": set(),
        }

        self.server_url = (
            "http://127.0.0.1:5000/upload/ThermalCameraStatus"  # Flask 伺服器的 URL
        )

        self.server_thermal_stream_url = (
            "http://127.0.0.1:5000/upload/ThermalCameraStream/"
        )

        # 設定每小時發送一次
        # self.target_times = [time(hour, 0) for hour in range(1, 24)]
        # 每分鐘發送 (Demo 用)
        self.target_times = [
            time(hour, minute) for hour in range(0, 24) for minute in range(0, 60)
        ]

        self.send_ipt430m_thermal_stream_timmer = self.create_timer(
            0.1, self.send_ipt430m_thermal_stream_callback
        )
        self.send_ds4025ft_thermal_stream_timmer = self.create_timer(
            0.1, self.send_ds4025ft_thermal_stream_callback
        )

        self.timer = self.create_timer(60, self.check_and_upload_at_target_times)

        # self.send_ipt430m_server_timmer = self.create_timer(0.5, self.send_ipt430m_server_callback)
        # self.send_ds4025ft_server_timmer = self.create_timer(0.5, self.send_ds4025ft_server_callback)

    def ipt430m_thermal_img_callback(self, msg):
        self.ipt430m_thermal_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
        self.last_ipt430m_update = self.get_clock().now()

    def ipt430m_hot_spot_callback(self, msg):
        self.ipt430m_thermal_hot_spot_temp = msg.data
        self.last_ipt430m_update = self.get_clock().now()

    def ds4025ft_thermal_img_callback(self, msg):
        self.ds4025ft_thermal_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
        self.last_ds4025ft_update = self.get_clock().now()

    def ds4025ft_hot_spot_callback(self, msg):
        self.ds4025ft_thermal_hot_spot_temp = msg.data
        self.last_ds4025ft_update = self.get_clock().now()

    def send_ipt430m_thermal_stream_callback(self):

        current_time = self.get_clock().now()

        ipt430m_encoded_img = None

        if (
            current_time - self.last_ipt430m_update
        ).nanoseconds / 1e9 > self.data_timeout_sec:
            self.get_logger().error("IPT430M data timeout")
            self.ipt430m_thermal_img = None
            self.ipt430m_thermal_hot_spot_temp = None
        else:
            resized_ipt430m_img = self.resize_with_aspect_ratio(
                self.ipt430m_thermal_img, 720, 720
            )
            _, ipt430m_buffer = cv2.imencode(".jpg", resized_ipt430m_img)
            ipt430m_encoded_img = base64.b64encode(ipt430m_buffer).decode("utf-8")

        try:
            response_ipt430m = post_to_server(
                url=self.server_thermal_stream_url + "ipt430m",
                json={
                    "thermal_img": ipt430m_encoded_img,
                    "hot_spot_temp": self.ipt430m_thermal_hot_spot_temp,
                },
            )

            if response_ipt430m.status_code == 200:
                self.get_logger().info("IPT430M thermal image uploaded successfully")
            else:
                self.get_logger().error(
                    f"Failed to upload IPT430M thermal image: {response_ipt430m.status_code}"
                )
        except Exception as e:
            self.get_logger().error(f"Error in send_thermal_stream_callback: {e}")

    def send_ds4025ft_thermal_stream_callback(self):
        current_time = self.get_clock().now()

        ds4025ft_encoded_img = None

        if (
            current_time - self.last_ds4025ft_update
        ).nanoseconds / 1e9 > self.data_timeout_sec:
            self.get_logger().error("DS4025FT data timeout")
            self.ds4025ft_thermal_img = None
            self.ds4025ft_thermal_hot_spot_temp = None
        else:
            resized_ds4025ft_img = self.resize_with_aspect_ratio(
                self.ds4025ft_thermal_img, 480, 480
            )
            _, ds4025ft_buffer = cv2.imencode(".jpg", resized_ds4025ft_img)
            ds4025ft_encoded_img = base64.b64encode(ds4025ft_buffer).decode("utf-8")

        try:
            response_ds4025ft = post_to_server(
                url=self.server_thermal_stream_url + "ds4025ft",
                json={
                    "thermal_img": ds4025ft_encoded_img,
                    "hot_spot_temp": self.ds4025ft_thermal_hot_spot_temp,
                },
            )

            if response_ds4025ft.status_code == 200:
                self.get_logger().info("DS4025FT thermal image uploaded successfully")
            else:
                self.get_logger().error(
                    f"Failed to upload DS4025FT thermal image: {response_ds4025ft.status_code}"
                )
        except Exception as e:
            self.get_logger().error(f"Error in send_thermal_stream_callback: {e}")

    def check_and_upload_at_target_times(self):
        now = datetime.now().time()
        if any(
            now.hour == t.hour and now.minute == t.minute for t in self.target_times
        ):
            self.upload_server()

    def upload_server(self):
        # 上傳 IPT430M 數據
        self.process_and_upload(
            source="ipt430m",
            last_update=self.last_ipt430m_update,
            hot_spot_temp=self.ipt430m_thermal_hot_spot_temp,
            status_dict=self.ipt430m_status_dict,
        )

        # 上傳 DS4025FT 數據
        self.process_and_upload(
            source="ds4025ft",
            last_update=self.last_ds4025ft_update,
            hot_spot_temp=self.ds4025ft_thermal_hot_spot_temp,
            status_dict=self.ds4025ft_status_dict,
        )

    def process_and_upload(self, source, last_update, hot_spot_temp, status_dict):
        """
        處理並上傳熱成像數據的通用函式

        :param source: 資料來源（ipt430m / ds4025ft）
        :param last_update: 最近一次數據更新時間
        :param img: 熱成像圖片
        :param hot_spot_temp: 熱點溫度
        :param status_dict: 狀態字典
        :param target_size: 影像目標尺寸 (寬, 高)
        """
        current_time = self.get_clock().now()

        # 設置上傳時間
        status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)

        # 檢查是否超時
        if (current_time - last_update).nanoseconds / 1e9 > self.data_timeout_sec:
            self.get_logger().error(f"{source.upper()} data timeout")
            status_dict.update(
                {
                    "hot_spot_temp": None,
                    "thermal_img": False,
                    "error_code": [
                        ERROR_CODE.THERMAL_IMG_ERROR,
                        ERROR_CODE.THERMAL_HOT_SPOT_TEMP_ERROR,
                    ],
                }
            )
        else:
            status_dict.update(
                {"hot_spot_temp": hot_spot_temp, "thermal_img": True, "error_code": []}
            )

        # 將 error_code 轉為 list 確保 JSON 格式
        status_dict["error_code"] = list(status_dict["error_code"])

        # 發送數據到伺服器
        try:
            print_json(status_dict)
            response = post_to_server(url=self.server_url, json=status_dict)

            if response.status_code == 200:
                self.get_logger().info(
                    f"{source.upper()} image and data uploaded successfully"
                )
            else:
                self.get_logger().error(
                    f"Failed to upload {source.upper()}: {response.status_code}"
                )
        except Exception as e:
            self.get_logger().error(f"Error uploading {source.upper()}: {e}")

    def resize_with_aspect_ratio(
        self, image, target_width, target_height, color=(255, 255, 255)
    ):
        """
        將圖片縮放到指定大小，保持比例，並使用指定顏色填充剩餘部分。

        :param image: 輸入圖片
        :param target_width: 目標寬度
        :param target_height: 目標高度
        :param color: 填充的顏色 (B, G, R)
        :return: 調整後的圖片
        """
        if image is not None:

            h, w = image.shape[:2]

            # 計算縮放比例
            scale = min(target_width / w, target_height / h)
            new_w = int(w * scale)
            new_h = int(h * scale)

            # 縮放圖片
            resized_image = cv2.resize(
                image, (new_w, new_h), interpolation=cv2.INTER_AREA
            )

            # 創建新的圖像並填充背景色
            top = (target_height - new_h) // 2
            bottom = target_height - new_h - top
            left = (target_width - new_w) // 2
            right = target_width - new_w - left
            padded_image = cv2.copyMakeBorder(
                resized_image,
                top,
                bottom,
                left,
                right,
                cv2.BORDER_CONSTANT,
                value=color,
            )

            return padded_image
        


    # ------------------------------------ 用不到 ----------------------------------- #

    # def send_ipt430m_server_callback(self):

    #     current_time = self.get_clock().now()

    #     self.ipt430m_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(
    #         current_time
    #     )

    #     # 檢查 IPT430M 是否超時
    #     if (
    #         current_time - self.last_ipt430m_update
    #     ).nanoseconds / 1e9 > self.data_timeout_sec:
    #         self.get_logger().error("IPT430M data timeout")
    #         self.ipt430m_status_dict.update(
    #             {
    #                 "hot_spot_temp": None,
    #                 "thermal_img": False,
    #                 "error_code": [
    #                     ERROR_CODE.THERMAL_IMG_ERROR,
    #                     ERROR_CODE.THERMAL_HOT_SPOT_TEMP_ERROR,
    #                 ],
    #             }
    #         )
    #         self.ipt430m_thermal_img = None
    #         self.ipt430m_thermal_hot_spot_temp = None
    #     else:
    #         self.ipt430m_status_dict.update(
    #             {"hot_spot_temp": self.ipt430m_thermal_hot_spot_temp, "error_code": []}
    #         )

    #     # 準備 IPT430M 數據
    #     if self.ipt430m_thermal_img is not None:
    #         resized_ipt430m_img = self.resize_with_aspect_ratio(
    #             self.ipt430m_thermal_img, 720, 720
    #         )
    #         _, ipt430m_buffer = cv2.imencode(".jpg", resized_ipt430m_img)
    #         ipt430m_encoded_img = base64.b64encode(ipt430m_buffer).decode("utf-8")
    #         self.ipt430m_status_dict["thermal_img"] = ipt430m_encoded_img
    #     else:
    #         self.ipt430m_status_dict["thermal_img"] = None

    #     # 發送 IPT430M 數據
    #     try:
    #         print_json(self.ipt430m_status_dict, exclude_key="thermal_img")
    #         response_ipt430m = post_to_server(
    #             url=self.server_url, json=self.ipt430m_status_dict
    #         )

    #         if response_ipt430m.status_code == 200:
    #             self.get_logger().info("IPT430M image and data uploaded successfully")
    #         else:
    #             self.get_logger().error(
    #                 f"Failed to upload IPT430M: {response_ipt430m.status_code}"
    #             )
    #     except Exception as e:
    #         self.get_logger().error(f"Error in send_server_callback: {e}")

    # def send_ds4025ft_server_callback(self):

    #     current_time = self.get_clock().now()

    #     self.ds4025ft_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(
    #         current_time
    #     )

    #     # 檢查 DS4025FT 是否超時
    #     if (
    #         current_time - self.last_ds4025ft_update
    #     ).nanoseconds / 1e9 > self.data_timeout_sec:
    #         self.get_logger().error("DS4025FT data timeout")
    #         self.ds4025ft_status_dict.update(
    #             {
    #                 "hot_spot_temp": None,
    #                 "thermal_img": False,
    #                 "error_code": [
    #                     ERROR_CODE.THERMAL_IMG_ERROR,
    #                     ERROR_CODE.THERMAL_HOT_SPOT_TEMP_ERROR,
    #                 ],
    #             }
    #         )
    #         self.ds4025ft_thermal_img = None
    #         self.ds4025ft_thermal_hot_spot_temp = None
    #     else:
    #         self.ds4025ft_status_dict.update(
    #             {"hot_spot_temp": self.ds4025ft_thermal_hot_spot_temp, "error_code": []}
    #         )

    #     # 準備 DS4025FT 數據
    #     if self.ds4025ft_thermal_img is not None:
    #         resized_ds4025ft_img = self.resize_with_aspect_ratio(
    #             self.ds4025ft_thermal_img, 480, 480
    #         )
    #         _, ds4025ft_buffer = cv2.imencode(".jpg", resized_ds4025ft_img)
    #         ds4025ft_encoded_img = base64.b64encode(ds4025ft_buffer).decode("utf-8")
    #         self.ds4025ft_status_dict["thermal_img"] = ds4025ft_encoded_img
    #     else:
    #         self.ds4025ft_status_dict["thermal_img"] = None

    #     self.ds4025ft_status_dict["error_code"] = list(
    #         self.ds4025ft_status_dict["error_code"]
    #     )
    #     self.ipt430m_status_dict["error_code"] = list(
    #         self.ipt430m_status_dict["error_code"]
    #     )

    #     print_json(self.ds4025ft_status_dict, exclude_key="thermal_img")
    #     # 發送 DS4025FT 數據
    #     try:
    #         response_ds4025ft = post_to_server(
    #             url=self.server_url, json=self.ds4025ft_status_dict  # 發送 JSON
    #         )
    #         if response_ds4025ft.status_code == 200:
    #             self.get_logger().info("DS4025FT image and data uploaded successfully")
    #         else:
    #             self.get_logger().error(
    #                 f"Failed to upload DS4025FT: {response_ds4025ft.status_code}"
    #             )

    #     except Exception as e:
    #         self.get_logger().error(f"Error in send_server_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = check_thermal_camera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    # [INFO] [1734087299.131832361] [check_thermal_camera]: DS4025FT image and data uploaded successfully
    # {
    #     "upload_time": "2024-12-13T18:54:59.117609+08:00",
    #     "source": "ds4025ft",
    #     "hot_spot_temp": 25.299999237060547,
    #     "thermal_img": true,
    #     "error_code": []
    # }
    # [INFO] [1734087299.139642283] [check_thermal_camera]: IPT430M image and data uploaded successfully
    # {
    #     "upload_time": "2024-12-13T18:54:59.117609+08:00",
    #     "source": "ipt430m",
    #     "hot_spot_temp": 14.300000190734863,
    #     "thermal_img": true,
    #     "error_code": []
    # }
