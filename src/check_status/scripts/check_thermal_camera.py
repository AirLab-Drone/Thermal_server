#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import requests
import cv2


from check_status_py.error_code import *
from check_status_py.tools import post_to_server, check_port_exists, ros2_time_to_taiwan_timezone, print_json



class check_thermal_camera(Node):
    def __init__(self):
        super().__init__('check_thermal_camera')

        # ---------------------------------- IPT430M --------------------------------- #
        self.ipt430m_thermal_img_subscription = self.create_subscription(
            Image,
            '/thermal_IPT430M/thermal_image',
            self.ipt430m_thermal_img_callback,
            10)

        self.ipt430m_hot_spot_temp_subscription = self.create_subscription(
            Float32,
            '/thermal_IPT430M/hot_spot_temperature',
            self.ipt430m_hot_spot_callback,
            10)
        
        self.ipt430m_thermal_img = None
        self.ipt430m_thermal_hot_spot_temp = None

        # --------------------------------- DS4025FT --------------------------------- #
        self.ds4025ft_thermal_img_subscription = self.create_subscription(
            Image,
            '/thermal_DS4025FT/thermal_image',
            self.ds4025ft_thermal_img_callback,
            10)
        
        self.ds4025ft_hot_spot_temp_subscription = self.create_subscription(
            Float32,
            '/thermal_DS4025FT/hot_spot_temperature',
            self.ds4025ft_hot_spot_callback,
            10)
        
        self.ds4025ft_thermal_img = None
        self.ds4025ft_thermal_hot_spot_temp = None
    


        # -------------------------------- send server ------------------------------- #

        # 設定時間間隔，檢查是否接收到新的數據
        self.data_timeout_sec = 5.0  # 設置超時時間
        self.last_ipt430m_update = self.get_clock().now()
        self.last_ds4025ft_update = self.get_clock().now()

        self.ipt430m_status_dict = {
            "upload_time": None,
            "source": "ipt430m",
            "hot_spot_temp": None,
            "thermal_img": None,
            "error_code": set()
        }        
        
        self.ds4025ft_status_dict = {
            "upload_time": None,
            "source": "ds4025ft",
            "hot_spot_temp": None,
            "thermal_img": None,
            "error_code": set()
        }

        
        self.server_url = "http://127.0.0.1:5000/upload_image"  # Flask 伺服器的 URL

        self.send_server_timmer = self.create_timer(0.5, self.send_server_callback)


        self.bridge = CvBridge()



    def ipt430m_thermal_img_callback(self, msg):
        self.ipt430m_thermal_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_ipt430m_update = self.get_clock().now()

    def ipt430m_hot_spot_callback(self, msg):
        self.ipt430m_thermal_hot_spot_temp = msg.data
        self.last_ipt430m_update = self.get_clock().now()

    def ds4025ft_thermal_img_callback(self, msg):
        self.ds4025ft_thermal_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_ds4025ft_update = self.get_clock().now()

    def ds4025ft_hot_spot_callback(self, msg):
        self.ds4025ft_thermal_hot_spot_temp = msg.data
        self.last_ds4025ft_update = self.get_clock().now()





    def send_server_callback(self):
        try:
            current_time = self.get_clock().now()

            self.ipt430m_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time).isoformat()
            self.ds4025ft_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time).isoformat()

            # 檢查 IPT430M 是否超時
            if (current_time - self.last_ipt430m_update).nanoseconds / 1e9 > self.data_timeout_sec:
                self.get_logger().error("IPT430M data timeout")
                self.ipt430m_status_dict.update({
                    "hot_spot_temp": None,
                    "thermal_img": False,
                    "error_code": [ERROR_CODE.THERMAL_IMG_ERROR, ERROR_CODE.THERMAL_HOT_SPOT_TEMP_ERROR]
                })
                self.ipt430m_thermal_img = None
                self.ipt430m_thermal_hot_spot_temp = None
            else:
                self.ipt430m_status_dict.update({
                    "hot_spot_temp": self.ipt430m_thermal_hot_spot_temp,
                    "thermal_img": True,
                    "error_code": []
                })

            # 檢查 DS4025FT 是否超時
            if (current_time - self.last_ds4025ft_update).nanoseconds / 1e9 > self.data_timeout_sec:
                self.get_logger().error("DS4025FT data timeout")
                self.ds4025ft_status_dict.update({
                    "hot_spot_temp": None,
                    "thermal_img": False,
                    "error_code": [ERROR_CODE.THERMAL_IMG_ERROR, ERROR_CODE.THERMAL_HOT_SPOT_TEMP_ERROR]
                })
                self.ds4025ft_thermal_img = None
                self.ds4025ft_thermal_hot_spot_temp = None
            else:
                self.ds4025ft_status_dict.update({
                    "hot_spot_temp": self.ds4025ft_thermal_hot_spot_temp,
                    "thermal_img": True,
                    "error_code": []
                })

            # 確認影像是否為 None，避免處理 None 導致的錯誤
            if self.ds4025ft_thermal_img is not None:
                resized_ds4025ft_img = self.resize_with_aspect_ratio(self.ds4025ft_thermal_img, 720, 720)
                _, ds4025ft_buffer = cv2.imencode('.jpg', resized_ds4025ft_img)
                ds4025ft_jpg_as_text = ds4025ft_buffer.tobytes()
                files_ds4025ft = {
                    "file": ("ds4025ft_thermal_image.jpg", ds4025ft_jpg_as_text, "image/jpeg")
                }
            else:
                files_ds4025ft = {}

            if self.ipt430m_thermal_img is not None:
                resized_ipt430m_img = self.resize_with_aspect_ratio(self.ipt430m_thermal_img, 720, 720)
                _, ipt430m_buffer = cv2.imencode('.jpg', resized_ipt430m_img)
                ipt430m_jpg_as_text = ipt430m_buffer.tobytes()
                files_ipt430m = {
                    "file": ("ipt430m_thermal_image.jpg", ipt430m_jpg_as_text, "image/jpeg")
                }
            else:
                files_ipt430m = {}


            # 發送 DS4025FT 數據
            response_ds4025ft = requests.post(
                self.server_url,
                files=files_ds4025ft,
                data={**self.ds4025ft_status_dict}
            )
            if response_ds4025ft.status_code == 200:
                self.get_logger().info("DS4025FT image and data uploaded successfully")
                print_json(self.ds4025ft_status_dict)
            else:
                self.get_logger().error(f"Failed to upload DS4025FT: {response_ds4025ft.status_code}")

            # 發送 IPT430M 數據
            response_ipt430m = requests.post(
                self.server_url,
                files=files_ipt430m,
                data={**self.ipt430m_status_dict}
            )
            if response_ipt430m.status_code == 200:
                self.get_logger().info("IPT430M image and data uploaded successfully")
                print_json(self.ipt430m_status_dict)
            else:
                self.get_logger().error(f"Failed to upload IPT430M: {response_ipt430m.status_code}")


        except Exception as e:
            self.get_logger().error(f"Error in send_server_callback: {e}")





    def resize_with_aspect_ratio(self, image, target_width, target_height, color=(255, 255, 255)):
        """
        將圖片縮放到指定大小，保持比例，並使用指定顏色填充剩餘部分。

        :param image: 輸入圖片
        :param target_width: 目標寬度
        :param target_height: 目標高度
        :param color: 填充的顏色 (B, G, R)
        :return: 調整後的圖片
        """
        h, w = image.shape[:2]

        # 計算縮放比例
        scale = min(target_width / w, target_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        # 縮放圖片
        resized_image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # 創建新的圖像並填充背景色
        top = (target_height - new_h) // 2
        bottom = target_height - new_h - top
        left = (target_width - new_w) // 2
        right = target_width - new_w - left
        padded_image = cv2.copyMakeBorder(
            resized_image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
        )

        return padded_image





def main(args=None):
    rclpy.init(args=args)
    node = check_thermal_camera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # IPT430M Status:
    # {
    #     "upload_time": "2024-12-10T23:20:25.215824+08:00",
    #     "hot_spot_temp": 20.700000762939453,
    #     "thermal_img": true,
    #     "error_code": []
    # }
    # DS4025FT Status:
    # {
    #     "upload_time": "2024-12-10T23:20:25.215824+08:00",
    #     "hot_spot_temp": 23.299999237060547,
    #     "thermal_img": true,
    #     "error_code": []
    # }
