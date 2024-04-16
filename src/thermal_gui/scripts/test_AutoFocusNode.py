#!/usr/bin/env python3

import sys
import cv2

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from thermal_msgs.srv import AutoFocus


# --------------------------------- 發送自動對焦請求 --------------------------------- #
# ros2 service call /auto_focus thermal_msgs/srv/AutoFocus "{auto_focus: 'auto focus'}"



# --------------------------------- Thermal UGI node -------------------------------- #
class ThermalUGI(Node):
    def __init__(self):
        super().__init__('thermal_ugi')

        self.sub_thermal_image = self.create_subscription(
            Image, 
            '/thermal_image', 
            self.thermal_image_callback, 10
        )
        self.sub_thermal_image  # prevent unused variable warning

        self.cv_bridge = CvBridge()

        self.sub_hot_spot_temperature = self.create_subscription(
            Float32,
            '/hot_spot_temperature',
            self.hot_spot_temperature_callback, 10
        )
        self.sub_hot_spot_temperature   # prevent unused variable warning


        # 發送自動對焦請求
        self.auto_focus_client = self.create_client(
            AutoFocus, 
            '/auto_focus'
        )
        while not self.auto_focus_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AutoFocus.Request()


    def thermal_image_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            # 將ROS圖像訊息轉換為OpenCV影像
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # 在這裡你可以對cv_image進行任何你想做的處理，比如顯示、儲存、或者進行電腦視覺處理
            # 這裡只是簡單地顯示影像的寬高
            height, width, _ = cv_image.shape
            self.get_logger().info('Image height: {}, width: {}'.format(height, width))
        except Exception as e:
            self.get_logger().error('Error processing the image: {}'.format(e))


    def hot_spot_temperature_callback(self, msg):
        pass


    def auto_focus(self):
        self.req.auto_focus = 'auto focus'
        future = self.auto_focus_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# -------------------------------- Thermal GUI ------------------------------- #
class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('PyQt視窗')
        layout = QVBoxLayout()

        # 添加圖片
        pixmap = QPixmap('/home/yuan/server_ws/src/thermal_gui/scripts/test.jpg')  # 請替換為您的圖片路徑
        image_label = QLabel(self)
        image_label.setPixmap(pixmap)
        layout.addWidget(image_label)

        # 添加按鈕
        self.button = QPushButton('按鈕')
        layout.addWidget(self.button)

        self.setLayout(layout)

        # 設定固定視窗大小
        self.setFixedSize(1280, 720)    



def main(args=None):
    rclpy.init(args=args)
    thermal_gui = ThermalUGI()
    rclpy.spin(thermal_gui)
    thermal_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

