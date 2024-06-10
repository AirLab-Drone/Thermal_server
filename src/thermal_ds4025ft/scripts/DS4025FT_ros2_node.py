#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# from package name.module import class
from thermal_ds4025ft.thermal_ds4025ft import Thermal_DS4025FT



class DS4025FT_ros2_node(Node):

    def __init__(self, vcap):
        super().__init__('thermal_camera_node')
        self.bridge = CvBridge()
        self.vcap = vcap
        
        self.image_pub = self.create_publisher(Image, 'thermal_image', 10)
    
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.image_callback)

    def image_callback(self):
        ret, frame = self.vcap.read()
        if ret:
            # frame = cv2.resize(frame, (640, 480))
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.get_logger().info('Publishing: Thermal Image')
            self.image_pub.publish(img_msg)
        else:
            print('[Error] Failed to read frame from camera')

def main(args=None):
    vcap  = cv2.VideoCapture('rtsp://admin:admin@192.168.1.108/cam/realmonitor?channel=2&subtype=0')
    if vcap.isOpened():
        print(f'[Info] VideoCapture is opened')

        rclpy.init(args=args)
        node = DS4025FT_ros2_node(vcap)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # main()
    account = "admin"
    password = "admin"
    ip_address = "192.168.1.108"

    thermal_camera = Thermal_DS4025FT(account, password, ip_address)

    print("done")
