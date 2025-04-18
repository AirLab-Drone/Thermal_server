#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class clibration_compare(Node):
    def __init__(self):
        super().__init__('chessboard_detector')
        self.subscription = self.create_subscription(
            Image, 
            # '/thermal_IPT430M/thermal_image', 
            # '/thermal_DS4025FT/thermal_image', 
            '/coin417rg2_thermal/thermal_image',
            self.image_callback, 
            10
        )

        self.bridge = CvBridge()

        self.camera_matrix = np.array(
            [[539.02076073,   0.        , 192.92292919],
            [  0.        , 541.1118448 , 147.27743381],
            [  0.        ,   0.        ,   1.        ]]
        )

        self.dist_coeffs = np.array(
            [[-0.36509059,  0.27282213,  0.00089947, -0.00072984, -0.53693823]]
        )


    def image_callback(self, msg):
        """處理接收到的影像，並執行標定"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            cv2.imshow("origin_image", cv_image)
            cv2.imshow("undistorted_image", undistorted_image)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = clibration_compare()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
