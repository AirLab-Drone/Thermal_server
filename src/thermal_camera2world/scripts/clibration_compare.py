#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


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
            [[536.74326964,   0.        , 197.52180702],
            [  0.        , 540.84763833, 149.22759103],
            [  0.        ,   0.        ,   1.        ]]
        )

        self.dist_coeffs = np.array(
            [[-3.94827407e-01,  7.48477869e-01, -2.54931624e-03, -6.11382433e-04, -2.45142323e+00]]
        )

        device = 'coin417rg2'
        self.save_path = os.path.expanduser(f'~/calibration_data/{device}')


    def image_callback(self, msg):
        """處理接收到的影像，並執行標定"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            cv2.imshow("origin_image", cv_image)
            cv2.imshow("undistorted_image", undistorted_image)

            key = cv2.waitKey(1) & 0xFF


            if key == ord('p'):
                
                if cv2.imwrite(os.path.join(self.save_path, f'origin_image.jpg'), cv_image):
                    print(f"✅ 原始影像已儲存成功")
                if cv2.imwrite(os.path.join(self.save_path, f'undistorted_image.jpg'), undistorted_image):
                    print(f"✅ 去畸變影像已儲存成功")


        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = clibration_compare()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
