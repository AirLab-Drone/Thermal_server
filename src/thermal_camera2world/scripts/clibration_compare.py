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
            Image, '/thermal_IPT430M/thermal_image', self.image_callback, 10)
        self.bridge = CvBridge()

        self.camera_matrix = np.array(
                [[658.88240558,   0.        , 242.4072513 ],
                [  0.        , 665.44992269, 177.90754703],
                [  0.        ,   0.        ,   1.        ]]
        )

        self.dist_coeffs = np.array(
            [[-4.17136521e-01, -6.40505148e-01,  4.40717899e-04,  5.57044934e-04, 3.36013273e+00]]
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
