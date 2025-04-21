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
            '/thermal_DS4025FT/thermal_image', 
            # '/coin417rg2_thermal/thermal_image',
            self.image_callback, 
            10
        )

        self.bridge = CvBridge()

        self.camera_matrix = np.array(
[[1.90154620e+03, 0.00000000e+00, 4.04211204e+02],
 [0.00000000e+00, 1.86988014e+03, 1.00406185e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        )

        self.dist_coeffs = np.array(
[[ 2.08244236e+00, -1.40233268e+01,  2.18963499e-03,  2.07644024e-01,
   7.21430708e+01]]
        )

        device = 'ds4025ft'
        num_exp = 3
        self.save_path = os.path.expanduser(f'~/calibration_data/{device}/exp{str(num_exp)}/calibraion_result')


    def image_callback(self, msg):
        """處理接收到的影像，並執行標定"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            h, w = cv_image.shape[:2]
            scale = 1
            scaled_size = (w * scale, h * scale)


            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, scaled_size, 1, scaled_size)


            # 🧠 調整主點（cx, cy）到新圖像的中心
            # newcameramtx[0, 2] = scaled_size[0] / 2
            # newcameramtx[1, 2] = scaled_size[1] / 2

            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            # undistort
            mapx, mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, newcameramtx, scaled_size, cv2.CV_32FC1)
            dst = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)



            # crop the image
            # x, y, w, h = roi
            # dst = dst[y:y+h, x:x+w]

            cv2.imshow('undistorted_image_withBlack', dst)
            cv2.imshow("origin_image", cv_image)
            cv2.imshow("undistorted_image", undistorted_image)

            key = cv2.waitKey(1) & 0xFF


            if key == ord('p'):
                
                if cv2.imwrite(os.path.join(self.save_path, f'origin_image.jpg'), cv_image):
                    print(f"✅ 原始影像已儲存成功")
                if cv2.imwrite(os.path.join(self.save_path, f'undistorted_image.jpg'), undistorted_image):
                    print(f"✅ 去畸變影像已儲存成功")
                if cv2.imwrite(os.path.join(self.save_path, f'undistorted_image_withBlack.jpg'), dst):
                    print(f"✅ 去畸變影像(黑邊)已儲存成功")



        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = clibration_compare()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
