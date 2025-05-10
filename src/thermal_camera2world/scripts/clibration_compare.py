#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

'''
this node will publish the undistorted image to the topic
'''


class clibration_compare(Node):
    def __init__(self):
        super().__init__("clibration_compare")

        self.current_namespace = self.get_namespace()  # 設為類別屬性
        self.get_logger().info(f"Current namespace: {self.current_namespace}")

        self.subscription = self.create_subscription(
            Image,
            'thermal_image',
            # '/thermal_DS4025FT/thermal_image',
            # '/coin417rg2_thermal/thermal_image',
            self.image_callback,
            10,
        )

        self.clibration_img = self.create_publisher(
            Image,
            'clibration_image', 
            10
        )

        self.bridge = CvBridge()

        self.camera_matrix = np.array(
            [
                [661.65174598,   0.        , 244.59833055],
                [  0.        , 663.98307256, 177.34716411],
                [  0.        ,   0.        ,   1.        ]
            ]
        )

        self.dist_coeffs = np.array(
            [
                [
                    -5.38873919e-01,  
                    9.94539455e-01, 
                    -3.06958892e-03,  
                    6.97459193e-04, 
                    -3.22030148e+00
                ]
            ]
        )

        device = "ipt430m"
        num_exp = 1
        self.save_path = os.path.expanduser(
            f"~/calibration_data/{device}/exp{str(num_exp)}/calibraion_result"
        )

    def image_callback(self, msg):
        """處理接收到的影像，並執行標定"""


        

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            

            undistorted_image = cv2.undistort(
                cv_image, self.camera_matrix, self.dist_coeffs
            )

            msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.clibration_img.publish(msg)


            # h, w = cv_image.shape[:2]
            # scale = 1
            # scaled_size = (w * scale, h * scale)

            # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            #     self.camera_matrix, self.dist_coeffs, scaled_size, 1, scaled_size
            # )

            # # 🧠 調整主點（cx, cy）到新圖像的中心
            # # newcameramtx[0, 2] = scaled_size[0] / 2
            # # newcameramtx[1, 2] = scaled_size[1] / 2

            # undistorted_image = cv2.undistort(
            #     cv_image, self.camera_matrix, self.dist_coeffs
            # )

            # # undistort
            # mapx, mapy = cv2.initUndistortRectifyMap(
            #     self.camera_matrix,
            #     self.dist_coeffs,
            #     None,
            #     newcameramtx,
            #     scaled_size,
            #     cv2.CV_32FC1,
            # )
            # dst = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)

            # # crop the image
            # # x, y, w, h = roi
            # # dst = dst[y:y+h, x:x+w]

            # cv2.imshow("undistorted_image_withBlack", dst)
            # cv2.imshow("origin_image", cv_image)
            # cv2.imshow("undistorted_image", undistorted_image)

            # key = cv2.waitKey(1) & 0xFF

            # if key == ord("p"):

            #     if cv2.imwrite(
            #         os.path.join(self.save_path, f"origin_image.jpg"), cv_image
            #     ):
            #         print(f"✅ 原始影像已儲存成功")
            #     if cv2.imwrite(
            #         os.path.join(self.save_path, f"undistorted_image.jpg"),
            #         undistorted_image,
            #     ):
            #         print(f"✅ 去畸變影像已儲存成功")
            #     if cv2.imwrite(
            #         os.path.join(self.save_path, f"undistorted_image_withBlack.jpg"),
            #         dst,
            #     ):
            #         print(f"✅ 去畸變影像(黑邊)已儲存成功")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = clibration_compare()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
