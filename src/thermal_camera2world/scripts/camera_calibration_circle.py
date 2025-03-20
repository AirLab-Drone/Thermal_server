#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')

        # 訂閱熱像儀影像
        self.subscription = self.create_subscription(
            Image, 
            '/thermal_IPT430M/thermal_image', 
            self.image_callback, 
            10
        )
        self.subscription  

        self.bridge = CvBridge()


        col = 3
        row = 11
        # 設定標定板尺寸（內部圓點數量）
        self.pattern_size = (col, row)  # 或者根據實際標定板調整
        self.image_size = None  # 影像尺寸
        self.stored_images = 0

        self.obj_points = []  # 3D 世界座標
        self.img_points = []  # 2D 影像座標
        # 準備 3D 世界座標點
        objp = []
        for c in range(self.pattern_size[0]):
            for r in range(self.pattern_size[1]):
                if (r%2 == 0):
                    objp.append([r, c*2, 0])
                else:
                    objp.append([r, c*2+1, 0])

        objp = np.array(objp, np.float32)
        index = np.lexsort((objp[:, 2], objp[:, 1], objp[:, 0]))
        ans = objp[index]

        self.objp = ans


        # self.detector = cv2.SimpleBlobDetector_create(self.params)

        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("Threshold", "Trackbars", 0, 255, self.on_trackbar)

        cv2.createTrackbar("Min Area", "Trackbars", 10, 5000, self.nothing)  # minArea
        cv2.createTrackbar("Max Area", "Trackbars", 5000, 5000, self.nothing)  # maxArea
        cv2.createTrackbar("Min Circularity", "Trackbars", 70, 100, self.nothing)  # minCircularity
        cv2.createTrackbar("Min Inertia", "Trackbars", 50, 100, self.nothing)  # minInertiaRatio
        cv2.createTrackbar("Min Convexity", "Trackbars", 70, 100, self.nothing)  # minConvexity



        # print(self.objp)

        self.get_logger().info("Camera Calibration Node Initialized. Waiting for images...")

    def nothing(self, x):
        pass

    def on_trackbar(self):
        pass

    def preprocess_image(self, image):

        threshold_value = cv2.getTrackbarPos("Threshold", "Trackbars")

        inverted = cv2.bitwise_not(image)
        gray = cv2.bitwise_not(inverted)

        gray_blurred = cv2.medianBlur(gray, 5)


        # _, binary = cv2.threshold(gray_blurred, threshold_value, 255, cv2.THRESH_BINARY)


        return gray_blurred
    

    def detect_circle_grid(self, image):
        """使用 findCirclesGrid 偵測交錯排列的圓形標定板"""


        min_area = max(cv2.getTrackbarPos("Min Area", "Trackbars"), 1)
        max_area = max(cv2.getTrackbarPos("Max Area", "Trackbars"), 1)
        min_circularity = cv2.getTrackbarPos("Min Circularity", "Trackbars") / 100.0
        min_inertia = cv2.getTrackbarPos("Min Inertia", "Trackbars") / 100.0
        min_convexity = cv2.getTrackbarPos("Min Convexity", "Trackbars") / 100.0


        # 設定 SimpleBlobDetector 參數
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = min_area  # 根據滑桿調整最小面積
        params.maxArea = max_area  # 根據滑桿調整最大面積
        params.filterByCircularity = True
        params.minCircularity = min_circularity  # 根據滑桿調整圓形要求
        params.filterByInertia = True
        params.minInertiaRatio = min_inertia  # 根據滑桿調整慣性比
        params.filterByConvexity = True
        params.minConvexity = min_convexity  # 根據滑桿調整凸度要求

        # 建立 SimpleBlobDetector
        detector = cv2.SimpleBlobDetector_create(params)

        ret, centers = cv2.findCirclesGrid(
            image, 
            self.pattern_size,
            None,
            flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
            blobDetector=detector
        )


        Chessboard_img = image.copy()

        key = cv2.waitKey(1) & 0xFF
        if ret:

            for i, center in enumerate(centers):
                x, y = center[0]  # 提取每個圓心的 x 和 y 座標
                cv2.putText(Chessboard_img, str(i + 1), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.6, (0, 255, 0), 2, cv2.LINE_AA) 
            cv2.drawChessboardCorners(Chessboard_img, self.pattern_size, centers, ret)



            if key == ord('p'):

                # 如果偵測到圓點，將它們加入 img_points
                self.img_points.append(centers)
                self.obj_points.append(self.objp)

                # 繪製出來的角點
                self.stored_images += 1
                self.get_logger().info(f"✅ 儲存了 {self.stored_images} 張影像")
                
                if len(self.obj_points) >= 10:
                    self.calibrate_camera()


        # 偵測 keypoints
        keypoints = detector.detect(image)


        output = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 255, 0),
                               cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        
        return output, keypoints, Chessboard_img


    def image_callback(self, msg):
        """處理接收到的影像，並執行標定"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            if self.image_size is None:
                self.image_size = (cv_image.shape[1], cv_image.shape[0])  # 記錄影像尺寸

            processed_image = self.preprocess_image(cv_image)

            # 嘗試標定板偵測
            Circle_Grid, keypoint, Chessboard_img= self.detect_circle_grid(processed_image)
            cv2.imshow("processed_image", processed_image)
            cv2.imshow("origin_image", cv_image)
            cv2.imshow("Detected Circle Grid", Circle_Grid)
            cv2.imshow("Chessboard_img", Chessboard_img)

            # print(len(keypoint))
            
            # keypoints_float = np.array([kp.pt for kp in keypoint], dtype=np.float32)


            # key = cv2.waitKey(1) & 0xFF
            # if key == ord('p'):
            #     if len(keypoints_float) == self.pattern_size[0] * self.pattern_size[1]:
            #         self.img_points.append(keypoints_float)
            #         self.obj_points.append(self.objp)
            #         self.stored_images += 1
            #         self.get_logger().info(f"✅ 儲存了 {self.stored_images} 張影像")
                    
            #         # 如果已儲存 20 張影像，開始標定
            #         if self.stored_images >= 10:
            #             self.calibrate_camera()

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")

    def calibrate_camera(self):
        """計算相機內參與畸變參數"""
        self.get_logger().info("🔹 開始相機標定...")

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.image_size, None, None
        )

        if ret:
            self.get_logger().info(f"✅ 標定成功！重投影誤差: {ret}")

            # 使用 np.array2string() 格式化矩陣，並顯示逗號
            camera_matrix_str = np.array2string(camera_matrix, separator=', ')
            dist_coeffs_str = np.array2string(dist_coeffs, separator=', ')

            # 印出矩陣與畸變係數
            self.get_logger().info(f"📌 相機內參矩陣:\n{camera_matrix_str}")
            self.get_logger().info(f"📌 畸變係數:\n{dist_coeffs_str}")

            # 儲存標定結果
            # np.savez("calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

            # self.get_logger().info("✅ 標定數據已儲存到 calibration_data.npz")

        else:
            self.get_logger().error("❌ 標定失敗！")

        # 停止節點
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
