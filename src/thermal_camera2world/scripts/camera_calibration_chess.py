#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sklearn.cluster import KMeans


class ChessboardDetector(Node):
    def __init__(self):
        super().__init__('chessboard_detector')
        self.subscription = self.create_subscription(
            Image, '/thermal_IPT430M/thermal_image', self.image_callback, 10)
        self.bridge = CvBridge()

        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("Threshold", "Trackbars", 0, 255, self.on_trackbar)
        cv2.createTrackbar("threshold1", "Trackbars", 1, 800, self.on_trackbar)
        cv2.createTrackbar("threshold2", "Trackbars", 1, 800, self.on_trackbar)
        cv2.createTrackbar("line_threshold", "Trackbars", 1, 800, self.on_trackbar)
        cv2.createTrackbar("minLineLength", "Trackbars", 1, 800, self.on_trackbar)
        cv2.createTrackbar("maxLineGap", "Trackbars", 1, 800, self.on_trackbar)
        cv2.createTrackbar("kernel_size", "Trackbars", 1, 20, self.on_trackbar)




    def on_trackbar(self, val):
        pass

    def sobel(self, src_image, kernel_size):
        grad_x = cv2.Sobel(src_image, cv2.CV_16S, 1, 0, ksize=kernel_size, scale=1,
                        delta=0, borderType=cv2.BORDER_DEFAULT)
        grad_y = cv2.Sobel(src_image, cv2.CV_16S, 0, 1, ksize=kernel_size, scale=1, 
                        delta=0, borderType=cv2.BORDER_DEFAULT)
        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)

        grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)

        return grad
    def process_image(self, cv_image):
        """ 影像處理與即時參數調整 """
        threshold1 = max(cv2.getTrackbarPos("threshold1", "Trackbars"), 1)  # 225
        threshold2 = max(cv2.getTrackbarPos("threshold2", "Trackbars"), 1)  # 113
        kernel_size = max(cv2.getTrackbarPos("kernel_size", "Trackbars"), 1)  # 113
        if kernel_size % 2 == 0:
            kernel_size += 1  # 如果是偶數，則加 1 使其變為奇數
        kernel_size = max(kernel_size, 1)  # 確保大小不小於 1

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) 
        blur_image = cv2.blur(gray,(3,3))
            # detect corners
        soel_img = self.sobel(blur_image, kernel_size)

        # edges = cv2.Canny(soel_img, threshold1, threshold2)
        # edges = cv2.Canny(soel_img, 225, 113)

        return soel_img
    

    def find_line(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        line_threshold = max(cv2.getTrackbarPos("line_threshold", "Trackbars"), 1)
        minLineLength = max(cv2.getTrackbarPos("minLineLength", "Trackbars"), 1)
        maxLineGap = max(cv2.getTrackbarPos("maxLineGap", "Trackbars"), 1)

        lines = cv2.HoughLinesP(gray, 1, np.pi / 180, threshold=line_threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)

        # image_with_lines = np.zeros((img.shape[0], img.shape[1]))


        copyIMG = img.copy()

        

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(copyIMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
            # # 提取線段的中點和角度
            # line_features = []
            # for line in lines:
            #     x1, y1, x2, y2 = line[0]
            #     midpoint = [(x1 + x2) / 2, (y1 + y2) / 2]  # 計算線段中點
            #     angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi  # 計算線段角度（弧度轉度數）
            #     line_features.append([midpoint[0], midpoint[1], angle])

            # line_features = np.array(line_features)

            # # **確保 num_clusters 不超過 line_features 數量**
            # num_clusters = min(10, len(line_features))  # 避免 clusters > 樣本數

            # if num_clusters > 1:
            #     kmeans = KMeans(n_clusters=num_clusters, random_state=0, n_init=10).fit(line_features)
            #     labels = kmeans.labels_
            # else:
            #     labels = [0] * len(line_features)  # 只有一組資料時，所有樣本歸為同一群




            # for i, line in enumerate(lines):
            #     x1, y1, x2, y2 = line[0]
            #     if labels[i] % 2 == 0:  # 只選擇部分分群的線條，去掉重複的
            #         cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 1)  # 綠色線條

        return copyIMG
        

    def image_callback(self, msg):
        """ ROS 影像訂閱回調函數 """
        # self.get_logger().info('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        process_img = self.process_image(cv_image)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) 

        found, corners = cv2.findChessboardCorners(cv_image, (5, 5))
        cv2.drawChessboardCorners(cv_image, (5, 5), corners, found)
        # find_line_img = self.find_line(cv_image)

        # blur_image = cv2.blur(gray,(3,3))
        #     # detect corners
        # soel_img = self.sobel(blur_image, 3)

    
        cv2.imshow("origin", cv_image)
        cv2.imshow("process_img", process_img)
        # cv2.imshow("find_line_img", find_line_img)
        # cv2.imshow("sobel", soel_img)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    node = ChessboardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# 進行邊緣檢測
# edges = cv2.Canny(image, 50, 150)

# # 使用霍夫變換來檢測直線
# lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)

# # 建立一個空白圖像來標記交點
# intersection_image = np.zeros_like(image)

# # 存儲線段
# line_segments = []

# if lines is not None:
#     for line in lines:
#         x1, y1, x2, y2 = line[0]
#         line_segments.append(((x1, y1), (x2, y2)))

# # 找出線段的交點
# def line_intersection(line1, line2):
#     (x1, y1), (x2, y2) = line1
#     (x3, y3), (x4, y4) = line2

#     # 計算行列式
#     denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
#     if denominator == 0:
#         return None  # 平行

#     # 計算交點
#     px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
#     py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

#     return int(px), int(py)

# # 找出所有交點
# intersections = set()
# for i in range(len(line_segments)):
#     for j in range(i + 1, len(line_segments)):
#         point = line_intersection(line_segments[i], line_segments[j])
#         if point:
#             intersections.add(point)

# # 在圖片上標記交點
# for x, y in intersections:
#     cv2.circle(intersection_image, (x, y), 3, (255, 255, 255), -1)