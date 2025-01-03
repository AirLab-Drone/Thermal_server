#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from thermal_msgs.msg import ThermalAlert
import cv2
import numpy as np
from cv_bridge import CvBridge

class CompareThermalAlert(Node):
    def __init__(self):
        super().__init__("compare_thermal_alert")
        self.create_subscription(
            ThermalAlert, "/thermal_DS4025FT/thermal_alert", self.thermal_alert_callback, 10
        )
        self.create_subscription(
            ThermalAlert, "/thermal_IPT430M/thermal_alert", self.thermal_alert_callback, 10
        )
        self.thermal_alert_pub = self.create_publisher(ThermalAlert, "/thermal_alert", 10)
        self.high_quality_thermal_alert = ThermalAlert()

        self.create_timer(1, self.clean_alert)

    def clean_alert(self):
        self.high_quality_thermal_alert = ThermalAlert()

    def thermal_alert_callback(self, msg):
        self.get_logger().info("Thermal Alert: %s" % msg)
        if self.high_quality_thermal_alert.temperature < msg.temperature:
            self.high_quality_thermal_alert = msg
        self.thermal_alert_pub.publish(self.high_quality_thermal_alert)
        # # 畫出一個黑色框框裡面要包含座標和溫度的數值用不同顏色顯示
        # image = np.zeros((500, 500, 3), dtype=np.uint8)
        # cv2.putText(image, f"X: {self.high_quality_thermal_alert.x:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv2.putText(image, f"Y: {self.high_quality_thermal_alert.y:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv2.putText(image, f"Temperature: {self.high_quality_thermal_alert.temperature:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv2.imshow("Thermal Alert", image)
        # cv2.waitKey(1)

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = CompareThermalAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()