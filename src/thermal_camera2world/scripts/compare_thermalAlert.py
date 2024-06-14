#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from thermal_msgs.msg import ThermalAlert

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

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = CompareThermalAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()