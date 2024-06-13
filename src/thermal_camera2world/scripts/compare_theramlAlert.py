from rclpy import Node
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
        self.thermal+alert_pub = self.create_publisher(ThermalAlert, "/thermal_alert", 10)
        self.high_quality_thermal_alert = ThermalAlert() 

    def thermal_alert_callback(self, msg):
        self.get_logger().info("Thermal Alert: %s" % msg)
        if self.high_quality_thermal_alert.temperature < msg.temperature:
            self.high_quality_thermal_alert = msg
            self.thermal_alert_pub.publish(self.high_quality_thermal_alert)
