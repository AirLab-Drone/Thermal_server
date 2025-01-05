#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from thermal_msgs.msg import ThermalAlert

from mailjet_rest import Client
import os


class SendFireAlert(Node):
    def __init__(self):
        super().__init__("send_fire_alert")
        self.email_sent = False
        self.last_email_time = None
        # self.cooldown_period = Duration(seconds=300)    # 5 分鐘冷卻期
        self.cooldown_period = Duration(seconds=60)  # 1 分鐘冷卻期 (demo 用)

        self.create_subscription(
            ThermalAlert,
            "/thermal_alert",
            self.thermal_alert_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.timer = self.create_timer(60.0, self.cooldown_countdown)  # 每分鐘倒數計時

        # 讀取 Mailjet API 金鑰與密鑰
        self.mailjet_api_key = os.getenv("MAILJET_API_KEY")
        self.mailjet_api_secret = os.getenv("MAILJET_API_SECRET")

        if not self.mailjet_api_key or not self.mailjet_api_secret:
            self.get_logger().error("Mailjet API key or secret is not set.")
            raise RuntimeError("Missing Mailjet API credentials.")

        self.mailjet_client = Client(
            auth=(self.mailjet_api_key, self.mailjet_api_secret), version="v3.1"
        )


    def thermal_alert_callback(self, msg):
        current_time = self.get_clock().now()

        # 檢查冷卻期是否結束
        if self.last_email_time is None or (
            current_time - self.last_email_time > self.cooldown_period
        ):
            self.email_sent = False

        if not self.email_sent:
            self.send_email()
            self.email_sent = True
            self.last_email_time = current_time
            self.get_logger().info(
                f"Email sent at {current_time}. Next email allowed after cooldown."
            )
        else:
            self.cooldown_countdown()  # 呼叫倒數計時函式


    def cooldown_countdown(self):
        if self.email_sent and self.last_email_time:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.last_email_time
            elapsed_seconds = elapsed_time.nanoseconds / 1e9
            cooldown_seconds = self.cooldown_period.nanoseconds / 1e9
            remaining_seconds = cooldown_seconds - elapsed_seconds

            if remaining_seconds > 0:
                minutes = int(remaining_seconds // 60)
                seconds = int(remaining_seconds % 60)
                self.get_logger().info(
                    f"Cooldown active. Time remaining: {minutes}m {seconds}s"
                )
            else:
                self.email_sent = False
                self.get_logger().info("Cooldown expired. Email alert re-enabled.")


    def send_email(self):
        data = {
            "Messages": [
                {
                    "From": {
                        "Email": "airlab.dronedrone@gmail.com",
                        "Name": "Thermal Alert System",
                    },
                    # "To": [{"Email": "danny1.2104@gmail.com", "Name": "Danny"}],
                    "To": [{"Email": "wj582693@gmail.com", "Name": "Danny"}],
                    "Subject": "🔥 Fire Alert Detected!",
                    "TextPart": "A fire alert has been triggered by the thermal detection system. Please check the system immediately.",
                    "HTMLPart": "<h3>🔥 Fire Alert!</h3><p>A fire has been detected. Please respond immediately.</p>",
                }
            ]
        }

        result = self.mailjet_client.send.create(data=data)
        if result.status_code == 200:
            self.get_logger().info("Email successfully sent.")
        else:
            self.get_logger().error(f"Failed to send email. Error: {result.json()}")


def main(args=None):
    rclpy.init(args=args)
    node = SendFireAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
