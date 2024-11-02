#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time

class MavlinkHeartbeatMonitor(Node):
    def __init__(self):
        super().__init__('mavlink_heartbeat_monitor')

        port='/dev/ttyUSB0'
        baud=57600

        # 連接到 Mavlink，您可以調整此處的連線方式
        self.mavlink_connection = mavutil.mavlink_connection(port, baud)
        self.heartbeat_timeout = 5  # 超時時間（秒）

        # 最後一次收到心跳的時間
        self.last_heartbeat_time = time.time()

        # 定時器每秒檢查一次心跳訊息
        self.timer = self.create_timer(1.0, self.check_heartbeat)

    def check_heartbeat(self):
        """
        檢查心跳訊息，若超過指定時間未收到心跳，則視為斷線
        """
        current_time = time.time()

        # 嘗試接收 HEARTBEAT 訊息
        msg = self.mavlink_connection.recv_match(type="HEARTBEAT", blocking=False)

        if msg is not None:
            # 若接收到心跳訊息，更新最後一次接收時間
            self.last_heartbeat_time = current_time
            self.get_logger().info(f"收到心跳訊息，系統 ID: {msg.get_srcSystem()}, 組件 ID: {msg.get_srcComponent()}")
        else:
            # 若超過超時時間未收到心跳訊息，判定為斷線
            if current_time - self.last_heartbeat_time > self.heartbeat_timeout:
                self.get_logger().warn("Mavlink 連線已斷開或超時未收到心跳訊息")
                # 可選擇在此處執行重新連線操作，或發出其他警告

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkHeartbeatMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
