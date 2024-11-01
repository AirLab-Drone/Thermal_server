#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone_status_msgs.srv import CheckUSBDevices



class USBClient(Node):
    def __init__(self):
        super().__init__('usb_client')

        self.target_devices = {
            '0c45:6364': 'Microdia USB 2.0 Camera',
            '04b4:f8f8': 'Cypress Semiconductor Corp. GuideCamera'
        }

        self.cli = self.create_client(CheckUSBDevices, 'check_usb_devices')
        
        # 等待服務啟動
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服務尚未啟動...')
        
        # 初始化請求物件
        self.request = CheckUSBDevices.Request()
        
        # 設置每秒檢查一次
        self.timer = self.create_timer(1.0, self.check_usb_status)

    def check_usb_status(self):
        # 發送非同步請求
        self.future = self.cli.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # 取得回應
        response = future.result()
        if response.success:
            self.get_logger().info("All target devices are connected.")
        else:
            # self.get_logger().warning("Missing devices: " + ", ".join(response.missing_devices))
            # print(type(response.missing_devices))   # list

            for i in range(len(response.missing_devices)):
                print(response.missing_devices[i])
                print(type(response.missing_devices[i])) # str



def main(args=None):
    rclpy.init(args=args)
    usb_client = USBClient()
    rclpy.spin(usb_client)
    usb_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
