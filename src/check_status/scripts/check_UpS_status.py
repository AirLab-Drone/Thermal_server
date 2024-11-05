#!/usr/bin/env python3

'''
確認無線電的usb port
ls /dev/ttyUSB*

給權限
sudo chmod 777 /dev/ttyUSB0
'''

import copy
from datetime import datetime, timezone
import pytz
import json
import math
import time

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import Float32


from drone_status_msgs.srv import CheckUSBDevices

from check_status_py.error_code import *
from check_status_py.tools import send_json_to_server, ros2_time_to_taiwan_timezone




class check_UpS_status(Node):

    def __init__(self):
        super().__init__('check_UpS_status')

        
        self.up_squared_status_dict = {
            "upload_time": None,
            "up_squared_service": None,
            "rgb_status": None,
            "thermal_status": None,
            "error_code": []
        }

        # 斷線等待時間
        self.__interval_time = Duration(seconds=5)

        self.__server_url = ""


        # -------------------------------- up squared -------------------------------- #

        check_service_interval_time = 1
        check_port_interval_time = 10


        self.UpSquared_disconnected_start_time = None  # 初始化計時器

        self.cli = self.create_client(CheckUSBDevices, 'check_usb_devices')

        self.check_UpSquared_service_timer = self.create_timer(check_service_interval_time, self.check_UpSquared_service_callback)

        self.check_usb_status_timer = self.create_timer(check_port_interval_time, self.check_usb_status_callback)




    def check_UpSquared_service_callback(self):

        if not self.cli.service_is_ready():
            if self.UpSquared_disconnected_start_time is None:
                self.UpSquared_disconnected_start_time = self.get_clock().now()

            elapsed_time = self.get_clock().now() - self.UpSquared_disconnected_start_time
            self.get_logger().warn(f"USB 檢查服務不可用，已等待 {int(elapsed_time.nanoseconds / 1e9)} 秒")

            # upload time
            current_time = self.get_clock().now()
            upload_time = ros2_time_to_taiwan_timezone(current_time)


            if elapsed_time > self.__interval_time:
                self.get_logger().error(f"USB 檢查服務已超過 {int(elapsed_time.nanoseconds / 1e9)} 秒不可用，發送警告至伺服器...")

                up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)
                up_squared_status_dict["up_squared_service"] = False
                up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_SERVICE_ERROR)
                up_squared_status_dict["upload_time"] = upload_time
                send_json_to_server(url=self.__server_url, data=up_squared_status_dict)

                self.UpSquared_disconnected_start_time = self.get_clock().now()
        else:
            if self.UpSquared_disconnected_start_time is not None:
                self.get_logger().info("USB 檢查服務已連接。")
                self.UpSquared_disconnected_start_time = None
            return


    def check_usb_status_callback(self):
        """
        若服務可用，則發送非同步 USB 檢查請求。
        """
        if self.cli.service_is_ready():
            request = CheckUSBDevices.Request()
            future = self.cli.call_async(request)
            future.add_done_callback(self.response_callback)
        else:
            return




    def response_callback(self, future):
        """
        處理 USB 檢查服務的回應，並將結果發送到伺服器。
        """
        current_time = self.get_clock().now()

        response = future.result()

        up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)
        up_squared_status_dict["up_squared_service"] = True
        up_squared_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)


        if response.success:
            up_squared_status_dict["rgb_status"] = True
            up_squared_status_dict["thermal_status"] = True

        else:
            missing_devices = response.missing_devices
            device_names = [device.split(' (')[0] for device in missing_devices]


            if 'Microdia USB 2.0 Camera' in device_names:
                up_squared_status_dict["rgb_status"] = False
                up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_RGB_CAMERA_ERROR)
            else:
                up_squared_status_dict["rgb_status"] = True  # 確保狀態更新正確

            if 'Cypress Semiconductor Corp. GuideCamera' in device_names:
                up_squared_status_dict["thermal_status"] = False
                up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_THERMAL_CAMERA_ERROR)
            else:
                up_squared_status_dict["thermal_status"] = True      
            
            # print(json.dumps(up_squared_status_dict, indent=4, ensure_ascii=False))
        
        send_json_to_server(url=self.__server_url, data=up_squared_status_dict)

        # print(response)


            # print(device_names)
            # print(type(device_names))  # list
            

        # self.get_logger().info(self.drone_status_dict["usb_status"])







def main(args=None):
    rclpy.init(args=args)

    check_status = None  # 先初始化為 None
    try:
        check_status = check_UpS_status()
        rclpy.spin(check_status)
    except Exception as e:
        print(f"發生錯誤: {e}")
    finally:
        if check_status:  # 確認 check_status 已被初始化，才調用 destroy_node()
            check_status.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



