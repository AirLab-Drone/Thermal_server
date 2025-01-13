#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from drone_status_msgs.srv import CheckUSBDevices

from check_status_py.error_code import *
from check_status_py.tools import post_to_server, ros2_time_to_taiwan_timezone, print_json

from datetime import datetime, timezone, time
import json


class check_UpS_status(Node):

    def __init__(self):
        super().__init__("check_UpS_status")

        self.up_squared_status_dict = {
            "upload_time": None,
            "up_squared_service": None,
            "rgb_status": None,
            "thermal_status": None,
            "error_code": [],
        }

        # 每分鐘執行一次檢查
        self.target_times = [
            time(hour, minute) for hour in range(0, 24) for minute in range(0, 60)
        ]

        self.__server_url = "http://127.0.0.1:5000/upload/UpSquaredStatus"

        # 定時器：每秒檢查是否到達目標時間
        self.check_timer = self.create_timer(60.0, self.check_and_upload_at_target_times)

        # USB 檢查服務
        self.cli = self.create_client(CheckUSBDevices, "check_usb_devices")

    def check_and_upload_at_target_times(self):
        """
        每秒檢查當前時間，若符合目標時間則執行檢查並上傳。
        """
        now = datetime.now().time()
        if any(now.hour == t.hour and now.minute == t.minute for t in self.target_times):
            self.perform_checks()

    def perform_checks(self):
        """
        執行服務與設備檢查，並在所有檢查完成後上傳結果。
        """
        # 初始化字典
        self.up_squared_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(self.get_clock().now())
        self.up_squared_status_dict["error_code"] = []

        # 檢查服務狀態
        service_ready = self.check_UpSquared_service()

        if not service_ready:
            # 如果服務不可用，直接更新字典並上傳
            self.up_squared_status_dict["rgb_status"] = False
            self.up_squared_status_dict["thermal_status"] = False
            self.upload_to_server()
            return

        # 檢查設備狀態
        device_check_future = self.check_usb_status()

        if device_check_future:
            device_check_future.add_done_callback(self.finalize_and_upload)
        else:
            self.upload_to_server()

    def check_UpSquared_service(self):
        """
        確認 USB 檢查服務是否可用，最多嘗試 3 次，返回檢查結果。
        """
        max_retries = 3
        for attempt in range(max_retries):
            if self.cli.service_is_ready():
                self.get_logger().info("USB 檢查服務可用。")
                self.up_squared_status_dict["up_squared_service"] = True
                return True
            else:
                self.get_logger().warn(f"USB 檢查服務不可用，嘗試第 {attempt + 1} 次...")
                self.get_clock().sleep_for(Duration(seconds=1))  # 等待 1 秒後重試

        # 超過最大嘗試次數，判斷失敗
        self.get_logger().error("USB 檢查服務不可用，已超過最大嘗試次數。")
        self.up_squared_status_dict["up_squared_service"] = False
        self.up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_SERVICE_ERROR)
        return False

    def check_usb_status(self):
        """
        發送 USB 檢查請求，返回檢查未來結果。
        """
        request = CheckUSBDevices.Request()
        return self.cli.call_async(request)

    def finalize_and_upload(self, future):
        """
        整合設備檢查結果並上傳伺服器。
        """
        try:
            response = future.result()
            if response.success:
                self.up_squared_status_dict["rgb_status"] = True
                self.up_squared_status_dict["thermal_status"] = True
            else:
                missing_devices = response.missing_devices
                device_names = [device.split(' (')[0] for device in missing_devices]

                if 'Microdia USB 2.0 Camera' in device_names:
                    self.up_squared_status_dict["rgb_status"] = False
                    self.up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_RGB_CAMERA_ERROR)
                else:
                    self.up_squared_status_dict["rgb_status"] = True

                if 'Cypress Semiconductor Corp. GuideCamera' in device_names:
                    self.up_squared_status_dict["thermal_status"] = False
                    self.up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_THERMAL_CAMERA_ERROR)
                else:
                    self.up_squared_status_dict["thermal_status"] = True
        except Exception as e:
            self.get_logger().error(f"USB 檢查請求失敗: {e}")
            self.up_squared_status_dict["rgb_status"] = False
            self.up_squared_status_dict["thermal_status"] = False
            self.up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_SERVICE_ERROR)

        # 上傳至伺服器
        self.upload_to_server()

    def upload_to_server(self):
        """
        將結果上傳至伺服器。
        """
        try:
            print_json(self.up_squared_status_dict)
            response = post_to_server(url=self.__server_url, json=self.up_squared_status_dict)
            if response.status_code == 200:
                self.get_logger().info("已成功上傳伺服器。")
            else:
                self.get_logger().error(f"伺服器回應錯誤: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"上傳至伺服器時發生錯誤: {e}")


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


if __name__ == "__main__":
    main()









# ----------------------------------- 舊ㄉ程式 ----------------------------------- #



    # def check_UpSquared_service_callback(self):

    #     if not self.cli.service_is_ready():
    #         if self.UpSquared_disconnected_start_time is None:
    #             self.UpSquared_disconnected_start_time = self.get_clock().now()

    #         elapsed_time = (
    #             self.get_clock().now() - self.UpSquared_disconnected_start_time
    #         )
    #         self.get_logger().warn(
    #             f"USB 檢查服務不可用，已等待 {int(elapsed_time.nanoseconds / 1e9)} 秒"
    #         )

    #         # upload time
    #         current_time = self.get_clock().now()
    #         upload_time = ros2_time_to_taiwan_timezone(current_time)

    #         if elapsed_time > self.__interval_time:
    #             self.get_logger().error(
    #                 f"USB 檢查服務已超過 {int(elapsed_time.nanoseconds / 1e9)} 秒不可用，發送警告至伺服器..."
    #             )

    #             up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)
    #             up_squared_status_dict["up_squared_service"] = False
    #             up_squared_status_dict["error_code"].append(
    #                 ERROR_CODE.UP_SQUARE_SERVICE_ERROR
    #             )
    #             up_squared_status_dict["upload_time"] = upload_time

    #             try:
    #                 print_json(up_squared_status_dict)
    #                 if (
    #                     post_to_server(
    #                         url=self.__server_url, json=up_squared_status_dict
    #                     ).status_code
    #                     == 200
    #                 ):
    #                     self.get_logger().error("已成功發送警告至伺服器")
    #             except Exception as e:
    #                 self.get_logger().error(f"發生錯誤: {e}")

    #             self.UpSquared_disconnected_start_time = self.get_clock().now()
    #     else:
    #         if self.UpSquared_disconnected_start_time is not None:
    #             self.get_logger().info("USB 檢查服務已連接。")
    #             self.UpSquared_disconnected_start_time = None
    #         return

    # def check_usb_status_callback(self):
    #     """
    #     若服務可用，則發送非同步 USB 檢查請求。
    #     """
    #     if self.cli.service_is_ready():
    #         request = CheckUSBDevices.Request()
    #         future = self.cli.call_async(request)
    #         future.add_done_callback(self.response_callback)
    #     else:
    #         return

    # def response_callback(self, future):
    #     """
    #     處理 USB 檢查服務的回應，並將結果發送到伺服器。
    #     """
    #     current_time = self.get_clock().now()

    #     response = future.result()

    #     up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)
    #     up_squared_status_dict["up_squared_service"] = True
    #     up_squared_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(
    #         current_time
    #     )

    #     if response.success:
    #         up_squared_status_dict["rgb_status"] = True
    #         up_squared_status_dict["thermal_status"] = True

    #     else:
    #         missing_devices = response.missing_devices
    #         device_names = [device.split(" (")[0] for device in missing_devices]

    #         if "Microdia USB 2.0 Camera" in device_names:
    #             up_squared_status_dict["rgb_status"] = False
    #             up_squared_status_dict["error_code"].append(
    #                 ERROR_CODE.UP_SQUARE_RGB_CAMERA_ERROR
    #             )
    #         else:
    #             up_squared_status_dict["rgb_status"] = True  # 確保狀態更新正確

    #         if "Cypress Semiconductor Corp. GuideCamera" in device_names:
    #             up_squared_status_dict["thermal_status"] = False
    #             up_squared_status_dict["error_code"].append(
    #                 ERROR_CODE.UP_SQUARE_THERMAL_CAMERA_ERROR
    #             )
    #         else:
    #             up_squared_status_dict["thermal_status"] = True

    #         # print(json.dumps(up_squared_status_dict, indent=4, ensure_ascii=False))

    #     try:
    #         print_json(up_squared_status_dict)
    #         if (
    #             post_to_server(
    #                 url=self.__server_url, json=up_squared_status_dict
    #             ).status_code
    #             == 200
    #         ):
    #             self.get_logger().error("已成功發送警告至伺服器")
    #     except Exception as e:
    #         self.get_logger().error(f"發生錯誤: {e}")

    #     # print(response)

    #     # print(device_names)
    #     # print(type(device_names))  # list

    #     # self.get_logger().info(self.drone_status_dict["usb_status"])




