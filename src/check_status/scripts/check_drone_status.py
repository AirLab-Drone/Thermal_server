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


from check_status_py.error_code import *
from check_status_py.tools import send_json_to_server, check_port_exists, ros2_time_to_taiwan_timezone


'''
database 有三個table
    1. 飛控狀態 
    2. up squared 狀態 (RGB, Thermal) 
    3. 環境熱像儀狀態
''' 
# TODO: Envirmental thermal camera status check



class check_Drone_status(Node):

    def __init__(self):
        super().__init__('check_drone_status')

        self.drone_status_dict = {  
            "upload_time": None,
            "sensor_health": None,
            "battery_voltage": None,           # uint16_t      mV       invalid:UINT16_MAX
            "battery_current": None,           # int16_t	   cA	    invalid:-1
            "battery_remaining": None,         # int8_t	       %	    invalid:-1
            "gps_hdop": None,                  # 1E-2	                invalid:UINT16_MAX	GPS HDOP (unitless * 100)
            "gps_satellites_visible": None,    # uint8_t			    invalid:UINT8_MAX
            "attitude_roll": None,             # float	       rad	    Roll angle (-pi..+pi)
            "attitude_pitch": None,            # float	       rad	    Pitch angle (-pi..+pi)
            "attitude_yaw": None,              # float	       rad	    Yaw angle (-pi..+pi)   
            "servo_output_1": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "servo_output_2": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "servo_output_3": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "servo_output_4": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "servo_output_5": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "servo_output_6": None,            # uint16_t      us       PWM 通常在 1000 到 2000 微秒之間
            "error_code": []
            }
        

        self.__interval_time = Duration(seconds=5)

        self.__server_url = ""

        # ---------------------------------- mavlink --------------------------------- #
        # TODO: 幹你娘問題一堆 要重寫==
        
        self.mavlink_port = '/dev/drone_usb'
        self.mavlink_baud = 57600
    
        self.master = None

        self.MavLink_latest_status = None
        self.MavLink_error_code_list = []

        self.mavlink_timer = None
        self.mavlink_status = None  # 狀態標記：'not_found', 'connecting', 'connected'

                
        self.check_mavlink_connection_timer = self.create_timer(1, self.check_mavlink_connection_callback)

        self.check_drone_status_timer = self.create_timer(10, self.get_drone_status_callback)
     


    def check_mavlink_connection_callback(self):
        current_time = self.get_clock().now()

        # 設定上傳時間
        drone_status_dict = copy.deepcopy(self.drone_status_dict)
        drone_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)

        # 檢查 mavlink port 是否存在
        if not check_port_exists(self.mavlink_port):
            if self.mavlink_status != 'not_found':
                # 初次進入 "not_found" 狀態，設定計時器
                self.mavlink_status = 'not_found'
                self.mavlink_timer = current_time

            elapsed_time = current_time - self.mavlink_timer
            self.get_logger().warn(f"Port {self.mavlink_port} 不可用, 以等待 {int(elapsed_time.nanoseconds / 1e9)} 秒")

            if elapsed_time > self.__interval_time:
                drone_status_dict["error_code"].append(ERROR_CODE.MAVLINK_PORT_ERROR)
                self.get_logger().error(f"Port {self.mavlink_port} 已超過 {int(elapsed_time.nanoseconds / 1e9)} 秒不可用，發送警告至伺服器...")
                send_json_to_server(url=self.__server_url, data=drone_status_dict)
                
                # 重置計時器
                self.mavlink_timer = current_time
            return

        # Mavlink 第一次連接
        if self.master is None:
            if self.mavlink_status != 'connecting':
                # 進入 "connecting" 狀態
                self.mavlink_status = 'connecting'
                self.mavlink_timer = current_time
                self.get_logger().info(f"找到 {self.mavlink_port}，正在嘗試連接飛控...")

            if not self.__connect_mavlink(self.mavlink_port, self.mavlink_baud):
                elapsed_time = current_time - self.mavlink_timer
                self.get_logger().warn(f"無法連接到 Mavlink，已等待 {int(elapsed_time.nanoseconds / 1e9)} 秒")
                
                if elapsed_time > self.__interval_time:
                    drone_status_dict["error_code"].append(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
                    self.get_logger().error(f"無法在 {int(elapsed_time.nanoseconds / 1e9)} 秒內連接到 Mavlink，發送警告至伺服器...")
                    send_json_to_server(url=self.__server_url, data=drone_status_dict)
                    # 重置計時器
                    self.mavlink_timer = current_time
                    self.master = None
                return

        # 檢查連線狀態
        if not self.__check_heartbeat() and self.master is not None:
            if self.mavlink_status != 'connected':
                # 設定 "connected" 狀態
                self.mavlink_status = 'connected'
                self.mavlink_timer = current_time

            elapsed_time = current_time - self.mavlink_timer
            self.get_logger().warn(f"Mavlink 連線中斷，已等待 {int(elapsed_time.nanoseconds / 1e9)} 秒")

            if elapsed_time > self.__interval_time:
                drone_status_dict["error_code"].append(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
                self.get_logger().error(f"Mavlink 連線中斷超過 {int(elapsed_time.nanoseconds / 1e9)} 秒，發送警告至伺服器...")
                send_json_to_server(url=self.__server_url, data=drone_status_dict)

                # 重置計時器和狀態
                self.mavlink_timer = current_time
                self.master = None
                self.mavlink_status = 'not_found'  # 重置為斷開狀態

        else:

            # self.get_logger().info("Mavlink 連線正常")
            return






    def __connect_mavlink(self, port, baud) -> bool:
        """
        嘗試連接 MAVLink，若無法在指定時間內連接成功，則返回 False。
        """
        try:
            self.master = mavutil.mavlink_connection(port, baud)
            if not self.__wait_for_heartbeat():
                return False
            return True
        except Exception as e:
            # self.get_logger().error(f"無法連接到 Mavlink: {e}")
            return False






    def __wait_for_heartbeat(self, timeout=10):
        """
        等待心跳包來確認連接成功，超時後返回 False。
        :param timeout: 每次嘗試的等待時間（秒）
        :return: 連接成功返回 True，失敗則返回 False
        """
        try:
            self.get_logger().info("等待飛控心跳包...")
            
            # 開始倒數計時
            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                remaining_time = timeout - int(time.time() - start_time)
                self.get_logger().warn(f"剩餘時間: {remaining_time} 秒", throttle_duration_sec=1)

                if msg is not None and msg.get_srcSystem() > 0 and msg.get_srcComponent() > 0:
                    self.get_logger().info(f"已連接到系統 {msg.get_srcSystem()}, 組件 {msg.get_srcComponent()}")
                    return True

            self.get_logger().error("無效的心跳訊息，系統或組件 ID 無效")
        except Exception as e:
            self.get_logger().error(f"連接過程中出現錯誤: {e}")

        return False  # 超時後返回 False 以表示連接失敗




    def __check_heartbeat(self) -> bool:
        """
        定期檢查是否有心跳訊息，以確認連接狀態。
        """
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            return msg is not None and msg.get_srcSystem() > 0 and msg.get_srcComponent() > 0
        except Exception as e:
            # self.get_logger().error(f"檢查心跳訊息時出現錯誤: {e}")

            return False




    def __get_mavlink_message(self, message_type, timeout=5):
        try:
            msg = self.master.recv_match(type=message_type, blocking=True, timeout=timeout)
            # print(msg)
            return msg
        except Exception as e:
            print(f"獲取 {message_type} 消息時出現錯誤: {e}")
            return None
        

    def get_drone_status_callback(self):
        """
        獲取飛控的狀態信息。
        """
        if self.mavlink_status == 'connected':

            drone_status_dict = copy.deepcopy(self.drone_status_dict)

            try:
                sys_status = self.__get_mavlink_message('SYS_STATUS')
                if sys_status:
                    drone_status_dict["sensor_health"] = sys_status.onboard_control_sensors_health
                    
                    drone_status_dict["battery_voltage"] = sys_status.voltage_battery
                    drone_status_dict["battery_current"] = sys_status.current_battery
                    drone_status_dict["battery_remaining"] = sys_status.battery_remaining

                gps_raw_int = self.__get_mavlink_message('GPS_RAW_INT')
                if gps_raw_int:
                    drone_status_dict["gps_hdop"] = gps_raw_int.eph
                    drone_status_dict["gps_satellites_visible"] = gps_raw_int.satellites_visible

                attitude = self.__get_mavlink_message('ATTITUDE')
                if attitude:
                    drone_status_dict["attitude_roll"] = attitude.roll
                    drone_status_dict["attitude_pitch"] = attitude.pitch
                    drone_status_dict["attitude_yaw"] = attitude.yaw

                    # self.drone_status_dict["attitude_roll"] = round(attitude.roll, 3)
                    # self.drone_status_dict["attitude_pitch"] = round(attitude.pitch, 3)
                    # self.drone_status_dict["attitude_yaw"] = round(attitude.yaw, 3)

                servo_output_raw = self.__get_mavlink_message('SERVO_OUTPUT_RAW')
                if servo_output_raw:
                    for i in range(1, 7):
                        drone_status_dict[f"servo_output_{i}"] = getattr(servo_output_raw, f"servo{i}_raw")



                # print(json.dumps(self.drone_status_dict, indent=4, ensure_ascii=False))
                self.MavLink_latest_status = drone_status_dict
                
                # return self.drone_status_dict  # callback can't return value


            except Exception as e:
                print(f"錯誤: {e}")

            self.check_drone_status()


    def get_last_status(self) -> dict:
        if self.MavLink_latest_status:
            # print(json.dumps(self.MavLink_latest_status, indent=4, ensure_ascii=False))
            return self.MavLink_latest_status
        

    def check_drone_status(self):
        status = self.get_last_status()

        if status:
            drone_status_dict = copy.deepcopy(self.drone_status_dict)
            current_time = self.get_clock().now()
            drone_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)


            sensor_health = status["sensor_health"]
            # print(f"系統感測器健康狀態: {sensor_health}")

            # sensor_health check
            self.MavLink_error_code_list = self.parse_sensor_health(sensor_health)

            # !battery_voltage check 
            # 市電供電時 無法量測電壓 跳過檢查 
            # if status["battery_voltage"] < 44:
            #     self.error_code_list.append(ERROR_CODE.HEALTH_BATTERY_VOLTAGE_ERROR)
 
            # gps hdop check
            if status["gps_hdop"] == UINT16_MAX or status["gps_hdop"] > 100:
                self.MavLink_error_code_list.append(ERROR_CODE.GPS_HDOP_ERROR)
                self.get_logger().error(f"GPS HDOP 過高: {status['gps_hdop']/100}")

            # gps_satellites_visible check, only have 4 UWB
            if status["gps_satellites_visible"] == UINT8_MAX or status["gps_satellites_visible"] < 4:
                self.MavLink_error_code_list.append(ERROR_CODE.GPS_SATELLITES_VISIBLE_ERROR)
                self.get_logger().error(f"可見衛星數量過少: {status['gps_satellites_visible']}")

            # drone attitude check
            if abs(math.degrees(status["attitude_roll"])) > 15:
                self.MavLink_error_code_list.append(ERROR_CODE.ATTITUDE_ROLL_ERROR)
                self.get_logger().error(f"Roll 角度過大: {math.degrees(status['attitude_roll'])}")

            if abs(math.degrees(status["attitude_pitch"])) > 15:
                self.MavLink_error_code_list.append(ERROR_CODE.ATTITUDE_PITCH_ERROR)
                self.get_logger().error(f"Pitch 角度過大: {math.degrees(status['attitude_pitch'])}")

            # motor output check
            for i in range(1, 7):
                if status[f"servo_output_{i}"] < 1000 or status[f"servo_output_{i}"] > 2000:
                    self.MavLink_error_code_list.append(ERROR_CODE.MOTOR_OUTPUTS_ERROR)
                    self.get_logger().error(f"馬達 {i} 輸出異常: {status[f'servo_output_{i}']}")

            # if status["range_finder"] :

        

            drone_status_dict["error_code"] = self.MavLink_error_code_list

            send_json_to_server(url=self.__server_url, data=drone_status_dict)

            # print(json.dumps(self.drone_status_dict, indent=4, ensure_ascii=False))
            # print(self.error_code_list)
            


        
    # 解析感測器健康狀態
    def parse_sensor_health(self, sensors_health) -> list:
        '''
        :param sensors_health: sys_status.onboard_control_sensors_health 的值
        :return: 回傳每個感測器合在一起成功或錯誤代碼陣列
        '''

        erroe_code_list = []

        # print(f"系統感測器健康狀態: {sensors_health}")
        for flag, sensor in sensor_flags.items():
            if not sensors_health & flag:
                # print(f"{sensor[0]}: False")
                erroe_code_list.append(sensor[1])
            else:
                pass
                # print(f"{sensor[0]}: True")
                # erroe_code_list.append(ERROR_CODE.SUCCESS)
                
        # print(erroe_code_list)
        return erroe_code_list







def main(args=None):
    rclpy.init(args=args)

    check_status = None  # 先初始化為 None
    try:
        check_status = check_Drone_status()
        rclpy.spin(check_status)
    except Exception as e:
        print(f"發生錯誤: {e}")
    finally:
        if check_status:  # 確認 check_status 已被初始化，才調用 destroy_node()
            check_status.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    # print(len(sensor_flags))
    {
    "sensor_health": 1467088239,
    "battery_voltage": 46104,
    "battery_current": 158,
    "battery_remaining": 56,
    "gps_hdop": 30,
    "gps_satellites_visible": 4,
    "attitude_roll": -0.008461258374154568,
    "attitude_pitch": 0.018391260877251625,
    "attitude_yaw": 1.9347625970840454,
    "servo_output_1": 1000,
    "servo_output_2": 1000,
    "servo_output_3": 1000,
    "servo_output_4": 1000,
    "servo_output_5": 1000,
    "servo_output_6": 1000,
    "error_code": None
    }



