#!/usr/bin/env python3

'''
確認無線電的usb port
ls /dev/ttyUSB*

給權限
sudo chmod 777 /dev/ttyUSB0
'''

import json
import math
import time
import copy

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


from drone_status_msgs.srv import CheckUSBDevices

from check_status_py.error_code import *
from check_status_py.tools import send_json_to_server   


'''
# TODO:同時檢查是否有接上飛控跟電腦
database 有三個table
    1. 飛控狀態
    2. up squared 狀態 (RGB, Thermal) 
    3. 環境熱像儀狀態
''' 


# !寫兩個callback 用來檢查 up squared 和 FUC 是否有連接上
# !一個callback 用來檢查飛控狀態
# !一個callback 用來檢查USB狀態



class Check_status(Node):

    def __init__(self, port='/dev/ttyUSB0', baud=57600, interval_time=1):
        super().__init__('minimal_publisher')

        self.drone_status_dict = {  
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
        
        self.up_squared_status_dict = {
            "rgb_status": None,
            "thermal_status": None,
            "error_code": []
        }


        timeout_warning_sent = False
        # 初始化 USB 檢查服務
        start_time = time.time()
        self.cli = self.create_client(CheckUSBDevices, 'check_usb_devices')

        # TODO: 若是啟動後失敗 要寫偵測機制
        while not self.cli.wait_for_service(timeout_sec=1.0):
            # 計算等待時間
            elapsed_time = time.time() - start_time
            self.get_logger().info(f'等待 USB 檢查服務啟動... 已等待 {elapsed_time:.0f} 秒')

            # 當等待時間超過 10 秒且還未發送警告時，發送一次警告
            if elapsed_time > 5 and not timeout_warning_sent:
                self.get_logger().warn('等待 USB 檢查服務啟動超過 10 秒，發送警告至伺服器...')
                
                # 發送警告並且只增加一次錯誤代碼
                up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)
                up_squared_status_dict["error_code"].append(ERROR_CODE.UP_SQUARE_ERROR)
                send_json_to_server(url="", data=up_squared_status_dict)
                
                timeout_warning_sent = True  # 標記已發送警告

            # 如果已經發送過警告且再次嘗試重新連接，則重設計時器和警告狀態
            if elapsed_time > 5:  # 假設每 15 秒重設一次計時器
                start_time = time.time()
                timeout_warning_sent = False  # 重置以便再次發送警告
                

        # 定期檢查 USB 狀態 interval_time 秒
        self.create_timer(interval_time, self.check_usb_status)
        





        self.__connect_mavlink(port, baud)




        
        self.latest_status = None

        self.error_code_list = []
        
        # 秒取得一次
        self.timer = self.create_timer(interval_time, self.get_drone_status_callback)



    # TODO: 若是啟動後失敗 要寫偵測機制
    def __connect_mavlink(self, port='/dev/ttyUSB0', baud=57600):
        self.master = mavutil.mavlink_connection(port, baud)
        if not self.__wait_for_heartbeat():
            
            # TODO: 連接失敗, 發送給server

            error = ERROR_CODE.MAVLINK_CONNECTION_ERROR
            raise ConnectionError("無法與飛控建立連接")
        

        
    def __connect_mavlink(self, port='/dev/ttyUSB0', baud=57600):
        """
        持續嘗試連接 MAVLink，直到成功為止。
        """
        while True:
            self.master = mavutil.mavlink_connection(port, baud)
            if self.__wait_for_heartbeat():
                break  # 成功連接後退出循環
            else:
                # 發送連接失敗信息給 server（可根據實際需求實現）
                error = ERROR_CODE.MAVLINK_CONNECTION_ERROR
                self.get_logger().error("連接失敗，重新嘗試中...")
                time.sleep(5)  # 等待 5 秒後再次嘗試



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
                self.get_logger().info(f"剩餘時間: {remaining_time} 秒", throttle_duration_sec=1)

                if msg is not None and msg.get_srcSystem() > 0 and msg.get_srcComponent() > 0:
                    self.get_logger().info(f"已連接到系統 {msg.get_srcSystem()}, 組件 {msg.get_srcComponent()}")
                    return True

            self.get_logger().error("無效的心跳訊息，系統或組件 ID 無效")
        except Exception as e:
            self.get_logger().error(f"連接過程中出現錯誤: {e}")

        return False  # 超時後返回 False 以表示連接失敗


    def __wait_for_heartbeat(self, retries=3, timeout=10):
        """
        等待心跳包來確認連接成功，失敗則重試指定次數，並顯示倒數計時。
        :param retries: 重試次數
        :param timeout: 每次嘗試的等待時間（秒）
        :return: 連接成功返回 True，失敗則返回 False
        """
        for attempt in range(retries):
            try:
                self.get_logger().info(f"等待飛控心跳包...（嘗試 {attempt + 1}/{retries}）")
                
                # 開始倒數計時
                start_time = time.time()
                while time.time() - start_time < timeout:
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                    remaining_time = timeout - int(time.time() - start_time)
                    self.get_logger().info(f"剩餘時間: {remaining_time} 秒", throttle_duration_sec=1)

                    if msg is not None and msg.get_srcSystem() > 0 and msg.get_srcComponent() > 0:
                        self.get_logger().info(f"已連接到系統 {msg.get_srcSystem()}, 組件 {msg.get_srcComponent()}")
                        return True

                self.get_logger().error("無效的心跳訊息，系統或組件 ID 無效")
            except Exception as e:
                self.get_logger().error(f"第 {attempt + 1} 次連接失敗: {e}")

        return False  # 超過重試次數後，返回 False 以表示連接失敗



    def __get_mavlink_message(self, message_type, timeout=5):
        try:
            return self.master.recv_match(type=message_type, blocking=False, timeout=timeout)
        except Exception as e:
            print(f"獲取 {message_type} 消息時出現錯誤: {e}")
            return None
        

    def get_drone_status_callback(self):
        """
        獲取飛控的狀態信息。
        :return: 包含飛控狀態信息原始資料的字典
        """
        try:
            sys_status = self.__get_mavlink_message('SYS_STATUS')
            if sys_status:
                self.drone_status_dict["sensor_health"] = sys_status.onboard_control_sensors_health
                
                self.drone_status_dict["battery_voltage"] = sys_status.voltage_battery
                self.drone_status_dict["battery_current"] = sys_status.current_battery
                self.drone_status_dict["battery_remaining"] = sys_status.battery_remaining

            gps_raw_int = self.__get_mavlink_message('GPS_RAW_INT')
            if gps_raw_int:
                self.drone_status_dict["gps_hdop"] = gps_raw_int.eph
                self.drone_status_dict["gps_satellites_visible"] = gps_raw_int.satellites_visible

            attitude = self.__get_mavlink_message('ATTITUDE')
            if attitude:
                self.drone_status_dict["attitude_roll"] = attitude.roll
                self.drone_status_dict["attitude_pitch"] = attitude.pitch
                self.drone_status_dict["attitude_yaw"] = attitude.yaw

                # self.drone_status_dict["attitude_roll"] = round(attitude.roll, 3)
                # self.drone_status_dict["attitude_pitch"] = round(attitude.pitch, 3)
                # self.drone_status_dict["attitude_yaw"] = round(attitude.yaw, 3)

            servo_output_raw = self.__get_mavlink_message('SERVO_OUTPUT_RAW')
            if servo_output_raw:
                for i in range(1, 7):
                    self.drone_status_dict[f"servo_output_{i}"] = getattr(servo_output_raw, f"servo{i}_raw")



            # print(json.dumps(self.drone_status_dict, indent=4, ensure_ascii=False))
            self.latest_status = self.drone_status_dict
            
            # return self.drone_status_dict  # callback can't return value


        except Exception as e:
            print(f"錯誤: {e}")

        self.check_drone_status()


    def get_last_status(self) -> dict:
        if self.latest_status:
            return self.latest_status
        

    def check_drone_status(self):
        status = self.get_last_status()

        if status:
            sensor_health = status["sensor_health"]
            # print(f"系統感測器健康狀態: {sensor_health}")

            # sensor_health check
            self.error_code_list = self.parse_sensor_health(sensor_health)

            # !battery_voltage check 
            # 市電供電時 無法量測電壓 跳過檢查 
            # if status["battery_voltage"] < 44:
            #     self.error_code_list.append(ERROR_CODE.HEALTH_BATTERY_VOLTAGE_ERROR)
 
            # gps hdop check
            if status["gps_hdop"] == UINT16_MAX or status["gps_hdop"] > 100:
                self.error_code_list.append(ERROR_CODE.GPS_HDOP_ERROR)
                self.get_logger().error(f"GPS HDOP 過高: {status['gps_hdop']/100}")

            # gps_satellites_visible check, only have 4 UWB
            if status["gps_satellites_visible"] == UINT8_MAX or status["gps_satellites_visible"] < 4:
                self.error_code_list.append(ERROR_CODE.GPS_SATELLITES_VISIBLE_ERROR)
                self.get_logger().error(f"可見衛星數量過少: {status['gps_satellites_visible']}")

            # drone attitude check
            if abs(math.degrees(status["attitude_roll"])) > 15:
                self.error_code_list.append(ERROR_CODE.ATTITUDE_ROLL_ERROR)
                self.get_logger().error(f"Roll 角度過大: {math.degrees(status['attitude_roll'])}")

            if abs(math.degrees(status["attitude_pitch"])) > 15:
                self.error_code_list.append(ERROR_CODE.ATTITUDE_PITCH_ERROR)
                self.get_logger().error(f"Pitch 角度過大: {math.degrees(status['attitude_pitch'])}")

            # motor output check
            for i in range(1, 7):
                if status[f"servo_output_{i}"] < 1000 or status[f"servo_output_{i}"] > 2000:
                    self.error_code_list.append(ERROR_CODE.MOTOR_OUTPUTS_ERROR)
                    self.get_logger().error(f"馬達 {i} 輸出異常: {status[f'servo_output_{i}']}")

            # if status["range_finder"] :

        

            self.drone_status_dict["error_code"] = self.error_code_list
            


            print(json.dumps(self.drone_status_dict, indent=4, ensure_ascii=False))
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


    def check_usb_status(self):
        """
        發送非同步 USB 檢查請求，並在回應中更新 drone_status_dict。
        """

        request = CheckUSBDevices.Request()
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """
        處理 USB 檢查服務的回應，並將結果發送到伺服器。
        """

        response = future.result()
        up_squared_status_dict = copy.deepcopy(self.up_squared_status_dict)


        if response.success:
            up_squared_status_dict = self.up_squared_status_dict
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
        
        send_json_to_server(url="", data=up_squared_status_dict)

        # print(response)


            # print(device_names)
            # print(type(device_names))  # list
            

        # self.get_logger().info(self.drone_status_dict["usb_status"])







def main(args=None):
    rclpy.init(args=args)

    check_status = None  # 先初始化為 None
    try:
        check_status = Check_status()
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
