#!/usr/bin/env python3

# '''
# 確認無線電的usb port
# ls /dev/ttyUSB*

# 給權限
# sudo chmod 777 /dev/ttyUSB0
# '''




from datetime import datetime, timezone, time
import json
import math
import requests

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from check_status_py.error_code import *
from check_status_py.tools import post_to_server, check_port_exists, ros2_time_to_taiwan_timezone, print_json

class CheckDroneStatus(Node):

    def __init__(self):
        super().__init__('check_drone_status')

        self.drone_status_dict = {
            "upload_time": None,
            "mavlink_status": None,
            "sensor_health": None,
            "battery_voltage": None,
            "battery_current": None,
            "battery_remaining": None,
            "gps_hdop": None,
            "gps_satellites_visible": None,
            "attitude_roll": None,
            "attitude_pitch": None,
            "attitude_yaw": None,
            "servo_output_1": None,
            "servo_output_2": None,
            "servo_output_3": None,
            "servo_output_4": None,
            "servo_output_5": None,
            "servo_output_6": None,
            "error_code": set()
        }

        self.__server_url = "http://127.0.0.1:5000/upload/DroneStatus"

        self.mavlink_port = '/dev/drone_usb'  # wireless connection
        self.mavlink_baud = 57600
        self.master = None
        self.mavlink_status = None

        # 指定的上傳時間
        # self.target_times = [time(1, 0), 
        #                      time(2, 0), 
        #                      time(3, 0), 
        #                      time(4, 0)]
        

        # 每小時檢查一次
        self.target_times = [time(hour, 0) for hour in range(1, 24)]



        # 每分鐘檢查一次
        self.timer = self.create_timer(60, self.check_and_upload_at_target_times)

    def check_and_upload_at_target_times(self):
        now = datetime.now().time()
        if any(now.hour == t.hour and now.minute == t.minute for t in self.target_times):
            self.perform_check_and_upload()

    def perform_check_and_upload(self):
        try:
            self.check_mavlink_connection()
            upload_drone_status = self.drone_status_dict.copy()
            new_errors = self.check_drone_status(upload_drone_status)
            upload_drone_status["error_code"] = upload_drone_status["error_code"].union(new_errors)
            upload_drone_status["error_code"] = list(upload_drone_status["error_code"])

            # 確保 upload_time 是字串
            if isinstance(upload_drone_status["upload_time"], datetime):
                upload_drone_status["upload_time"] = upload_drone_status["upload_time"].isoformat()

            print_json(upload_drone_status)

            # 將資料發送到伺服器
            response = post_to_server(url=self.__server_url, json=upload_drone_status).status_code
            if response == 200:
                self.get_logger().info(f"上傳成功: {upload_drone_status}")
            else:
                self.get_logger().error(f"上傳失敗: {upload_drone_status}")

        except Exception as e:
            self.get_logger().error(f"{e}")

    def check_mavlink_connection(self):
        self.drone_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(self.get_clock().now())
        self.drone_status_dict["error_code"].clear()

        if not check_port_exists(self.mavlink_port):
            self.master = None
            self.mavlink_status = 'not_found'
            self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_PORT_ERROR)
            self.drone_status_dict["mavlink_status"] = self.mavlink_status
            return

        if self.master is None:
            self.mavlink_status = 'connecting'
            if not self.__connect_mavlink(self.mavlink_port, self.mavlink_baud):
                self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
                self.drone_status_dict["mavlink_status"] = self.mavlink_status
                self.master = None
                return
            else:
                self.get_logger().info(f"已連接到 pixhawk6c")

        if self.master is not None:
            self.mavlink_status = 'is_connected'
            self.drone_status_dict["mavlink_status"] = self.mavlink_status
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
                    self.master = None
                else:
                    self.update_status_from_messages()
            except Exception as e:
                self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
                self.master = None

    def update_status_from_messages(self):
        if 'ATTITUDE' in self.master.messages:
            attitude = self.master.messages['ATTITUDE']
            self.drone_status_dict.update({
                "attitude_roll": attitude.roll,
                "attitude_pitch": attitude.pitch,
                "attitude_yaw": attitude.yaw
            })

        if 'SYS_STATUS' in self.master.messages:
            sys_status = self.master.messages['SYS_STATUS']
            self.drone_status_dict.update({
                "sensor_health": sys_status.onboard_control_sensors_health,
                "battery_voltage": sys_status.voltage_battery,
                "battery_current": sys_status.current_battery,
                "battery_remaining": sys_status.battery_remaining
            })

        if 'GPS_RAW_INT' in self.master.messages:
            gps_raw = self.master.messages['GPS_RAW_INT']
            self.drone_status_dict.update({
                "gps_hdop": gps_raw.eph,
                "gps_satellites_visible": gps_raw.satellites_visible
            })

        if 'SERVO_OUTPUT_RAW' in self.master.messages:
            servo_output = self.master.messages['SERVO_OUTPUT_RAW']
            for i in range(1, 7):
                self.drone_status_dict[f"servo_output_{i}"] = getattr(servo_output, f"servo{i}_raw")

    def __connect_mavlink(self, port, baud) -> bool:
        try:
            self.master = mavutil.mavlink_connection(port, baud)
            if not self.master.wait_heartbeat(timeout=5):
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10, 1
                )
                return False
            else:
                return True
        except Exception:
            return False

    def check_drone_status(self, drone_status_dict) -> set:
        error_list_set = set()

        if drone_status_dict["gps_hdop"] is not None:
            if drone_status_dict["gps_hdop"] == UINT16_MAX or drone_status_dict["gps_hdop"] > 100:
                error_list_set.add(ERROR_CODE.GPS_HDOP_ERROR)

        if drone_status_dict["gps_satellites_visible"] is not None:
            if drone_status_dict["gps_satellites_visible"] == UINT8_MAX or drone_status_dict["gps_satellites_visible"] < 4:
                error_list_set.add(ERROR_CODE.GPS_SATELLITES_VISIBLE_ERROR)

        if drone_status_dict["attitude_roll"] is not None:
            if abs(math.degrees(drone_status_dict["attitude_roll"])) > 15:
                error_list_set.add(ERROR_CODE.ATTITUDE_ROLL_ERROR)

        if drone_status_dict["attitude_pitch"] is not None:
            if abs(math.degrees(drone_status_dict["attitude_pitch"])) > 15:
                error_list_set.add(ERROR_CODE.ATTITUDE_PITCH_ERROR)

        if drone_status_dict["servo_output_1"] is not None:
            for i in range(1, 7):
                if drone_status_dict[f"servo_output_{i}"] < 1000 or drone_status_dict[f"servo_output_{i}"] > 2000:
                    error_list_set.add(ERROR_CODE.MOTOR_OUTPUTS_ERROR)

        return error_list_set


def main(args=None):
    rclpy.init(args=args)
    check_status = CheckDroneStatus()
    rclpy.spin(check_status)
    check_status.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()








# ----------------------------------- 舊ㄉ程式 ----------------------------------- #


# from datetime import datetime, timezone
# import json
# import math
# import requests

# from pymavlink import mavutil

# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration


# from check_status_py.error_code import *
# from check_status_py.tools import post_to_server, check_port_exists, ros2_time_to_taiwan_timezone, print_json


# '''
# database 有三個table
#     1. 飛控狀態 
#     2. up squared 狀態 (RGB, Thermal) 
#     3. 環境熱像儀狀態
# ''' 

# # TODO: 一直要資料(並確認是否有連接) 但上傳的時間不一樣 ex:一天一次



# class check_Drone_status(Node):

#     def __init__(self):
#         super().__init__('check_drone_status')

#         self.drone_status_dict = {  
#             "upload_time": None,                            # datetime      Taiwan timezone
#             "mavlink_status": None,                         # str           mavlink status
#             "sensor_health": None,
#             "battery_voltage": None,                        # uint16_t      mV      invalid:UINT16_MAX
#             "battery_current": None,                        # int16_t	    cA	    invalid:-1
#             "battery_remaining": None,                      # int8_t	    %	    invalid:-1
#             "gps_hdop": None,                               # 1E-2	                invalid:UINT16_MAX	GPS HDOP (unitless * 100)
#             "gps_satellites_visible": None,                 # uint8_t			    invalid:UINT8_MAX
#             "attitude_roll": None,                          # float	        rad	    Roll angle (-pi..+pi)
#             "attitude_pitch": None,                         # float	        rad	    Pitch angle (-pi..+pi)
#             "attitude_yaw": None,                           # float	        rad	    Yaw angle (-pi..+pi)   
#             "servo_output_1": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "servo_output_2": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "servo_output_3": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "servo_output_4": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "servo_output_5": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "servo_output_6": None,                         # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
#             "error_code": set()                             # set           error code
#             }
        


#         self.__server_url = "http://127.0.0.1:5000/upload/DroneStatus"

#         # ---------------------------------- mavlink --------------------------------- #
#         # self.mavlink_port = '/dev/ttyACM0'      # wired connection
#         self.mavlink_port = '/dev/drone_usb'  # wireless connection
#         self.mavlink_baud = 57600
    
#         self.master = None

#         self.mavlink_status = None  # 狀態標記：'not_found', 'connecting', 'is_connected'


#         self.check_mavlink_connection_timer = self.create_timer(0.01, self.check_mavlink_connection_callback)

#         self.upload_to_server_timmer = self.create_timer(0.005, self.upload_to_server_callback)
     

#     def upload_to_server_callback(self):
#         try:
            
#             upload_drone_status = self.drone_status_dict.copy()

#             new_errors = self.check_drone_status(upload_drone_status)
#             upload_drone_status["error_code"] = upload_drone_status["error_code"].union(new_errors)
#             upload_drone_status["error_code"] = list(upload_drone_status["error_code"])


#             # 確保 upload_time 是字串，避免 JSON 序列化問題
#             if isinstance(upload_drone_status["upload_time"], datetime):
#                 upload_drone_status["upload_time"] = upload_drone_status["upload_time"].isoformat()

#             print_json(upload_drone_status)

#             # 將資料發送到伺服器
#             response = post_to_server(url=self.__server_url, json=upload_drone_status).status_code
#             if response == 200:
#                 self.get_logger().info(f"上傳成功: {upload_drone_status}")
#             else:
#                 self.get_logger().error(f"上傳失敗: {upload_drone_status}")

#         except Exception as e:
#             self.get_logger().error(f"{e}")







#     def check_mavlink_connection_callback(self):
#         current_time = self.get_clock().now()
        

#         self.drone_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)
#         self.drone_status_dict["error_code"].clear()


        
#         # --------------------------- 檢查 mavlink port 是否存在 --------------------------- #

#         if not check_port_exists(self.mavlink_port):
#             self.master = None
#             if self.mavlink_status != 'not_found':
#                 self.mavlink_status = 'not_found'
                
#             self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_PORT_ERROR)
#             self.drone_status_dict["mavlink_status"] = self.mavlink_status
#             return
        

#         # ------------------------------- Mavlink 第一次連接 ------------------------------ #

#         if self.master is None:
#             if self.mavlink_status != 'connecting':
#                 self.mavlink_status = 'connecting'
#                 self.get_logger().info(f"找到 {self.mavlink_port}，正在嘗試連接飛控...")
            
#             if not self.__connect_mavlink(self.mavlink_port, self.mavlink_baud):
#                 self.get_logger().warn(f"無法連接到 pixhawk6c, 等待重新連接...")
#                 self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
#                 self.drone_status_dict["mavlink_status"] = self.mavlink_status
#                 self.master = None
#                 return
#             else:
#                 self.get_logger().info(f"已連接到 pixhawk6c")



#         # ----------------------------------- 取得狀態 ----------------------------------- #
#         if self.master is not None:
            
#             self.mavlink_status = 'is_connected'
#             self.drone_status_dict["mavlink_status"] = self.mavlink_status
#             try:
                
#                 msg = self.master.recv_match(blocking=True, timeout=0.5)

#                 if not msg:
#                     self.get_logger().warn(f"未接收到訊息，嘗試重新連接...")
#                     self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
#                     self.master = None
#                     return
                
#                 else:
#                     if 'ATTITUDE' in self.master.messages:
#                         attitude = self.master.messages['ATTITUDE']
#                         self.drone_status_dict["attitude_roll"] = attitude.roll
#                         self.drone_status_dict["attitude_pitch"] = attitude.pitch
#                         self.drone_status_dict["attitude_yaw"] = attitude.yaw


#                     if 'SYS_STATUS' in self.master.messages:
#                         sys_status = self.master.messages['SYS_STATUS']
#                         self.drone_status_dict["sensor_health"] = sys_status.onboard_control_sensors_health
#                         self.drone_status_dict["battery_voltage"] = sys_status.voltage_battery
#                         self.drone_status_dict["battery_current"] = sys_status.current_battery
#                         self.drone_status_dict["battery_remaining"] = sys_status.battery_remaining

#                     if 'GPS_RAW_INT' in self.master.messages:
#                         gps_raw = self.master.messages['GPS_RAW_INT']
#                         self.drone_status_dict["gps_hdop"] = gps_raw.eph
#                         self.drone_status_dict["gps_satellites_visible"] = gps_raw.satellites_visible

#                     if 'SERVO_OUTPUT_RAW' in self.master.messages:
#                         servo_output = self.master.messages['SERVO_OUTPUT_RAW']
#                         for i in range(1, 7):
#                             self.drone_status_dict[f"servo_output_{i}"] = getattr(servo_output, f"servo{i}_raw")


#             except Exception as e:
#                 self.get_logger().warn(f"接收訊息時出現錯誤: {e}, 嘗試重新連接...")
#                 self.drone_status_dict["error_code"].add(ERROR_CODE.MAVLINK_CONNECTION_ERROR)
#                 self.master = None





#     def __connect_mavlink(self, port, baud) -> bool:
#         """
#         嘗試連接 MAVLink，若無法在指定時間內連接成功，則返回 False。
#         """
#         try:
#             self.master = mavutil.mavlink_connection(port, baud)
            
#             if not self.master.wait_heartbeat(timeout=5):
#                 # 啟用指定訊息類型的流速
#                 self.master.mav.request_data_stream_send(
#                     self.master.target_system,
#                     self.master.target_component,
#                     mavutil.mavlink.MAV_DATA_STREAM_ALL,
#                     10,  # 訊息頻率 (Hz)
#                     1    # 啟用
#                 )
#                 return False
#             else:
#                 return True
#         except Exception as e:
#             # self.get_logger().error(f"無法連接到 Mavlink: {e}")
#             return False




#     def check_drone_status(self, drone_status_dict) -> set:

#         current_time = self.get_clock().now()
#         drone_status_dict["upload_time"] = ros2_time_to_taiwan_timezone(current_time)
#         error_list_set = set()

#         if drone_status_dict["sensor_health"] is not None:
#             error_list_set.update(set(self.parse_sensor_health(drone_status_dict["sensor_health"])))
#             # print(self.parse_sensor_health(drone_status_dict["sensor_health"]))


#         # !battery_voltage check 
#         # 市電供電時 無法量測電壓 跳過檢查 
#         # if status["battery_voltage"] < 44:
#         #     self.error_code_list.append(ERROR_CODE.HEALTH_BATTERY_VOLTAGE_ERROR)


#         if drone_status_dict["gps_hdop"] is not None:
#             if drone_status_dict["gps_hdop"] == UINT16_MAX or drone_status_dict["gps_hdop"] > 100:
#                 error_list_set.add(ERROR_CODE.GPS_HDOP_ERROR)

#         if drone_status_dict["gps_satellites_visible"] is not None:
#             if drone_status_dict["gps_satellites_visible"] == UINT8_MAX or drone_status_dict["gps_satellites_visible"] < 4:
#                 error_list_set.add(ERROR_CODE.GPS_SATELLITES_VISIBLE_ERROR)

#         if drone_status_dict["attitude_roll"] is not None:
#             if abs(math.degrees(drone_status_dict["attitude_roll"])) > 15:
#                 error_list_set.add(ERROR_CODE.ATTITUDE_ROLL_ERROR)
        
#         if drone_status_dict["attitude_pitch"] is not None:
#             if abs(math.degrees(drone_status_dict["attitude_pitch"])) > 15:
#                 error_list_set.add(ERROR_CODE.ATTITUDE_PITCH_ERROR)
        
#         if drone_status_dict["servo_output_1"] is not None:
#             for i in range(1, 7):
#                 if drone_status_dict[f"servo_output_{i}"] < 1000 or drone_status_dict[f"servo_output_{i}"] > 2000:
#                     error_list_set.add(ERROR_CODE.MOTOR_OUTPUTS_ERROR)

#         return error_list_set
        



        
#     # 解析感測器健康狀態
#     def parse_sensor_health(self, sensors_health) -> list:
#         '''
#         :param sensors_health: sys_status.onboard_control_sensors_health 的值
#         :return: 回傳每個感測器合在一起成功或錯誤代碼陣列
#         '''

#         erroe_code_list = []

#         # print(f"系統感測器健康狀態: {sensors_health}")
#         for flag, sensor in sensor_flags.items():
#             if not sensors_health & flag:
#                 # print(f"{sensor[0]}: False")
#                 erroe_code_list.append(sensor[1])
#             else:
#                 pass
#                 # print(f"{sensor[0]}: True")
#                 # erroe_code_list.append(ERROR_CODE.SUCCESS)
                
#         # print(erroe_code_list)
#         return erroe_code_list







# def main(args=None):
#     rclpy.init(args=args)

#     check_status = None  # 先初始化為 None
#     try:
#         check_status = check_Drone_status()
#         rclpy.spin(check_status)
#     except Exception as e:
#         print(f"發生錯誤: {e}")
#     finally:
#         if check_status:  # 確認 check_status 已被初始化，才調用 destroy_node()
#             check_status.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#     # print(len(sensor_flags))
#     {
#         "upload_time": "2024-11-21T20:30:26.209132+08:00",
#         "mavlink_status": "is_connected",
#         "sensor_health": 1198562607,
#         "battery_voltage": 50233,
#         "battery_current": 148,
#         "battery_remaining": 93,
#         "gps_hdop": 255,
#         "gps_satellites_visible": 0,
#         "attitude_roll": -0.019706612452864647,
#         "attitude_pitch": 0.003123760223388672,
#         "attitude_yaw": 1.3226935863494873,
#         "servo_output_1": 1000,
#         "servo_output_2": 1000,
#         "servo_output_3": 1000,
#         "servo_output_4": 1000,
#         "servo_output_5": 1000,
#         "servo_output_6": 1000,
#         "error_code": [
#             128,
#             113,
#             114,
#             132,
#             133,
#             116
#         ]
#     }
