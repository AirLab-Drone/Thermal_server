'''
確認無線電的usb port
ls /dev/ttyUSB*

給權限
sudo chmod 777 /dev/ttyUSB0

'''

from pymavlink import mavutil
import json
import requests
import time

class MAVLinkHandler:

    def __init__(self, port='/dev/ttyUSB0', baud=57600):
        """
        初始化並建立與飛控的連接。
        :param port: 串口名稱
        :param baud: 波特率
        """
        self.master = mavutil.mavlink_connection(port, baud)
        if not self._wait_for_heartbeat():
            raise ConnectionError("無法與飛控建立連接")
        
        self.drone_status_dict = {  
            "sensor_health": 0,
            "battery_voltage": 0,           # uint16_t      mV      invalid:UINT16_MAX
            "battery_current": 0,           # int16_t	    cA	    invalid:-1
            "battery_remaining": 0,         # int8_t	    %	    invalid:-1
            "gps_hdop": 0,                  # 1E-2	                invalid:UINT16_MAX	GPS HDOP (unitless * 100)
            "gps_satellites_visible": 0,    # uint8_t			    invalid:UINT8_MAX
            "attitude_roll": 0,             # float	        rad	    Roll angle (-pi..+pi)
            "attitude_pitch": 0,            # float	        rad	    Pitch angle (-pi..+pi)
            "attitude_yaw": 0,              # float	        rad	    Yaw angle (-pi..+pi)   
            "servo_output_1": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            "servo_output_2": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            "servo_output_3": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            "servo_output_4": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            "servo_output_5": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            "servo_output_6": 0,            # uint16_t      us      PWM 通常在 1000 到 2000 微秒之間
            
            }

    def _wait_for_heartbeat(self, retries=3):
        """
        等待心跳包來確認連接成功，失敗則重試指定次數。
        :param retries: 重試次數
        :return: 連接成功返回 True，失敗則返回 False
        """
        for attempt in range(retries):
            try:
                print(f"等待飛控心跳包...（嘗試 {attempt + 1}/{retries}）")
                self.master.wait_heartbeat(timeout=10)
                print(f"已連接到系統 {self.master.target_system}, 組件 {self.master.target_component}")
                return True
            except Exception as e:
                print(f"第 {attempt + 1} 次連接失敗: {e}")
                if attempt == retries - 1:
                    return False
        
    def _get_mavlink_message(self, message_type, timeout=5):
        try:
            return self.master.recv_match(type=message_type, blocking=True, timeout=timeout)
        except Exception as e:
            print(f"獲取 {message_type} 消息時出現錯誤: {e}")
            return None

    def get_drone_status(self) -> dict:
        """
        獲取飛控的狀態信息。
        :return: 包含飛控狀態信息原始資料的字典
        """
        try:
            sys_status = self._get_mavlink_message('SYS_STATUS')
            if sys_status:
                self.drone_status_dict["sensor_health"] = sys_status.onboard_control_sensors_health
                
                self.drone_status_dict["battery_voltage"] = sys_status.voltage_battery
                self.drone_status_dict["battery_current"] = sys_status.current_battery
                self.drone_status_dict["battery_remaining"] = sys_status.battery_remaining

            gps_raw_int = self._get_mavlink_message('GPS_RAW_INT')
            if gps_raw_int:
                self.drone_status_dict["gps_hdop"] = gps_raw_int.eph
                self.drone_status_dict["gps_satellites_visible"] = gps_raw_int.satellites_visible

            attitude = self._get_mavlink_message('ATTITUDE')
            if attitude:
                self.drone_status_dict["attitude_roll"] = attitude.roll
                self.drone_status_dict["attitude_pitch"] = attitude.pitch
                self.drone_status_dict["attitude_yaw"] = attitude.yaw

                # self.drone_status_dict["attitude_roll"] = round(attitude.roll, 3)
                # self.drone_status_dict["attitude_pitch"] = round(attitude.pitch, 3)
                # self.drone_status_dict["attitude_yaw"] = round(attitude.yaw, 3)

            servo_output_raw = self._get_mavlink_message('SERVO_OUTPUT_RAW')
            if servo_output_raw:
                for i in range(1, 7):
                    self.drone_status_dict[f"servo_output_{i}"] = getattr(servo_output_raw, f"servo{i}_raw")


            print(json.dumps(self.drone_status_dict, indent=4, ensure_ascii=False))
            return self.drone_status_dict

        except Exception as e:
            print(f"錯誤: {e}")


    def send_to_server(self, data: dict):
        """
        將飛控狀態信息發送到服務器。
        :param data: 包含飛控狀態信息的字典
        """
        try:
            response = requests.post("http://127.0.0.1:5000/", json=data)
            if response.status_code == 200:
                print("數據發送成功")
            else:
                print(f"數據發送失敗: {response.status_code}")
        except requests.RequestException as e:
            print(f"數據發送失敗: {e}")



    def __debug_function(self, type):
        try:
            debug_status = self._get_mavlink_message(type)
            if debug_status:
                print(f"溫度: {debug_status.temperature}")
                print(f"電壓: {debug_status.voltages}")
                print(f"電流: {debug_status.current_battery}")
                print(f"剩餘電量: {debug_status.battery_remaining}")
        except Exception as e:
            print(f"錯誤: {e}")





if __name__ == "__main__":
    try:
        mavlink_handler = MAVLinkHandler(port='/dev/ttyUSB0', baud=57600)
        while True:
            # mavlink_handler._MAVLinkHandler__debug_function('BATTERY_STATUS')
            mavlink_handler.send_to_server(mavlink_handler.get_drone_status())
            time.sleep(1)  # 每隔 1 秒更新一次狀態


        
    except ConnectionError as e:
        print(f"程序錯誤: {e}")
    except KeyboardInterrupt:
        print("監控終止")





    test_dict = {
    "sensor_health": 1198562671,
    "battery_voltage": 49134,
    "battery_current": 155,
    "battery_remaining": 69,
    "gps_hdop": 82,
    "gps_satellites_visible": 4,
    "attitude_roll": -0.011641320772469044,
    "attitude_pitch": 0.004098756238818169,
    "attitude_yaw": 2.275703191757202,
    "servo_output_1": 1000,
    "servo_output_2": 1000,
    "servo_output_3": 1000,
    "servo_output_4": 1000,
    "servo_output_5": 1000,
    "servo_output_6": 1000
    }
