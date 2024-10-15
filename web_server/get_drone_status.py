from pymavlink import mavutil
import time

from utils import tools


class MAVLinkHandler:
    __mav_state_dict = {
        0: "MAV_STATE_UNINIT",
        1: "MAV_STATE_BOOT",
        2: "MAV_STATE_CALIBRATING",
        3: "MAV_STATE_STANDBY",
        4: "MAV_STATE_ACTIVE",
        5: "MAV_STATE_CRITICAL",
        6: "MAV_STATE_EMERGENCY",
        7: "MAV_STATE_POWEROFF",
        8: "MAV_STATE_FLIGHT_TERMINATION"
    }

    def __init__(self, port='/dev/ttyUSB0', baud=57600):
        """
        初始化並建立與飛控的連接。
        :param port: 串口名稱
        :param baud: 波特率
        """
        self.master = mavutil.mavlink_connection(port, baud)
        if not self._wait_for_heartbeat():
            raise ConnectionError("無法與飛控建立連接")

    def _wait_for_heartbeat(self):
        """
        等待心跳包來確認連接成功，失敗則返回 False。
        :return: 連接成功返回 True，失敗則返回 False
        """
        try:
            print("等待飛控心跳包...")
            self.master.wait_heartbeat(timeout=10)  # 設置超時時間
            print(f"已連接到系統 {self.master.target_system}, 組件 {self.master.target_component}")
            return True
        except Exception as e:
            print(f"連接失敗: {e}")
            return False

    def get_sys_status(self):
        """
        獲取飛控的系統狀態信息。
        """
        try:
            sys_status = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if sys_status:
                battery_voltage =  sys_status.voltage_battery
                battery_current =  sys_status.current_battery
                battery_remaining = sys_status.battery_remaining
                sensors_health = sys_status.onboard_control_sensors_health
                sensors_enabled = sys_status.onboard_control_sensors_enabled
                sensors_present = sys_status.onboard_control_sensors_present
                # tools.parse_sensor_health(sensors_health)
                # print(f"感測器健康狀態: {sensors_health} \
                #         sensors_enabled: {sensors_enabled} \
                #         sensors_present: {sensors_present}")

                print("sensors_health")
                tools.parse_sensor_health(sensors_health)

                print("sensors_enabled")
                tools.parse_sensor_health(sensors_enabled)

                print("sensors_present")
                tools.parse_sensor_health(sensors_present)

                # print(f"感測器健康狀態: {sensors_health}, 剩餘電池: {battery_remaining}%")
            else:
                print("未接收到 SYS_STATUS 消息。")
        except Exception as e:
            print(f"錯誤: {e}")


    def monitor_mav_state(self):
        """
        持續監控 MAVLink 系統狀態。
        """
        while True:
            self.get_sys_status()
            print("-----------------------------------------------------------------\n\n")
            time.sleep(1)

if __name__ == "__main__":
    try:
        mavlink_handler = MAVLinkHandler(port='/dev/ttyUSB0', baud=57600)
        mavlink_handler.monitor_mav_state()
    except ConnectionError as e:
        print(f"程序錯誤: {e}")
    except KeyboardInterrupt:
        print("監控終止")
