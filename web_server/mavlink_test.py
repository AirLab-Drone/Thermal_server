'''
確認無線電的usb port
ls /dev/ttyUSB*

給權限
sudo chmod 777 /dev/ttyUSB0


'''


from pymavlink import mavutil
import math
import time

from utils import tools



# 設定串口和波特率
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# 等待心跳包確認連接成功
print("等待飛控心跳包...")
master.wait_heartbeat()
print(f"已連接到系統 {master.target_system}, 組件 {master.target_component}")


def get_system_info():
    # 獲取系統信息
    system_info = master.mav.srcSystem
    print(f"系統信息: {system_info}")

    # 獲取組件信息
    component_info = master.mav.srcComponent
    print(f"組件信息: {component_info}")

    # 獲取飛控版本信息
    version_info = master.mav.version
    print(f"飛控版本信息: {version_info}")






def mavlink_recive_test():
    while True:
        try:
            # 嘗試接收任何消息
            message = master.recv_match(blocking=False, timeout=10)
            if message:
                print(f"Received message: {message.get_type()} - {message}")
            else:
                print("未接收到任何消息...")
            time.sleep(1)  # 稍作延遲避免過度輸出
        except Exception as e:
            print(f"錯誤: {e}")


def get_prearm_status():
    """獲取飛控的主要狀態信息"""
    while True:
        try:
            # 獲取電池狀態
            battery_status = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
            if battery_status:
                voltage_battery = battery_status.voltages[0] / 1000.0  # 轉換成伏特
                current_battery = battery_status.current_battery / 100.0  # 轉換成安培
                battery_remaining = battery_status.battery_remaining  # 剩餘百分比
                print(f"電池電壓: {voltage_battery}V, 電池電流: {current_battery}A, 剩餘電量: {battery_remaining}%")

            # 獲取姿態信息
            attitude = master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
            if attitude:
                pitch = math.degrees(attitude.pitch)  # 轉換成角度
                roll = math.degrees(attitude.roll)
                yaw = math.degrees(attitude.yaw)
                print(f"姿態 - Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}")

            # 獲取 GPS 狀態
            gps_status = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps_status:
                print(f"GPS 固定類型: {gps_status.fix_type}, 可見衛星數: {gps_status.satellites_visible}")

            # 獲取飛控狀態提示
            statustext = master.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
            if statustext:
                print(f"狀態消息: {statustext.text}")

        except Exception as e:
            print(f"錯誤: {e}")
        time.sleep(1)




def get_mavlink_messages():

    # 列出所有可用的 MAVLink 消息
    for msg_name in dir(mavutil.mavlink):
        if msg_name.startswith("MAVLINK_MSG_ID"):
            print(msg_name)

    print("-----------------")



def get_battery_state():
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

    # 等待飛行器心跳包
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    try:
        # 獲取電池電壓和電流
        battery_status = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=wait_time)
        if battery_status:
            voltage_battery = battery_status.voltages[0] / 1000.0
            current_battery = battery_status.current_battery / 100.0
            battery_remaining = battery_status.battery_remaining
            print(f"電池電壓: {voltage_battery}V, 電池電流: {current_battery}A, 剩餘電量: {battery_remaining}%")

            # print(f"Battery Voltage: {voltage_battery}V, Battery Current: {current_battery}A")

    except Exception as e:
        print(f"Error receiving sensor data: {e}")

if __name__ == "__main__":
    try:
        get_system_info()
    except KeyboardInterrupt:
        print("終止監控")