'''
確認無線電的usb port
ls /dev/ttyUSB*

給權限
sudo chmod 777 /dev/ttyUSB0


'''



from pymavlink import mavutil
import math


wait_time = 5  # 等待時間（秒）


def get_prearm_status():
    # 連接到 ArduPilot，這裡假設是通過 USB 串口（可以根據實際情況修改端口）
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

    # 等待飛行器心跳包
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    

    while True:
        # 獲取感測器的數據，這些數據可以來自不同的 MAVLink 消息
        try:
            # 獲取電池電壓和電流
            battery_status = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=wait_time)
            if battery_status:
                voltage_battery = battery_status.voltages[0] / 1000.0
                current_battery = battery_status.current_battery / 100.0
                battery_remaining = battery_status.battery_remaining
                print("電池：")
                print(f"電池電壓: {voltage_battery}V, 電池電流: {current_battery}A, 剩餘電量: {battery_remaining}%\n")
            
            # 獲取 ATTITUDE（姿態）數據
            attitude = master.recv_match(type='ATTITUDE', blocking=True, timeout=wait_time)
            if attitude:
                pitch = attitude.pitch
                roll = attitude.roll
                yaw = attitude.yaw
                pitch_deg = math.degrees(pitch)
                roll_deg = math.degrees(roll)
                yaw_deg = math.degrees(yaw)
                print("姿態：")
                print(f"Attitude: Pitch={pitch_deg:.2f}, Roll={roll_deg:.2f}, Yaw={yaw_deg:.2f}\n")
                
            # 馬達輸出
            message = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=5)
            if message:
                # PPM調變如下：1000微秒：0%，2000微秒：100%
                print("馬達：")
                print(f"Motor 1 output: {message.servo1_raw} us")
                print(f"Motor 2 output: {message.servo2_raw} us")
                print(f"Motor 3 output: {message.servo3_raw} us")
                print(f"Motor 4 output: {message.servo4_raw} us")
                print(f"Motor 5 output: {message.servo5_raw} us")
                print(f"Motor 6 output: {message.servo6_raw} us")
                print("\n")

            # 檢查系統狀態
            sys_status = master.recv_match(type='SYS_STATUS', blocking=True, timeout=wait_time)
            if sys_status:
                print("Checking sensors status...")
                if sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                    print("3D Gyroscope is healthy")
                if sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS:
                    print("GPS is healthy")
            # 檢查 GPS 狀態
            gps_status = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=wait_time)
            if gps_status:
                print(f"GPS fix type: {gps_status.fix_type}, Satellites visible: {gps_status.satellites_visible}")
                if gps_status.fix_type >= 3:
                    print("GPS lock acquired")

            # 顯示任何狀態提示
            status_text = master.recv_match(type='STATUSTEXT', blocking=True, timeout=wait_time)
            if status_text:
                print(f"Status message: {status_text.text}")

        except Exception as e:
            print(f"Error receiving sensor data: {e}")




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
        get_prearm_status()
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
