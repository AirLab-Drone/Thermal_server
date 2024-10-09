from pymavlink import mavutil
import math


wait_time = 5  # 等待時間（秒）


def get_prearm_status():
    # 連接到 ArduPilot，這裡假設是通過 USB 串口（可以根據實際情況修改端口）
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

    # 等待飛行器心跳包
    master.wait_heartbeat()
    print("Heartbeat received. Connected to the vehicle.")

    while True:
        # 獲取感測器的數據，這些數據可以來自不同的 MAVLink 消息
        try:
            # 獲取電池電壓和電流
            battery_status = master.recv_match(type='SYS_STATUS', blocking=True, timeout=wait_time)
            if battery_status:
                voltage_battery = battery_status.voltage_battery / 1000.0  # 轉換為伏特
                current_battery = battery_status.current_battery / 100.0   # 轉換為安培
                print(f"Battery Voltage: {voltage_battery}V, Battery Current: {current_battery}A")
            
            # 獲取 ATTITUDE（姿態）數據
            attitude = master.recv_match(type='ATTITUDE', blocking=True, timeout=wait_time)
            if attitude:
                pitch = attitude.pitch
                roll = attitude.roll
                yaw = attitude.yaw
                pitch_deg = math.degrees(pitch)
                roll_deg = math.degrees(roll)
                yaw_deg = math.degrees(yaw)
                print(f"Attitude: Pitch={pitch_deg:.2f}, Roll={roll_deg:.2f}, Yaw={yaw_deg:.2f}")
                
            message = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=5)
            if message:
                for i in range(6):  # 6個馬達
                    output = getattr(message, f'servo{i+1}_raw', None)
                    if output is not None:
                        # print(f"Motor {i + 1} output: {output} us")    # output 是 PWM 脈衝寬度 (us) 1000-2000
                        pass
            else:
                print("No motor status message received.")

        except Exception as e:
            print(f"Error receiving sensor data: {e}")




def get_mavlink_messages():

    # 列出所有可用的 MAVLink 消息
    for msg_name in dir(mavutil.mavlink):
        if msg_name.startswith("MAVLINK_MSG_ID"):
            print(msg_name)


if __name__ == "__main__":
    try:
        get_prearm_status()
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
