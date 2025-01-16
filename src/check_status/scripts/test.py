from pymavlink import mavutil
import time
import math
import threading
from queue import Queue

# 全局變數來保存 master 物件
master = None
message_queue = Queue()  # 使用佇列來緩衝接收的訊息

# 接收訊息的執行緒函數
def receive_messages():
    global master
    while True:
        try:
            if master is not None:
                msg = master.recv_match(blocking=False)
                if msg is not None:  # 確保 msg 不是 None
                    message_queue.put(msg)  # 將訊息加入佇列
        except Exception as e:
            print(f"接收訊息時出現錯誤: {e}")
            master = None
            time.sleep(1)  # 出錯後等待一段時間再繼續嘗試

        time.sleep(0.005)  # 減少延遲時間來提高訊息接收頻率

# 處理訊息的執行緒函數
def process_messages():
    while True:
        try:
            while not message_queue.empty():
                msg = message_queue.get()
                msg_id = msg.get_msgId()
                
                # 根據 msg_id 來處理 ATTITUDE 和其他訊息
                if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE:
                    roll_deg = math.degrees(msg.roll)
                    pitch_deg = math.degrees(msg.pitch)
                    yaw_deg = math.degrees(msg.yaw)
                    # print(f"姿態 - Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
                elif msg_id == mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS:
                    battery_remaining = msg.battery_remaining
                    voltage_battery = msg.voltage_battery / 1000.0  # 轉換為 V
                    # print(f"電池狀態 - 電壓: {voltage_battery:.2f}V, 剩餘電量: {battery_remaining}%")
                    sensor_health = msg.onboard_control_sensors_health
                    print(sensor_health)
                # 添加其他訊息類型的處理邏輯...
        except Exception as e:
            print(f"處理訊息時出現錯誤: {e}")

        time.sleep(0.001)  # 避免 CPU 過高

# 連接到 Pixhawk 的函數
def connect_to_pixhawk():
    global master
    while master is None:
        try:
            print("嘗試連接到 Pixhawk...")
            master = mavutil.mavlink_connection('/dev/drone_usb', baud=57600)  # 提高波特率以減少延遲
            master.wait_heartbeat(timeout=5)  # 等待心跳訊號
            print(f"已連接到系統 ID: {master.target_system}, 組件 ID: {master.target_component}")

            # 啟用指定訊息類型的流速，增加頻率
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # 姿態訊息
                50,  # 訊息頻率 (Hz)
                1    # 啟用
            )

            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  # 系統狀態訊息
                100,  # 訊息頻率 (Hz)
                1    # 啟用
            )

            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,  # GPS 和額外訊息
                20,  # 訊息頻率 (Hz)
                1    # 啟用
            )
        except Exception as e:
            print(f"連接失敗: {e}")
            master = None
            time.sleep(5)  # 重試前等待 5 秒

# 主程式
if __name__ == "__main__":
    # 啟動接收訊息的執行緒
    threading.Thread(target=receive_messages, daemon=True).start()
    # 啟動處理訊息的執行緒
    threading.Thread(target=process_messages, daemon=True).start()

    # 不斷嘗試連接到 Pixhawk
    while True:
        if master is None:
            connect_to_pixhawk()
        else:
            time.sleep(1)  # 主執行緒可處理其他邏輯或等待
