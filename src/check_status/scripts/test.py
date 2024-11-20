#!/usr/bin/env python3
from pymavlink import mavutil
import time
import math
import threading

# 全局變數來保存 master 物件
master = None

# 接收訊息的執行緒函數
def receive_messages():
    global master
    while True:
        try:
            if master is not None:
                msg = master.recv_match(blocking=False)
                if msg:
                    print(f"{msg}")
            time.sleep(0.1)  # 避免 CPU 過高
        except Exception as e:
            print(f"接收訊息時出現錯誤: {e}")
            master = None
            time.sleep(1)

# 連接到 Pixhawk 的函數
def connect_to_pixhawk():
    global master
    while master is None:
        try:
            print("嘗試連接到 Pixhawk...")
            master = mavutil.mavlink_connection('/dev/drone_usb', baud=57600)
            master.wait_heartbeat(timeout=10)
            print(f"已連接到系統 ID: {master.target_system}, 組件 ID: {master.target_component}")

            # 啟用指定訊息類型的流速
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # 訊息頻率 (Hz)
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

    # 不斷嘗試連接到 Pixhawk
    while True:
        if master is None:
            connect_to_pixhawk()
        else:
            # 主執行緒可處理其他邏輯
            try:
                msg = master.recv_match(blocking=False)
                if not msg:
                    print("未接收到訊息，重新連接...")
                    master = None
            except Exception as e:
                print(f"檢查訊息時出現錯誤: {e}")
                master = None
            time.sleep(1)  # 每秒檢查一次連線狀態
