

# sys_status.onboard_control_sensors_health 的值
# from https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR


# sensor_flags = {
#     0x01: "3D Gyro",
#     0x02: "3D Accelerometer",
#     0x04: "3D Magnetometer",
#     0x08: "Absolute Pressure",
#     0x10: "Differential Pressure",
#     0x20: "GPS",
#     0x40: "Optical Flow",
#     0x80: "Vision Position",
#     0x100: "Laser Position",
#     0x200: "External Ground Truth",
#     0x400: "3D Angular Rate Control",
#     0x800: "Attitude Stabilization",
#     0x1000: "Yaw Position",
#     0x2000: "Z/Altitude Control",
#     0x4000: "X/Y Position Control",
#     0x8000: "Motor Outputs",
#     0x10000: "RC Receiver",
#     0x20000: "2nd 3D Gyro",
#     0x40000: "2nd 3D Accelerometer",
#     0x80000: "2nd 3D Magnetometer",
#     0x100000: "Geofence",
#     0x200000: "AHRS",
#     0x400000: "Terrain",
#     0x800000: "Reverse Motor",
#     0x1000000: "Logging",
#     0x2000000: "Battery",
#     0x4000000: "Proximity",
#     0x8000000: "Satellite Communication",
#     0x10000000: "Pre-arm Check",
#     0x20000000: "Obstacle Avoidance",
#     0x40000000: "Propulsion",
#     0x80000000: "Extended Bit-field Used"
# }

sensor_flags = {
    0x01: "3D 陀螺儀",
    0x02: "3D 加速度計",
    0x04: "3D 磁力計",
    0x08: "絶對壓力",
    0x10: "差壓計",
    0x20: "GPS",
    0x40: "光流",
    0x80: "視覺定位",
    0x100: "雷射定位",
    0x200: "外部地面真值",
    0x400: "3D 角速度控制",
    0x800: "姿態穩定",
    0x1000: "偏航位置",
    0x2000: "Z 軸/高度控制",
    0x4000: "X/Y 軸位置控制",
    0x8000: "馬達輸出",
    0x10000: "RC 接收器",
    0x20000: "第二 3D 陀螺儀",
    0x40000: "第二 3D 加速度計",
    0x80000: "第二 3D 磁力計",
    0x100000: "地理圍欄",
    0x200000: "AHRS（姿態與航向參考系統）",
    0x400000: "地形",
    0x800000: "反向馬達",
    0x1000000: "日誌記錄",
    0x2000000: "電池",
    0x4000000: "接近感測",
    0x8000000: "衛星通信",
    0x10000000: "起飛前檢查",
    0x20000000: "障礙避免",
    0x40000000: "推進系統",
    0x80000000: "擴展位元字段已使用"
}

# 解析感測器健康狀態
def parse_sensor_health(sensors_health):
    
    print(f"系統感測器健康狀態: {sensors_health}")
    for flag, sensor in sensor_flags.items():
        if sensors_health & flag:
            print(f"{sensor}: True")
            # print(f"{sensor}: True, {sensors_health & flag}")
        else:
            print(f"{sensor}: False")
            # print(f"{sensor}: False, {sensors_health & flag}")


if __name__ == "__main__":
    # sensors_health = 1196465487  # 你的 onboard_control_sensors_health 數值
    sensors_health = 2147483649  # 你的 onboard_control_sensors_health 數值
    parse_sensor_health(sensors_health)

