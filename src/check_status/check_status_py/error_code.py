#!/usr/bin/env python3

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


import pprint 
pp = pprint.PrettyPrinter(indent=4)


UINT16_MAX = (1 << 16) - 1  # 65535
UINT8_MAX = (1 << 8) - 1  # 255


class ERROR_CODE:
    SUCCESS = 0

    # ------------------------------------ FCU ----------------------------------- #

    # sensor health
    HEALTH_GYRO_3D_ERROR = 100
    HEALTH_ACCELEROMETER_3D_ERROR = 101
    HEALTH_MAGNETOMETER_3D_ERROR = 102
    HEALTH_ABSOLUTE_PRESSURE_ERROR = 103
    HEALTH_DIFFERENTIAL_PRESSURE_ERROR = 104       # no use
    HEALTH_GPS_ERROR = 105
    HEALTH_OPTICAL_FLOW_ERROR = 106                # no use
    HEALTH_VISION_POSITION_ERROR = 107             # no use
    HEALTH_LASER_POSITION_ERROR = 108
    HEALTH_EXTERNAL_GROUND_TRUTH_ERROR = 109       # no use
    HEALTH_ANGULAR_RATE_CONTROL_ERROR = 110
    HEALTH_ATTITUDE_STABILIZATION_ERROR = 111
    HEALTH_YAW_POSITION_ERROR = 112
    HEALTH_ALTITUDE_CONTROL_ERROR = 113
    HEALTH_POSITION_CONTROL_ERROR = 114
    HEALTH_MOTOR_OUTPUTS_ERROR = 115
    HEALTH_RC_RECEIVER_ERROR = 116
    HEALTH_SECOND_3D_GYRO_ERROR = 117              # no use
    HEALTH_SECOND_3D_ACCELEROMETER_ERROR = 118     # no use
    HEALTH_SECOND_3D_MAGNETOMETER_ERROR = 119      # no use
    HEALTH_GEOFENCE_ERROR = 120                    # no use
    HEALTH_AHRS_ERROR = 121
    HEALTH_TERRAIN_ERROR = 122
    HEALTH_REVERSE_MOTOR_ERROR = 123               # no use
    HEALTH_LOGGING_ERROR = 124                     # no use
    HEALTH_BATTERY_ERROR = 125
    HEALTH_PROXIMITY_ERROR = 126                   # no use
    HEALTH_SATELLITE_COMMUNICATION_ERROR = 127     # no use
    HEALTH_PRE_ARM_CHECK_ERROR = 128
    HEALTH_OBSTACLE_AVOIDANCE_ERROR = 129          # no use
    HEALTH_PROPULSION_ERROR = 130
    HEALTH_EXTENDED_BIT_FIELD_USED_ERROR = 131     # no use

    GPS_HDOP_ERROR = 132
    GPS_SATELLITES_VISIBLE_ERROR = 133

    ATTITUDE_ROLL_ERROR = 134
    ATTITUDE_PITCH_ERROR = 135

    MOTOR_OUTPUTS_ERROR = 136

    MAVLINK_PORT_ERROR = 137
    MAVLINK_CONNECTION_ERROR = 138

    
    # ------------------------------ up squared i12 ------------------------------ #

    UP_SQUARE_SERVICE_ERROR = 200
    UP_SQUARE_RGB_CAMERA_ERROR = 201
    UP_SQUARE_THERMAL_CAMERA_ERROR = 202

    # ------------------------------ Thermal camera ------------------------------ #
    THERMAL_IMG_ERROR = 300
    THERMAL_HOT_SPOT_TEMP_ERROR = 301





# --------------------------- complete sensor_flags -------------------------- #
sensor_flags = {
    0x01: ["3D 陀螺儀", ERROR_CODE.HEALTH_GYRO_3D_ERROR],
    0x02: ["3D 加速度計", ERROR_CODE.HEALTH_ACCELEROMETER_3D_ERROR],
    0x04: ["3D 磁力計", ERROR_CODE.HEALTH_MAGNETOMETER_3D_ERROR],
    0x08: ["絶對壓力", ERROR_CODE.HEALTH_ABSOLUTE_PRESSURE_ERROR],
    0x10: ["差壓計", ERROR_CODE.HEALTH_DIFFERENTIAL_PRESSURE_ERROR],
    0x20: ["GPS", ERROR_CODE.HEALTH_GPS_ERROR],
    0x40: ["光流", ERROR_CODE.HEALTH_OPTICAL_FLOW_ERROR],
    0x80: ["視覺定位", ERROR_CODE.HEALTH_VISION_POSITION_ERROR],
    0x100: ["雷射定位", ERROR_CODE.HEALTH_LASER_POSITION_ERROR],
    0x200: ["外部地面真值", ERROR_CODE.HEALTH_EXTERNAL_GROUND_TRUTH_ERROR],
    0x400: ["3D 角速度控制", ERROR_CODE.HEALTH_ANGULAR_RATE_CONTROL_ERROR],
    0x800: ["姿態穩定", ERROR_CODE.HEALTH_ATTITUDE_STABILIZATION_ERROR],
    0x1000: ["偏航位置", ERROR_CODE.HEALTH_YAW_POSITION_ERROR],
    0x2000: ["Z 軸/高度控制", ERROR_CODE.HEALTH_ALTITUDE_CONTROL_ERROR],
    0x4000: ["X/Y 軸位置控制", ERROR_CODE.HEALTH_POSITION_CONTROL_ERROR],
    0x8000: ["馬達輸出", ERROR_CODE.HEALTH_MOTOR_OUTPUTS_ERROR],
    0x10000: ["RC 接收器", ERROR_CODE.HEALTH_RC_RECEIVER_ERROR],
    0x20000: ["第二 3D 陀螺儀", ERROR_CODE.HEALTH_SECOND_3D_GYRO_ERROR],
    0x40000: ["第二 3D 加速度計", ERROR_CODE.HEALTH_SECOND_3D_ACCELEROMETER_ERROR],
    0x80000: ["第二 3D 磁力計", ERROR_CODE.HEALTH_SECOND_3D_MAGNETOMETER_ERROR],
    0x100000: ["地理圍欄", ERROR_CODE.HEALTH_GEOFENCE_ERROR],
    0x200000: ["AHRS（姿態與航向參考系統）", ERROR_CODE.HEALTH_AHRS_ERROR],
    0x400000: ["地形", ERROR_CODE.HEALTH_TERRAIN_ERROR],
    0x800000: ["反向馬達", ERROR_CODE.HEALTH_REVERSE_MOTOR_ERROR],
    0x1000000: ["日誌記錄", ERROR_CODE.HEALTH_LOGGING_ERROR],
    0x2000000: ["電池", ERROR_CODE.HEALTH_BATTERY_ERROR],
    0x4000000: ["接近感測", ERROR_CODE.HEALTH_PROXIMITY_ERROR],
    0x8000000: ["衛星通信", ERROR_CODE.HEALTH_SATELLITE_COMMUNICATION_ERROR],
    0x10000000: ["起飛前檢查", ERROR_CODE.HEALTH_PRE_ARM_CHECK_ERROR],
    0x20000000: ["障礙避免", ERROR_CODE.HEALTH_OBSTACLE_AVOIDANCE_ERROR],
    0x40000000: ["推進系統", ERROR_CODE.HEALTH_PROPULSION_ERROR],
    0x80000000: ["擴展位元字段已使用", ERROR_CODE.HEALTH_EXTENDED_BIT_FIELD_USED_ERROR]
}


# delete we not use sensor
del sensor_flags[0x10]  # 0x10: "Differential Pressure",
del sensor_flags[0x40]  # 0x40: "Optical Flow",
del sensor_flags[0x80]  # 0x80: "Vision Position",
del sensor_flags[0x200]  # 0x200: "External Ground Truth",
del sensor_flags[0x20000]  # 0x20000: "2nd 3D Gyro",
del sensor_flags[0x40000]  # 0x40000: "2nd 3D Accelerometer",
del sensor_flags[0x80000]  # 0x80000: "2nd 3D Magnetometer",
del sensor_flags[0x100000]  # 0x100000: "Geofence",
del sensor_flags[0x800000]  # 0x800000: "Reverse Motor",
del sensor_flags[0x1000000]  # 0x1000000: "Logging",
del sensor_flags[0x4000000]  # 0x4000000: "Proximity",
del sensor_flags[0x8000000]  # 0x8000000: "Satellite Communication",
del sensor_flags[0x20000000]  # 0x20000000: "Obstacle Avoidance",
del sensor_flags[0x80000000]  # 0x80000000: "Extended Bit-field Used"


# print number of sensor_flags
# print(len(sensor_flags))
# print(sensor_flags)






if __name__ == "__main__":
    # sensors_health = 1467088239  # 你的 onboard_control_sensors_health 數值
    sensors_health = 1198562671  # 你的 onboard_control_sensors_health 數值

    # 解析感測器健康狀態
    def parse_sensor_health(sensors_health):
        '''
        :param sensors_health: sys_status.onboard_control_sensors_health 的值
        :return: 回傳每個感測器合在一起成功或錯誤代碼陣列
        '''

        erroe_code_list = []

        print(f"系統感測器健康狀態: {sensors_health}")
        for flag, sensor in sensor_flags.items():
            if sensors_health & flag:
                print(f"{sensor[0]}: True")
                # erroe_code_list.append(ERROR_CODE.SUCCESS)
            else:
                print(f"{sensor[0]}: False")
                erroe_code_list.append(sensor[1])
                
        print(erroe_code_list)

    parse_sensor_health(sensors_health)
