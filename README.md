# Thermal_server 



### Introduction

This is respository for the thermal server project. It use robot operating system (ROS) to communicate with the thermal cameras and convert the temperature data from the camera pixel coordinates to the world coordinates. It contains the following packages:

- thermal_camera2world:This package is used to convert the host temperature coordinates from the camera pixel coordinates to the world coordinates.
- thermal_ipt430m:This package is used to communicate with the thermal camera IPT430M.
- thermal_msgs:This package defines the messages used in the thermal server project.
- thermal_ds4025ft:This package is used to communicate with the thermal camera DS4025FT.
- ~~thermal_gui:Is not used in this project.~~
- ~~thermal_im4:This package is used to communicate with the thermal camera IM4. Is not used in this project.~~


### How to use

clone the repository into your colcon workspace:

```
mkdir -p ~/{youer workspace name} && cd ~/{youer workspace name}
git clone https://github.com/AirLab-Drone/Thermal_server.git/.
```

colcon build the workspace:
```
colcon build
```

----


### web server 

使用flask的網頁伺服器, 並且結合資料庫(SQLite),

## 目前要檢查的項目：
## 無人機
### 無人機上的感測器  
| 感測器/控制項                                     | Value       | Description                              | 健康狀態 (`sensors_health`) | 啟用狀態 (`sensors_enabled`) | 存在狀態 (`sensors_present`) |
|:---------------------------------------------------|:-------------|:------------------------------------------|:----------------------------|:------------------------------|:------------------------------|
| 3D 陀螺儀 (MAV_SYS_STATUS_SENSOR_3D_GYRO)          | 1           | 0x01 3D gyro                             | 1                          | 1                            | 1                            |
| 3D 加速度計 (MAV_SYS_STATUS_SENSOR_3D_ACCEL)       | 2           | 0x02 3D accelerometer                    | 1                          | 1                            | 1                            |
| 3D 磁力計 (MAV_SYS_STATUS_SENSOR_3D_MAG)           | 4           | 0x04 3D magnetometer                     | 1                          | 1                            | 1                            |
| 絶對壓力 (MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) | 8           | 0x08 absolute pressure                   | 1                          | 1                            | 1                            |
| 差壓計 (MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE) | 16         | 0x10 differential pressure               | 0                          | 0                            | 0                            |
| GPS (MAV_SYS_STATUS_SENSOR_GPS)                    | 32          | 0x20 GPS                                 | 1                          | 1                            | 1                            |
| 光流 (MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)          | 64          | 0x40 optical flow                        | 1                          | 1                            | 1                            |
| 視覺定位 (MAV_SYS_STATUS_SENSOR_VISION_POSITION)   | 128         | 0x80 computer vision position            | 0                          | 0                            | 0                            |
| 雷射定位 (MAV_SYS_STATUS_SENSOR_LASER_POSITION)    | 256         | 0x100 laser based position               | 1                          | 1                            | 1                            |
| 外部地面真值 (MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH) | 512     | 0x200 external ground truth             | 0                          | 0                            | 0                            |
| 3D 角速度控制 (MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL) | 1024   | 0x400 3D angular rate control           | 1                          | 1                            | 1                            |
| 姿態穩定 (MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) | 2048      | 0x800 attitude stabilization            | 1                          | 1                            | 1                            |
| 偏航位置 (MAV_SYS_STATUS_SENSOR_YAW_POSITION)      | 4096        | 0x1000 yaw position                      | 1                          | 1                            | 1                            |
| Z 軸/高度控制 (MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL) | 8192     | 0x2000 z/altitude control               | 1                          | 1                            | 1                            |
| X/Y 軸位置控制 (MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) | 16384   | 0x4000 x/y position control             | 1                          | 1                            | 1                            |
| 馬達輸出 (MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)    | 32768       | 0x8000 motor outputs / control           | 1                          | 1                            | 1                            |
| RC 接收器 (MAV_SYS_STATUS_SENSOR_RC_RECEIVER)      | 65536       | 0x10000 RC receiver                      | 1                          | 1                            | 1                            |
| 第二 3D 陀螺儀 (MAV_SYS_STATUS_SENSOR_3D_GYRO2)    | 131072      | 0x20000 2nd 3D gyro                      | 0                          | 0                            | 0                            |
| 第二 3D 加速度計 (MAV_SYS_STATUS_SENSOR_3D_ACCEL2) | 262144      | 0x40000 2nd 3D accelerometer             | 0                          | 0                            | 0                            |
| 第二 3D 磁力計 (MAV_SYS_STATUS_SENSOR_3D_MAG2)     | 524288      | 0x80000 2nd 3D magnetometer              | 0                          | 0                            | 0                            |
| 地理圍欄 (MAV_SYS_STATUS_GEOFENCE)                 | 1048576     | 0x100000 geofence                        | 1                          | 0                            | 1                            |
| AHRS（姿態與航向參考系統） (MAV_SYS_STATUS_AHRS)   | 2097152     | 0x200000 AHRS subsystem health           | 1                          | 1                            | 1                            |
| 地形 (MAV_SYS_STATUS_TERRAIN)                      | 4194304     | 0x400000 Terrain subsystem health        | 1                          | 1                            | 1                            |
| 反向馬達 (MAV_SYS_STATUS_REVERSE_MOTOR)            | 8388608     | 0x800000 Motors are reversed             | 0                          | 0                            | 0                            |
| 日誌記錄 (MAV_SYS_STATUS_LOGGING)                  | 16777216    | 0x1000000 Logging                        | 1                          | 0                            | 1                            |
| 電池 (MAV_SYS_STATUS_SENSOR_BATTERY)               | 33554432    | 0x2000000 Battery                        | 1                          | 1                            | 1                            |
| 接近感測 (MAV_SYS_STATUS_SENSOR_PROXIMITY)        | 67108864    | 0x4000000 Proximity                      | 1                          | 0                            | 0                            |
| 衛星通信 (MAV_SYS_STATUS_SENSOR_SATCOM)            | 134217728   | 0x8000000 Satellite Communication        | 0                          | 0                            | 0                            |
| 起飛前檢查 (MAV_SYS_STATUS_PREARM_CHECK)           | 268435456   | 0x10000000 Pre-arm check status          | 1                          | 1                            | 1                            |
| 障礙避免 (MAV_SYS_STATUS_OBSTACLE_AVOIDANCE)      | 536870912   | 0x20000000 Avoidance/collision prevention| 0                          | 0                            | 0                            |
| 推進系統 (MAV_SYS_STATUS_SENSOR_PROPULSION)       | 1073741824  | 0x40000000 Propulsion                    | 1                          | 1                            | 1                            |
| 擴展位元字段已使用 (MAV_SYS_STATUS_EXTENSION_USED) | 2147483648  | 0x80000000 Extended bit-field used       | 0                          | 0                            | 0                            |






| 檢查項目 | mavlink messages |
| :-----|:-----|
| 系統狀態 | MAV_STATE|
| 無人機模式(stable, guided ) | MAV_MODE_FLAG |
| Compass健康狀態 | |
| GPS健康狀態 | |
|||
|||
|||
|||
|||

RC Channels 
Board voltage
Hardware safety switch
Rangefinder
無人機電壓
無人機電流
無人機剩餘電量
無人機馬達


### 環境熱像儀


TODO:preArm check: ARMING_CHECK

ARMING_CHECK:
```
Barometer (氣壓計)
Compass (指南針)
GPS lock (GPS 鎖定)
INS (慣性導航系統)
Parameters (參數)
RC Channels (遙控通道)
Board voltage (板電壓)
Battery Level (電池電量)
Logging Available (可用日誌)
Hardware safety switch (硬體安全開關)
GPS Configuration (GPS 配置)
System (系統)
Mission (任務)
Rangefinder (測距儀)
Camera (相機)
AuxAuth (輔助認證)
VisualOdometry (視覺里程計)
FFT (快速傅立葉變換)
```
