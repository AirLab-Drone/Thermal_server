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
| 感測器/控制項                                     | 存在狀態 (`present`) | 啟用狀態 (`enabled`) | 健康狀態 (`health`) | Value       | Description                              |
|:---------------------------------------------------|:------------------------------|:----------------------------|:----------------------------|:-------------|:------------------------------------------|
| 3D 陀螺儀 (3D_GYRO)          | 1                            | 1                          | 1                          | 1           | 0x01 3D gyro                             |
| 3D 加速度計 (3D_ACCEL)       | 1                            | 1                          | 1                          | 2           | 0x02 3D accelerometer                    |
| 3D 磁力計 (3D_MAG)           | 1                            | 1                          | 1                          | 4           | 0x04 3D magnetometer                     |
| 絶對壓力 (ABSOLUTE_PRESSURE) | 1                            | 1                          | 1                          | 8           | 0x08 absolute pressure                   |
| 差壓計 (DIFFERENTIAL_PRESSURE) | 0                            | 0                          | 0                          | 16         | 0x10 differential pressure               |
| GPS (GPS)                    | 1                            | 1                          | 1                          | 32          | 0x20 GPS                                 |
| 光流 (OPTICAL_FLOW)          | 1                            | 1                          | 1                          | 64          | 0x40 optical flow                        |
| 視覺定位 (VISION_POSITION)   | 0                            | 0                          | 0                          | 128         | 0x80 computer vision position            |
| 雷射定位 (LASER_POSITION)    | 1                            | 1                          | 1                          | 256         | 0x100 laser based position               |
| 外部地面真值 (EXTERNAL_GROUND_TRUTH) | 0                            | 0                          | 0                          | 512     | 0x200 external ground truth             |
| 3D 角速度控制 (ANGULAR_RATE_CONTROL) | 1                            | 1                          | 1                          | 1024   | 0x400 3D angular rate control           |
| 姿態穩定 (ATTITUDE_STABILIZATION) | 1                            | 1                          | 1                          | 2048      | 0x800 attitude stabilization            |
| 偏航位置 (YAW_POSITION)      | 1                            | 1                          | 1                          | 4096        | 0x1000 yaw position                      |
| Z 軸/高度控制 (Z_ALTITUDE_CONTROL) | 1                            | 1                          | 1                          | 8192     | 0x2000 z/altitude control               |
| X/Y 軸位置控制 (XY_POSITION_CONTROL) | 1                            | 1                          | 1                          | 16384   | 0x4000 x/y position control             |
| 馬達輸出 (MOTOR_OUTPUTS)    | 1                            | 1                          | 1                          | 32768       | 0x8000 motor outputs / control           |
| RC 接收器 (RC_RECEIVER)      | 1                            | 1                          | 1                          | 65536       | 0x10000 RC receiver                      |
| 第二 3D 陀螺儀 (3D_GYRO2)    | 0                            | 0                          | 0                          | 131072      | 0x20000 2nd 3D gyro                      |
| 第二 3D 加速度計 (3D_ACCEL2) | 0                            | 0                          | 0                          | 262144      | 0x40000 2nd 3D accelerometer             |
| 第二 3D 磁力計 (3D_MAG2)     | 0                            | 0                          | 0                          | 524288      | 0x80000 2nd 3D magnetometer              |
| 地理圍欄 (GEOFENCE)                 | 1                            | 0                          | 1                          | 1048576     | 0x100000 geofence                        |
| AHRS（姿態與航向參考系統） (AHRS)   | 1                            | 1                          | 1                          | 2097152     | 0x200000 AHRS subsystem health           |
| 地形 (TERRAIN)                      | 1                            | 1                          | 1                          | 4194304     | 0x400000 Terrain subsystem health        |
| 反向馬達 (REVERSE_MOTOR)            | 0                            | 0                          | 0                          | 8388608     | 0x800000 Motors are reversed             |
| 日誌記錄 (LOGGING)                  | 1                            | 0                          | 1                          | 16777216    | 0x1000000 Logging                        |
| 電池 (BATTERY)               | 1                            | 1                          | 1                          | 33554432    | 0x2000000 Battery                        |
| 接近感測 (PROXIMITY)        | 0                            | 0                          | 1                          | 67108864    | 0x4000000 Proximity                      |
| 衛星通信 (SATCOM)            | 0                            | 0                          | 0                          | 134217728   | 0x8000000 Satellite Communication        |
| 起飛前檢查 (PREARM_CHECK)           | 1                            | 1                          | 1                          | 268435456   | 0x10000000 Pre-arm check status          |
| 障礙避免 (OBSTACLE_AVOIDANCE)      | 0                            | 0                          | 0                          | 536870912   | 0x20000000 Avoidance/collision prevention|
| 推進系統 (PROPULSION)       | 1                            | 1                          | 1                          | 1073741824  | 0x40000000 Propulsion                    |
| 擴展位元字段已使用 (EXTENSION_USED) | 0                            | 0                          | 0                          | 2147483648  | 0x80000000 Extended bit-field used       |





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
