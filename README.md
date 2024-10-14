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


### web server 

使用flask的網頁伺服器, 並且結合資料庫(SQLite),

## 目前要檢查的項目：
### 無人機

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
