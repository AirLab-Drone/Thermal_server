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

TODO:preArm check: ARMING_CHECK