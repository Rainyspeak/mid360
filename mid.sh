#!/bin/bash
# 一键启动: 先起 mavros(后台), 等 5 秒再起定位链路(前台)
roslaunch mavros px4.launch &
sleep 5
roslaunch lidar_to_mavros lidar_to_mavros.launch
