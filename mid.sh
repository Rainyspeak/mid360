#!/bin/bash
roslaunch mavros px4.launch &sleep 5;
roslaunch lidar_to_mavros lidar_to_mavros.launch;
