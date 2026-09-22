# mid360 — Mid-360 + FAST-LIO + PX4 机载定位链路

上游溯源见 [UPSTREAM.md](UPSTREAM.md)，节点参数说明见 [lidar_to_mavros/README.md](lidar_to_mavros/README.md)。

## 依赖

```bash
sudo apt install -y build-essential cmake git python3-dev \
  libeigen3-dev libboost-all-dev libapr1-dev python3-catkin-tools \
  ros-noetic-pcl-ros ros-noetic-pcl-msgs ros-noetic-tf \
  ros-noetic-eigen-conversions \
  ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-foxglove-bridge
```

```bash
# Livox-SDK2（必须，源码装到 /usr/local/lib）
git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd Livox-SDK2
mkdir build && cd build && cmake .. && make -j4
sudo make install && sudo ldconfig
```

## 编译

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <本仓库地址> mid_livox
cd ~/catkin_ws

# 官方 build.sh 的用法是 ./build.sh ROS1 → catkin_make -DROS_EDITION=ROS1
# 这里等价换成 catkin build 直接传 ROS 版本；不传 -DROS_EDITION=ROS1 则 fast_lio 报 CustomMsg.h 找不到
catkin build --cmake-args -DROS_EDITION=ROS1
source devel/setup.bash
```

## 运行

```bash
roslaunch lidar_to_mavros lidar_to_mavros.launch
```
