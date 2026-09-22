# mid360 — Mid-360 + FAST-LIO + PX4 机载定位链路

Livox Mid-360 激光雷达 + FAST-LIO 2.0 激光惯性里程计 + PX4 视觉定位的机载部署仓库。三个 ROS1 包平铺在本仓库根目录，clone 即为完整 catkin 工作空间源码。

| 目录 | 说明 |
|------|------|
| `livox_ros_driver2/` | Mid-360 驱动，发布 CustomMsg 点云与内置 IMU（上游 [Livox-SDK/livox_ros_driver2]） |
| `fast_lio/` | 激光惯性里程计与建图，输出 `/Odometry`（上游 [hku-mars/FAST_LIO]，含重力自适应安装角估计等本地改动，见 [UPSTREAM.md](UPSTREAM.md)） |
| `lidar_to_mavros/` | 自研包：里程计 → `mavros/vision_pose/pose`（位置 EKF、零漂校准、杆臂补偿、Foxglove 远程可视化） |

上游仓库、基线 commit 与本地改动清单见 [UPSTREAM.md](UPSTREAM.md)。

## 1. 依赖

### 1.1 系统与 ROS

- **Ubuntu 20.04 + ROS Noetic**（desktop-full 推荐）
  - livox_ros_driver2 官方要求：Ubuntu 20.04 for ROS Noetic（[ROS Noetic 安装](https://wiki.ros.org/noetic/Installation)）
  - fast_lio 官方要求：Ubuntu ≥ 16.04、ROS ≥ Melodic、PCL ≥ 1.8、Eigen ≥ 3.3.4（20.04 apt 默认版本即满足）
- 编译工具链：`gcc`（C++14）、`cmake ≥ 3.0`、`python3-catkin-tools`（使用 `catkin build` 时）

### 1.2 Livox-SDK2（唯一需要源码安装的依赖）

livox_ros_driver2 的 CMakeLists 在 `/usr/local/lib` 下查找静态库 `liblivox_lidar_sdk_static.a`，**不装必挂**。按 [Livox-SDK2 官方说明](https://github.com/Livox-SDK/Livox-SDK2)安装：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j4
sudo make install && sudo ldconfig   # 装到 /usr/local/lib 与 /usr/local/include
```

### 1.3 apt 依赖一键安装

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-dev \
  libeigen3-dev libboost-all-dev libapr1-dev python3-catkin-tools \
  ros-noetic-pcl-ros ros-noetic-pcl-msgs ros-noetic-tf \
  ros-noetic-eigen-conversions \
  ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-foxglove-bridge
```

| 依赖 | 谁需要 | 出处 |
|------|--------|------|
| `libeigen3-dev`（Eigen ≥ 3.3.4） | fast_lio、lidar_to_mavros | fast_lio 官方 Prerequisites |
| `ros-noetic-pcl-ros` / `libpcl-dev`（PCL ≥ 1.8） | fast_lio、livox_ros_driver2 | fast_lio 官方 Prerequisites；pcl_ros 会带上 libpcl 1.10 |
| `libboost-all-dev`（Boost ≥ 1.54, system/thread/chrono） | livox_ros_driver2 | 其 CMakeLists `find_package(Boost 1.54 REQUIRED ...)` |
| `python3-dev` | fast_lio（`find_package(PythonLibs REQUIRED)`） | 其 CMakeLists |
| `libapr1-dev` | livox_ros_driver2（pkg-config 检查 apr-1） | 上游依赖 |
| `ros-noetic-eigen-conversions` | fast_lio（CMakeLists REQUIRED，**容易漏**） | 本仓库本地改动引入 |
| `ros-noetic-foxglove-bridge` | lidar_to_mavros 运行时（远程可视化） | 运行依赖，编译不强制 |

## 2. 编译

### 2.1 推荐方式：catkin build

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <本仓库地址> mid_livox
cd ~/catkin_ws

# 关键：必须传 ROS_EDITION，livox_ros_driver2 的 CMakeLists 按它选择 ROS1/ROS2 分支
catkin config --cmake-args -DROS_EDITION=ROS1
catkin build livox_ros_driver2 fast_lio lidar_to_mavros
source devel/setup.bash
```

> **为什么必须 `-DROS_EDITION=ROS1`**：`livox_ros_driver2/CMakeLists.txt` 整体包在 `if(ROS_EDITION STREQUAL "ROS1")` 里，不传参时两个分支都不执行，驱动变成空工程、不生成 `CustomMsg` 消息，下游 fast_lio 编译报 `livox_ros_driver2/CustomMsg.h: No such file or directory`。`catkin config` 配一次即可，后续 `catkin build` 会沿用。
> 仓库里的 `livox_ros_driver2/package.xml` 已是 ROS1 版本（与 `package_ROS1.xml` 一致），无需手动替换。

编译顺序无需关心：catkin 自动先编 livox_ros_driver2（生成消息），再编 fast_lio（依赖 CustomMsg）和 lidar_to_mavros。

### 2.2 备选方式：官方 build.sh（catkin_make）

livox 官方 README 的方式，从 `livox_ros_driver2/` 目录用 `catkin_make` 编译整个工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src/mid_livox/livox_ros_driver2
./build.sh ROS1
source ~/catkin_ws/devel/setup.bash
```

注意：`build.sh` 每次运行会**先删除整个 `build/`、`devel/`** 再全量重编，且 `catkin_make` 与 `catkin build`（catkin_tools）的构建产物互不兼容——**两种方式二选一，不要混用**。日常迭代推荐 2.1。

机载电脑内存紧张时（PCL + ikd-Tree 模板编译吃内存），用 `catkin build -j2` 或加大 swap。

## 3. 运行

一键启动（内含雷达驱动 + fast_lio + 定位转发 + Foxglove 桥，IMU/点云话题在 `fast_lio/config/mid360.yaml` 中配置）：

```bash
roslaunch lidar_to_mavros lidar_to_mavros.launch
```

分步启动（调试时）：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch   # 驱动，xfer_format=1 发布 CustomMsg
roslaunch fast_lio mapping_mid360.launch         # 里程计/建图，rviz:=true 可开 RViz
rosrun lidar_to_mavros lidar_to_mavros           # 转发 mavros/vision_pose/pose
```

- 雷达 IP / 主机 IP 等连接配置在 `livox_ros_driver2/config/MID360_config.json`
- 杆臂补偿、EKF、零漂校准参数在 `lidar_to_mavros/launch/lidar_to_mavros.launch`
- 节点详细说明（EKF 调参、Foxglove 远程可视化连接方式）见 [lidar_to_mavros/README.md](lidar_to_mavros/README.md)
- 进入 PX4 还需 mavros 在跑（不在本仓库范围）

## 4. 常见编译问题

| 现象 | 原因与解决 |
|------|-----------|
| fast_lio 报 `livox_ros_driver2/CustomMsg.h: No such file or directory` | `catkin build` 没传 `-DROS_EDITION=ROS1`，livox_ros_driver2 空工程未生成消息。按 2.1 执行 `catkin config --cmake-args -DROS_EDITION=ROS1` 后删 `build/livox_ros_driver2` 重编 |
| livox_ros_driver2 配置阶段报 `liblivox_lidar_sdk_static.a` 找不到 | Livox-SDK2 未安装。按 1.2 源码编译安装后 `sudo ldconfig` |
| fast_lio 配置阶段报 `eigen_conversions` 找不到 | `sudo apt install ros-noetic-eigen-conversions` |
| catkin_make / build.sh 与 catkin build 混用后工作空间异常 | 两者产物不兼容。删掉 `build/ devel/` 后只用一种方式重编 |
| 编译中途被 kill（cc1plus killed） | 内存不足。`catkin build -j2` 或加 swap |

[Livox-SDK/livox_ros_driver2]: https://github.com/Livox-SDK/livox_ros_driver2
[hku-mars/FAST_LIO]: https://github.com/hku-mars/FAST_LIO
