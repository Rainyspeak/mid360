# 上游溯源

本仓库（mid_livox）由以下上游项目平铺合并而来（2026-09-19 扁平化，去除上游 git 历史，本地修改全部保留在工作区文件中）：

| 目录 | 上游仓库 | 基线 commit |
|------|---------|------------|
| fast_lio/ | https://github.com/hku-mars/FAST_LIO.git | 7cc4175de6f8ba2edf34bab02a42195b141027e9 |
| fast_lio/include/ikd-Tree/ | https://github.com/hku-mars/ikd-Tree.git （分支 fast_lio，原子模块引入） | e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4 |
| livox_ros_driver2/ | https://github.com/Livox-SDK/livox_ros_driver2.git | 6b9356cadf77084619ba406e6a0eb41163b08039 |

lidar_to_mavros/ 为自研包，无上游。

如需与上游对比差异：克隆上游仓库并 checkout 到上表 commit，与对应目录做 diff 即可。

主要本地改动：

- **fast_lio**：livox_ros_driver2 消息类型迁移（preprocess.*）；重力自适应安装角估计与输出层倾角校正（IMU_Processing.hpp、laserMapping.cpp，含 5 秒质量门控初始化与 /mount_tilt_rpy 话题）。
- **livox_ros_driver2**：MID360_config.json 雷达外参配置、build.sh 适配。
- **lidar_to_mavros**：零漂校准、杆臂补偿、输出层位置 EKF（100Hz 高频预测输出 + 马氏门限剔野值）。

## Linux 部署编译

```bash
# 依赖: ROS1 (noetic), Livox-SDK2
cd ~/catkin_ws/src
git clone <本仓库> mid_livox
cd ~/catkin_ws
catkin build livox_ros_driver2 fast_lio lidar_to_mavros
source devel/setup.bash
```
