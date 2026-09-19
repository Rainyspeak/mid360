# lidar_to_mavros 节点说明

## 功能概述

该节点负责将 FAST-LIO 输出的雷达里程计数据转换为飞控可用的视觉位姿数据，包含三个核心功能：

### 1. 位置 EKF 滤波 (Pose EKF)
三轴独立的「位置+速度」匀速(CV)模型卡尔曼滤波：里程计(约10Hz)到达时做更新，发布循环外推到当前时刻输出。**姿态不滤，直接透传**。

**作用**：
- 10Hz 阶梯位置 → 高频连续平滑输出
- 马氏门限剔除里程计野值跳变（瞬时野值匀速滑行过渡，连续拒收超限自动重置跟随真实大位移）
- 里程计短时丢帧时匀速外推补齐

**参数配置**：
- `publish_rate`: 发布频率/Hz (默认: 100.0)。100Hz 为上限：PX4 EKF2 对外部视觉的有效带宽约 30-50Hz，更高只增加链路流量无收益
- `kf_en`: 是否启用位置 EKF (默认: true)。置 false 且 publish_rate=30 即恢复旧版行为
- `kf_sigma_a`: 过程噪声，加速度白噪声 (m/s²，默认: 2.0)。调小更平滑（滞后增大），调大更紧跟测量
- `kf_sigma_r`: 观测噪声，位置测量 std (m，默认: 0.03)，按里程计位置噪声水平设置
- `kf_gate_thresh`: 野值门限，新息马氏距离平方 (默认: 11.35 = 3自由度chi2 99%)
- `kf_max_rejects`: 连续拒收上限 (默认: 15，约1.5s)，超过后重置到当前测量
- `kf_extrap_max`: 丢帧时最大外推时长/秒 (默认: 0.5)，超出后位置保持在外推边界不再前冲

### 2. 零漂校准 (Zero Drift Calibration)
开机时收集静止状态下的位置数据，计算平均值作为零漂偏移量，后续输出减去该偏移。

**参数配置**：
- `zero_drift_calib_en`: 是否启用零漂校准 (默认: true)
- `zero_drift_calib_time`: 校准时长/秒 (默认: 2.0)
- `zero_drift_motion_thresh`: 运动检测阈值/米 (默认: 0.05)

**校准流程**：
1. 启动后自动开始校准，要求飞机保持静止
2. 收集指定时长的位置数据
3. 若检测到运动超过阈值，校准失败，输出原始数据
4. 校准成功后，后续输出自动减去零漂偏移

### 3. 杠杆臂补偿 (Lever Arm Compensation)
补偿雷达与飞控安装位置差异导致的位置误差，采用**绝对补偿策略**。

**参数配置**：
- `lidar_offset_en`: 是否启用补偿 (默认: true)
- `lidar_offset_x`: 机体系 X 方向偏移/米，向前为正
- `lidar_offset_y`: 机体系 Y 方向偏移/米，向左为正
- `lidar_offset_z`: 机体系 Z 方向偏移/米，向上为正

**补偿原理**：
- FAST-LIO 输出的是雷达坐标系位置
- 飞控需要的是飞控中心位置
- 通过当前姿态四元数将机体系偏移旋转到世界系，然后从雷达位置减去该偏移
- 公式：`P_飞控 = P_雷达 - R(姿态) × offset(机体系)`

**关键特性**：
- **动态补偿**：每一帧根据当前姿态实时计算偏移量
- **绝对补偿**：不依赖开机姿态，任意姿态下都精确补偿
- **解决杠杆效应**：当无人机俯仰/横滚时，补偿量自动随姿态变化，避免位置估计漂移

## 数据流

```
FAST-LIO (/Odometry, 约10Hz)
    ↓
位置 EKF (更新+剔野值, 姿态透传)
    ↓
零漂校准 (减去开机零点偏移)
    ↓
杠杆臂补偿 (雷达位置 → 飞控位置)
    ↓
mavros/vision_pose/pose (publish_rate 高频外推发布, 发送给飞控)
```

## 启动示例

```bash
roslaunch lidar_to_mavros lidar_to_mavros.launch
```

在 launch 文件中配置参数：

```xml
<node name="lidar_to_mavros" pkg="lidar_to_mavros" type="lidar_to_mavros" output="screen">
    <!-- 零漂校准 -->
    <param name="zero_drift_calib_en" value="true"/>
    <param name="zero_drift_calib_time" value="2.0"/>
    <param name="zero_drift_motion_thresh" value="0.05"/>
    
    <!-- 杠杆臂补偿 (根据实际安装测量) -->
    <param name="lidar_offset_en" value="true"/>
    <param name="lidar_offset_x" value="0.10"/>  <!-- 雷达在飞控前方10cm -->
    <param name="lidar_offset_y" value="0.00"/>
    <param name="lidar_offset_z" value="-0.05"/> <!-- 雷达在飞控下方5cm -->
</node>
```

## 测量安装偏移的方法

1. 确定飞控中心位置（通常是IMU位置）
2. 测量雷达中心相对飞控中心在机体系的偏移
3. 机体系定义：X轴向前，Y轴向左，Z轴向上
4. 填入对应参数

## 注意事项

- **开机静止**：启用零漂校准时，开机后必须保持飞机静止2秒
- **参数精度**：杠杆臂偏移参数需精确测量，误差会导致姿态变化时位置估计漂移
- **坐标系一致**：确保 FAST-LIO 和飞控使用相同的世界坐标系定义（通常是 ENU）
- **时间戳变化**：启用 EKF 后发布的时间戳为外推目标时刻（约等于当前时刻），比旧版（沿用雷达帧时间戳，滞后约0.1s+处理延迟）更新鲜。若飞控端位置融合出现抖动，需重调 `EKF2_EV_DELAY`
- **回退开关**：`kf_en=false` 且 `publish_rate=30` 即完全恢复旧版行为，用于飞行对比测试
