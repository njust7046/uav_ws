# uav_ws — 室内无人机自主避障飞行系统

基于 PX4 + MAVROS + Livox MID-360 + FAST_LIO + EGO-Planner 的室内无人机自主导航避障 ROS 工作空间。

## 系统架构

```
MID-360 激光雷达
    ↓
FAST_LIO（定位 + 点云建图）
    ├── /Odometry        → lidar_to_mavros → /mavros/vision_pose/pose → PX4（外部定位）
    ├── /Odometry        → lidar_to_mavros → /Odom_high_freq          → px4ctrl
    └── /cloud_registered                  → ego_planner_node（障碍物地图）
                                                    ↓
                                             traj_server → /position_cmd
                                                    ↓
                                               px4ctrl → /mavros/setpoint_raw/attitude → PX4
```

## 包结构

```
src/
├── ego_planner/            # EGO-Planner 规划核心
│   ├── bspline_opt         # B 样条轨迹优化
│   ├── drone_detect        # 无人机检测（多机避碰）
│   ├── path_searching      # 路径搜索（A*）
│   ├── plan_env            # 环境建图（ESDF/占用栅格）
│   ├── plan_manage         # 规划状态机 + launch 文件
│   └── traj_utils          # 轨迹工具库
├── ego_bringup             # 系统启动 launch 文件集合
├── px4ctrl                 # 姿态控制器（位置指令 → 姿态目标）
├── lidar_to_mavros         # FAST_LIO 里程计桥接节点
├── waypoint_flight         # 简单航点飞行节点（不含避障）
├── quadrotor_msgs          # 自定义消息类型
├── uav_utils               # 通用工具库
└── cmake_utils             # CMake 工具
```

## 依赖

| 依赖 | 说明 |
|------|------|
| ROS Noetic | 基础框架 |
| MAVROS | MAVLink 通信桥接 |
| FAST_LIO | 激光雷达里程计与建图 |
| livox_ros_driver2 | Livox MID-360 驱动 |
| robot_bringup | MAVROS px4.launch |
| Eigen3 | 矩阵运算 |
| PCL 1.7+ | 点云处理 |
| OpenCV | 图像处理（plan_env 依赖） |

## 编译

```bash
cd ~/uav_ws
catkin_make
source devel/setup.bash
```

## 使用方法

系统分四个终端依次启动：

### 终端 1：传感器链路 + 定位

启动 MAVROS、MID-360 驱动、FAST_LIO 建图、lidar_to_mavros 桥接。

```bash
roslaunch ego_bringup ego_lidar.launch
```

等待日志出现 `lidar_to_mavros started` 且 FAST_LIO 输出里程计后再继续。

### 终端 2：px4ctrl 控制器

启动姿态控制器，订阅 `/position_cmd` 并向 PX4 发送姿态目标。

```bash
roslaunch ego_bringup ego_ctrl.launch
```

等待日志出现 `[PX4CTRL] RC received` 且 FCU 连接成功后继续。

### 终端 3：起飞

发送起飞指令，无人机爬升至 `takeoff_height`（默认 0.5m）后悬停。

```bash
roslaunch ego_bringup ego_takeoff.launch
```

### 终端 4：EGO-Planner 规划

启动规划节点，无人机开始按预设航点自主避障飞行。

```bash
roslaunch ego_bringup ego_planner.launch
```

### 降落

```bash
roslaunch ego_bringup ego_land.launch
```

## 关键参数配置

### px4ctrl 控制器参数

文件：`px4ctrl/config/ctrl_param_fpv.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mass` | 2.32 | 无人机质量 (kg)，**必须根据实际修改** |
| `hover_percentage` | 0.5 | 悬停油门百分比，**需根据实际标定** |
| `takeoff_height` | 0.5 | 自动起飞高度 (m) |
| `takeoff_land_speed` | 0.3 | 起降速度 (m/s) |
| `max_angle` | 30 | 最大姿态角限制 (°) |
| `low_voltage` | 21.6 | 低电压报警阈值 (V)，6S 电池 |
| `Kp0/1/2` | 2.0 | 位置环 PID 增益 |
| `Kv0/1/2` | 1.5 | 速度环 PID 增益 |

> `hover_percentage` 设置过大会导致起飞过猛，过小则无法起飞。建议先在 Stabilize 模式下测量实际悬停油门后填入。

### EGO-Planner 规划参数

文件：`ego_planner/plan_manage/launch/run_in_exp.launch`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_size_x/y` | 30 | 地图范围 (m)，根据实际场地修改 |
| `map_size_z` | 1.2 | 地图高度范围 (m) |
| `max_vel` | 0.7 | 最大飞行速度 (m/s) |
| `max_acc` | 0.5 | 最大加速度 (m/s²) |
| `planning_horizon` | 7.5 | 规划视野 (m)，建议为感知范围的 1.5 倍 |
| `flight_type` | 2 | 1: RViz 2D Nav Goal 指定终点；2: 预设航点列表 |
| `point_num` | 4 | 预设航点数量 |
| `point0_x/y/z` | — | 各航点坐标 (m)，odom 坐标系绝对坐标 |

文件：`ego_planner/plan_manage/launch/advanced_param_exp.xml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_map/resolution` | 0.15 | 占用栅格分辨率 (m) |
| `grid_map/obstacles_inflation` | 0.6 | 障碍物膨胀半径 (m)，建议 ≥ 机体半径 |
| `grid_map/local_update_range_x/y` | 20 | 局部地图更新范围 (m) |
| `grid_map/local_update_range_z` | 1.2 | 局部地图高度范围 (m) |

## ROS 话题总览

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/Odometry` | `nav_msgs/Odometry` | FAST_LIO | lidar_to_mavros, ego_planner |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST_LIO | ego_planner |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | lidar_to_mavros | MAVROS→PX4 |
| `/Odom_high_freq` | `nav_msgs/Odometry` | lidar_to_mavros | px4ctrl |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | traj_server | px4ctrl |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | px4ctrl | MAVROS→PX4 |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | ego_takeoff/land.launch | px4ctrl |

## 调参建议

| 问题现象 | 建议调整 |
|----------|---------|
| 起飞过猛 / 无法起飞 | 标定 `hover_percentage` |
| 飞行位置漂移 | 检查 FAST_LIO 初始化，调整 `Kp`/`Kv` |
| 规划频繁失败 / 卡死 | 减小 `max_vel`，增大 `obstacles_inflation` |
| 绕障路径过于保守 | 减小 `obstacles_inflation` |
| 飞行轨迹抖动 | 减小 `max_vel` 和 `max_acc` |
| 无法感知近处障碍 | 减小 `grid_map/resolution` |

## 注意事项

- 首次飞行前务必在地面验证 `hover_percentage`，错误值可能导致失控。
- `ego_lidar.launch` 启动后需等待 FAST_LIO 完成初始化（约 5～10 秒），再启动 `ego_ctrl.launch`。
- EGO-Planner 的航点坐标（`point_x/y/z`）为 odom 坐标系下的绝对坐标，原点为 FAST_LIO 启动时的位置。
- `flight_type=1` 时可通过 RViz 的 2D Nav Goal 实时指定目标点，适合调试；`flight_type=2` 按预设航点列表自动飞行。
- RC 遥控器需保持开启，紧急情况可随时切回手动模式接管。
