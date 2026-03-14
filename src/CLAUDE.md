# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## 项目概述

ROS (catkin) 工作空间，用于无人机室内自主巡检。单一功能包 `indoor_inspection`，基于 PX4 + MAVROS + Livox MID-360 激光雷达 + FAST_LIO 定位实现航点覆盖飞行。

## 构建与运行

```bash
# 编译（在工作空间根目录）
cd ~/uav_ws && catkin_make

# 加载工作空间环境
source ~/uav_ws/devel/setup.bash

# 阶段一：手动建图飞行（仅传感器链路，不含巡检节点）
roslaunch indoor_inspection mapping.launch

# 阶段二：完整自主巡检（全部组件）
roslaunch indoor_inspection bringup.launch

# 阶段三：仅启动巡检节点（传感器链路已在运行时）
roslaunch indoor_inspection inspection.launch
```

无自动化测试，通过实飞验证。

## 系统架构

```
PX4 飞控 ←──MAVROS──→ inspection_node
              ↑
MID-360 → FAST_LIO → lidar_to_mavros
(雷达驱动)  (定位建图)    (位姿桥接)
```

bringup.launch 按顺序启动：MAVROS → MID-360 驱动 → FAST_LIO → lidar_to_mavros → inspection_node。

## 关键文件

- `indoor_inspection/src/inspection_node.cpp` — 巡检主节点（C++11）。状态机流程：等待 FCU 连接 → 记录起飞位置 → 预发 setpoint（100次，PX4 要求）→ 切 OFFBOARD → 解锁 → 起飞爬升 → 依次飞往各航点 → 返回起飞点 → AUTO.LAND 降落。
- `indoor_inspection/config/waypoints.yaml` — 飞行参数（`flight_height`、`reach_threshold`、`hover_time`）及航点列表，航点为相对起飞点的 x, y 偏移量。
- `indoor_inspection/launch/` — 三个 launch 文件，分别用于建图、完整启动、仅巡检。

## ROS 接口

| 方向 | 话题/服务 | 类型 |
|------|----------|------|
| 发布 | `/mavros/setpoint_position/local` (20Hz) | `geometry_msgs/PoseStamped` |
| 订阅 | `/mavros/state` | `mavros_msgs/State` |
| 订阅 | `/mavros/local_position/odom` | `nav_msgs/Odometry` |
| 服务 | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` |
| 服务 | `/mavros/set_mode` | `mavros_msgs/SetMode` |

## 开发注意事项

- C++11 标准，catkin 构建。依赖：roscpp, geometry_msgs, mavros_msgs, nav_msgs, std_msgs, tf。
- 航点坐标为相对起飞位置的偏移量（起飞位置在节点启动时从 FAST_LIO 里程计记录），节点自动叠加偏移。
- 航点到达判定使用欧氏距离，阈值为 `reach_threshold`。
- 切 OFFBOARD 前预发 100 次 setpoint 是 PX4 的硬性要求，不可删除。
