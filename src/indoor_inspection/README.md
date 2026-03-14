# indoor_inspection — 室内定点覆盖巡检飞行

基于 PX4 + MAVROS + MID-360 + FAST_LIO + lidar_to_mavros 的室内无人机定点覆盖飞行 ROS 包。

## 文件结构

```
indoor_inspection/
├── CMakeLists.txt                # 构建配置
├── package.xml                   # ROS 包描述
├── config/
│   └── waypoints.yaml            # 航点与飞行参数配置
├── launch/
│   ├── mapping.launch            # 阶段一：手动飞行建图
│   ├── bringup.launch            # 阶段二：一键启动完整巡检链路
│   └── inspection.launch         # 仅启动巡检节点（需其他组件已运行）
└── src/
    └── inspection_node.cpp       # 巡检主节点
```

## 系统架构

`bringup.launch` 按顺序启动以下组件：

```
PX4 飞控 ←──MAVROS──→ inspection_node
                ↑
MID-360 → FAST_LIO → lidar_to_mavros
(雷达驱动)  (定位建图)    (位姿桥接)
```

| 启动顺序 | 组件 | 来源 |
|---------|------|------|
| 1 | MAVROS | `robot_bringup/px4.launch` |
| 2 | MID-360 驱动 | `livox_ros_driver2/msg_MID360.launch` |
| 3 | FAST_LIO 定位 | `fast_lio/mapping_mid360.launch` |
| 4 | 位姿桥接 | `lidar_to_mavros` 节点 |
| 5 | 巡检节点 | `indoor_inspection/inspection.launch` |

## 巡检节点工作流程

```
等待FCU连接 → 记录起飞位置 → 预发setpoint(100次)
    → 切OFFBOARD → 解锁 → 起飞到目标高度
    → 依次飞往各航点（距离<阈值=到达，悬停N秒）
    → 返回起飞点上方 → AUTO.LAND降落
```

### ROS 接口

| 方向 | 话题/服务 | 类型 |
|------|----------|------|
| 发布 | `/mavros/setpoint_position/local` (20Hz) | `geometry_msgs/PoseStamped` |
| 订阅 | `/mavros/state` | `mavros_msgs/State` |
| 订阅 | `/mavros/local_position/odom` | `nav_msgs/Odometry` |
| 服务 | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` |
| 服务 | `/mavros/set_mode` | `mavros_msgs/SetMode` |

## 配置说明

`config/waypoints.yaml` 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `flight_height` | 1.2 | 飞行高度(m) |
| `reach_threshold` | 0.3 | 到达判定欧氏距离阈值(m) |
| `hover_time` | 3.0 | 每个航点悬停时间(s) |
| `waypoints` | — | 相对起飞点的 {x, y} 偏移列表(m) |

## 使用方法

整体分两个阶段：先建图摸清环境，再根据地图规划航点执行巡检。

### 阶段一：建图

用遥控器手动飞行，FAST_LIO 实时建图，了解房间尺寸和障碍物分布。

```bash
cd ~/uav_ws && catkin_make && source devel/setup.bash

# 启动传感器链路（不含巡检节点）
roslaunch indoor_inspection mapping.launch

# 另一个终端打开 rviz 查看实时点云地图
rviz  # 添加 PointCloud2 话题: /cloud_registered
```

手动飞行覆盖整个区域后：
1. 在 rviz 中观察点云地图，确认房间边界和障碍物位置
2. 用 `rostopic echo /mavros/local_position/odom` 在关键位置记录坐标
3. 计算各航点相对起飞点的 x, y 偏移量

### 阶段二：规划航点

根据建图结果编辑 `config/waypoints.yaml`：

```yaml
flight_height: 1.2
reach_threshold: 0.3
hover_time: 3.0

waypoints:                 # 根据实际地图规划的覆盖路径
  - {x: 2.0, y: 0.0}      # 沿 x 方向前进 2m
  - {x: 2.0, y: 2.0}      # 横移 2m
  - {x: 0.0, y: 2.0}      # 返回 x=0
  - {x: 0.0, y: 4.0}      # 继续横移
  - {x: 2.0, y: 4.0}      # 蛇形覆盖
```

航点坐标是相对起飞位置的偏移量，节点启动时自动记录起飞位置并叠加偏移。典型的覆盖策略是蛇形路径（弓字形），间距根据传感器覆盖范围确定。

### 阶段三：执行巡检

```bash
# 完整启动（含全部传感器链路 + 巡检）
roslaunch indoor_inspection bringup.launch

# 或：传感器链路已在运行时，仅启动巡检
roslaunch indoor_inspection inspection.launch
```

## 实飞建议

1. 首次测试用 2-3 个近距离航点（如 1m 间距），确认行为正常
2. 地面预检：启动节点后 `rostopic echo /mavros/setpoint_position/local` 确认发布正常
3. 遥控器保持在手，随时可切回手动模式
4. `reach_threshold` 建议 0.2~0.5m，过小可能导致航点判定困难
