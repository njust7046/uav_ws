# waypoint_flight

基于 PX4 + MAVROS 的航点自主飞行 ROS 包。从 YAML 配置文件读取航点列表，控制无人机依次飞往各航点，完成后自动返回起飞点并降落。

## 系统架构

```
MID-360 → FAST_LIO → lidar_to_mavros → MAVROS → PX4
                                            ↑
                              waypoint_flight_node（发布 setpoint）
```

## 文件结构

```
waypoint_flight/
├── config/
│   └── waypoints.yaml          # 飞行参数与航点列表
├── launch/
│   └── waypoint_flight.launch  # 启动节点
└── src/
    └── waypoint_flight_node.cpp
```

## ROS 接口

| 方向 | 话题 / 服务 | 类型 |
|------|------------|------|
| 发布 | `/mavros/setpoint_position/local` (20Hz) | `geometry_msgs/PoseStamped` |
| 订阅 | `/mavros/state` | `mavros_msgs/State` |
| 订阅 | `/mavros/local_position/odom` | `nav_msgs/Odometry` |
| 服务 | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` |
| 服务 | `/mavros/set_mode` | `mavros_msgs/SetMode` |

## 配置说明

`config/waypoints.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `flight_height` | 1.2 | 飞行高度 (m)，相对起飞点 |
| `reach_threshold` | 0.3 | 到达判定距离阈值 (m) |
| `hover_time` | 3.0 | 每个航点悬停时间 (s) |
| `waypoints` | — | 航点列表，每项为相对起飞点的 `x`、`y` 偏移 (m) |

航点坐标以起飞时记录的位置为原点，x 为东向，y 为北向，所有航点共用同一飞行高度。

示例：

```yaml
flight_height: 1.2
reach_threshold: 0.3
hover_time: 3.0

waypoints:
  - {x: 2.0, y: 0.0}
  - {x: 2.0, y: 2.0}
  - {x: 0.0, y: 2.0}
```

## 状态机流程

```
等待 FCU 连接
    ↓
等待 FAST_LIO 初始化（记录起飞原点）
    ↓
预发 setpoint 100 次（约 5s，PX4 硬性要求）
    ↓
请求切入 OFFBOARD 模式
    ↓
请求解锁（Arm）
    ↓
爬升至 flight_height
    ↓
依次飞往各航点 → 到达后悬停 hover_time 秒
    ↓
返回起飞点上方
    ↓
切 AUTO.LAND → 落地自动上锁 → 结束
```

## 使用方法

### 前提

传感器链路已通过 `ego_bringup/ego_lidar.launch` 启动（MAVROS + MID-360 + FAST_LIO + lidar_to_mavros）。

### 编译

```bash
cd ~/uav_ws && catkin_make --pkg waypoint_flight
source devel/setup.bash
```

### 启动

```bash
roslaunch waypoint_flight waypoint_flight.launch
```

## 注意事项

- 起飞前确保 FAST_LIO 已完成初始化（`/Odometry` 话题有非零输出），节点会自动等待。
- 航点坐标为相对起飞点的偏移，节点启动位置即为原点，无需手动设置绝对坐标。
- `reach_threshold` 设置过大会导致航点提前判定到达，过小在定位抖动时可能长时间无法到达，建议 0.2～0.5m。
- 切 OFFBOARD 前的 100 次预发是 PX4 的硬性要求，不可删除或缩短。
