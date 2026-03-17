# obstacle_avoidance — VFH+ 室外避障演示包

基于 VFH+（Vector Field Histogram Plus）算法的无人机室外避障演示 ROS 包。无人机起飞后自主避障飞向目标点，到达后自动降落。

## 系统架构

```
PX4 飞控 ←──MAVROS──→ avoidance_node ←── /cloud_registered (点云)
              ↑
MID-360 → FAST_LIO → lidar_to_mavros
(雷达驱动)  (定位建图)    (位姿桥接)
```

本包与 `indoor_inspection` 完全独立，共享同一传感器链路（MAVROS + FAST_LIO），互不影响。

## 文件结构

```
obstacle_avoidance/
├── CMakeLists.txt              # 构建配置（C++14，含 PCL 依赖）
├── package.xml                 # 包声明
├── README.md                   # 本文件
├── config/
│   └── avoidance.yaml          # VFH+ 参数 + 飞行参数
├── launch/
│   └── avoidance_demo.launch   # 启动文件
└── src/
    └── avoidance_node.cpp      # 避障主节点
```

## VFH+ 算法原理

每个控制周期（20Hz）执行以下步骤：

### 1. 多层点云切片
从 `/cloud_registered` 话题获取 FAST_LIO 输出的全局点云，切取三层水平薄片（低/中/高），覆盖飞行高度上下不同区域。低矮障碍物和高处树枝都能检测到。低于起飞点 + `ground_clearance` 的点自动过滤为地面，避免室外不平整地面误触发。

### 2. 极坐标直方图构建
以无人机当前位置为中心，将切片内的点投影到 360° 极坐标系，划分为 72 个扇区（每个 5°），统计每个扇区内的障碍物点数。

### 3. 二值化 + 膨胀
扇区内点数超过阈值（默认 3）的标记为"阻塞"。阻塞扇区向两侧各膨胀 `enlarge_sectors` 个扇区（默认 2 = 10°），给障碍物增加角度安全边界，防止贴边飞行。

### 4. Gap 搜索
扫描所有扇区，找出连续空闲扇区组成的"通道"（gap）。扫描从第一个阻塞扇区开始，正确处理 0°/360° 环绕边界。

### 5. 代价函数选择最优方向
对每个 gap 的中心和两侧边缘方向，计算加权代价：

```
cost = w_target × |方向 - 目标方向| + w_smooth × |方向 - 当前航向| + w_prev × |方向 - 上帧方向|
```

- `w_target`（默认 5.0）：偏向目标方向，确保不会绕远路
- `w_smooth`（默认 2.0）：偏向当前航向，减少急转弯
- `w_prev`（默认 2.0）：偏向上一帧选择，保证方向连续性

选择代价最小的方向，经 EMA 低通滤波（`direction_alpha`）平滑后作为本帧前进方向。

### 6. 自适应速度 Setpoint 生成
沿选定方向生成步进目标点，步进距离根据最近障碍物距离动态缩放：
- 障碍物 > `slowdown_radius`(3m)：全速（`step_distance` = 1.5m）
- 障碍物在 `safety_radius` ~ `slowdown_radius` 之间：线性减速到 20%
- 障碍物 < `safety_radius`(1m)：紧急悬停

航向（yaw）自动朝向飞行方向，确保雷达前方感知范围最大化。

### 安全机制

- **紧急悬停**：最近障碍物距离 < `safety_radius` 时原地悬停
- **全阻塞悬停**：所有扇区均被阻塞时原地悬停
- **超时后退脱困**：悬停（紧急或全阻塞）持续超过 `stuck_timeout`(3s) 后，沿来时路径反方向后退 `retreat_distance`(2m)，后退过程同样使用 VFH+ 避障
- **点云时效性检查**：超过 `cloud_timeout`(1s) 未收到新点云则标记无效，降级为直飞模式并发出警告
- **地面点过滤**：低于起飞点 + `ground_clearance`(0.3m) 的点自动过滤，避免室外地面误检测

<!-- APPEND_MARKER -->

## 状态机流程

```
┌─────────────┐
│ 等待FCU连接  │ ← 阻塞直到 MAVROS 连上 PX4
└──────┬──────┘
       ▼
┌──────────────────┐
│ 等待FAST_LIO初始化│ ← 阻塞直到收到首个非零里程计
└──────┬───────────┘
       ▼
┌─────────────────┐
│ 预发setpoint(5s) │ ← PX4 硬性要求，否则拒绝切 OFFBOARD
└──────┬──────────┘
       ▼
┌──────────────────┐
│ OFFBOARD + 解锁   │ ← 5秒间隔重试，防止请求洪泛
│ + 起飞爬升        │ ← 到达目标高度后进入下一阶段
└──────┬───────────┘
       ▼
┌──────────────────────────────────────┐
│ VFH+ 避障导航                         │
│                                      │
│  正常飞行 ──障碍物近──→ 减速飞行       │
│     ↑                    │           │
│     │              太近/全阻塞        │
│     │                    ▼           │
│  脱困完成 ←──超时──── 悬停等待        │
│     │                    │           │
│     └──── VFH+后退 ←────┘           │
│                                      │
│  2D距离 < reach_threshold → 到达目标  │
└──────┬───────────────────────────────┘
       ▼
┌─────────────┐
│ 悬停稳定(2s) │ ← 在目标点上方定点悬停
└──────┬──────┘
       ▼
┌─────────────┐
│ AUTO.LAND   │ ← PX4 原生降落模式，自动检测着地并上锁
└─────────────┘
```

<!-- APPEND_MARKER_2 -->

## 参数配置

所有参数在 `config/avoidance.yaml` 中配置：

### 飞行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `flight_height` | 2.0 | 飞行高度(m) |
| `reach_threshold` | 0.5 | 到达判定阈值(m) |
| `hover_time` | 2.0 | 到达目标后悬停时间(s) |
| `goal_x` | 10.0 | 目标点前方距离(m)，相对起飞点 |
| `goal_y` | 0.0 | 目标点侧向距离(m)，相对起飞点 |

### VFH+ 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_range` | 6.0 | 障碍物检测最大距离(m) |
| `safety_radius` | 1.0 | 紧急悬停距离(m) |
| `slowdown_radius` | 3.0 | 减速开始距离(m) |
| `num_sectors` | 72 | 直方图扇区数（72 = 每扇区5°） |
| `obstacle_threshold` | 3 | 扇区阻塞判定点数阈值 |
| `enlarge_sectors` | 2 | 阻塞扇区膨胀数（2 = 10°安全边界） |
| `slice_half_height` | 0.5 | 点云切片半高度(m) |
| `w_target` | 5.0 | 目标方向权重 |
| `w_smooth` | 2.0 | 航向平滑权重 |
| `w_prev` | 2.0 | 方向连续性权重 |

### 多层切片参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `multi_layer` | true | 启用三层点云切片 |
| `layer_offset` | 0.5 | 额外层与飞行高度的偏移(m) |
| `ground_clearance` | 0.3 | 地面过滤余量(m)，低于此高度的点视为地面 |

<!-- APPEND_MARKER_3 -->

### 导航与安全参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `step_distance` | 1.5 | 每步前进距离(m) |
| `direction_alpha` | 0.3 | 方向 EMA 滤波系数（0~1，越小越平滑） |
| `stuck_timeout` | 3.0 | 悬停超时触发后退(s) |
| `retreat_distance` | 2.0 | 后退脱困距离(m) |
| `cloud_timeout` | 1.0 | 点云超时判定(s) |

## ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| 订阅 | `/mavros/state` | `mavros_msgs/State` | 飞控状态（连接、解锁、模式） |
| 订阅 | `/mavros/local_position/odom` | `nav_msgs/Odometry` | FAST_LIO 里程计位姿 |
| 订阅 | `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST_LIO 全局注册点云 |
| 发布 | `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 位置+航向指令(20Hz) |
| 服务 | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁 |
| 服务 | `/mavros/set_mode` | `mavros_msgs/SetMode` | 飞行模式切换 |

## 使用方法

### 编译

```bash
cd ~/uav_ws && catkin_make
source devel/setup.bash
```

### 运行

```bash
# 终端1：启动传感器链路（MAVROS + 雷达 + FAST_LIO）
roslaunch indoor_inspection mapping.launch

# 终端2：启动避障演示
roslaunch obstacle_avoidance avoidance_demo.launch
```

### 自定义目标点

编辑 `config/avoidance.yaml`：

```yaml
goal_x: 15.0    # 前方 15m
goal_y: 5.0     # 右侧 5m
```

或通过命令行覆盖：

```bash
roslaunch obstacle_avoidance avoidance_demo.launch _goal_x:=15.0 _goal_y:=5.0
```

## 实飞测试建议

1. **首次测试**：设置短距离目标（`goal_x: 3.0`），无障碍物，验证起飞→直飞→降落流程
2. **避障验证**：在飞行路径上放置障碍物（纸箱、柱子等），观察绕行行为
3. **参数调优**：
   - 绕行幅度过大 → 增大 `w_target`
   - 转弯太急 → 增大 `w_smooth`
   - 方向抖动 → 减小 `direction_alpha`（如 0.15）或增大 `w_prev`
   - 漏检障碍物 → 降低 `obstacle_threshold` 或增大 `max_range`
   - 地面误检测 → 增大 `ground_clearance`（如 0.5）
   - 贴边飞行 → 增大 `enlarge_sectors`（如 3）
   - 减速不够 → 增大 `slowdown_radius`（如 4.0）
4. **安全注意**：确保遥控器随时可切回手动模式（MANUAL/STABILIZED）

## 依赖

- ROS Noetic
- MAVROS
- PCL 1.10+
- FAST_LIO（提供 `/cloud_registered` 和 `/mavros/local_position/odom`）
- PX4 飞控固件

## 许可证

MIT
