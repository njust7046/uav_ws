# circle_crossing

无人机穿越圆圈障碍物飞行节点。从起飞点起飞，依次穿越多个不同高度的圆圈，到达终点后爬升至安全高度直线飞回起飞点自动降落。

## 原理

这是一个带独立高度的航点导航任务，不是避障任务。圆圈位置预先已知，无人机只需精确飞过每个圆心。

核心设计：
- 每个圆圈航点带独立 z 高度，适应不同高度的圆圈
- 每个圆圈前后自动插入过渡点（approach / depart），确保无人机以水平姿态穿越圆心
- 穿越时不悬停，连续飞行
- 返回时先爬升到安全高度（高于所有圆圈），避免撞到圆圈支架

## 系统架构

```
MID-360 → FAST_LIO(定位) → lidar_to_mavros(桥接) → MAVROS → PX4
(雷达驱动)   (激光里程计)      (位姿格式转换)        (MAVLink)  (飞控)
```

- 定位完全依赖 FAST_LIO 激光雷达里程计，不使用 GPS
- lidar_to_mavros 将 FAST_LIO 的 `/Odometry` 通过 TF 转换后发布到 `/mavros/vision_pose/pose`
- PX4 的 EKF2 融合 vision pose 作为位置估计来源
- 不依赖 PCL/点云处理，纯航点飞行

## 状态机流程

```
等待FCU连接 → 等待FAST_LIO初始化 → 预发setpoint(5s) → OFFBOARD+解锁
→ 起飞到 takeoff_height
→ 对每个圆圈: 进入过渡点 → 圆心 → 离开过渡点
→ 飞到终点
→ 爬升到 safe_return_height
→ 直线飞回起飞点上方
→ AUTO.LAND
```

## 配置

编辑 `config/circle_crossing.yaml`：

```yaml
takeoff_height: 2.0        # 起飞高度(m)
safe_return_height: 3.0    # 返回安全高度(m)，应高于所有圆圈
reach_threshold: 0.3       # 航点到达判定阈值(m)
approach_distance: 1.5     # 圆圈前后过渡点距离(m)

# 圆圈列表（相对起飞点偏移）
circles:
  - {x: 3.0,  y: 0.0, z: 1.5}
  - {x: 6.0,  y: 0.0, z: 2.0}
  - {x: 9.0,  y: 0.0, z: 1.2}

# 终点
goal_x: 12.0
goal_y: 0.0
```

### 参数说明

| 参数 | 含义 | 建议值 |
|------|------|--------|
| `takeoff_height` | 起飞后悬停高度 | 根据第一个圆圈高度调整 |
| `safe_return_height` | 返回时飞行高度，必须高于所有圆圈 | 最高圆圈 z + 1.0m |
| `reach_threshold` | 到达航点的判定距离 | 0.2~0.5m，圆圈越小越要小 |
| `approach_distance` | 过渡点到圆心的距离 | 1.0~2.0m，越大水平飞行段越长 |

### 圆圈坐标含义

- `x` — 相对起飞点的前方距离（米），正值为前方

- `y` — 相对起飞点的侧向距离（米），正值为左侧
- `z` — 圆心离地面的高度（米）

## 如何获取圆圈坐标

### 方法一：卷尺量测（简单直接）

在草坪上摆好圆圈后，用卷尺量测：
1. 确定起飞点位置
2. 量每个圆圈圆心到起飞点的前方距离（x）和侧向距离（y）
3. 量每个圆圈圆心离地面的高度（z），从地面到圆心
4. 量终点到起飞点的距离，填入 `goal_x` / `goal_y`

### 方法二：FAST_LIO 定位读取（更精确）

适用于圆圈不在一条直线上、或需要更高精度的场景：

```bash
# 终端1：启动传感器链路
roslaunch indoor_inspection mapping.launch

# 终端2：手持无人机走到每个位置，读取坐标
rostopic echo -n 1 /Odometry
```

记录每个位置 `pose.pose.position` 中的 x, y, z 值。第一个记录点作为起飞点，后续坐标减去起飞点坐标即为 yaml 中的偏移量。z 值（圆心离地高度）建议用卷尺直接量，更准确。

注意：`/Odometry` 是 FAST_LIO 直接输出的话题。`/mavros/local_position/odom` 需要飞控在线才有数据，离线标定时请使用 `/Odometry`。

## 使用

```bash
cd ~/uav_ws && catkin_make
source devel/setup.bash

# 终端1：启动传感器链路
roslaunch indoor_inspection mapping.launch

# 终端2：启动穿越圆圈任务
roslaunch circle_crossing circle_crossing.launch
```

## PX4 飞控配置

本方案使用 FAST_LIO 视觉定位而非 GPS，需要配置 PX4 的 EKF2：

- `EKF2_AID_MASK`：启用 vision position 和 vision yaw，关闭 GPS fusion
- `EKF2_EV_CTRL`（PX4 v1.14+）：启用 external vision
- `EKF2_HGT_MODE`：设为 Vision（使用视觉高度而非气压计）

配置完成后飞控的位置估计完全来自 FAST_LIO 激光里程计，不需要 GPS 模块。

## 户外飞行注意事项

### FAST_LIO 户外定位特性

FAST_LIO 在户外可以正常工作，但定位精度取决于环境中的几何特征：

**效果好的场景：**
- 周围有建筑物、树木、围栏等结构物提供丰富的点云特征
- 圆圈支架本身也是有效的特征点
- 飞行范围在几十米以内，累积漂移可控

**效果差的场景：**
- 空旷草坪，四周没有任何物体，雷达扫不到足够特征
- 长距离飞行（几十米以上），里程计累积漂移会增大
- 强风天气导致点云畸变

### 户外飞行前检查清单

1. **环境特征评估**：实飞前先手持无人机在场地走一圈，运行 `rostopic echo /Odometry` 观察位置输出是否稳定。如果坐标跳变或漂移明显，说明环境特征不足
2. **增加环境特征**：如果草坪过于空旷，可以在飞行路径两侧放置纸箱、锥桶等物体，给雷达增加参考特征
3. **FAST_LIO 初始化**：启动后需要缓慢移动无人机（或雷达），让 FAST_LIO 积累足够点云完成初始化，初始化完成前 `/Odometry` 输出全零
4. **遥控器准备**：确保遥控器在手，随时可切手动模式（如 STABILIZED 或 POSITION）接管飞行
5. **电池电量**：确认电池电量充足，穿越任务全程约需 3~5 分钟
6. **风速**：建议风速 < 3 级，大风会影响穿越精度和定位稳定性

## 安全说明

- `safe_return_height` 必须设置为高于所有圆圈的高度，避免回程撞到圆圈支架
- 过渡点确保无人机水平穿越圆心，减少因倾斜角度导致撞圈的风险
- `reach_threshold` 默认 0.3m，在圆圈直径通常 > 1m 的情况下有足够余量；如果圆圈较小，应适当减小此值
- 遥控器随时可切手动模式接管，这是最重要的安全保障
- 首次飞行建议将圆圈间距加大、飞行速度放慢，确认系统稳定后再调整到目标参数
- 确保飞行场地周围无人员靠近飞行路径
