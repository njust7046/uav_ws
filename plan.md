# VFH+ 避障节点优化计划

## 优化目标

针对之前分析的 5 个主要问题，在 `avoidance_node.cpp` 和 `avoidance.yaml` 中进行改进。仅修改这两个文件，不动其他文件。

## 优化项

### 1. 距离自适应速度控制（解决"没有速度规划"问题）

当前：固定 1.5m 步进，靠近障碍物时不减速。

优化：step_distance 根据最近障碍物距离动态缩放。

```
实际步进 = step_distance × clamp((min_obs_dist - safety_radius) / (slowdown_radius - safety_radius), 0.2, 1.0)
```

- 障碍物距离 > slowdown_radius(默认 3.0m)：全速
- 距离在 safety_radius ~ slowdown_radius 之间：线性减速到 20%
- 距离 < safety_radius：紧急悬停（已有）

新增参数：`slowdown_radius: 3.0`

### 2. 直方图膨胀（解决"安全余量不足"问题）

当前：只看单个扇区是否阻塞，无人机可能贴着障碍物边缘飞。

优化：阻塞扇区向两侧各膨胀 `enlarge_sectors`（默认 2 个 = 10°），相当于给障碍物加了角度安全边界。在二值化之后、gap 搜索之前执行膨胀。

新增参数：`enlarge_sectors: 2`

### 3. 方向输出低通滤波（解决"方向抖动"问题）

当前：每帧直接使用 VFH+ 输出方向，帧间可能跳变。

优化：对选定方向做指数移动平均（EMA）滤波：

```
smoothed_angle = prev_angle + alpha × normalize(raw_angle - prev_angle)
```

alpha 默认 0.3，越小越平滑。

新增参数：`direction_alpha: 0.3`

### 4. 全阻塞后退恢复（解决"U形障碍物卡死"问题）

当前：全阻塞时原地悬停，永远等待。

优化：全阻塞持续超过 `stuck_timeout`（默认 3s）后，沿来时方向的反方向后退 `retreat_distance`（默认 2.0m），尝试脱困。后退完成后重新尝试 VFH+ 导航。

新增参数：`stuck_timeout: 3.0`，`retreat_distance: 2.0`

### 5. 多层点云切片（解决"只有 2D 避障"问题）

当前：只切飞行高度 ±0.5m 一层。

优化：切 3 层（低/中/高），任一层检测到障碍物都计入直方图：
- 低层：cur_z - 0.8m ~ cur_z - 0.2m（检测低矮障碍物）
- 中层：cur_z - slice_half ~ cur_z + slice_half（原有）
- 高层：cur_z + 0.2m ~ cur_z + 0.8m（检测高处障碍物如树枝）

新增参数：`multi_layer: true`，`layer_offset: 0.5`（额外层与飞行高度的偏移）

## 修改文件

1. `src/obstacle_avoidance/config/avoidance.yaml` — 新增 5 个参数
2. `src/obstacle_avoidance/src/avoidance_node.cpp` — 修改 VFH+ 函数和导航循环

## 不修改的文件

- CMakeLists.txt、package.xml、launch 文件 — 无需改动
- indoor_inspection 包 — 完全不动
