# 无障碍赛道 Gap 定向速度方案与曲率转向后续计划

## 1. 文档范围

本文记录 Lab 4 在 `levine_blocked` 无障碍地图上的新控制方案，包括：

- 已完成并验证的 gap 定向净空速度控制；
- 直道最高速度与原有安全弯速的协调方法；
- 15 Hz LiDAR 条件下的三圈测试结果；
- 尚未实现的 Ackermann 曲率转向方案及后续验证计划。

当前目标只针对无障碍 Levine 环路。密集障碍物、disparity extender、复杂环境切换和侧墙保护不在本方案范围内。

## 2. 原问题

基础 FTG 已经能够稳定完成无障碍地图，但车辆进入弯道时速度下降过多。原控制器使用车头正前方 `±10°` 雷达窗口的第 10 百分位距离计算正常速度：

```text
forward_clearance = percentile(front_window, 10)
clearance_speed = gain × (forward_clearance - stop_distance)
```

这种方法在直道上合理，但在弯道上测量了错误的方向。车辆已经选择了指向弯道内部的 gap，正前方雷达却会看到外侧墙壁。结果是：

- 所选行驶方向仍有 4–6 m 净空；
- 正前方可能只剩 1–2 m；
- 安全的弯道被误判成需要低速行驶。

这与 L05 课件第 34–35 页的建议不一致。课件要求根据“所选 gap 内的障碍物距离”决定速度，而不是始终根据车体正前方距离决定速度。

## 3. 已完成的新方案

### 3.1 保留基础 FTG 转向路径

新方案没有重新启用之前试验过的复杂附加算法。当前转向流程仍然是：

1. 清理无效 LiDAR 数据并限制最大量程；
2. 只保留车辆前方视场；
3. 为最近局部障碍建立安全气泡；
4. 找出能够容纳车辆的最大连续 gap；
5. 在 gap 内对深度进行 21 点平滑并选择稳定目标点；
6. 将目标方向限制在最大转角范围内。

以下算法仍然不参与当前无障碍地图控制：

- disparity extender；
- 简单/复杂环境模式切换；
- 侧墙保护；
- 最大距离截断后取 gap 中心；
- 新转向低通滤波和转向变化率限制。

### 3.2 测量所选方向的净空

获得目标索引 `best_index` 后，控制器在目标方向附近取一个角度窗口：

```text
窗口中心：best_index
窗口范围：目标方向 ±5°
窗口边界：不得超出当前选中的 gap
```

目标净空定义为窗口内有效距离的第 20 百分位：

```text
target_clearance = percentile(valid_target_window, 20)
```

不直接使用单条最远射线，因为单条射线可能从墙角或门缝穿过。第 20 百分位能够过滤偶然的远距离射线，同时不会像取最小值那样被少量噪声过度限制。

### 3.3 分离安全停车与正常限速

正前方距离仍然保留，但只作为最后一道紧急停车条件：

```text
forward_clearance <= 0.45 m  →  停车
```

正常行驶速度不再由 `forward_clearance` 连续限制，而是由 `target_clearance` 决定。这样车辆可以在正前方接近弯道外墙时，继续根据弯道内部的真实可行空间行驶。

### 3.4 Gap 净空分段速度

目标方向净空通过分段线性函数转换成速度上限：

| 目标方向净空 | 速度上限 |
|---:|---:|
| 0.45 m | 0.0 m/s |
| 0.60 m | 0.4 m/s |
| 1.20 m | 最高速度的 30% |
| 2.50 m | 最高速度的 60% |
| 4.50 m 及以上 | 最高速度 |

在无障碍地图 `max_speed = 5.0 m/s` 时，对应速度为：

```text
0.45 m → 0.0 m/s
0.60 m → 0.4 m/s
1.20 m → 1.5 m/s
2.50 m → 3.0 m/s
4.50 m → 5.0 m/s
```

区间之间使用线性插值。

### 3.5 直道 5 m/s 与原弯速的协调

仅把 `max_speed` 从 3 提升到 5，并把 `turn_slowdown` 保持为 0.65，并不能真正保留原来的弯速。原公式会随最高速度一起放大所有弯道速度。例如相同的 `0.24 rad` 转角：

```text
max_speed = 3.0 时：约 1.62 m/s
max_speed = 5.0 时：约 2.70 m/s
```

第一次 5 m/s 测试因此在第一处急弯失效。最终方案引入 `corner_speed_reference = 3.0 m/s`：

```text
normalized_turn = abs(steering) / max_steering

corner_speed = max(
    min_speed,
    corner_speed_reference × (1 - turn_slowdown × normalized_turn)
)
```

接近直行时，再用高斯权重平滑加入 3–5 m/s 的直道增量：

```text
straight_boost =
    (max_speed - corner_speed_reference)
    × exp(-(abs(steering) / straight_boost_angle)²)

turn_speed_limit = corner_speed + straight_boost
```

当前 `straight_boost_angle = 0.10 rad`。因此：

- 接近零转角时允许达到 5 m/s；
- 中等转角时直道增量快速衰减；
- 急弯速度回到之前已经验证过的 3 m/s 参数曲线。

最终速度为：

```text
command_speed = min(turn_speed_limit, gap_speed_limit)
```

恢复模式仍然使用独立的低速限制。

## 4. 当前参数

`levine_blocked_launch.py` 使用：

```python
{
    'clearance_speed_gain': 1.0,
    'corner_speed_reference': 3.0,
    'max_speed': 5.0,
    'min_speed': 0.4,
    'turn_slowdown': 0.65,
}
```

gap 定向速度的默认参数位于 `reactive_node.py`：

```text
target_clearance_half_angle = 5°
target_clearance_percentile = 20
gap_speed_near_distance = 0.6 m
gap_speed_low_distance = 1.2 m
gap_speed_medium_distance = 2.5 m
gap_speed_fast_distance = 4.5 m
gap_speed_low_fraction = 0.3
gap_speed_medium_fraction = 0.6
```

## 5. 15 Hz 验证结果

测试条件：

- 地图：`levine_blocked`；
- 起点：`(-12.0, 0.0, 0.0)`；
- 方向：逆时针；
- LiDAR 发布频率：约 15 Hz；
- 连续运行：3 圈。

结果：

| 圈数 | 圈速 |
|---:|---:|
| 1 | 33.92 s |
| 2 | 34.27 s |
| 3 | 33.17 s |

其他统计：

```text
碰撞：无
扫描中断：无
最高指令速度：5.00 m/s
全程平均指令速度：2.14 m/s
abs(steering) >= 0.10 时的平均指令速度：1.65 m/s
```

之前 3 m/s 基准的三圈为 45.09 s、42.14 s 和 44.53 s。新方案将稳定圈速降低到约 33–34 s。

## 6. 已实现的曲率转向（默认关闭）

### 6.1 动机

当前控制器直接把目标 LiDAR 方位角当作 Ackermann 前轮转角：

```text
steering = target_bearing
```

这种映射简单有效，但没有显式考虑轴距、目标点距离和车辆的非完整约束。更严格的方法应先把目标点转换为期望曲率，再转换成前轮转角。

### 6.2 候选数学模型

设目标点在车体坐标系中为：

```text
x = target_distance × cos(target_bearing)
y = target_distance × sin(target_bearing)
```

Pure Pursuit 形式的目标曲率为：

```text
curvature = 2y / (x² + y²)
```

根据自行车模型和轴距 `L` 转换成前轮转角：

```text
steering = atan(L × curvature)
```

另一种实现是不直接使用原始目标距离，而是在目标方向上设置固定或随速度变化的前视距离 `Ld`：

```text
curvature = 2 × sin(target_bearing) / Ld
steering = atan(L × curvature)
```

### 6.3 默认策略与验证状态

曲率转向已经以独立参数接入，但默认保持关闭。三圈数据表明，紧弯处最终速度确实主要受转角限速约束；当前直接角度映射已经在 15 Hz 下稳定完成约 33 秒圈速，因此它仍是回退基线。

如果直接使用完整目标距离计算 Pure Pursuit 曲率，5–6 m 的远目标会产生很小的转角，可能导致车辆在急弯中转向过晚。因此在没有系统确定前视距离之前，不应直接替换当前转向映射。

### 6.4 后续实施方案

曲率功能现在作为独立、默认关闭的实验参数实现，不能直接覆盖已验证控制器。

建议步骤：

1. 已添加 `use_curvature_steering` 参数，默认 `False`；
2. 已添加 `curvature_lookahead` 参数，默认 0.65 m，初始测试范围建议 0.55–0.80 m；
3. 从 `best_index` 获得目标方位角；
4. 使用固定前视距离计算曲率和前轮转角；
5. 继续应用现有 `max_steering` 限制；
6. 不同时修改 gap 选择和速度参数；
7. 记录直接角度与曲率角度的差值、转角饱和比例和车辆横向位置；
8. 先完成一圈无碰撞测试，再运行三圈；
9. 只有在三圈均无碰撞且最快圈优于 33.17 s 时，才考虑设为默认；
10. 若发生转向过晚、外墙碰撞或摆动增加，立即保持当前直接角度方案。

还可以在固定前视距离稳定后，再测试随速度变化的前视距离：

```text
Ld = clip(Ld_min + k_speed × speed, Ld_min, Ld_max)
```

速度越高使用越远的前视点可以降低抖动，但可能造成急弯转向不足，所以必须晚于固定前视距离方案验证。

## 7. 验收标准

任何后续曲率版本至少必须满足：

- 15 Hz LiDAR 下连续完成 3 圈；
- 全程无碰撞、无扫描停滞；
- 不重新引入明显左右摆动；
- 三圈最慢圈不超过 35 s；
- 最快圈优于当前 33.17 s 才认为具有实际收益；
- 失败时可以只关闭一个参数恢复当前验证版本。

## 8. 相关文件

- `gap_follow/scripts/reactive_node.py`：目标净空、gap 速度和转角速度计算；
- `gap_follow/launch/levine_blocked_launch.py`：无障碍地图参数；
- `gap_follow/VERIFIED_BASELINE.md`：历史基准和本轮三圈结果。

