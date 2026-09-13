# 系统架构

## 总体数据流

```
Phone GPS (UDP NMEA)
        ↓
gps_udp_receiver          [phone_gps_bridge]
        ↓
    /gps/fix
        ↓
┌───────────────────────────────────────────────────┐
│ gps_global_converter                              │
│   WGS84 → ENU (true north)                        │
│   TF: earth → map                                 │
│   /global_pose                                    │
└───────────────────────────────────────────────────┘
        ↓
gps_enu_converter           (legacy /open_nav/pose, open_map)
        ↓
┌───────────────────────────────────────────────────┐
│ gps_fastlio_fusion                                │
│   /gps/fix + /Odometry (FAST-LIO)                 │
│   → /open_nav/odom                                │
│   TF: map → open_odom → open_base                 │
└───────────────────────────────────────────────────┘
        ↓
┌───────────────────────────────────────────────────┐
│ waypoint_tracker                                  │
│   Pure Pursuit → /open_nav/cmd_vel_raw            │
│   /open_nav/mission_status                        │
└───────────────────────────────────────────────────┘
        ↓
lidar_obstacle_avoid
        ↓
    /open_nav/cmd_vel
        ↓
tron_controller_bridge → WebSocket → TRON1
```

## TF 树

### 全球导航模式（gps_global_nav.launch）

```
earth
 └── map              (真北 ENU，首帧 GPS 为原点)
      └── open_odom   (gps_fastlio_fusion)
           └── open_base
```

### 官方 FAST-LIO（不修改，并行存在）

```
map
 └── camera_init
      └── body1
```

两套 TF **互不冲突**。

## 节点说明

| 节点 | 包 | 输入 | 输出 | 作用 |
|------|-----|------|------|------|
| `gps_udp_receiver` | phone_gps_bridge | UDP 10110 | `/gps/fix` | 手机 GPS 接入 |
| `gps_global_converter` | tron_open_space_nav | `/gps/fix` | `/global_pose`, TF | 全球真北 ENU 层 |
| `gps_enu_converter` | tron_open_space_nav | `/gps/fix` | `/open_nav/pose` | 兼容旧 open_map 可视化 |
| `gps_fastlio_fusion` | tron_open_space_nav | `/gps/fix`, `/Odometry` | `/open_nav/odom`, TF | GPS 位置校正 + 里程计融合 |
| `gps_waypoint_collector` | tron_open_space_nav | `/move_base_simple/goal` | markers | **单点** RViz 目标 |
| `gps_multi_waypoint_collector` | tron_open_space_nav | `/multi_navi_goals/mission` | `/open_nav/waypoints`, yaml | **多点**任务收集 |
| `waypoint_tracker` | tron_open_space_nav | `/open_nav/odom`, goals | `/open_nav/cmd_vel_raw`, status | Pure Pursuit 顺序导航 |
| `lidar_obstacle_avoid` | tron_open_space_nav | cmd_vel_raw, `/livox/lidar` | `/open_nav/cmd_vel` | 前向避障 |
| `tron_controller_bridge` | tron_open_space_nav | `/open_nav/cmd_vel` | WebSocket | TRON1 运动控制 |

## 航点输入模式

| waypoint_source | 话题 | 行为 |
|-----------------|------|------|
| `single_goal` | `/move_base_simple/goal` | 单点，到达后停止 |
| `multi_goal` | `/open_nav/waypoints` | P1→P2→… 顺序执行 |
| `manual_yaml` | `config/gps_waypoints.yaml` | 启动时加载文件 |

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gps/fix` | NavSatFix | 手机 GPS |
| `/global_pose` | PoseStamped | 机器人在 map 下 ENU 位置 |
| `/open_nav/odom` | Odometry | 融合定位 |
| `/open_nav/waypoints` | PoseArray | 多点任务（map 坐标） |
| `/open_nav/mission_status` | MissionStatus | 当前航点索引与状态 |
| `/open_nav/cmd_vel` | Twist | 最终速度指令 |

## MissionStatus

```
uint32 current_index   # 当前目标索引 (0-based)
uint32 total           # 总航点数
string state           # idle | navigating | completed
```

## Launch 文件

| Launch | 用途 |
|--------|------|
| `gps_global_nav.launch` | **室外全球导航**（机器人默认） |
| `open_nav.launch` | legacy open_map 模式 / 仿真 |
