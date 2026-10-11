# GPS-TRON1

TRON1 的 ROS1 Noetic 户外 GPS 导航、FAST-LIO 融合定位、局部碰撞安全、
调试可视化和统一启动工程。

本仓库保留现有 waypoint 导航架构，不引入 `move_base`、TEB 或 Nav2，
也不包含或修改官方 Livox、FAST-LIO、机器人底盘 SDK 源码。官方传感器与定位包
作为外部 workspace 依赖，由本仓库的 overlay 启动层调用。

> 当前状态：仅允许 `TEST` / `dry_run=true` 联调。传感器持续运行、GNSS 静止漂移、
> 航向转换实机验证全部通过前，不应启动 REAL 导航。

## 系统架构

```text
南方 GNSS USB / 手机 NetGPS
              |
              v
          /gps/fix
              |
              v
     gps_global_converter
              |
              v
/open_nav/gps_filtered_pose       /Odometry (FAST-LIO FRD)
              |                         |
              +------> gps_fastlio_fusion
                              |
                              v
                      /open_nav/odom
                              |
                              v
                       waypoint_tracker
                              |
                              v
                  /open_nav/cmd_vel_raw
                              |
                              v
                     collision_safety
                              |
                              v
                    /open_nav/cmd_vel
                              |
                              v
                  tron_controller_bridge
```

局部轨迹规划器当前保持 `SHADOW`：

```text
/open_nav/odom + /open_nav/mission_path + /livox/lidar_filter1
                              |
                              v
                 local_trajectory_planner
                              |
                              +--> /open_nav/local_plan
                              +--> /open_nav/local_plan_candidates
                              +--> /open_nav/local_planner/status
```

它只发布路径和诊断，不发布任何 `cmd_vel`。

## 仓库内容

```text
catkin_ws/
├── runtime_scripts/                   # 用户直接执行的统一运行入口
└── src/
    ├── phone_gps_bridge/              # TCP、UDP、USB 串口 NMEA -> ROS
    ├── tron_open_nav/                 # 早期兼容包
    ├── tron_open_space_nav/           # GPS导航、融合、任务、控制桥与Mapviz
    ├── tron_sensor_bridge/            # 官方 Livox/FAST-LIO overlay 启动适配
    ├── tron_local_collision_safety/   # 最终速度安全仲裁层
    ├── tron_local_trajectory_planner/ # 可视化 DWA 风格 SHADOW planner
    └── tron_nav_dashboard/            # 地面站综合监控 Dashboard
```

### 外部依赖

机器人环境需要提供：

- Ubuntu 20.04、ROS Noetic
- `livox_ros_driver2`
- `fast_lio`
- `fast_lio_localization_sc_qn`
- TRON1 官方驱动、描述与底盘控制环境
- Mapviz、RViz、PyQt5（仅地面站需要）

这些官方包不复制进本仓库。可通过 `LIVOX_WS`、`FASTLIO_WS` 指定其 workspace。

## 关键 ROS 接口

| Topic | 类型/用途 | 主要发布者 |
|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | GNSS bridge |
| `/gps/course` | RMC 地面航迹角，仅作诊断 | GNSS bridge |
| `/Odometry` | FAST-LIO 原始 odometry | `/laserMapping` |
| `/livox/lidar_filter1` | `PointCloud2`，frame 通常为 `body` | FAST-LIO localization |
| `/open_nav/odom` | ROS REP-103 map 融合位姿 | `gps_fastlio_fusion` |
| `/open_nav/mission_path` | 已确认的全局任务路径 | mission manager |
| `/open_nav/cmd_vel_raw` | waypoint tracker 原始速度 | `waypoint_tracker` |
| `/open_nav/cmd_vel` | 最终安全速度 | `collision_safety` |
| `/open_nav/obstacle_status` | `FREE/SLOW/STOP` | `collision_safety` |
| `/open_nav/local_plan` | SHADOW 最优局部轨迹 | local planner |
| `/open_nav/heading/status` | raw/nav/global 航向状态 | fusion |

完整 bringup 中 `/collision_safety` 必须是 `/open_nav/cmd_vel` 的唯一发布者；
旧 `/lidar_obstacle_avoid` 会被禁用，但源码保留用于历史兼容。

## 构建

工程保持 `catkin_make_isolated`，不使用 `catkin_make`、`catkin build` 或
`--install`：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --force-cmake
```

只重建本仓库关键包：

```bash
catkin_make_isolated --pkg phone_gps_bridge --force-cmake
catkin_make_isolated --pkg tron_open_space_nav --force-cmake
catkin_make_isolated --pkg tron_sensor_bridge --force-cmake
catkin_make_isolated --pkg tron_local_collision_safety --force-cmake
catkin_make_isolated --pkg tron_local_trajectory_planner --force-cmake
catkin_make_isolated --pkg tron_nav_dashboard --force-cmake
```

不要连续 source 各包的 `setup.bash`。`catkin_make_isolated --pkg` 生成的 setup
会重建自己的 underlay 链，可能使其他包从 `ROS_PACKAGE_PATH` 消失。
所有运行脚本通过 `local_setup.bash` 和 `.catkin` marker 统一组合环境。

## 部署到机器人

机器人标准路径为 `/home/guest/catkin_ws`。从仓库副本部署时：

```bash
rsync -a --exclude='__pycache__/' --exclude='*.pyc' \
  catkin_ws/src/ guest@10.192.1.3:/home/guest/catkin_ws/src/

rsync -a \
  catkin_ws/runtime_scripts/ \
  guest@10.192.1.3:/home/guest/catkin_ws/runtime_scripts/
```

随后在机器人上重建修改过的包，并确保脚本可执行：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --pkg phone_gps_bridge --force-cmake
catkin_make_isolated --pkg tron_open_space_nav --force-cmake
catkin_make_isolated --pkg tron_sensor_bridge --force-cmake
catkin_make_isolated --pkg tron_local_collision_safety --force-cmake
catkin_make_isolated --pkg tron_local_trajectory_planner --force-cmake
chmod +x ~/catkin_ws/runtime_scripts/*.sh
```

## 启动

### 推荐：统一 TEST bringup

脚本不会启动第二个 `roscore`，机器人 ROS master 必须已存在：

```bash
~/catkin_ws/runtime_scripts/start_full_bringup.sh test
```

该命令启动非 GUI 机器人组件：

1. Livox 驱动
2. FAST-LIO localization
3. 南方 GNSS 串口接收
4. GPS/FAST-LIO 融合与 waypoint 导航
5. SHADOW local planner
6. collision safety
7. `dry_run=true` controller bridge

检查完整链路：

```bash
~/catkin_ws/runtime_scripts/check_full_bringup.sh
```

### 分模块诊断

每条命令都自动构造 ROS 环境，不需要预先 source：

```bash
~/catkin_ws/runtime_scripts/start_livox.sh
~/catkin_ws/runtime_scripts/start_localization.sh
~/catkin_ws/runtime_scripts/start_phone_gps.sh serial
~/catkin_ws/runtime_scripts/start_gps_nav.sh
~/catkin_ws/runtime_scripts/start_collision_safety.sh
~/catkin_ws/runtime_scripts/check_runtime.sh
```

`start_localization.sh` 必须先于 collision safety，因为
`/fast_lio_localization_sc_qn_node` 同时提供 TF 和
`/livox/lidar_filter1`。

其他 GNSS 模式：

```bash
~/catkin_ws/runtime_scripts/start_phone_gps.sh tcp
~/catkin_ws/runtime_scripts/start_phone_gps.sh udp
```

南方 GNSS 默认串口：

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00
```

可用 `GNSS_SERIAL_DEVICE` 覆盖。应配置 `dialout`/udev 权限，不要把密码或
长期 `chmod 666` 写进部署脚本。

## Dashboard 与可视化

Dashboard 只在开发电脑/GPS-TRON1 容器运行：

```bash
/root/catkin_ws/runtime_scripts/start_nav_dashboard.sh
```

主要功能：

- 中文自适应界面、离线武汉瓦片地图
- GNSS、FAST-LIO、融合定位和任务路径
- 地图选点、撤销、清空、确认、暂停、继续
- `body` 局部 LiDAR 点云与完整过滤计数
- SHADOW 候选轨迹和最优 local plan
- collision safety 与速度链诊断
- 人工绝对航向标定
- 导航调试 rosbag 记录

Dashboard 不发布 `/open_nav/cmd_vel` 或 `/open_nav/cmd_vel_raw`，只通过
现有 collector 接口发布 `/open_nav/goal_add`。

独立可视化入口：

```bash
~/catkin_ws/runtime_scripts/start_mapviz_navigation_test.sh
~/catkin_ws/runtime_scripts/start_navigation_debug_rviz.sh
```

地图颜色：绿色 GNSS、青色 filtered GPS、黄色实际融合轨迹、蓝色任务路径、
品红色 local plan、红色机器人融合姿态、灰色原始 FAST-LIO。

## 坐标系与航向

TRON1 当前 FAST-LIO `/Odometry` 的 `camera_init/body1` 使用 FRD：

```text
+X forward, +Y right, +Z down
```

导航 `body/open_base/map` 使用 ROS REP-103 FLU：

```text
+X forward/east, +Y left/north, +Z up
```

`gps_fastlio_fusion` 使用统一基变换 `S=diag(1,-1,-1)` 同时转换 translation、
orientation、linear velocity 和 angular velocity。禁止只在 Dashboard 中把 yaw
取反来掩盖底层错误。

绝对航向由 session 级人工标定提供：

```text
yaw_offset = wrap(global_heading - normalized_fastlio_yaw)
global_yaw = wrap(normalized_fastlio_yaw + yaw_offset)
```

FAST-LIO 重启、时间戳回退、序号重置或大幅位姿跳变后，状态自动回到
`UNALIGNED`。REAL mission 在未标定时会被 Dashboard 阻止。

## 碰撞安全策略

`tron_local_collision_safety` 使用 `/livox/lidar_filter1`，默认局部 frame 为
`body`。点云已经是 `body` 时直接处理，不经过错误的 `body -> open_base`
千米级 TF。

处理链：

```text
PointCloud2
 -> finite/height/range/self filter
 -> 0.05m local occupancy
 -> footprint + braking envelope + forward corridor
 -> trajectory collision check
 -> FREE / SLOW / STOP
```

- `FREE`：完整透传 Twist
- `SLOW`：所有速度分量同比缩放，保持方向和曲率
- `STOP`：发布全零 Twist
- 点云超时或 TF 失败：fail-safe `STOP`
- 不自动左绕、右绕或改变 waypoint 目标

为避免 TRON1 进入不可执行的低速踏步区，SLOW 前进输出下限默认为
`0.35 m/s`；真正碰撞风险仍然直接 STOP。

## 定位保护

实机记录曾出现 FAST-LIO 几乎静止但 GNSS 漂移约 35m，旧融合逻辑累计约
28m GPS correction。当前默认：

```yaml
stationary_correction_gain: 0.0
```

FAST-LIO 判断静止时冻结 GPS correction，避免把静止 GNSS 漂移变成机器人
运动；机器人运动时继续使用正常 correction gain。该保护不能替代 GNSS 数据
质量检查。

## TEST 验收清单

在考虑 REAL 前至少确认：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /livox/lidar_filter1
rostopic hz /gps/fix
rostopic hz /open_nav/odom
rostopic echo /open_nav/heading/status
rostopic echo /open_nav/obstacle_status
rostopic info /open_nav/cmd_vel
```

必须满足：

- `/livox/lidar_filter1` 持续约 10Hz，不能长时间 timeout
- `/Odometry` 持续更新
- `/collision_safety` 是 `/open_nav/cmd_vel` 唯一发布者
- `/lidar_obstacle_avoid` 未运行
- 顺时针旋转时 normalized/global yaw 减小
- 逆时针旋转时 normalized/global yaw 增加
- 航向已人工标定且 Dashboard 箭头与物理方向一致
- 空场 `FREE`、接近障碍 `SLOW`、碰撞风险 `STOP`
- `controller_dry_run=true`

## 当前已知问题

截至当前版本仍未完成 REAL 验收：

1. 最近实机日志中 `/livox/lidar_filter1` 曾连续超时 100 秒以上，必须先定位
   FAST-LIO localization/点云发布中断原因。
2. GNSS 静止测试存在几十米漂移；融合层已冻结静止 correction，但 GNSS 接收、
   covariance、FIX 类型和多路径问题仍需单独处理。
3. 新的 FRD -> FLU 航向转换已通过 bag、官方 TF 和单元测试，仍需部署后完成
   物理顺/逆时针 TEST 验证。
4. 静止检测当前主要参考 FAST-LIO `twist.linear.x`。若官方 twist 不可靠，后续应
   与 pose 增量联合判断。
5. 局部轨迹规划器仍为 SHADOW，不能接管控制。

在上述问题关闭前，不要运行：

```bash
~/catkin_ws/runtime_scripts/start_full_bringup.sh real
```

## 离线测试

```bash
python3 catkin_ws/src/phone_gps_bridge/test/test_nmea_parser.py
python3 catkin_ws/src/tron_open_space_nav/scripts/test_geodetic_utils.py
python3 catkin_ws/src/tron_open_space_nav/scripts/test_fastlio_frame.py
python3 catkin_ws/src/tron_open_space_nav/scripts/test_heading_alignment.py
python3 catkin_ws/src/tron_nav_dashboard/scripts/test_dashboard_arrow.py
```

分析实机 bag：

```bash
python3 catkin_ws/src/tron_open_space_nav/scripts/analyze_heading_direction.py \
  /path/to/tron_nav_YYYYMMDD_HHMMSS.bag
```

更详细的模块说明位于各 package 的 `README.md` 和
`catkin_ws/src/tron_open_space_nav/docs/`。
