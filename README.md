# GPS-TRON1

TRON1 四足机器人室外 GPS 导航工程，运行于 Ubuntu 20.04 / ROS Noetic。
工程保留轻量 GPS 航点导航架构，不引入 `move_base`、TEB 或 DWA。

## 系统架构

```text
手机 GPS / 南方 GNSS
        |
        v
     /gps/fix
        |
        v
GPS + FAST-LIO fusion ---- /open_nav/odom
        |
        v
 waypoint_tracker
        |
        v
/open_nav/cmd_vel_raw
        |
        v
local collision safety
        |
        v
 /open_nav/cmd_vel
        |
        v
TRON controller bridge
```

传感器链：

```text
Livox MID360 -> FAST-LIO localization
              |- /Odometry
              |- /cloud_registered
              `- /livox/lidar_filter1
```

`/fast_lio_localization_sc_qn_node` 同时提供定位 TF 和
`/livox/lidar_filter1`，因此 localization 必须先于 collision safety 启动。

## 主要目录

```text
catkin_ws/
  runtime_scripts/                  可部署的统一启动入口
  src/
    phone_gps_bridge/               手机/串口 GNSS 输入
    tron_sensor_bridge/             官方传感器链适配
    tron_open_space_nav/            GPS 融合、航点跟踪、Mapviz
    tron_local_collision_safety/    独立局部碰撞安全层
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- `catkin_make_isolated`，使用 `devel_isolated`
- Livox ROS Driver 2
- FAST-LIO localization
- Mapviz、RViz 及相关 ROS 插件

不要连续 source 各 package 的 `setup.bash`。isolated package 的 setup 会重建
underlay，可能让之前加载的 package 消失。运行脚本会通过各 package 的
`local_setup.bash` 统一构造环境。

## 部署

```bash
git clone https://github.com/potatoHu-a/GPS-TRON1.git
mkdir -p ~/catkin_ws/src ~/catkin_ws/run
rsync -a GPS-TRON1/catkin_ws/src/ ~/catkin_ws/src/
rsync -a GPS-TRON1/catkin_ws/runtime_scripts/ ~/catkin_ws/run/
chmod +x ~/catkin_ws/run/*.sh

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated
```

构建仍使用 `catkin_make_isolated`，不需要 install space。日常启动不需要手工
source ROS 环境，也不要把 `rosrun <package> <startup_script>` 作为入口。

## 实机启动顺序

每条命令在独立终端执行：

```bash
# 1. 只启动 Livox 驱动
~/catkin_ws/run/start_livox.sh

# 2. 只启动 FAST-LIO localization
~/catkin_ws/run/start_localization.sh

# 3. 选择一种 GPS 输入
~/catkin_ws/run/start_phone_gps.sh tcp
# ~/catkin_ws/run/start_phone_gps.sh udp
# ~/catkin_ws/run/start_phone_gps.sh serial

# 4. 启动 GPS 导航，默认 dry_run:=true
~/catkin_ws/run/start_gps_nav.sh

# 5. 启动独立碰撞安全层
~/catkin_ws/run/start_collision_safety.sh
```

`start_livox.sh` 不启动 FAST-LIO、GPS fusion 或导航。
`start_localization.sh` 不重复启动 Livox 或 GPS fusion。脚本会检查已有节点，
避免重复启动造成 TF 和 frame 冲突。

`start_collision_safety.sh` 会等待
`/fast_lio_localization_sc_qn_node` 发布 `/livox/lidar_filter1`，停止旧的
`/lidar_obstacle_avoid`，并确认 `/open_nav/cmd_vel` 没有其他 publisher 后才启动。

首次实机测试必须保留 `dry_run:=true`。完成点云、TF、状态和速度检查后，才应按
现场流程解除 dry-run。

运行检查：

```bash
~/catkin_ws/run/check_runtime.sh
```

## GPS 输入模式

```bash
# 手机 NetGPS TCP，默认模式
~/catkin_ws/run/start_phone_gps.sh tcp

# 旧 UDP 模式
~/catkin_ws/run/start_phone_gps.sh udp

# 南方 GNSS USB 串口，默认 115200 8N1
~/catkin_ws/run/start_phone_gps.sh serial
```

串口路径和波特率可覆盖：

```bash
GNSS_SERIAL_DEVICE=/dev/ttyUSB0 \
GNSS_SERIAL_BAUDRATE=115200 \
~/catkin_ws/run/start_phone_gps.sh serial
```

## Mapviz 与路径含义

导航调试视图：

```bash
~/catkin_ws/run/start_mapviz_navigation_test.sh
```

颜色定义：

| 颜色 | Topic | 含义 |
|---|---|---|
| 绿色 | `/gps/fix` | GNSS 原始轨迹 |
| 黄色 | `/open_nav/path` | 机器人已经走过的实际轨迹 |
| 蓝色 | `/open_nav/mission_path` | 已确认的任务航点连线 |
| 红色 | `/Odometry` | 当前高频机器人姿态，历史缓冲为 1 |

当前 `waypoint_tracker` 使用 Pure Pursuit 根据航点和前视点直接生成速度，不是
move_base/TEB 局部规划器。因此系统不会发布一条实时绕障规划曲线。蓝色
`/open_nav/mission_path` 是机器人将依次到达的目标航点连线，可作为当前架构下的
“计划路线”；黄色 `/open_nav/path` 用来对比实际行驶结果。

正式 `mapviz.mvc` 中还包含：

- 青色 `/open_nav/gps_filtered_pose`：GPS 转换/滤波位置
- 红色 `/open_nav/odom`：融合里程计历史箭头
- 浅蓝色 `/open_nav/mission_path`：任务路径
- 蓝色 TF `open_base`：机器人 TF 姿态

原红色箭头呈圆形拖尾主要是 `/open_nav/odom` 的显示缓冲为 200，并不表示 TF
只在低频更新。新的导航测试配置使用 `/Odometry` 且只保留当前一帧。

## RViz 调试

```bash
~/catkin_ws/run/start_navigation_debug_rviz.sh
```

配置包含 TF、RobotModel、原始 `/Odometry`、实际路径和 Livox PointCloud2。
点云自动按以下顺序选择：

1. `/livox/lidar_filter1`
2. `/cloud_registered`
3. `/livox/lidar`

只有类型为 `sensor_msgs/PointCloud2` 的 topic 才会交给 RViz PointCloud2 插件。

## 关键 Topic

| Topic | 类型 | 用途 |
|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS/GNSS 定位 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 原始里程计 |
| `/livox/lidar_filter1` | `sensor_msgs/PointCloud2` | localization 输出的安全层点云 |
| `/open_nav/odom` | `nav_msgs/Odometry` | GPS/FAST-LIO 融合位姿 |
| `/open_nav/mission_path` | `nav_msgs/Path` | 已确认任务航点连线 |
| `/open_nav/path` | `nav_msgs/Path` | 实际行驶历史轨迹 |
| `/open_nav/cmd_vel_raw` | `geometry_msgs/Twist` | waypoint tracker 原始速度 |
| `/open_nav/cmd_vel` | `geometry_msgs/Twist` | 安全仲裁后速度 |
| `/open_nav/obstacle_status` | `ObstacleStatus` | FREE/SLOW/STOP 或旧避障状态 |
| `/open_nav/obstacle_markers` | `visualization_msgs/MarkerArray` | 障碍调试显示 |

## 避障速度调试

旧 `lidar_obstacle_avoid` 的速度参数位于：

```text
catkin_ws/src/tron_open_space_nav/config/obstacle_speed.yaml
```

```yaml
slow_distance: 2.0
slowdown_factor: 0.35
stop_distance: 0.8
obstacle_speed_scale: 1.0
```

`obstacle_speed_scale: 1.0` 保持原行为。该参数只缩放障碍影响状态下的距离相关
速度，输出仍受原始命令和 `max_linear_vel` 限制，STOP 逻辑不变。

新部署优先使用 `tron_local_collision_safety`。不要让它和
`lidar_obstacle_avoid` 同时发布 `/open_nav/cmd_vel`。

## 常用诊断

```bash
rostopic hz /gps/fix
rostopic hz /livox/lidar_filter1
rostopic hz /Odometry
rostopic hz /open_nav/odom
rostopic hz /tf

rostopic info /open_nav/cmd_vel
rostopic echo /open_nav/obstacle_status
rosrun tf tf_echo map open_base
rosrun tf tf_echo open_base livox_frame
```

`/open_nav/cmd_vel` 在新安全层运行时应只有 `/collision_safety` 一个 publisher。

更详细的显示说明见
`catkin_ws/src/tron_open_space_nav/docs/NAV_VISUALIZATION_DEBUG.md`，运行环境说明见
`catkin_ws/runtime_scripts/README.md`。
