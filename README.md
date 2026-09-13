# GPS-TRON1 机器人端导航工程

这是 TRON1 机器人当前使用的 ROS1 Noetic 导航工作区源码备份，包含手机 GPS 接入、GPS/FAST-LIO 融合、航点跟踪、LiDAR 避障、TRON 控制桥接以及 Mapviz 地面站配置。

仓库目标是让另一台 TRON1 机器人可以更容易复现当前部署。

## 目录结构

```text
catkin_ws/
  src/
    phone_gps_bridge/        # 手机 NetGPS/NMEA -> ROS GPS topic
    tron_open_nav/           # 早期 GPS open navigation 包
    tron_open_space_nav/     # 当前 GPS 全局导航、融合、航点、避障、Mapviz
```

## 已包含的主要功能

- `phone_gps_bridge`
  - NetGPS TCP 接收节点：`netgps_tcp_receiver.py`
  - 保留旧 UDP 接收节点：`gps_udp_receiver.py`
  - 发布 `/gps/fix`
  - 发布 `/gps/heading`、`/gps/magnetic_heading`、`/gps/course`

- `tron_open_space_nav`
  - GPS WGS84 到 true-north ENU 转换
  - FAST-LIO odometry 与 GPS 融合
  - 多航点导航
  - `/open_nav/cmd_vel_raw -> /open_nav/cmd_vel` LiDAR 避障层
  - TRON1 WebSocket 控制桥接
  - Mapviz 地面站与武汉瓦片地图

## 依赖环境

- Ubuntu 20.04
- ROS1 Noetic
- catkin isolated build
- TRON1 机器人本体 SDK/驱动环境
- Livox MID360 驱动
- FAST-LIO localization 工程
- Mapviz 与相关插件

常见 ROS 依赖可按实际系统补装：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-mapviz \
  ros-noetic-mapviz-plugins \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-visualization-msgs
```

## 部署到新机器人

在目标机器人上：

```bash
cd ~
git clone https://github.com/potatoHu-a/GPS-TRON1.git
cd ~/GPS-TRON1/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated
source devel_isolated/setup.bash
```

如果需要放回标准路径：

```bash
mkdir -p ~/catkin_ws/src
rsync -a ~/GPS-TRON1/catkin_ws/src/ ~/catkin_ws/src/
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated
source devel_isolated/setup.bash
```

## 启动顺序

以下命令按当前现场网络写法整理。远程登录目标为：

```bash
ssh guest@10.192.1.3
```

密码请按现场机器人账户配置输入，不建议写入公开仓库。

### 1. 开启 Livox MID360 雷达

在机器人端 SSH 终端 1：

```bash
ssh guest@10.192.1.3
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 2. 开启 FAST-LIO 重定位

在机器人端 SSH 终端 2：

```bash
ssh guest@10.192.1.3
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch fast_lio_localization_sc_qn run.launch lidar:=livox_mid360
```

### 3. 开启手机 GPS

手机端打开 NetGPS，当前默认配置：

- 手机 IP：`172.18.125.125`
- TCP Port：`10110`
- Sentence Output Interval：建议 `1000 ms`

机器人端启动 GPS bridge：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch phone_gps_bridge netgps.launch
```

可检查：

```bash
rostopic echo /gps/fix
rostopic echo /gps/heading
rostopic echo /gps/magnetic_heading
rostopic echo /gps/course
```

### 4. 开启 GPS 全局导航

首次部署和调试必须使用 `dry_run:=true`，确认 `/open_nav/cmd_vel` 正常后再考虑实机控制。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true
```

### 5. 本地电脑开启 Mapviz/RViz 地面站

在本地电脑或可访问 ROS master 的终端：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch tron_open_space_nav mapviz_ground_station.launch \
  tile_root:=/root/catkin_ws/src/tron_open_space_nav/maps/wuhan_tiles \
  launch_mapviz:=true
```

如果源码部署在普通用户目录，可把 `tile_root` 改成：

```bash
tile_root:=~/catkin_ws/src/tron_open_space_nav/maps/wuhan_tiles
```

## 关键话题

```text
/gps/fix                         sensor_msgs/NavSatFix
/gps/heading                     std_msgs/Float64, true north heading
/gps/magnetic_heading            std_msgs/Float64, magnetic heading diagnostic
/gps/course                      std_msgs/Float64, RMC/VTG course when available

/open_nav/cmd_vel_raw            waypoint_tracker 原始速度
/open_nav/cmd_vel                避障层输出速度
/open_nav/obstacle_status        tron_open_space_nav/ObstacleStatus
/open_nav/obstacle_markers       visualization_msgs/MarkerArray
```

## LiDAR 避障说明

当前避障层不引入 `move_base`、DWA、TEB、Nav2 或全局 costmap。

处理链路：

```text
waypoint_tracker
  -> /open_nav/cmd_vel_raw
  -> lidar_obstacle_avoid
  -> /open_nav/cmd_vel
  -> tron_controller_bridge
```

避障逻辑：

- 点云 TF 到 `base_frame`，默认 `open_base`
- 在 `open_base` 中按 `+X` 前、`+Y` 左、`+Z` 上处理
- ROI 与高度过滤
- `ground_z_max` 过滤地面点
- footprint corridor：`robot_width / 2 + safety_margin`
- 左/中/右三区统计 `min_distance`、`point_count`、`occupancy`
- 状态机：`CLEAR`、`SLOW`、`AVOID_LEFT`、`AVOID_RIGHT`、`BLOCKED`、`SENSOR_TIMEOUT`
- 点云超时进入 fail-safe 停车

配置文件：

```text
catkin_ws/src/tron_open_space_nav/config/obstacle_avoid.yaml
```

## NetGPS TCP 说明

`phone_gps_bridge` 使用 TCP client 连接 NetGPS TCP server。

默认配置：

```text
catkin_ws/src/phone_gps_bridge/config/netgps.yaml
```

特性：

- 自动连接 `172.18.125.125:10110`
- TCP 断开后自动重连
- 接收超时不视为断线
- TCP stream buffer 按完整 NMEA sentence 分句
- 校验 NMEA checksum
- GGA/RMC 发布 `/gps/fix`
- HDT 发布 true north `/gps/heading`
- HDG 发布诊断 `/gps/magnetic_heading`
- RMC/VTG course 有效时发布 `/gps/course`

## 常用验证命令

```bash
rostopic hz /gps/fix
rostopic hz /gps/heading
rostopic echo /open_nav/obstacle_status
rostopic echo /open_nav/cmd_vel
rostopic echo /Odometry -n 1
rostopic echo /livox/lidar/header -n 1
rosrun tf tf_echo open_base livox_frame
```

## 安全提醒

- 初次部署保持 `dry_run:=true`
- 不要在未确认 GPS、FAST-LIO、TF、雷达和避障状态前启动实机运动
- 公开仓库不要提交机器人 SSH 密码、token、WiFi 密码或现场账号密钥

