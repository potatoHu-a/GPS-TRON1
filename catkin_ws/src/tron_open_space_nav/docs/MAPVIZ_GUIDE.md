# Mapviz 地面站指南

Mapviz 在**开发电脑 Docker** 运行，连接**机器人 ROS Master**。  
**不修改**机器人导航 TF 树：`earth → map → open_odom → open_base`。

---

## 设计说明

| 组件 | 作用 |
|------|------|
| `swri_transform_util/initialize_origin.py` | 官方 WGS84 初始化：latched `/local_xy_origin` + `map→map__identity` TF |
| `open_nav_path_publisher.py` | `/open_nav/odom` → `/open_nav/path`（`map` 帧轨迹） |
| `start_mapviz_when_ready.py` | 等待 origin/GPS/TF 就绪后再启动 Mapviz |
| `tile_http_server.py` | 可选 HTTP 瓦片服务（`file://` 已验证时可不依赖） |

**不要**发布 `wgs84→map` 或 `gps_local→map`。  
**不要**使用 `/gps/path`（`gps_local` 帧，与导航 TF 不连通）。

---

## initialize_origin.py（ROS Noetic 2.15.x 实际接口）

脚本路径：`/opt/ros/noetic/lib/swri_transform_util/initialize_origin.py`  
（**与 Kinetic/Indigo 旧版不同**，无 `--help`，不使用 `gps` remap）

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `~local_xy_frame` | string | `map` | LocalXY 对应的 ROS 帧 |
| `~local_xy_origin` | string | `auto` | `auto` 或 `local_xy_origins` 中的 name |
| `~local_xy_origins` | list | — | 命名原点：`name/latitude/longitude/altitude/heading` |
| `~local_xy_navsatfix_topic` | string | `fix` | **NavSatFix** 输入（本系统设为 `/gps/fix`） |
| `~local_xy_gpsfix_topic` | string | `gps` | **gps_common/GPSFix** 输入（可选） |
| `~local_xy_custom_topic` | string | `None` | 自定义消息（可选） |

| 话题 | 方向 | 说明 |
|------|------|------|
| `/gps/fix`（经 `local_xy_navsatfix_topic`） | 订阅 | `auto` 模式下取首帧有效 NavSatFix |
| `/local_xy_origin` | 发布 latched | `PoseStamped`：x=lon, y=lat, z=alt |
| `/diagnostics` | 发布 | LocalXY Origin 状态 |

`OriginManager.start()` 持续广播 **`map → map__identity`** 单位 TF，供 `Wgs84Transformer` 初始化（**不是** `wgs84→map`）。

### 与机器人已锁定 origin 对齐

若机器人已运行 `gps_global_converter` 并锁定 origin，请从参数服务器读取后传入 launch：

```bash
rosparam get /tron_open_space_nav/origin_lat
rosparam get /tron_open_space_nav/origin_lon
rosparam get /tron_open_space_nav/origin_alt

roslaunch tron_open_space_nav mapviz_ground_station.launch \
  local_xy_origin:=robot \
  origin_lat:=... origin_lon:=... origin_alt:=... \
  tile_root:=/path/to/tiles launch_mapviz:=true
```

---

## 安装（开发电脑）

```bash
sudo apt install -y \
  ros-noetic-mapviz \
  ros-noetic-mapviz-plugins \
  ros-noetic-tile-map \
  ros-noetic-swri-transform-util
```

---

## 启动

### 机器人端（导航，无 Mapviz）

```bash
roslaunch phone_gps_bridge phone_gps_test.launch
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false use_fake_fastlio:=false dry_run:=true
```

### 开发电脑 Docker（Mapviz 地面站）

```bash
export ROS_MASTER_URI=http://10.192.1.3:11311
export ROS_IP=<DEV_PC_IP>
source ~/catkin_ws/devel_isolated/setup.bash

roslaunch tron_open_space_nav mapviz_ground_station.launch \
  tile_root:=/root/catkin_ws/src/tron_open_space_nav/maps/wuhan_tiles \
  launch_mapviz:=true
```

仅启动：`mapviz_initialize_origin`、`tile_http_server`、`open_nav_path_publisher`、可选 `mapviz_launcher`（就绪后 exec mapviz）。

`launch_mapviz:=true` 时由 `start_mapviz_when_ready.py` 等待 `/local_xy_origin`、`/gps/fix` 和 `map→open_base` TF 全部就绪，延迟 1 秒后启动 Mapviz，避免首次灰屏。

`use_origin_fallback:=true` 时改用 `mapviz_tf_broadcaster.py`（默认 **不启用**）。

---

## 验证命令

```bash
rostopic echo /local_xy_origin -n 1
rostopic echo /open_nav/path -n 1
rostopic echo /open_nav/odom -n 1
rostopic echo /gps/fix -n 1
```

Mapviz 预期：

- Tile Map Status = **OK**
- NavSat 有数据
- Path 订阅 `/open_nav/path`，无 `gps_local → map` 错误
- Odometry 正常
- 日志出现 `Wgs84Transformer initialized`，不再出现 `No transform between /wgs84 and map`

**不要**用 `rosrun tf tf_echo wgs84 map` 验证。

---

## mapviz.mvc 插件

| 插件 | Topic |
|------|-------|
| Tile Map | `file://.../wuhan_tiles/{level}/{x}/{y}.png`, max_zoom=18 |
| NavSat | `/gps/fix` |
| Path | `/open_nav/path` |
| Odometry | `/open_nav/odom` |
| Marker | `/open_nav/waypoint_markers` |
| TF Frame | `open_base` |

Fixed Frame: `map` · Target Frame: `open_base`（**不要用** legacy `open_map`）

若 `~/.mapviz_config` 残留旧 Target Frame `open_map`，请删除或改用 `-d config/rviz/mapviz.mvc` 启动。

---

## 故障排查

| 现象 | 处理 |
|------|------|
| `Wgs84Transformer not initialized` | 确认 `swri_transform_util` 已安装；`initialize_origin` 在运行 |
| `No transform wgs84/map` | 检查 `/local_xy_origin` 与 `map→map__identity` TF |
| Path `gps_local→map` | 确认 Path 插件订阅 `/open_nav/path`，不是 `/gps/path` |
| Tile Map 空白 | 检查 `tile_root`、8088 端口 |
