# GPS 卫星地图显示说明

## 容器内现状

默认 **GPS-TRON1** Docker 镜像 **未安装 mapviz**，也 **未安装 pyproj**。

### 容器内可选 Python 依赖

```bash
pip3 install pyproj
```

安装后可在 `gps_global.yaml` 中设置 `use_utm: true` 使用 UTM 坐标（默认 ENU 已满足室外短距离导航）。

---

## 方案 A：RViz（容器内默认可用，推荐）

RViz 在开发电脑手动启动（机器人默认 `use_rviz:=false`），Fixed Frame = **`map`**（真北 ENU）。

| 操作 | 说明 |
|------|------|
| Fixed Frame | 设为 `map` |
| 2D Nav Goal | 工具栏 **2D Nav Goal**，在 Grid 上连续点击 P1→P2→P3→P4 |
| 显示 | `/global_pose` 当前 GPS 位置；`/open_nav/waypoint_markers` 航点标记 |

点击后自动：

1. 写入 `config/gps_waypoints.yaml`（lat/lon）
2. 发布 `/open_nav/waypoints` 供 Pure Pursuit 跟踪

---

## 方案 B：Mapviz 卫星底图（需在宿主机安装）

Mapviz **不支持在容器内自动下载中国区域卫星瓦片**，需用户自行准备瓦片服务或离线包。

### 宿主机 Ubuntu 22.04 安装

```bash
sudo apt update
sudo apt install -y ros-noetic-mapviz ros-noetic-mapviz-plugins ros-noetic-tile-map
```

### 中国卫星影像来源（需自行合规使用）

| 来源 | 说明 |
|------|------|
| 天地图 | 需 API Key，WMTS/WMS 转 XYZ 瓦片 |
| 高德卫星 | 无官方 ROS 接口，常用做法：下载 XYZ 瓦片到本地 HTTP 服务 |
| OpenStreetMap | 非卫星，但可快速验证 mapviz 流程 |

### 本地瓦片 + Mapviz 地面站

瓦片目录结构：`tiles/{z}/{x}/{y}.png`（max zoom 18）。

开发电脑 Docker 连接机器人 ROS Master，**不修改机器人 TF 树**：

```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<DEV_PC_IP>
source ~/catkin_ws/devel_isolated/setup.bash

roslaunch tron_open_space_nav mapviz_ground_station.launch \
  tile_root:=/path/to/tiles \
  launch_mapviz:=true
```

由官方 `swri_transform_util/initialize_origin.py` 初始化 WGS84（latched `/local_xy_origin` + `map→map__identity` TF）。  
轨迹显示用 `/open_nav/path`（`map` 帧），**不用** `/gps/path`。

验证：

```bash
rostopic echo /local_xy_origin -n 1
rostopic echo /open_nav/path -n 1
rostopic echo /open_nav/odom -n 1
rostopic echo /gps/fix -n 1
```

详见 [MAPVIZ_GUIDE.md](MAPVIZ_GUIDE.md)。

---

## 方案 C：multi_navi_goals RViz 插件（可选）

仓库中若存在 `rviz_navi_multi_goals_pub_plugin`（桌面 catkin_ws 已有），可编译进工作空间：

```bash
cd ~/catkin_ws/src
# 解压 rviz_navi_multi_goals_pub_plugin-master.zip
cd ~/catkin_ws && catkin_make_isolated
```

RViz 中添加 Panel：**MultiNaviGoalsPanel**

- 用 **2D Nav Goal** 添加多个点
- 点击 **Start** 发布 `/multi_navi_goals/mission`
- `gps_waypoint_collector` 已订阅该话题，会自动转换并保存 GPS 航点

---

## 无法自动下载时的建议流程

1. 在 RViz `map` 坐标系下用 Grid + GPS 轨迹验证相对位置
2. 现场采集 GPS 航点（手机 + RViz 点击）
3. 需要卫星图时，在宿主机用 mapviz + 本地瓦片
4. 将 `gps_waypoints.yaml` 复制到机器人部署
