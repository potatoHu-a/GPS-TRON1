# GPS 全球导航启动说明

> **已合并至 [RUN_GUIDE.md](RUN_GUIDE.md)**，本文保留作快速参考。

# GPS 全球导航启动说明（快速参考）

机器人无显示器（SSH 环境）时，导航节点在机器人上运行，RViz 在开发电脑上手动启动。

---

## 机器人端（Ubuntu 20.04，无 DISPLAY）

### 前置

1. 手机 GPS UDP → `/gps/fix`（`phone_gps_bridge`）
2. FAST-LIO 已发布 `/Odometry`
3. Livox 已发布 `/livox/lidar`（避障需要）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
```

### 启动 GPS 桥接（若未运行）

```bash
roslaunch phone_gps_bridge phone_gps_test.launch
```

### 启动全球导航（不启动 RViz）

```bash
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true
```

默认 `use_rviz:=false`，不会启动 `gps_global_rviz`，避免 `qt.qpa.xcb: could not connect to display`。

### 启动节点列表

| 节点 | 说明 |
|------|------|
| `gps_global_converter` | WGS84 → 真北 ENU `/global_pose`，TF `earth→map` |
| `gps_waypoint_collector` | RViz 航点收集（订阅远程 RViz 话题） |
| `gps_enu_converter` | `/gps/fix` → `/open_nav/pose` |
| `gps_fastlio_fusion` | GPS + FAST-LIO → `/open_nav/odom` |
| `waypoint_tracker` | Pure Pursuit → `/open_nav/cmd_vel_raw` |
| `lidar_obstacle_avoid` | 避障 → `/open_nav/cmd_vel` |
| `nav_mode_manager` | 模式管理（预留） |
| `tron_controller_bridge` | WebSocket 控制（dry_run 时不连） |

### 验证（机器人 SSH 终端）

```bash
rosnode list | grep -E 'rviz|gps_global'
rostopic echo /global_pose -n 1
rostopic echo /open_nav/cmd_vel -n 1
```

期望：`rosnode list` 中 **无** `gps_global_rviz` 或 `/rviz`。

---

## 电脑端（有显示器）

### 1. 连接机器人 ROS Master

```bash
export ROS_MASTER_URI=http://<机器人IP>:11311
export ROS_IP=<本机局域网IP>
# 示例：export ROS_MASTER_URI=http://10.192.1.3:11311
```

### 2. 加载工作空间（可选，用于 rospack find）

若本机也有相同 catkin 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
```

### 3. 手动启动 RViz

```bash
rviz -d $(rospack find tron_open_space_nav)/config/gps_global_nav.rviz
```

若本机无 rospack，使用绝对路径：

```bash
rviz -d /path/to/tron_open_space_nav/config/gps_global_nav.rviz
```

### 4. RViz 操作

1. Fixed Frame 设为 **`map`**
2. 工具栏选择 **2D Nav Goal**
3. 在 Grid 上连续点击 P1、P2、P3、P4
4. 航点写入机器人上 `config/gps_waypoints.yaml`，并发布 `/open_nav/waypoints`

### 5. 电脑端验证

```bash
rostopic echo /global_pose -n 1
rostopic echo /open_nav/waypoints -n 1
```

---

## 可选：在本机 launch 中启动 RViz

仅在**有 DISPLAY 的环境**（开发机、容器带 X11）：

```bash
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true \
  use_rviz:=true
```

---

## dry_run 与真实运动

| 参数 | 效果 |
|------|------|
| `dry_run:=true` | 不连 WebSocket，机器人不动，仅看 `/open_nav/cmd_vel` |
| `dry_run:=false` | 连接 TRON WebSocket，机器人执行导航（需先 P→G 预备/行走） |
