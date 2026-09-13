# tron_open_space_nav

LimX TRON1 室外 GPS 开放空间导航系统（ROS Noetic）。

基于手机 GPS + FAST-LIO 里程计 + Pure Pursuit 航点跟踪 + 激光避障，独立于官方 `move_base` 栈运行。

## 功能

- 手机 GPS（UDP）→ `/gps/fix`
- WGS84 → 真北 ENU 全球坐标（`earth` → `map`）
- GPS + FAST-LIO 融合 → `/open_nav/odom`
- **单点导航**：RViz `2D Nav Goal` → `/move_base_simple/goal`
- **多点顺序导航**：P1 → P2 → P3 → P4 自动依次到达
- LiDAR 避障 → `/open_nav/cmd_vel`
- TRON1 WebSocket 控制桥接

## 系统架构

```
Phone GPS → gps_udp_receiver → /gps/fix
                ↓
         gps_global_converter → earth→map, /global_pose
                ↓
         gps_fastlio_fusion + /Odometry → /open_nav/odom
                ↓
         waypoint_tracker → /open_nav/cmd_vel_raw
                ↓
         lidar_obstacle_avoid → /open_nav/cmd_vel
                ↓
         tron_controller_bridge → TRON1
```

详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 硬件需求

| 组件 | 说明 |
|------|------|
| LimX TRON1 | 轮足机器人 + 感知拓展套件 |
| Livox MID360 | `/livox/lidar`、`/Odometry`（FAST-LIO） |
| Android 手机 | NMEA UDP → 端口 10110 |
| 开发电脑 | 有显示器，运行 RViz（可选） |

## 软件环境

- Ubuntu 20.04（机器人）/ 20.04 或 22.04（开发机）
- ROS Noetic
- Python 3.8+
- 依赖包：`phone_gps_bridge`（同工作空间）

## 快速安装

```bash
# 1. 安装 ROS Noetic（见 docs/INSTALL.md）
# 2. 克隆工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <your-repo>/phone_gps_bridge.git
# tron_open_space_nav 位于 phone_gps_bridge/catkin_ws/src/ 或单独 clone

# 3. 编译
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated
source ~/catkin_ws/devel_isolated/setup.bash

# 4. 编辑机器人配置
cp $(rospack find tron_open_space_nav)/config/robot.example.yaml \
   $(rospack find tron_open_space_nav)/config/robot.yaml
# 修改 ws_url 和 accid
```

完整步骤：[docs/INSTALL.md](docs/INSTALL.md)

## 启动流程

### 机器人（无 RViz）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash

# GPS
roslaunch phone_gps_bridge phone_gps_test.launch

# 全球导航（另开终端，需 FAST-LIO 已运行）
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false use_fake_fastlio:=false dry_run:=true
```

### 开发电脑（RViz）

```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<DEV_PC_IP>
source ~/catkin_ws/devel_isolated/setup.bash
rviz -d $(rospack find tron_open_space_nav)/config/rviz/gps_global_nav.rviz
```

完整说明：[docs/RUN_GUIDE.md](docs/RUN_GUIDE.md)

## 多点导航测试

1. RViz Fixed Frame = `map`
2. 用 **2D Nav Goal** 依次点击 P1、P2、P3（收集模式见 RUN_GUIDE）
3. 发布 `/multi_navi_goals/mission` 或使用 Multi Goals 插件 **Start**
4. 观察 `/open_nav/mission_status`：`current_index` 递增
5. 机器人依次到达各点

## 部署检查

```bash
rosrun tron_open_space_nav check_system.sh
```

## 文档索引

| 文档 | 内容 |
|------|------|
| [docs/INSTALL.md](docs/INSTALL.md) | 新机器部署 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 节点与数据流 |
| [docs/RUN_GUIDE.md](docs/RUN_GUIDE.md) | 运行与 RViz 分离 |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | 故障排查 |
| [docs/MAPVIZ_GUIDE.md](docs/MAPVIZ_GUIDE.md) | Mapviz 卫星地图可视化 |
| [docs/GPS_SATELLITE_MAP.md](docs/GPS_SATELLITE_MAP.md) | 卫星瓦片补充说明 |

## 兼容模式

旧版本地导航（`open_map` frame）仍可用：

```bash
roslaunch tron_open_space_nav open_nav.launch
```
