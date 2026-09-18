# GPS-TRON 实机联调流程

本文档用于第一次机器人联调。开始时必须保持 `dry_run:=true`，确认传感器、局部坐标系、速度仲裁和唯一发布者正确后，才能进入有人监护的低速测试。

## 坐标系约定

- 全局导航继续使用 `map -> open_odom -> open_base`。
- `/livox/lidar_filter1` 的 `frame_id` 应为 `body`。
- 局部碰撞检测、footprint、预测轨迹、Marker 和 `ObstacleStatus` 均使用 `body`。
- 点云已经是 `body` 时不会查询 TF；只有未来点云使用其他 frame 时，安全层才查询 `source_frame -> body`。
- 局部预测固定从 `body` 原点 `(x=0, y=0, yaw=0)` 开始，不使用 `open_base` 的全局坐标。

## 环境和安全前提

公共运行脚本会自动构造 ROS 和全部 isolated package 环境，不需要手工 source，也不使用 `rosrun` 作为最外层入口。默认从脚本所在目录推导工作区，也可通过 `TRON_WS=/path/to/catkin_ws` 覆盖。

`config/test_mode.yaml` 中的 `0.2 m/s` 和 `0.2 rad/s` 是第一次测试的验收上限。阶段 2.4.1 不修改导航速度，所以这些值不会自动钳制 `waypoint_tracker`。如果 topic 实测超限，禁止关闭 `dry_run`。

清空机器人周围区域，准备急停，并安排一人观察机器人、一人操作终端。

## 独立启动命令

按以下顺序在独立终端启动。

### 1. 官方 Livox 驱动

```bash
~/catkin_ws/run/start_livox.sh
```

该脚本只启动 Livox 官方驱动，不启动 FAST-LIO、GPS fusion 或导航。确认点云及 frame：

```bash
rostopic hz /livox/lidar_filter1
rostopic echo -n 1 /livox/lidar_filter1/header
```

期望类型为 `sensor_msgs/PointCloud2` 且 `frame_id: body`。

### 2. FAST-LIO localization

```bash
~/catkin_ws/run/start_localization.sh
```

该脚本只启动官方 FAST-LIO localization。它不会启动 Livox driver 或 GPS fusion；后者仍由 GPS 导航 launch 负责。

`/fast_lio_localization_sc_qn_node` 不仅提供 localization TF，也负责发布安全层唯一使用的 `/livox/lidar_filter1`。因此必须先成功启动 localization，再启动 collision safety，不能跳过或颠倒这两个步骤。

确认：

```bash
rostopic hz /Odometry
rostopic hz /livox/lidar_filter1
```

### 3. 手机 GPS

默认使用已验证的 NetGPS TCP 配置，服务器地址从现有 `netgps.yaml` 读取：

```bash
~/catkin_ws/run/start_phone_gps.sh tcp
```

需要 UDP 时：

```bash
~/catkin_ws/run/start_phone_gps.sh udp
```

确认：

```bash
rostopic hz /gps/fix
```

### 4. GPS 导航

```bash
~/catkin_ws/run/start_gps_nav.sh
```

脚本固定使用：

```text
use_fake_gps:=false
use_fake_fastlio:=false
dry_run:=true
```

它不会自动关闭 dry-run。确认上层命令：

```bash
rostopic echo /open_nav/cmd_vel_raw
```

### 5. 新碰撞安全层

```bash
~/catkin_ws/run/start_collision_safety.sh
```

该脚本会自动调用 `stop_legacy_obstacle.sh`，停止 `/lidar_obstacle_avoid`，检查 `/open_nav/cmd_vel` 没有未知发布者，然后使用 `test_mode.yaml` 启动新安全层。

启动脚本会先等待 `/fast_lio_localization_sc_qn_node` 成为 `/livox/lidar_filter1` 的 publisher。依赖尚未就绪时会持续显示：

```text
waiting for fast_lio_localization_sc_qn_node to provide /livox/lidar_filter1
```

此时不会启动安全层，避免点云缺失导致误判。可按 `Ctrl+C` 取消等待并检查 localization。

确认：

```bash
rostopic info /open_nav/cmd_vel
rostopic echo /open_nav/obstacle_status
```

`/open_nav/cmd_vel` 的 publisher 必须只有 `/collision_safety`。

### 6. Mapviz

在带图形界面的工作站或 GUI 容器运行：

```bash
~/catkin_ws/run/start_mapviz.sh
```

脚本保留已有 `ROS_IP`；未设置时，从到机器人 ROS Master `10.192.1.3` 的路由自动获取本机 IPv4。瓦片目录通过 `rospack find tron_open_space_nav` 动态定位。

`start_collision_safety.sh` 会自动处理 `/lidar_obstacle_avoid`，不会停止 `waypoint_tracker`、`patrol_mission_manager` 或 `tron_controller_bridge`，无需单独执行环境或清理命令。

## 完整运行检查

```bash
~/catkin_ws/run/check_runtime.sh
```

它会检查 ROS 网络、关键节点、topic 类型/频率/publisher、`map -> open_base`、`body -> livox_frame`，以及 `/open_nav/cmd_vel` 唯一发布者。点云已经在 `body` 时，`body -> livox_frame` 不是碰撞检测依赖，仅用于核对传感器 TF。

## 低速测试准入条件

保持 `dry_run:=true`，同时观察：

```bash
rostopic echo /open_nav/obstacle_status
rostopic echo /open_nav/cmd_vel_raw
rostopic echo /open_nav/cmd_vel
```

进入实机运动前必须满足：

- `/open_nav/cmd_vel` 只有 `/collision_safety` 发布。
- `/livox/lidar_filter1/header.frame_id` 为 `body`。
- `ObstacleStatus.header.frame_id` 为 `body`。
- 无障碍时状态稳定为 `FREE`，障碍接近时为 `SLOW` 或 `STOP`。
- `abs(linear.x) <= 0.2`，`abs(angular.z) <= 0.2`。
- 点云持续更新，`pointcloud_age < 0.5`。
- `STOP` 时所有输出速度为零。

如果 `/open_nav/cmd_vel_raw` 或 `/open_nav/cmd_vel` 出现 `0.5 m/s`，说明上游仍使用正常导航速度。`test_mode.yaml` 不会自动降为 `0.2 m/s`；在没有单独配置低速命令源前，不允许把 `dry_run` 改为 `false`。

第一次解除 dry-run 后只执行短距离直行，操作员必须保持急停可用。禁止高速、无人监护、狭窄空间和直接执行长航点任务。

## 必须停止测试的情况

- `/open_nav/cmd_vel` 存在多个 publisher。
- 点云 frame 为空，或非 `body` 且 `source_frame -> body` TF 不可用。
- 点云超时导致状态为 `STOP`。
- `cmd_output_linear` 在 `STOP` 状态不为零。
- 输出速度超过第一次测试上限。

结束时按相反顺序停止 Mapviz、安全层、导航、GPS 和传感器，并确认机器人控制端收到零速度。
