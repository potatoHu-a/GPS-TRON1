# 故障排查

## RViz：could not connect to display

**原因**：在无 DISPLAY 的机器人 SSH 上启动了 RViz。

**解决**：使用默认 `use_rviz:=false`，在开发电脑单独启动 RViz。见 [RUN_GUIDE.md](RUN_GUIDE.md)。

---

## /open_nav/odom 无输出

1. 检查 GPS 原点：

```bash
rosparam get /tron_open_space_nav/origin_ready   # 应为 true
rostopic hz /gps/fix
```

2. 检查 FAST-LIO：

```bash
rostopic hz /Odometry
```

3. 融合需 **GPS 原点就绪** 且 **收到 Odometry** 后才开始发布。

---

## 多点导航不切换航点

```bash
rostopic echo /open_nav/mission_status
rosparam get /waypoint_tracker/goal_tolerance
```

- 确认 `waypoint_source=multi_goal`
- 增大 `goal_tolerance`（默认 1.0 m）
- 确认 `/open_nav/waypoints` 已发布：

```bash
rostopic echo /open_nav/waypoints -n 1
```

---

## 点击 RViz 无反应

| 模式 | RViz Goal 话题 |
|------|----------------|
| multi_goal | `/open_nav/goal_add` + `rosservice call .../send_mission` |
| single_goal | `/move_base_simple/goal` |

检查 Fixed Frame = `map`。

---

## TF 冲突 / TF_REPEATED_DATA

开放导航使用 `open_base`，官方使用 `body1`，不应冲突。

```bash
rosrun tf view_frames
rosrun tf tf_echo map open_base
```

---

## WebSocket 连接失败

编辑 `config/robot.yaml`：

```yaml
ws_url: "ws://<ROBOT_WLAN_IP>:5000"
accid: "<YOUR_ACCID>"
```

dry_run 模式下不会连接 WebSocket，可忽略。

---

## check_system.sh 报错

```bash
echo $ROS_MASTER_URI
rosnode list
rosrun tron_open_space_nav check_system.sh
```

确保在 **机器人 roscore 所在网络** 执行，且导航栈已启动。

---

## 编译 MissionStatus 找不到

```bash
cd ~/catkin_ws
catkin_make_isolated
source ~/catkin_ws/devel_isolated/setup.bash
rosmsg show tron_open_space_nav/MissionStatus
```
