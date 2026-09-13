# 运行指南

## 架构：机器人 + 电脑分离

| 设备 | 运行内容 | DISPLAY |
|------|----------|---------|
| TRON1 开发板 | roscore、GPS、FAST-LIO、gps_global_nav | 无 |
| 开发电脑 | RViz | 有 |

---

## 一、机器人端

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=<ROBOT_IP>
```

### 1. GPS

```bash
roslaunch phone_gps_bridge phone_gps_test.launch
```

### 2. 全球导航（FAST-LIO 需已运行）

```bash
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true
```

默认 **不启动 RViz**（`use_rviz:=false`）。

### 3. 系统检查

```bash
rosrun tron_open_space_nav check_system.sh
```

---

## 二、开发电脑端

```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<DEV_PC_IP>
source ~/catkin_ws/devel_isolated/setup.bash

rviz -d $(rospack find tron_open_space_nav)/config/rviz/gps_global_nav.rviz
```

RViz 设置：

- **Fixed Frame** = `map`
- 默认 **2D Nav Goal** 发布到 `/open_nav/goal_add`（多点累积模式）

---

## 三、单点导航

```bash
# 机器人端
roslaunch tron_open_space_nav gps_global_nav.launch \
  waypoint_source:=single_goal dry_run:=true
```

RViz 中将 **2D Nav Goal** 话题改为 `/move_base_simple/goal`，点击一次即可导航到该点。

---

## 四、多点顺序导航（P1→P2→P3→P4）

默认 `waypoint_source:=multi_goal`。

### 方式 A：RViz 累积 + Send Mission（无需插件）

1. RViz **2D Nav Goal** → `/open_nav/goal_add`（默认已配置）
2. 依次点击 P1、P2、P3、P4
3. 发送任务：

```bash
rosservice call /gps_multi_waypoint_collector/send_mission "{}"
```

4. 机器人自动依次导航，观察：

```bash
rostopic echo /open_nav/mission_status
```

### 方式 B：Multi Goals RViz 插件

若已编译 `navi_multi_goals_pub_rviz_plugin`：

1. RViz 添加 Panel → MultiNaviGoalsPanel
2. 用 2D Nav Goal 添加多个点
3. 点击 **Start** → 发布 `/multi_navi_goals/mission`
4. `gps_multi_waypoint_collector` 自动处理

### 方式 C：manual_yaml

编辑 `config/gps_waypoints.yaml` 后：

```bash
roslaunch tron_open_space_nav gps_global_nav.launch waypoint_source:=manual_yaml
```

---

## 五、验证命令

```bash
rostopic echo /global_pose -n 1
rostopic echo /open_nav/odom -n 1
rostopic echo /open_nav/mission_status -n 1
rostopic echo /open_nav/cmd_vel
rosrun tf tf_echo earth map
rosrun tf tf_echo map open_base
```

---

## 六、真实运动

确认 dry_run 测试通过后：

```bash
roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false use_fake_fastlio:=false dry_run:=false
```

机器人需先完成预备（P）和行走模式（G），参见 TRON1-keyboard 文档。
