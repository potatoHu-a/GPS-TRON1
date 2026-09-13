# Mapviz 多点巡航指南

在 Mapviz 卫星地图上点击设置 P1→P2→P3，复用已验证的 `gps_multi_waypoint_collector` + `waypoint_tracker`。

## Mapviz 点击方法

1. 启动 Mapviz 地面站（含 `point_click_publisher` 插件）
2. 在卫星地图上依次点击航点
3. 每次点击发布 `/mapviz/waypoint_click`（`PointStamped`, `frame=map`）
4. `mapviz_click_to_goal` 转为 `/open_nav/goal_add`（`PoseStamped`）
5. 点击过程中 Mapviz 显示 pending 航点 P1/P2/P3 及连线

## 一次性导航

```bash
# 发送当前点击的 mission（不立即运动，state=READY）
rosservice call /open_nav/mission/send "{}"

# 开始顺序巡航 P1->P2->P3->END
rosservice call /open_nav/mission/start_patrol "{patrol_mode: 'once', loop_count: 0}"
```

兼容 RViz：直接 `rosservice call /gps_multi_waypoint_collector/send_mission "{}"` 仍会立即开始导航。

## Loop 巡航

```bash
rosservice call /open_nav/mission/send "{}"
rosservice call /open_nav/mission/start_patrol "{patrol_mode: 'loop', loop_count: 0}"
```

`loop_count=0` 无限循环；`loop_count=N` 循环 N 次。

## Ping-pong 巡航

```bash
rosservice call /open_nav/mission/start_patrol "{patrol_mode: 'pingpong', loop_count: 0}"
```

顺序：P1→P2→P3→P2→P1→…

## pause / resume / stop / clear

```bash
rosservice call /open_nav/mission/pause "{}"
rosservice call /open_nav/mission/resume "{}"
rosservice call /open_nav/mission/stop "{}"
rosservice call /open_nav/mission/clear "{}"
```

pause/stop/clear 会立即发布零速度。

## 保存 / 加载

```bash
rosservice call /open_nav/mission/save "{}"
rosservice call /open_nav/mission/load "{}"
```

默认文件：`config/patrol_waypoints.yaml`（含 map x/y + latitude/longitude）。

## dry_run 测试

机器人端 `dry_run:=true` 时导航逻辑与 `/open_nav/cmd_vel_raw` 正常，机器人不实际运动。

## 实机前检查

- [ ] `/gps/fix` 有效
- [ ] `/open_nav/odom` frame=map
- [ ] TF `map→open_base` 连通
- [ ] `/local_xy_origin` 已发布
- [ ] `dry_run:=false` 前确认航点与障碍
- [ ] loop 模式确认 `loop_count` 与场地安全
