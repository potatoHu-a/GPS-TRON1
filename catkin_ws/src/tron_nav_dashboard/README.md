# TRON1 Navigation Dashboard

Laptop-only PyQt5 dashboard for observing GPS-TRON1 navigation. It is not
included by `tron_full_bringup.launch` and must not run on the robot Orin.

## Safety boundary

The dashboard and `navigation_debug_monitor.py` do not advertise
`/open_nav/cmd_vel` or `/open_nav/cmd_vel_raw`. The GUI never changes
`/tron_controller_bridge/dry_run`. Mission buttons call only services already
provided by the navigation stack; unavailable services remain disabled.
The only command-like topic published by the GUI is `/open_nav/goal_add`
(`geometry_msgs/PoseStamped`) for adding a map-frame waypoint to the existing
`gps_multi_waypoint_collector`.

## Start

```bash
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --pkg tron_nav_dashboard --force-cmake
/root/catkin_ws/runtime_scripts/start_nav_dashboard.sh
```

The script connects to `http://10.192.1.3:11311`, uses
`ROS_IP=10.192.1.194`, and defaults to `DISPLAY=:1`. It never starts a ROS
master.

The interface defaults to Simplified Chinese and automatically selects an
available CJK font. Set `language: en_US` in `config/dashboard.yaml` to use
English instead.

The laptop container needs a CJK font package once:

```bash
apt-get update
apt-get install -y fonts-noto-cjk
```

## LiDAR TF diagnostics

The right panel subscribes to `/livox/lidar_filter1` and transforms its
`PointCloud2` data to the configured `lidar_display_frame` for visualization
only. The default is `body`, because the robot's `/livox/lidar_filter1`
messages are already expressed in that local frame. When the input frame is
`body`, no TF lookup is performed and the local range is measured around
`(0, 0)`. Only a different input frame triggers TF conversion. The dashboard
does not change the global `map -> open_odom -> open_base` chain.
The panel reports:

- `SYNC`: exact cloud timestamp transform succeeded;
- `LATEST`: exact lookup failed and the latest visualization-only TF was used;
- `FAILED`: neither lookup was available and that frame was discarded.

It also shows point-cloud age, fallback count, and the latest TF/cloud time
difference, plus raw, finite, height-filtered, range-filtered, and displayed
point counts. Future extrapolation warnings are throttled to avoid log
flooding.
Invalid indoor GNSS (`NO_FIX`, NaN latitude/longitude or covariance) is simply
omitted from map geometry; it does not create a Qt NaN drawing operation.

## Map colors

- Green: raw GNSS current point and recent scatter; raw trail is opt-in.
- Cyan: filtered GNSS pose.
- Yellow: fused actual path.
- Blue: confirmed mission path and numbered waypoints.
- Magenta: selected local plan and highlighted local target.
- Red: current fused robot pose.
- Gray: raw FAST-LIO pose.

## Operations

- 开始选点：进入地图选点模式；短按添加航点，拖动仍然平移地图。
- 结束选点：保留 collector 中的待确认航点并进入 READY。
- 撤销上一点：`/open_nav/mission/delete_waypoint`
- Clear Mission: `/open_nav/mission/clear`
- Confirm Mission: first `/open_nav/mission/send`, then
  `/open_nav/mission/start_patrol` after an explicit safety confirmation.
- Pause Mission: `/open_nav/mission/pause`
- Resume Mission: `/open_nav/mission/resume`
- Reset Localization Stats: `/open_nav/localization/reset_distance`
- 标定当前朝向：地图点击机器人正前方参考点，确认后调用
  `/open_nav/heading/set_alignment`
- 清除航向标定：`/open_nav/heading/clear_alignment`
- Start/Stop rosbag: laptop subprocess recording the configured diagnostic topics

## Heading calibration

FAST-LIO yaw is local. Before REAL navigation, align FAST-LIO yaw to the
ENU/map frame from the dashboard:

1. Start the stack in TEST mode (`dry_run=true`).
2. Confirm the FUSION card shows `航向未标定`.
3. Place the robot facing a real-world landmark 20–50 m ahead.
4. Click `标定当前朝向`, then click that landmark on the map.
5. Confirm the preview arrow and dialog, then apply the offset.
6. Verify `航向已标定`, `/open_nav/heading/status.aligned=true`, and the red
   fused arrow matches the real heading.
7. Manually rotate the robot clockwise ~30°, then counter-clockwise ~30°.
   Compare `/Odometry`, `/open_nav/odom`, and the dashboard arrow. All three
   must change in the same direction. Do not invert yaw signs unless this test
   proves a sign bug.

Mission safety:

- REAL mode (`dry_run=false`) + UNALIGNED: mission start is blocked.
- TEST mode (`dry_run=true`) + UNALIGNED: mission start shows a warning only.

Unit tests:

```bash
python3 src/tron_open_space_nav/scripts/test_heading_alignment.py
python3 src/tron_nav_dashboard/scripts/test_dashboard_arrow.py
```

Synthetic ROS check (requires `gps_fastlio_fusion` running):

```bash
/root/catkin_ws/runtime_scripts/test_heading_alignment_ros.sh
```

地图点击坐标严格按以下顺序转换：屏幕像素 → Web Mercator 世界像素 →
WGS84 经纬度 → 当前 `/tron_open_space_nav/origin_*` 定义的 ENU/map 坐标。
Dashboard 不保存独立航点列表；蓝色待确认路线来自 `/open_nav/waypoints`，
正式任务路线来自 `/open_nav/mission_path`。

调试 bag 默认保存在 `/root/catkin_ws/bags`，文件名格式为
`tron_nav_YYYYMMDD_HHMMSS.bag`。
