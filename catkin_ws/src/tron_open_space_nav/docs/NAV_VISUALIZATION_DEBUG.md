# Navigation Visualization Debug

## Existing Mapviz displays

The formal `config/rviz/mapviz.mvc` uses the following sources:

| Color | Display | Topic or frame | Publisher/source |
| --- | --- | --- | --- |
| Green | Raw GPS points | `/gps/fix` | `netgps_tcp_receiver`, `serial_gnss_receiver`, or the selected phone GPS receiver |
| Cyan | Filtered GPS points | `/open_nav/gps_filtered_pose` | `gps_global_converter` |
| Yellow | Actual accumulated track | `/open_nav/path` | `open_nav_path_publisher`, accumulated from `/open_nav/odom` |
| Red | Odometry arrow history | `/open_nav/odom` | `gps_fastlio_fusion` |
| Light blue | Sent mission path | `/open_nav/mission_path` | `gps_multi_waypoint_collector` |
| Blue | Robot TF arrow | frame `open_base` | TF published by `gps_fastlio_fusion` |

The standalone `mapviz_gnss_test.mvc` contains only the Wuhan tile map and
green `/gps/fix` history.

The apparent red-arrow delay is primarily a display configuration issue:
the formal Mapviz config retains 200 odometry arrows. During a turn those
historical orientations form a circle. It is not a single delayed robot
arrow. `gps_fastlio_fusion` publishes `/open_nav/odom`, `map -> open_odom`,
and `open_odom -> open_base` on every `/Odometry` callback, so no deliberate
low-frequency timer exists in that code path.

Check actual robot rates before drawing conclusions about network or sensor
latency:

```bash
rostopic hz /Odometry
rostopic hz /open_nav/odom
rostopic hz /tf
rostopic info /Odometry
rostopic info /open_nav/odom
```

## Mapviz navigation test

The independent navigation test view uses:

- green: `/gps/fix`;
- yellow: actual track `/open_nav/path`;
- blue: planned mission `/open_nav/mission_path`;
- red: current high-rate `/Odometry` pose with a one-arrow buffer.

Start it with:

```bash
~/catkin_ws/run/start_mapviz_navigation_test.sh
```

The launcher looks for `/open_nav/path`, `/open_nav/tracker_path`, then
`/waypoint_tracker/path` and remaps the display to the first available
`nav_msgs/Path`. It separately prefers `/open_nav/mission_path` for the
planned path. It reuses an existing `/local_xy_origin`; if none exists, it
starts the standard Mapviz origin initializer so raw NavSat fixes can be
projected without changing the navigation TF implementation.

## Obstacle speed overlay

`config/obstacle_speed.yaml` is loaded after the main obstacle configuration.
The effective distance-controlled speed is:

```text
min(raw command, max_linear_vel,
    max(distance_scale, slowdown_factor)
    * max_linear_vel * obstacle_speed_scale)
```

`obstacle_speed_scale: 1.0` preserves the existing behavior. Values above
`1.0` reduce the severity of slowdown but never bypass STOP or the configured
maximum velocity. `slowdown_factor` is the lower bound for the distance scale.
With the defaults, `0.35 * 0.5 = 0.175 m/s`, which remains below
`min_effective_forward_vel=0.2` and is converted to zero just as before.

## RViz navigation debug

```bash
~/catkin_ws/run/start_navigation_debug_rviz.sh
```

The launcher selects the first available `sensor_msgs/PointCloud2` topic from
`/livox/lidar_filter1`, `/cloud_registered`, and `/livox/lidar`, then remaps
the configured point-cloud display. RobotModel requires `robot_description`;
TF, point cloud, and odometry displays remain usable if that parameter is not
available.
