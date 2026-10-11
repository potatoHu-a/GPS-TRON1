# TRON Local Collision Safety

This package is a velocity safety arbiter for the GPS waypoint navigation
stack. It keeps the existing GPS-TRON ROS interfaces and adapts the safety
ideas used by TRON1's official `sentry_navigation` stack without bringing in
`move_base`, costmap_2d, or TEB.

## Architecture

```text
/livox/lidar_filter1 (PointCloud2)
              |
              v
TF to body + height/range/footprint filtering
              |
              v
0.05 m local obstacle grid
              |
              v
physical + inflated footprint and forward safety corridor
              |
              v
velocity-aware swept trajectory checking
              |
              v
        STOP / SLOW / FREE
              |
/open_nav/cmd_vel_raw -> arbitration -> /open_nav/cmd_vel
```

The official reference uses a 1.2 x 0.5 m footprint, a rolling local
costmap, footprint clearing, a 0.65 m inflation layer, and TEB trajectory
feasibility with `min_obstacle_dist: 0.5`. Inflation is a graded costmap field,
not an extra hard collision radius. This package therefore measures clearance
from the footprint, uses the TEB clearance plus a small margin for `SLOW`, and
reserves `STOP` for the physical footprint or a velocity-dependent braking
envelope. It never selects a left/right avoidance direction and does not alter
the waypoint target.

The checked official launch activates `/scan` as its obstacle source; a Livox
`PointCloud2` source is present in the costmap configuration but is not selected
by `observation_sources`. Official STOP/SLOW/FREE messages do not exist: TEB
rejects infeasible trajectories and `move_base` produces velocity, followed by
an EMA smoother. Here those behaviors are mapped to explicit safety states while
retaining the GPS-TRON PointCloud2 and velocity-arbitration interfaces.

`FREE` passes the complete `Twist` through. `SLOW` scales all components by
the same factor so direction and curvature are preserved. `STOP` publishes a
zero `Twist`. Missing/stale point cloud or unavailable TF is fail-safe `STOP`.

## Parameters

Main settings are in `config/collision.yaml`:

- `base_frame`: frame in which footprint and collision checks are evaluated.
- `footprint`: TRON1 body polygon, default 1.2 x 0.5 m.
- `height_filter`: obstacle height band after TF conversion.
- `range_filter`: minimum and maximum accepted point range.
- `grid`: local bounds and 0.05 m occupancy resolution.
- `obstacle_filter/min_points_per_cell`: optional isolated-return rejection.
- `inflation_radius`: official costmap visualization radius; not a STOP range.
- `min_obstacle_dist`: official TEB footprint-to-obstacle clearance.
- `safety_margin`: extra margin added to `min_obstacle_dist` for `SLOW`.
- `hard_stop_margin`: small hard clearance outside the physical footprint.
- `braking/deceleration`: official TEB longitudinal acceleration limit used as
  the conservative braking deceleration.
- `braking/reaction_time`: command/sensor reaction allowance used in the
  dynamic stopping envelope.
- `forward_corridor`: directional corridor dimensions and activation speed.
- `prediction/horizon`, `prediction/dt`: commanded-velocity trajectory model.
- `slow_scale_min`, `slow_scale_max`: output scaling range in `SLOW`.
- `pointcloud_timeout`, `tf_timeout`: fail-safe sensor and transform limits.

ROS interfaces remain:

```text
subscribe /livox/lidar_filter1
subscribe /open_nav/cmd_vel_raw
publish   /open_nav/cmd_vel
publish   /open_nav/obstacle_status
publish   /open_nav/obstacle_markers
```

Markers show filtered obstacle points, the physical footprint, the predicted
trajectory, the inflation envelope, and the forward corridor.

## Real Robot Test

Start each command in a separate terminal, in this order:

```bash
~/catkin_ws/run/start_livox.sh
~/catkin_ws/run/start_localization.sh
~/catkin_ws/run/start_gps_nav.sh
~/catkin_ws/run/start_collision_safety.sh
```

Localization must be running before collision safety. The node
`/fast_lio_localization_sc_qn_node` provides both the localization TF chain and
the only expected `/livox/lidar_filter1` publisher. The collision startup
script waits for that publisher rather than starting with a misleading cloud
timeout.

Begin with `dry_run:=true`. Check `/open_nav/obstacle_status` and RViz markers
before enabling the controller. Test an empty corridor (`FREE`), an obstacle
inside the slow envelope (`SLOW`), and a footprint/trajectory collision
(`STOP`). Side and rear obstacles outside the swept footprint and active travel
corridor must remain `FREE`.
