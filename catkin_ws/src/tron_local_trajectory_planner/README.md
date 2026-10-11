# TRON Local Trajectory Planner

Visualization-only DWA-style trajectory sampler for GPS-TRON1. Phase 1 does
not publish `geometry_msgs/Twist` and cannot take control of the robot.

## Interfaces

Inputs:

- `/open_nav/odom` (`nav_msgs/Odometry`)
- `/open_nav/mission_path` (`nav_msgs/Path`)
- `/livox/lidar_filter1` (`sensor_msgs/PointCloud2`)

Outputs:

- `/open_nav/local_plan` (`nav_msgs/Path`)
- `/open_nav/local_plan_candidates` (`visualization_msgs/MarkerArray`)
- `/open_nav/local_planner/status` (`diagnostic_msgs/DiagnosticArray`)

The node samples linear and angular velocity pairs, predicts 2.5 seconds of
motion, rejects footprint collisions, and scores valid trajectories using
obstacle clearance, goal heading, progress, and velocity smoothness.

Frames are explicit: `global_frame` (default `map`) is used for mission and
odometry geometry, while `local_planning_frame` (default `body`) is used for
LiDAR obstacles, footprint checks, sampled trajectories, and local-plan
output. If the cloud already has the local frame, no TF lookup is performed.
The status diagnostic reports the source cloud frame, whether processing was
`DIRECT` or used `TF`, and raw, finite, height-filtered, range-filtered,
self-filtered, and voxel point counts. On the robot, `/livox/lidar_filter1`
is published in `body`, so the planner intentionally does not transform it to
`open_base`. This keeps the local obstacle coordinates consistent with the
predicted footprint at `(0, 0)`.

`/open_nav/local_plan` and `/open_nav/local_plan_candidates` are local-frame
visualization outputs. They must be displayed with `body` as their fixed
frame (or transformed by a consumer that knows the robot pose); they are not
global mission paths. The mission path and odometry remain global/navigation
inputs and are only used to select a local goal.

## Build and run

```bash
cd ~/catkin_ws
catkin_make_isolated --pkg tron_local_trajectory_planner --force-cmake
source devel_isolated/tron_local_trajectory_planner/setup.bash
roslaunch tron_local_trajectory_planner local_trajectory_planner.launch rviz:=true
```

Verify that the node has no velocity publisher:

```bash
rosnode info /local_trajectory_planner
rostopic echo /open_nav/local_planner/status
rostopic hz /open_nav/local_plan
```

The node is SHADOW-only. Its ROS graph contains no publisher for
`/open_nav/cmd_vel` or `/open_nav/cmd_vel_raw`; it only publishes the three
visualization/status topics listed above. A healthy robot diagnostic should
show `cloud_frame: body`, `cloud_transform: DIRECT`, non-zero filtered points,
and `candidate_count: 66` with the default `6 x 11` sampler.
