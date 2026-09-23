# GPS-TRON Runtime Scripts

This directory is the deployable, package-independent runtime entrypoint for a
`catkin_make_isolated` workspace. Copy its contents to the workspace `run/`
directory on the target machine:

```bash
mkdir -p /home/guest/catkin_ws/run
cp -a runtime_scripts/. /home/guest/catkin_ws/run/
chmod +x /home/guest/catkin_ws/run/*.sh
```

No manual ROS setup and no top-level `rosrun` are required. Start modules in
separate terminals in this order:

```bash
~/catkin_ws/run/start_livox.sh
~/catkin_ws/run/start_localization.sh
~/catkin_ws/run/start_phone_gps.sh tcp
~/catkin_ws/run/start_gps_nav.sh
~/catkin_ws/run/start_collision_safety.sh
~/catkin_ws/run/start_mapviz.sh
~/catkin_ws/run/check_runtime.sh
```

GPS input modes:

```bash
# Existing phone NetGPS TCP mode (also the default when omitted)
~/catkin_ws/run/start_phone_gps.sh tcp

# South GNSS USB serial mode, 115200 8N1
~/catkin_ws/run/start_phone_gps.sh serial
```

The serial mode defaults to
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00`. RMC course over
ground is published on `/gps/course`; it is not true robot heading and is never
published on `/gps/heading`. True heading requires a verified HDT/HDG or future
dual-antenna attitude sentence.

Use `TRON_WS=/path/to/catkin_ws` when the scripts are not installed directly
below the intended workspace. External official workspaces may be supplied as
`LIVOX_WS` and `FASTLIO_WS`; their `local_setup.bash` files are used when
present.

`start_livox.sh` only starts the official Livox driver. `start_localization.sh`
only starts the official FAST-LIO localization launch. It provides both the
localization TF chain and `/livox/lidar_filter1`; that topic's only expected
publisher is `/fast_lio_localization_sc_qn_node`. Therefore
`start_localization.sh` must be running before `start_collision_safety.sh`.

`start_collision_safety.sh` waits until
`/fast_lio_localization_sc_qn_node` publishes `/livox/lidar_filter1`. It does
not start the safety node while that dependency is missing, avoiding an
immediate point-cloud timeout and misleading `STOP` state.

GPS fusion remains part of `start_gps_nav.sh`, so it is not started a second
time by the localization entrypoint. Existing `/livox_lidar_publisher2`, `/livox_driver`,
`/laserMapping`, `/fast_lio_localization_sc_qn_node`, and
`/gps_fastlio_fusion` nodes are detected and never started twice.
