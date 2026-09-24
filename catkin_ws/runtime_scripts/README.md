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

## Local South GNSS Mapviz Test

This standalone mode displays only the Wuhan offline tile map and `/gps/fix`.
It does not start or require robot navigation, FAST-LIO, GPS fusion,
`map`, or `open_base`.

The installed Mapviz NavSat plugin cannot consume latitude/longitude until its
`LocalXyUtil` has a geographic origin. The test launch therefore uses the
official `swri_transform_util/initialize_origin.py` node with the geographic
center of `wuhan_tiles/metadata.json` as a dedicated `gnss_local` projection
origin. This does not fabricate a GPS fix or a robot transform and is isolated
from the formal `map` navigation frame. Mapviz Fixed Frame and Target Frame
are both `wgs84`; `gnss_local` is used only inside the standard geographic
transformer.

On the host, expose the receiver as a TCP stream:

```bash
sudo chmod 666 /dev/ttyACM0
socat -d -d \
  TCP-LISTEN:10110,reuseaddr \
  FILE:/dev/ttyACM0,b115200,raw,echo=0
```

In container terminal 1, start the local ROS master:

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore
```

In container terminal 2, start the existing TCP receiver:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
rosrun phone_gps_bridge netgps_tcp_receiver.py \
  _host:=127.0.0.1 \
  _port:=10110 \
  _frame_id:=gps_link \
  _validate_checksum:=true \
  _publish_magnetic_heading:=true
```

Allow the container to use the host X server:

```bash
xhost +local:root
```

Then, in container terminal 3:

```bash
~/catkin_ws/runtime_scripts/start_mapviz_gnss_test.sh
```

The script uses only `http://127.0.0.1:11311`; it never starts another
`roscore`. If `/gps/fix` is not available yet it warns and still starts
Mapviz. An indoor no-fix message (`STATUS_NO_FIX` with NaN coordinates) is
correct: no GNSS point appears until the receiver obtains a valid outdoor fix.
The tile map remains available because its projection origin is fixed to the
center of the Wuhan tile set. With valid fixes, green points show the latest
600 samples so stationary drift can be inspected.

For a static accuracy experiment, fix the antenna outdoors for 5-10 minutes
and record the raw observations:

```bash
rosbag record -O south_gnss_static /gps/fix /gps/course
```

Use the bag to calculate horizontal standard deviation, RMS, maximum drift,
95% position spread, and compare them with GST covariance. Visual spread in
Mapviz alone is not an absolute accuracy measurement.
