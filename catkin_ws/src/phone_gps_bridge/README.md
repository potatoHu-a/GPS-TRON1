# Phone and Serial GNSS Bridge

`phone_gps_bridge` publishes the same ROS GPS interfaces from either a NetGPS
TCP stream or a USB serial NMEA receiver. TCP remains the default mode.

## Start

```bash
# Existing phone NetGPS input
~/catkin_ws/run/start_phone_gps.sh tcp

# South GNSS USB serial input
~/catkin_ws/run/start_phone_gps.sh serial
```

Serial defaults to:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
```

Override without editing source:

```bash
GNSS_SERIAL_DEVICE=/dev/ttyACM0 \
GNSS_SERIAL_BAUDRATE=115200 \
~/catkin_ws/run/start_phone_gps.sh serial
```

The runtime entrypoint expects the existing robot ROS master. It does not start
`roscore`. For the GPS-TRON1 ground computer deployment:

```bash
export ROS_MASTER_URI=http://10.192.1.3:11311
export ROS_IP=10.192.1.194
```

## ROS Interfaces

- `/gps/fix` (`sensor_msgs/NavSatFix`): GGA/RMC position and fix validity.
- `/gps/course` (`std_msgs/Float64`): RMC/VTG course over ground.
- `/gps/heading` (`std_msgs/Float64`): published only from a real HDT sentence.
- `/gps/magnetic_heading` (`std_msgs/Float64`): published only from HDG.

RMC course over ground is not true vehicle heading and is never copied to
`/gps/heading`. The current South receiver stream has not yet provided a
verified dual-antenna heading sentence, so no true heading is expected.

GGA quality `0` and RMC status `V` publish `STATUS_NO_FIX` with NaN position;
they are never represented as a valid `(0, 0)` fix. GGA quality is logged at
debug level and is not mislabeled as RTK. Unknown proprietary `$PSIC`
sentences are ignored safely.

## GST Covariance

GST fields 7, 6, and 8 are the one-sigma longitude, latitude, and altitude
errors. A recent GST sample is mapped to the NavSatFix ENU diagonal as:

```text
east variance  = longitude_sigma^2
north variance = latitude_sigma^2
up variance    = altitude_sigma^2
```

This uses `COVARIANCE_TYPE_DIAGONAL_KNOWN`. If GST is missing or stale, the
existing HDOP estimate is retained as `COVARIANCE_TYPE_APPROXIMATED`; without
either input, covariance remains unknown.

## Docker Device Access

The container must be created with access to the USB character device. Do not
change permissions to `0666`. Prefer a device mapping and `dialout` membership.
The stable `/dev/serial/by-id` symlink also needs to be visible inside the
container; alternatively set `GNSS_SERIAL_DEVICE=/dev/ttyACM0`.
