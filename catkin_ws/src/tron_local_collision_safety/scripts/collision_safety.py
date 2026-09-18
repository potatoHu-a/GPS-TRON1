#!/usr/bin/env python3
"""Arbitrate velocity using local occupancy and predicted footprint clearance."""

import math
import struct
import threading

import numpy as np
import rosgraph
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from tron_local_collision_safety.msg import ObstacleStatus


class CollisionSafetyNode:
    """Arbitrate velocity from local occupancy and predicted footprint clearance."""

    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/livox/lidar_filter1")
        self.input_cmd_topic = rospy.get_param(
            "~input_cmd_topic", "/open_nav/cmd_vel_raw"
        )
        self.output_cmd_topic = rospy.get_param(
            "~output_cmd_topic", "/open_nav/cmd_vel"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/open_nav/obstacle_status"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/open_nav/obstacle_markers"
        )
        self.base_frame = rospy.get_param("~base_frame", "body")

        self.footprint = np.asarray(
            rospy.get_param(
                "~footprint",
                [[-0.6, -0.25], [-0.6, 0.25], [0.6, 0.25], [0.6, -0.25]],
            ),
            dtype=np.float64,
        )
        self.prediction_horizon = float(
            rospy.get_param("~prediction/horizon", 2.0)
        )
        self.prediction_dt = float(rospy.get_param("~prediction/dt", 0.1))
        self.min_z = float(rospy.get_param("~height_filter/min_z", -0.2))
        self.max_z = float(rospy.get_param("~height_filter/max_z", 1.5))
        self.min_range = float(rospy.get_param("~range_filter/min_range", 0.1))
        self.max_range = float(rospy.get_param("~range_filter/max_range", 8.0))
        self.grid_x_min = float(rospy.get_param("~grid/x_min", -2.0))
        self.grid_x_max = float(rospy.get_param("~grid/x_max", 8.0))
        self.grid_y_min = float(rospy.get_param("~grid/y_min", -3.0))
        self.grid_y_max = float(rospy.get_param("~grid/y_max", 3.0))
        self.resolution = float(rospy.get_param("~grid/resolution", 0.05))
        self.safety_distance = float(rospy.get_param("~safety_distance", 0.5))
        self.slow_scale_min = float(rospy.get_param("~slow_scale_min", 0.35))
        self.slow_scale_max = float(rospy.get_param("~slow_scale_max", 0.70))
        self.pointcloud_timeout = float(
            rospy.get_param("~pointcloud_timeout", 0.8)
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))
        self.status_publish_rate = float(
            rospy.get_param("~status_publish_rate", 10.0)
        )
        self.marker_max_points = int(rospy.get_param("~marker_max_points", 3000))

        self._validate_parameters()
        self.grid_width = int(
            math.ceil((self.grid_x_max - self.grid_x_min) / self.resolution)
        )
        self.grid_height = int(
            math.ceil((self.grid_y_max - self.grid_y_min) / self.resolution)
        )
        self.cell_radius = self.resolution * math.sqrt(2.0) * 0.5

        self.lock = threading.RLock()
        self.last_cloud_time = None
        self.last_cmd_time = None
        self.raw_cmd = None
        self.cloud_fault = "waiting for point cloud"
        self.occupancy = np.zeros((self.grid_height, self.grid_width), dtype=bool)
        self.occupied_centers = np.empty((0, 2), dtype=np.float64)
        self.marker_points = np.empty((0, 3), dtype=np.float64)
        self.obstacle_point_count = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._warn_existing_cmd_publishers()
        self.status_pub = rospy.Publisher(
            self.status_topic, ObstacleStatus, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=10)
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber(
            self.input_cmd_topic, Twist, self._cmd_callback, queue_size=10
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0 / self.status_publish_rate), self._timer_callback
        )

        rospy.loginfo(
            "[collision_safety] velocity arbitration: cloud=%s cmd_in=%s cmd_out=%s "
            "frame=%s grid=%dx%d",
            self.cloud_topic,
            self.input_cmd_topic,
            self.output_cmd_topic,
            self.base_frame,
            self.grid_width,
            self.grid_height,
        )

    def _validate_parameters(self):
        if (
            self.footprint.ndim != 2
            or self.footprint.shape[0] < 3
            or self.footprint.shape[1] != 2
        ):
            raise ValueError("~footprint must contain at least three [x, y] vertices")
        if not np.isfinite(self.footprint).all():
            raise ValueError("~footprint vertices must be finite")
        if self.prediction_horizon <= 0.0:
            raise ValueError("~prediction/horizon must be positive")
        if self.prediction_dt <= 0.0 or self.prediction_dt > self.prediction_horizon:
            raise ValueError("~prediction/dt must be positive and no greater than horizon")
        if not self.min_z < self.max_z:
            raise ValueError("height_filter min_z must be smaller than max_z")
        if self.min_range < 0.0 or self.min_range >= self.max_range:
            raise ValueError("range_filter requires 0 <= min_range < max_range")
        if not self.grid_x_min < self.grid_x_max or not self.grid_y_min < self.grid_y_max:
            raise ValueError("grid minimums must be smaller than maximums")
        if self.resolution <= 0.0:
            raise ValueError("grid resolution must be positive")
        if self.safety_distance < 0.0:
            raise ValueError("safety_distance must not be negative")
        if not 0.0 < self.slow_scale_min <= self.slow_scale_max <= 1.0:
            raise ValueError(
                "slow scales must satisfy 0 < slow_scale_min <= slow_scale_max <= 1"
            )
        if self.pointcloud_timeout <= 0.0 or self.tf_timeout < 0.0:
            raise ValueError("pointcloud_timeout must be positive and tf_timeout nonnegative")
        if self.status_publish_rate <= 0.0 or self.marker_max_points <= 0:
            raise ValueError("publish rate and marker_max_points must be positive")

    def _warn_existing_cmd_publishers(self):
        resolved_topic = rospy.resolve_name(self.output_cmd_topic)
        try:
            publishers, _subscribers, _services = rosgraph.Master(
                rospy.get_name()
            ).getSystemState()
        except Exception as exc:  # ROS master failures must not bypass safe startup.
            rospy.logwarn(
                "[collision_safety] unable to check existing publishers on %s: %s",
                resolved_topic,
                exc,
            )
            return
        existing_nodes = []
        for topic, nodes in publishers:
            if rospy.resolve_name(topic) == resolved_topic:
                existing_nodes.extend(nodes)
        if existing_nodes:
            rospy.logwarn(
                "[collision_safety] %s already has publisher(s): %s; "
                "multiple velocity publishers can produce unsafe arbitration",
                resolved_topic,
                ", ".join(sorted(set(existing_nodes))),
            )

    def _cloud_callback(self, msg):
        receipt_time = rospy.Time.now()
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self._set_cloud_fault(receipt_time, "point cloud frame_id is empty")
            return

        try:
            points = np.asarray(
                list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float64,
            )
        except (KeyError, ValueError, TypeError, struct.error) as exc:
            self._set_cloud_fault(receipt_time, "invalid PointCloud2: {}".format(exc))
            return

        if points.size == 0:
            points = np.empty((0, 3), dtype=np.float64)
        else:
            points = points.reshape((-1, 3))

        try:
            points = self._transform_points(points, source_frame, msg.header.stamp)
        except (tf2_ros.TransformException, ValueError) as exc:
            self._set_cloud_fault(receipt_time, "TF {} -> {} failed: {}".format(source_frame, self.base_frame, exc))
            return

        points = self._filter_points(points)
        occupancy, occupied_centers = self._build_occupancy(points)
        marker_points = self._limit_marker_points(points)

        with self.lock:
            self.last_cloud_time = receipt_time
            self.cloud_fault = None
            self.occupancy = occupancy
            self.occupied_centers = occupied_centers
            self.marker_points = marker_points
            self.obstacle_point_count = len(points)

    def _set_cloud_fault(self, receipt_time, reason):
        with self.lock:
            self.last_cloud_time = receipt_time
            self.cloud_fault = reason
            self.occupancy.fill(False)
            self.occupied_centers = np.empty((0, 2), dtype=np.float64)
            self.marker_points = np.empty((0, 3), dtype=np.float64)
            self.obstacle_point_count = 0
        rospy.logwarn_throttle(1.0, "[collision_safety] %s", reason)

    def _transform_points(self, points, source_frame, stamp):
        if source_frame == self.base_frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            source_frame,
            stamp,
            rospy.Duration(self.tf_timeout),
        ).transform
        quaternion = np.array(
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
            dtype=np.float64,
        )
        norm = np.linalg.norm(quaternion)
        if norm < 1.0e-12 or not math.isfinite(norm):
            raise ValueError("TF contains an invalid quaternion")
        quaternion /= norm
        rotation = self._quaternion_matrix(quaternion)
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            dtype=np.float64,
        )
        return np.matmul(points, rotation.T) + translation

    @staticmethod
    def _quaternion_matrix(quaternion):
        x, y, z, w = quaternion
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _filter_points(self, points):
        if points.size == 0:
            return points
        finite = np.isfinite(points).all(axis=1)
        squared_range = np.einsum("ij,ij->i", points, points)
        selected = (
            finite
            & (points[:, 2] >= self.min_z)
            & (points[:, 2] <= self.max_z)
            & (squared_range >= self.min_range * self.min_range)
            & (squared_range <= self.max_range * self.max_range)
            & (points[:, 0] >= self.grid_x_min)
            & (points[:, 0] < self.grid_x_max)
            & (points[:, 1] >= self.grid_y_min)
            & (points[:, 1] < self.grid_y_max)
        )
        points = points[selected]
        if points.size == 0:
            return points.reshape((0, 3))
        self_points = self._points_in_polygon(points[:, :2], self.footprint)
        return points[~self_points]

    def _build_occupancy(self, points):
        occupancy = np.zeros((self.grid_height, self.grid_width), dtype=bool)
        if points.size == 0:
            return occupancy, np.empty((0, 2), dtype=np.float64)
        x_indices = np.floor((points[:, 0] - self.grid_x_min) / self.resolution).astype(np.int32)
        y_indices = np.floor((points[:, 1] - self.grid_y_min) / self.resolution).astype(np.int32)
        occupancy[y_indices, x_indices] = True
        y_occupied, x_occupied = np.nonzero(occupancy)
        centers = np.column_stack(
            (
                self.grid_x_min + (x_occupied + 0.5) * self.resolution,
                self.grid_y_min + (y_occupied + 0.5) * self.resolution,
            )
        )
        # Clearing by cell center mirrors costmap footprint clearing and avoids
        # reintroducing robot returns through grid quantization.
        keep = ~self._points_in_polygon(centers, self.footprint)
        if not keep.all():
            occupancy[y_occupied[~keep], x_occupied[~keep]] = False
            centers = centers[keep]
        return occupancy, centers

    def _limit_marker_points(self, points):
        if len(points) <= self.marker_max_points:
            return points.copy()
        indices = np.linspace(0, len(points) - 1, self.marker_max_points, dtype=np.int64)
        return points[indices]

    def _cmd_callback(self, msg):
        with self.lock:
            self.raw_cmd = self._copy_twist(msg)
            self.last_cmd_time = rospy.Time.now()

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            cloud_time = self.last_cloud_time
            cmd_time = self.last_cmd_time
            cloud_fault = self.cloud_fault
            occupied = self.occupied_centers.copy()
            marker_points = self.marker_points.copy()
            obstacle_point_count = self.obstacle_point_count
            raw_cmd = None if self.raw_cmd is None else self._copy_twist(self.raw_cmd)

        cloud_age = self._age(now, cloud_time)
        cmd_age = self._age(now, cmd_time)
        velocity = 0.0 if raw_cmd is None else raw_cmd.linear.x
        yaw_rate = 0.0 if raw_cmd is None else raw_cmd.angular.z
        trajectory = self._predict_trajectory(velocity, yaw_rate)

        if cloud_time is None:
            state, risk, front_distance = "STOP", 1.0, -1.0
            fault = cloud_fault
        elif cloud_fault is not None:
            state, risk, front_distance = "STOP", 1.0, -1.0
            fault = cloud_fault
        elif cloud_age > self.pointcloud_timeout:
            state, risk, front_distance = "STOP", 1.0, -1.0
            fault = "point cloud timeout ({:.3f}s)".format(cloud_age)
        else:
            state, risk = self._evaluate_state(occupied, trajectory)
            front_distance = self._front_distance(occupied)
            fault = None

        output_cmd = None
        if raw_cmd is not None:
            output_cmd = self._arbitrate_cmd(raw_cmd, state, risk)
            self.cmd_pub.publish(output_cmd)

        status = ObstacleStatus()
        status.header.stamp = now
        status.header.frame_id = self.base_frame
        status.state = state
        status.front_distance = front_distance
        status.obstacle_points = int(obstacle_point_count)
        status.collision_risk = risk
        status.pointcloud_age = cloud_age
        status.cmd_age = cmd_age
        status.cmd_raw_linear = 0.0 if raw_cmd is None else raw_cmd.linear.x
        status.cmd_output_linear = 0.0 if output_cmd is None else output_cmd.linear.x
        status.cmd_raw_angular = 0.0 if raw_cmd is None else raw_cmd.angular.z
        status.cmd_output_angular = 0.0 if output_cmd is None else output_cmd.angular.z
        self.status_pub.publish(status)
        self.marker_pub.publish(self._make_markers(now, marker_points, trajectory, state))

        if fault:
            rospy.logwarn_throttle(1.0, "[collision_safety] STOP: %s", fault)

    def _arbitrate_cmd(self, raw_cmd, state, collision_risk):
        if state == "FREE":
            return self._copy_twist(raw_cmd)
        if state == "SLOW":
            risk = float(np.clip(collision_risk, 0.0, 1.0))
            scale = self.slow_scale_max - (
                self.slow_scale_max - self.slow_scale_min
            ) * risk
            return self._scaled_twist(raw_cmd, scale)
        if state != "STOP":
            rospy.logerr_throttle(
                1.0,
                "[collision_safety] unknown state %s; publishing STOP",
                state,
            )
        return Twist()

    @staticmethod
    def _copy_twist(source):
        return CollisionSafetyNode._scaled_twist(source, 1.0)

    @staticmethod
    def _scaled_twist(source, scale):
        output = Twist()
        output.linear.x = source.linear.x * scale
        output.linear.y = source.linear.y * scale
        output.linear.z = source.linear.z * scale
        output.angular.x = source.angular.x * scale
        output.angular.y = source.angular.y * scale
        output.angular.z = source.angular.z * scale
        return output

    def _predict_trajectory(self, velocity, yaw_rate):
        count = int(math.floor(self.prediction_horizon / self.prediction_dt + 1.0e-9))
        trajectory = np.empty((count + 1, 3), dtype=np.float64)
        x_position = 0.0
        y_position = 0.0
        yaw = 0.0
        trajectory[0] = (x_position, y_position, yaw)
        for index in range(1, count + 1):
            next_yaw = yaw + yaw_rate * self.prediction_dt
            if abs(yaw_rate) < 1.0e-6:
                x_position += velocity * math.cos(yaw) * self.prediction_dt
                y_position += velocity * math.sin(yaw) * self.prediction_dt
            else:
                radius = velocity / yaw_rate
                x_position += radius * (math.sin(next_yaw) - math.sin(yaw))
                y_position -= radius * (math.cos(next_yaw) - math.cos(yaw))
            yaw = next_yaw
            trajectory[index] = (x_position, y_position, yaw)
        return trajectory

    def _evaluate_state(self, occupied, trajectory):
        if len(occupied) == 0:
            return "FREE", 0.0

        current_clearance = self._footprint_clearance(occupied, trajectory[0])
        if current_clearance <= self.cell_radius:
            return "STOP", 1.0

        minimum_clearance = current_clearance
        for pose in trajectory[1:]:
            minimum_clearance = min(
                minimum_clearance, self._footprint_clearance(occupied, pose)
            )
            if minimum_clearance <= self.cell_radius:
                return "SLOW", 1.0

        effective_clearance = max(0.0, minimum_clearance - self.cell_radius)
        if effective_clearance < self.safety_distance:
            risk = 1.0 - effective_clearance / max(self.safety_distance, 1.0e-9)
            return "SLOW", float(np.clip(risk, 0.0, 1.0))
        return "FREE", 0.0

    def _footprint_clearance(self, occupied, pose):
        x_position, y_position, yaw = pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        delta_x = occupied[:, 0] - x_position
        delta_y = occupied[:, 1] - y_position
        local_points = np.column_stack(
            (cosine * delta_x + sine * delta_y, -sine * delta_x + cosine * delta_y)
        )
        if self._points_in_polygon(local_points, self.footprint).any():
            return 0.0
        return self._minimum_polygon_distance(local_points, self.footprint)

    @staticmethod
    def _points_in_polygon(points, polygon):
        if len(points) == 0:
            return np.zeros(0, dtype=bool)
        inside = np.zeros(len(points), dtype=bool)
        x_coord = points[:, 0]
        y_coord = points[:, 1]
        previous = polygon[-1]
        for current in polygon:
            x_first, y_first = previous
            x_second, y_second = current
            crosses = (y_first > y_coord) != (y_second > y_coord)
            denominator = y_second - y_first
            if abs(denominator) < 1.0e-12:
                denominator = math.copysign(1.0e-12, denominator if denominator else 1.0)
            boundary_x = (x_second - x_first) * (y_coord - y_first) / denominator + x_first
            inside ^= crosses & (x_coord < boundary_x)
            previous = current
        return inside

    @staticmethod
    def _minimum_polygon_distance(points, polygon):
        minimum_squared = np.full(len(points), np.inf, dtype=np.float64)
        previous = polygon[-1]
        for current in polygon:
            segment = current - previous
            length_squared = float(np.dot(segment, segment))
            if length_squared <= 1.0e-18:
                previous = current
                continue
            offsets = points - previous
            factors = np.clip(np.matmul(offsets, segment) / length_squared, 0.0, 1.0)
            closest = previous + factors[:, np.newaxis] * segment
            squared = np.einsum("ij,ij->i", points - closest, points - closest)
            minimum_squared = np.minimum(minimum_squared, squared)
            previous = current
        return float(math.sqrt(float(np.min(minimum_squared))))

    def _front_distance(self, occupied):
        if len(occupied) == 0:
            return float("inf")
        front_x = float(np.max(self.footprint[:, 0]))
        min_y = float(np.min(self.footprint[:, 1]))
        max_y = float(np.max(self.footprint[:, 1]))
        corridor = occupied[
            (occupied[:, 0] >= front_x)
            & (occupied[:, 1] >= min_y)
            & (occupied[:, 1] <= max_y)
        ]
        if len(corridor) == 0:
            return float("inf")
        return max(0.0, float(np.min(corridor[:, 0]) - front_x - self.resolution * 0.5))

    @staticmethod
    def _age(now, last_time):
        if last_time is None:
            return -1.0
        return max(0.0, (now - last_time).to_sec())

    def _make_markers(self, stamp, obstacle_points, trajectory, state):
        markers = MarkerArray()
        markers.markers.append(self._obstacle_marker(stamp, obstacle_points))
        markers.markers.append(self._footprint_marker(stamp, state))
        markers.markers.append(self._trajectory_marker(stamp, trajectory))
        return markers

    def _base_marker(self, stamp, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "collision_safety"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.25)
        return marker

    def _obstacle_marker(self, stamp, obstacle_points):
        marker = self._base_marker(stamp, 0, Marker.POINTS)
        marker.scale.x = max(0.02, self.resolution)
        marker.scale.y = max(0.02, self.resolution)
        marker.color.r = 0.95
        marker.color.g = 0.15
        marker.color.b = 0.10
        marker.color.a = 0.85
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in obstacle_points]
        return marker

    def _footprint_marker(self, stamp, state):
        marker = self._base_marker(stamp, 1, Marker.LINE_STRIP)
        marker.scale.x = 0.04
        marker.color.a = 1.0
        colors = {
            "FREE": (0.10, 0.90, 0.20),
            "SLOW": (1.00, 0.70, 0.05),
            "STOP": (1.00, 0.05, 0.05),
        }
        marker.color.r, marker.color.g, marker.color.b = colors[state]
        vertices = np.vstack((self.footprint, self.footprint[0]))
        marker.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in vertices]
        return marker

    def _trajectory_marker(self, stamp, trajectory):
        marker = self._base_marker(stamp, 2, Marker.LINE_STRIP)
        marker.scale.x = 0.035
        marker.color.r = 0.10
        marker.color.g = 0.45
        marker.color.b = 1.00
        marker.color.a = 1.0
        marker.points = [Point(x=float(x), y=float(y), z=0.06) for x, y, _yaw in trajectory]
        return marker


def main():
    rospy.init_node("collision_safety")
    try:
        CollisionSafetyNode()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("[collision_safety] invalid configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
