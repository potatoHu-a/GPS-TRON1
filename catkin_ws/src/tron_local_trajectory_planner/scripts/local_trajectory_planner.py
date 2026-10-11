#!/usr/bin/env python3

import math
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))


class LocalTrajectoryPlanner(object):
    """Visualization-only DWA-style trajectory sampler."""

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/open_nav/odom")
        self.path_topic = rospy.get_param(
            "~mission_path_topic", "/open_nav/mission_path"
        )
        self.cloud_topic = rospy.get_param(
            "~cloud_topic", "/livox/lidar_filter1"
        )
        self.local_plan_topic = rospy.get_param(
            "~local_plan_topic", "/open_nav/local_plan"
        )
        self.candidates_topic = rospy.get_param(
            "~candidates_topic", "/open_nav/local_plan_candidates"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/open_nav/local_planner/status"
        )

        self.global_frame = rospy.get_param("~global_frame", "map").lstrip("/")
        self.local_planning_frame = rospy.get_param(
            "~local_planning_frame", "body"
        ).lstrip("/")
        self.planning_rate = float(rospy.get_param("~planning_rate", 5.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.5))
        self.path_timeout = float(rospy.get_param("~path_timeout", 0.0))
        self.cloud_timeout = float(rospy.get_param("~pointcloud_timeout", 0.8))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))

        self.horizon = float(rospy.get_param("~prediction/horizon", 2.5))
        self.dt = float(rospy.get_param("~prediction/dt", 0.1))
        self.min_linear = float(
            rospy.get_param("~sampling/min_linear_velocity", 0.0)
        )
        self.max_linear = float(
            rospy.get_param("~sampling/max_linear_velocity", 0.5)
        )
        self.linear_samples = int(rospy.get_param("~sampling/linear_samples", 6))
        self.max_angular = float(
            rospy.get_param("~sampling/max_angular_velocity", 0.5)
        )
        self.angular_samples = int(
            rospy.get_param("~sampling/angular_samples", 11)
        )

        self.robot_length = float(rospy.get_param("~footprint/length", 1.2))
        self.robot_width = float(rospy.get_param("~footprint/width", 0.5))
        self.safety_margin = float(
            rospy.get_param("~footprint/safety_margin", 0.08)
        )
        self.half_length = 0.5 * self.robot_length + self.safety_margin
        self.half_width = 0.5 * self.robot_width + self.safety_margin

        self.min_z = float(rospy.get_param("~obstacle_filter/min_z", -0.4))
        self.max_z = float(rospy.get_param("~obstacle_filter/max_z", 0.6))
        self.min_range = float(
            rospy.get_param("~obstacle_filter/min_range", 0.1)
        )
        self.max_range = float(
            rospy.get_param("~obstacle_filter/max_range", 8.0)
        )
        self.voxel_size = float(
            rospy.get_param("~obstacle_filter/voxel_size", 0.1)
        )
        self.max_obstacle_points = int(
            rospy.get_param("~obstacle_filter/max_points", 6000)
        )

        self.weight_clearance = float(
            rospy.get_param("~scoring/obstacle_clearance", 0.45)
        )
        self.weight_heading = float(rospy.get_param("~scoring/goal_heading", 0.25))
        self.weight_progress = float(
            rospy.get_param("~scoring/goal_progress", 0.20)
        )
        self.weight_smoothness = float(rospy.get_param("~scoring/smoothness", 0.10))
        self.desired_clearance = float(
            rospy.get_param("~scoring/desired_clearance", 0.8)
        )
        self.local_goal_distance = float(
            rospy.get_param("~scoring/local_goal_distance", 2.5)
        )
        self.marker_lifetime = float(rospy.get_param("~markers/lifetime", 0.35))
        self.marker_width = float(rospy.get_param("~markers/line_width", 0.025))

        self._validate_parameters()

        self.lock = threading.RLock()
        self.odom = None
        self.odom_received = None
        self.mission_points = np.empty((0, 2), dtype=np.float64)
        self.mission_frame = ""
        self.path_received = None
        self.obstacles = np.empty((0, 2), dtype=np.float64)
        self.cloud_raw_points = 0
        self.cloud_finite_points = 0
        self.cloud_height_points = 0
        self.cloud_range_points = 0
        self.cloud_self_points = 0
        self.cloud_voxel_points = 0
        self.cloud_received = None
        self.cloud_fault = "waiting for point cloud"
        self.cloud_source_frame = ""
        self.cloud_transform_mode = "WAITING"

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.plan_pub = rospy.Publisher(
            self.local_plan_topic, Path, queue_size=1
        )
        self.candidates_pub = rospy.Publisher(
            self.candidates_topic, MarkerArray, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DiagnosticArray, queue_size=10
        )

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.path_topic, Path, self._path_callback, queue_size=1)
        rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=1
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.planning_rate), self._planning_callback
        )

        rospy.loginfo(
            "[local_trajectory_planner] visualization only: odom=%s path=%s "
            "cloud=%s plan=%s samples=%dx%d horizon=%.2fs",
            self.odom_topic,
            self.path_topic,
            self.cloud_topic,
            self.local_plan_topic,
            self.linear_samples,
            self.angular_samples,
            self.horizon,
        )
        rospy.logwarn(
            "[local_trajectory_planner] phase 1 does not publish cmd_vel"
        )

    def _validate_parameters(self):
        if self.planning_rate <= 0.0:
            raise ValueError("planning_rate must be positive")
        if self.horizon <= 0.0 or self.dt <= 0.0 or self.dt > self.horizon:
            raise ValueError("prediction horizon/dt are invalid")
        if self.linear_samples < 2 or self.angular_samples < 3:
            raise ValueError("sampling requires at least 2 linear and 3 angular samples")
        if self.min_linear < 0.0 or self.max_linear < self.min_linear:
            raise ValueError("linear velocity range is invalid")
        if self.max_angular <= 0.0:
            raise ValueError("max angular velocity must be positive")
        if self.robot_length <= 0.0 or self.robot_width <= 0.0:
            raise ValueError("footprint dimensions must be positive")
        if self.min_z >= self.max_z or self.min_range >= self.max_range:
            raise ValueError("obstacle filter bounds are invalid")
        if self.voxel_size <= 0.0 or self.max_obstacle_points <= 0:
            raise ValueError("obstacle downsampling parameters are invalid")
        weights = (
            self.weight_clearance,
            self.weight_heading,
            self.weight_progress,
            self.weight_smoothness,
        )
        if min(weights) < 0.0 or sum(weights) <= 0.0:
            raise ValueError("scoring weights must be non-negative and non-zero")
        self.weight_sum = sum(weights)

    def _odom_callback(self, msg):
        with self.lock:
            self.odom = msg
            self.odom_received = rospy.Time.now()

    def _path_callback(self, msg):
        points = np.asarray(
            [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses],
            dtype=np.float64,
        )
        if points.size == 0:
            points = np.empty((0, 2), dtype=np.float64)
        else:
            points = points.reshape((-1, 2))
        with self.lock:
            self.mission_points = points
            self.mission_frame = msg.header.frame_id
            self.path_received = rospy.Time.now()

    def _cloud_callback(self, msg):
        if not msg.header.frame_id:
            with self.lock:
                self.cloud_fault = "point cloud frame_id is empty"
                self.cloud_received = rospy.Time.now()
                self.cloud_source_frame = ""
                self.cloud_transform_mode = "FAILED"
            return

        try:
            source_frame = msg.header.frame_id.lstrip("/")
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        msg, field_names=("x", "y", "z"), skip_nans=False
                    )
                ),
                dtype=np.float64,
            )
            if points.size == 0:
                filtered = np.empty((0, 2), dtype=np.float64)
                raw_count = 0
                finite_count = 0
                height_count = 0
                range_count = 0
                self_count = 0
                voxel_count = 0
                transform_mode = "DIRECT" if source_frame == self.local_planning_frame else "TF"
            else:
                points = points.reshape((-1, 3))
                raw_count = len(points)
                finite_mask = np.isfinite(points).all(axis=1)
                points = points[finite_mask]
                finite_count = len(points)
                points, transform_mode = self._transform_cloud(points, msg.header)
                filtered, counts = self._filter_obstacles(points)
                height_count, range_count, self_count, voxel_count = counts
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as exc:
            with self.lock:
                self.cloud_fault = "point cloud transform/filter failed: {}".format(exc)
                self.cloud_received = rospy.Time.now()
                self.cloud_source_frame = source_frame
                self.cloud_transform_mode = "FAILED"
            rospy.logwarn_throttle(1.0, "[local_trajectory_planner] %s", self.cloud_fault)
            return

        with self.lock:
            self.obstacles = filtered
            self.cloud_received = rospy.Time.now()
            self.cloud_fault = None
            self.cloud_raw_points = raw_count
            self.cloud_finite_points = finite_count
            self.cloud_height_points = height_count
            self.cloud_range_points = range_count
            self.cloud_self_points = self_count
            self.cloud_voxel_points = voxel_count
            self.cloud_source_frame = source_frame
            self.cloud_transform_mode = transform_mode

    def _transform_cloud(self, points, header):
        source_frame = header.frame_id.lstrip("/")
        target_frame = self.local_planning_frame
        if source_frame == target_frame:
            return points, "DIRECT"

        stamp = header.stamp if header.stamp != rospy.Time(0) else rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.ExtrapolationException:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        matrix = self._transform_matrix(transform.transform)
        homogeneous = np.column_stack((points, np.ones(points.shape[0])))
        return homogeneous.dot(matrix.T)[:, :3], "TF"

    @staticmethod
    def _transform_matrix(transform):
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        return np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), transform.translation.x],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), transform.translation.y],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), transform.translation.z],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _filter_obstacles(self, points):
        distances = np.hypot(points[:, 0], points[:, 1])
        height_mask = (
            (points[:, 2] >= self.min_z)
            & (points[:, 2] <= self.max_z)
        )
        filtered = points[height_mask]
        height_count = len(filtered)
        distances = np.hypot(filtered[:, 0], filtered[:, 1])
        range_mask = (distances >= self.min_range) & (distances <= self.max_range)
        filtered = filtered[range_mask]
        range_count = len(filtered)
        if filtered.size == 0:
            return np.empty((0, 2), dtype=np.float64), (
                height_count, range_count, 0, 0
            )

        outside_self = ~(
            (np.abs(filtered[:, 0]) <= self.half_length)
            & (np.abs(filtered[:, 1]) <= self.half_width)
        )
        filtered = filtered[outside_self]
        self_count = len(filtered)
        if filtered.size == 0:
            return np.empty((0, 2), dtype=np.float64), (
                height_count, range_count, self_count, 0
            )

        voxel_keys = np.floor(filtered[:, :2] / self.voxel_size).astype(np.int64)
        _, indices = np.unique(voxel_keys, axis=0, return_index=True)
        filtered = filtered[np.sort(indices), :2]
        voxel_count = len(filtered)
        if len(filtered) > self.max_obstacle_points:
            sample_indices = np.linspace(
                0, len(filtered) - 1, self.max_obstacle_points, dtype=np.int64
            )
            filtered = filtered[sample_indices]
        return filtered, (height_count, range_count, self_count, voxel_count)

    def _planning_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            odom = self.odom
            odom_received = self.odom_received
            mission_points = self.mission_points.copy()
            mission_frame = self.mission_frame
            path_received = self.path_received
            obstacles = self.obstacles.copy()
            cloud_received = self.cloud_received
            cloud_fault = self.cloud_fault

        odom_age = self._age(now, odom_received)
        path_age = self._age(now, path_received)
        cloud_age = self._age(now, cloud_received)
        fault = self._input_fault(
            odom,
            odom_age,
            mission_points,
            mission_frame,
            path_age,
            cloud_age,
            cloud_fault,
        )
        if fault is not None:
            self._publish_empty(now, "WAITING", fault, odom_age, path_age, cloud_age)
            return

        odom_frame = (odom.header.frame_id or self.global_frame).lstrip("/")
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        robot_yaw = yaw_from_quaternion(odom.pose.pose.orientation)

        try:
            mission_points = self._transform_path_points(
                mission_points, mission_frame, odom_frame, odom.header.stamp
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._publish_empty(
                now,
                "TF_ERROR",
                "mission path transform failed: {}".format(exc),
                odom_age,
                path_age,
                cloud_age,
            )
            return

        goal_global = self._select_local_goal(
            mission_points, np.asarray([robot_x, robot_y], dtype=np.float64)
        )
        goal_local = self._global_to_local(
            goal_global, robot_x, robot_y, robot_yaw
        )
        current_v = odom.twist.twist.linear.x
        current_w = odom.twist.twist.angular.z
        candidates = self._sample_candidates(
            obstacles, goal_local, current_v, current_w
        )
        valid = [candidate for candidate in candidates if candidate["valid"]]
        if not valid:
            self._publish_empty(
                now,
                "NO_VALID_TRAJECTORY",
                "all sampled trajectories collide",
                odom_age,
                path_age,
                cloud_age,
                candidates,
                obstacles,
                goal_local,
            )
            return

        best = max(valid, key=lambda candidate: candidate["score"])
        path = self._trajectory_path(best["trajectory"], now)
        self.plan_pub.publish(path)
        self.candidates_pub.publish(
            self._candidate_markers(now, candidates, best)
        )
        self._publish_status(
            now,
            "OK",
            "visualization-only best trajectory selected",
            odom_age,
            path_age,
            cloud_age,
            len(obstacles),
            len(candidates),
            len(valid),
            best,
            goal_local,
        )

    def _input_fault(
        self,
        odom,
        odom_age,
        mission_points,
        mission_frame,
        path_age,
        cloud_age,
        cloud_fault,
    ):
        if odom is None:
            return "waiting for {}".format(self.odom_topic)
        if odom_age > self.odom_timeout:
            return "odometry timeout ({:.3f}s)".format(odom_age)
        if len(mission_points) == 0:
            return "waiting for non-empty {}".format(self.path_topic)
        if not mission_frame:
            return "mission path frame_id is empty"
        if self.path_timeout > 0.0 and path_age > self.path_timeout:
            return "mission path timeout ({:.3f}s)".format(path_age)
        if cloud_age < 0.0:
            return "waiting for {}".format(self.cloud_topic)
        if cloud_fault is not None:
            return cloud_fault
        if cloud_age > self.cloud_timeout:
            return "point cloud timeout ({:.3f}s)".format(cloud_age)
        return None

    def _transform_path_points(self, points, source_frame, target_frame, stamp):
        source_frame = source_frame.lstrip("/")
        target_frame = target_frame.lstrip("/")
        if source_frame == target_frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp if stamp != rospy.Time(0) else rospy.Time(0),
            rospy.Duration(self.tf_timeout),
        ).transform
        yaw = yaw_from_quaternion(transform.rotation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.asarray([[cosine, -sine], [sine, cosine]])
        translation = np.asarray(
            [transform.translation.x, transform.translation.y], dtype=np.float64
        )
        return points.dot(rotation.T) + translation

    def _select_local_goal(self, path, robot):
        if len(path) == 1:
            return path[0]

        best_distance = float("inf")
        best_segment = 0
        best_fraction = 0.0
        best_projection = path[0]
        for index in range(len(path) - 1):
            start = path[index]
            delta = path[index + 1] - start
            length_sq = float(delta.dot(delta))
            fraction = 0.0 if length_sq < 1.0e-9 else clamp(
                float((robot - start).dot(delta) / length_sq), 0.0, 1.0
            )
            projection = start + fraction * delta
            distance = float(np.linalg.norm(robot - projection))
            if distance < best_distance:
                best_distance = distance
                best_segment = index
                best_fraction = fraction
                best_projection = projection

        remaining = self.local_goal_distance
        current = best_projection
        segment = best_segment
        segment_end = path[segment + 1]
        available = float(np.linalg.norm(segment_end - current))
        if remaining <= available and available > 1.0e-9:
            return current + (remaining / available) * (segment_end - current)
        remaining -= available
        segment += 1

        while segment < len(path) - 1:
            start = path[segment]
            end = path[segment + 1]
            length = float(np.linalg.norm(end - start))
            if remaining <= length and length > 1.0e-9:
                return start + (remaining / length) * (end - start)
            remaining -= length
            segment += 1
        return path[-1]

    @staticmethod
    def _global_to_local(point, robot_x, robot_y, robot_yaw):
        delta_x = point[0] - robot_x
        delta_y = point[1] - robot_y
        cosine = math.cos(robot_yaw)
        sine = math.sin(robot_yaw)
        return np.asarray(
            [cosine * delta_x + sine * delta_y,
             -sine * delta_x + cosine * delta_y],
            dtype=np.float64,
        )

    def _sample_candidates(self, obstacles, goal, current_v, current_w):
        linear_values = np.linspace(
            self.min_linear, self.max_linear, self.linear_samples
        )
        angular_values = np.linspace(
            -self.max_angular, self.max_angular, self.angular_samples
        )
        candidates = []
        for velocity in linear_values:
            for yaw_rate in angular_values:
                trajectory = self._simulate(float(velocity), float(yaw_rate))
                collision, clearance = self._trajectory_clearance(
                    trajectory, obstacles
                )
                score = -1.0
                if not collision:
                    score = self._score_trajectory(
                        trajectory,
                        clearance,
                        goal,
                        float(velocity),
                        float(yaw_rate),
                        current_v,
                        current_w,
                    )
                candidates.append(
                    {
                        "velocity": float(velocity),
                        "yaw_rate": float(yaw_rate),
                        "trajectory": trajectory,
                        "clearance": clearance,
                        "valid": not collision,
                        "score": score,
                    }
                )
        return candidates

    def _simulate(self, velocity, yaw_rate):
        count = int(math.floor(self.horizon / self.dt + 1.0e-9))
        trajectory = np.empty((count + 1, 3), dtype=np.float64)
        x_position = 0.0
        y_position = 0.0
        yaw = 0.0
        trajectory[0] = (x_position, y_position, yaw)
        for index in range(1, count + 1):
            next_yaw = yaw + yaw_rate * self.dt
            if abs(yaw_rate) < 1.0e-6:
                x_position += velocity * math.cos(yaw) * self.dt
                y_position += velocity * math.sin(yaw) * self.dt
            else:
                radius = velocity / yaw_rate
                x_position += radius * (math.sin(next_yaw) - math.sin(yaw))
                y_position -= radius * (math.cos(next_yaw) - math.cos(yaw))
            yaw = next_yaw
            trajectory[index] = (x_position, y_position, yaw)
        return trajectory

    def _trajectory_clearance(self, trajectory, obstacles):
        if len(obstacles) == 0:
            return False, self.max_range
        minimum = float("inf")
        for x_position, y_position, yaw in trajectory:
            delta_x = obstacles[:, 0] - x_position
            delta_y = obstacles[:, 1] - y_position
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            local_x = cosine * delta_x + sine * delta_y
            local_y = -sine * delta_x + cosine * delta_y
            outside_x = np.maximum(np.abs(local_x) - self.half_length, 0.0)
            outside_y = np.maximum(np.abs(local_y) - self.half_width, 0.0)
            clearances = np.hypot(outside_x, outside_y)
            pose_minimum = float(np.min(clearances))
            minimum = min(minimum, pose_minimum)
            if pose_minimum <= 1.0e-6:
                return True, 0.0
        return False, minimum

    def _score_trajectory(
        self,
        trajectory,
        clearance,
        goal,
        velocity,
        yaw_rate,
        current_v,
        current_w,
    ):
        final_x, final_y, final_yaw = trajectory[-1]
        initial_distance = float(np.linalg.norm(goal))
        final_delta = goal - np.asarray([final_x, final_y])
        final_distance = float(np.linalg.norm(final_delta))
        progress = clamp(
            (initial_distance - final_distance) / max(self.local_goal_distance, 0.1),
            0.0,
            1.0,
        )
        if final_distance < 1.0e-6:
            heading_score = 1.0
        else:
            target_heading = math.atan2(final_delta[1], final_delta[0])
            heading_error = abs(normalize_angle(target_heading - final_yaw))
            heading_score = 1.0 - heading_error / math.pi
        clearance_score = clamp(
            clearance / max(self.desired_clearance, 1.0e-3), 0.0, 1.0
        )
        velocity_span = max(self.max_linear - self.min_linear, 0.1)
        angular_span = max(2.0 * self.max_angular, 0.1)
        smoothness = math.exp(
            -abs(velocity - current_v) / velocity_span
            -abs(yaw_rate - current_w) / angular_span
        )
        weighted = (
            self.weight_clearance * clearance_score
            + self.weight_heading * heading_score
            + self.weight_progress * progress
            + self.weight_smoothness * smoothness
        )
        return weighted / self.weight_sum

    def _trajectory_path(self, trajectory, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.local_planning_frame
        for x_local, y_local, yaw_local in trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_local
            pose.pose.position.y = y_local
            pose.pose.orientation = quaternion_from_yaw(yaw_local)
            path.poses.append(pose)
        return path

    def _candidate_markers(self, stamp, candidates, best=None):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for marker_id, candidate in enumerate(candidates, 1):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.local_planning_frame
            marker.ns = "local_plan_candidates"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(self.marker_lifetime)
            marker.scale.x = self.marker_width
            if not candidate["valid"]:
                marker.color.r = 0.9
                marker.color.g = 0.1
                marker.color.b = 0.1
                marker.color.a = 0.18
            else:
                score = clamp(candidate["score"], 0.0, 1.0)
                marker.color.r = 1.0 - score
                marker.color.g = score
                marker.color.b = 0.25
                marker.color.a = 0.45
            if candidate is best:
                marker.scale.x = self.marker_width * 2.5
                marker.color.r = 0.05
                marker.color.g = 0.95
                marker.color.b = 1.0
                marker.color.a = 1.0
            marker.points = [
                Point(x=float(x), y=float(y), z=0.08)
                for x, y, _yaw in candidate["trajectory"]
            ]
            markers.markers.append(marker)
        return markers

    def _publish_empty(
        self,
        now,
        state,
        message,
        odom_age,
        path_age,
        cloud_age,
        candidates=None,
        obstacles=None,
        goal=None,
    ):
        empty = Path()
        empty.header.stamp = now
        empty.header.frame_id = self.local_planning_frame
        self.plan_pub.publish(empty)
        self.candidates_pub.publish(
            self._candidate_markers(now, candidates or [], None)
        )
        self._publish_status(
            now,
            state,
            message,
            odom_age,
            path_age,
            cloud_age,
            0 if obstacles is None else len(obstacles),
            0 if candidates is None else len(candidates),
            0 if candidates is None else sum(c["valid"] for c in candidates),
            None,
            np.zeros(2) if goal is None else goal,
        )
        rospy.logwarn_throttle(2.0, "[local_trajectory_planner] %s: %s", state, message)

    def _publish_status(
        self,
        stamp,
        state,
        message,
        odom_age,
        path_age,
        cloud_age,
        obstacle_count,
        candidate_count,
        valid_count,
        best,
        goal,
    ):
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "local_trajectory_planner"
        status.hardware_id = "TRON1"
        status.level = (
            DiagnosticStatus.OK if state == "OK" else DiagnosticStatus.WARN
        )
        status.message = "{}: {}".format(state, message)
        values = {
            "mode": "visualization_only",
            "global_frame": self.global_frame,
            "local_planning_frame": self.local_planning_frame,
            "cloud_frame": self.cloud_source_frame or "unknown",
            "cloud_transform": self.cloud_transform_mode,
            "state": state,
            "odom_age": "{:.3f}".format(odom_age),
            "path_age": "{:.3f}".format(path_age),
            "pointcloud_age": "{:.3f}".format(cloud_age),
            "obstacle_points": str(obstacle_count),
            "raw_points": str(self.cloud_raw_points),
            "finite_points": str(self.cloud_finite_points),
            "height_filtered_points": str(self.cloud_height_points),
            "range_filtered_points": str(self.cloud_range_points),
            "self_filtered_points": str(self.cloud_self_points),
            "voxel_points": str(self.cloud_voxel_points),
            "candidate_count": str(candidate_count),
            "valid_candidate_count": str(valid_count),
            "local_goal_x": "{:.3f}".format(float(goal[0])),
            "local_goal_y": "{:.3f}".format(float(goal[1])),
            "publishes_cmd_vel": "false",
        }
        if best is not None:
            values.update(
                {
                    "best_linear_velocity": "{:.3f}".format(best["velocity"]),
                    "best_angular_velocity": "{:.3f}".format(best["yaw_rate"]),
                    "best_score": "{:.4f}".format(best["score"]),
                    "best_clearance": "{:.3f}".format(best["clearance"]),
                }
            )
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array.status.append(status)
        self.status_pub.publish(array)

    @staticmethod
    def _age(now, received):
        if received is None:
            return -1.0
        return max(0.0, (now - received).to_sec())


def main():
    rospy.init_node("local_trajectory_planner")
    try:
        LocalTrajectoryPlanner()
    except ValueError as exc:
        rospy.logfatal("[local_trajectory_planner] invalid configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
