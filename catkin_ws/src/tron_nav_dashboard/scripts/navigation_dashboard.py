#!/usr/bin/env python3

import collections
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import threading

import numpy as np
import rospy
import rosgraph
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from PyQt5 import QtCore, QtGui, QtWidgets
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import MarkerArray

from tron_nav_dashboard.dashboard_geometry import arrow_delta_screen

from tron_local_collision_safety.msg import ObstacleStatus
from tron_open_space_nav.msg import MissionStatus, HeadingStatus
from tron_open_space_nav.srv import (
    DeleteWaypoint,
    MissionPatrolStart,
    ResetLocalizationDistance,
    SetHeadingAlignment,
)


EARTH_RADIUS = 6378137.0
TILE_SIZE = 256.0
BAG_TOPICS = [
    "/gps/fix",
    "/gps/course",
    "/Odometry",
    "/open_nav/odom",
    "/open_nav/gps_filtered_pose",
    "/open_nav/waypoints",
    "/open_nav/mission_path",
    "/open_nav/mission_status",
    "/open_nav/local_plan",
    "/open_nav/cmd_vel_raw",
    "/open_nav/cmd_vel",
    "/open_nav/obstacle_status",
    "/open_nav/local_planner/status",
    "/open_nav/navigation_debug",
    "/open_nav/heading/status",
    "/tf",
    "/tf_static",
]

UI_LANGUAGE = "zh_CN"
UI_TEXT = {
    "zh_CN": {
        "window_title": "TRON1 导航监控面板",
        "map": "地图",
        "follow": "跟随",
        "free_view": "自由浏览",
        "lidar_planner": "雷达 / 局部规划",
        "waiting": "等待数据",
        "operations": "任务操作",
        "center": "居中到机器人",
        "follow_robot": "跟随机器人",
        "raw_gnss_trail": "显示原始 GNSS 轨迹",
        "waiting_debug": "正在等待 /open_nav/navigation_debug",
        "confirm_mission": "确认任务",
        "clear_mission": "清空航点",
        "pause_mission": "暂停任务",
        "resume_mission": "继续任务",
        "reset_localization": "重置定位统计",
        "calibrate_heading": "标定当前朝向",
        "clear_heading": "清除航向标定",
        "heading_hint": "请点击机器人当前正前方尽可能远的位置，建议20m以上。",
        "heading_calibration_mode": "航向标定模式",
        "heading_unaligned": "航向未标定",
        "heading_aligned": "航向已标定",
        "heading_blocked_real": "当前全局航向未标定，请先完成航向标定。",
        "heading_warn_test": "当前航向未标定，仅用于 TEST 模式调试。",
        "start_rosbag": "开始记录调试数据",
        "stop_rosbag": "停止记录调试数据",
        "start_editing": "开始选点",
        "finish_editing": "结束选点",
        "undo_waypoint": "撤销上一点",
        "editing_mode": "航点编辑模式",
        "mission_state": "任务状态：{}",
        "bag_idle": "未记录",
        "bag_recording": "● 正在记录",
        "bag_last_file": "最近文件：{}",
        "bag_file": "文件：{}",
        "bag_tooltip": "记录 GNSS、FAST-LIO、融合定位、规划、避障及控制 Topic，\n用于离线复现导航问题，不是屏幕录像。",
        "origin_unavailable": "无法选点：导航原点尚未建立",
        "finish_first": "请先结束选点，再确认任务",
        "no_waypoint": "当前没有可操作的航点",
        "confirm_start_title": "确认并开始导航",
        "confirm_start_text": "确认任务后将通过现有任务管理器启动导航。\n请确认路线和机器人周围环境安全。",
        "lidar": "雷达",
        "no_data": "无数据",
        "pointcloud": "点云",
        "points": "个点",
        "age": "年龄",
        "planner": "规划器",
        "valid": "有效",
        "service_unavailable": "服务不可用：{}",
        "calls_service": "调用服务：{}",
        "disabled_service": "已禁用：服务不可用（{}）",
        "card_gnss": "GNSS 卫星定位",
        "card_fastlio": "FAST-LIO 激光定位",
        "card_fusion": "融合定位",
        "card_planner": "局部规划器",
        "card_safety": "碰撞安全层",
        "card_controller": "机器人控制器",
    },
    "en_US": {},
}


def tr(key, fallback):
    return UI_TEXT.get(UI_LANGUAGE, {}).get(key, fallback)


def translated_state(value):
    if UI_LANGUAGE != "zh_CN":
        return value
    return {
        "FIX": "已定位（FIX）",
        "NO_FIX": "未定位（NO_FIX）",
        "FREE": "畅通（FREE）",
        "SLOW": "减速（SLOW）",
        "STOP": "停车（STOP）",
        "UNKNOWN": "未知（UNKNOWN）",
        "WAITING": "等待（WAITING）",
        "STALE": "数据过期（STALE）",
        "OK": "正常（OK）",
        "SHADOW": "影子模式（SHADOW）",
        "CONTROL": "控制模式（CONTROL）",
        "IDLE": "空闲（IDLE）",
        "EDITING": "选点中（EDITING）",
        "READY": "待确认（READY）",
        "RUNNING": "运行中（RUNNING）",
        "PAUSED": "已暂停（PAUSED）",
    }.get(value, value)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def arrow_tip_screen(point, yaw, length):
    """Map/ENU yaw to Qt screen delta: +x east/right, +y north/up on screen."""
    dx, dy = arrow_delta_screen(yaw, length)
    return QtCore.QPointF(
        point.x() + dx,
        point.y() + dy,
    )


def latlon_to_world(latitude, longitude, zoom):
    scale = TILE_SIZE * (2 ** zoom)
    x = (longitude + 180.0) / 360.0 * scale
    latitude = max(-85.05112878, min(85.05112878, latitude))
    lat_rad = math.radians(latitude)
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) * 0.5 * scale
    return x, y


def enu_to_latlon(east, north, origin_lat, origin_lon):
    latitude = origin_lat + math.degrees(north / EARTH_RADIUS)
    longitude = origin_lon + math.degrees(
        east / (EARTH_RADIUS * max(math.cos(math.radians(origin_lat)), 1.0e-6))
    )
    return latitude, longitude


def world_to_latlon(world_x, world_y, zoom):
    scale = TILE_SIZE * (2 ** zoom)
    longitude = world_x / scale * 360.0 - 180.0
    mercator_y = math.pi - 2.0 * math.pi * world_y / scale
    latitude = math.degrees(math.atan(math.sinh(mercator_y)))
    return latitude, longitude


def latlon_to_enu(latitude, longitude, origin_lat, origin_lon):
    north = math.radians(latitude - origin_lat) * EARTH_RADIUS
    east = (
        math.radians(longitude - origin_lon)
        * EARTH_RADIUS
        * math.cos(math.radians(origin_lat))
    )
    return east, north


def diagnostic_dict(message):
    result = {}
    if message is None:
        return result
    for status in message.status:
        section = status.name.rsplit("/", 1)[-1]
        result[section] = {
            "level": status.level,
            "message": status.message,
            **{entry.key: entry.value for entry in status.values},
        }
    return result


class RosCache(object):
    """Callbacks only cache data. Qt timers own all rendering."""

    def __init__(self, max_path_points, scatter_points, lidar_max_points, voxel_size,
                 lidar_display_frame="body", lidar_min_z=-0.4, lidar_max_z=1.0,
                 lidar_min_range=0.1, lidar_max_range=8.0):
        self.lock = threading.RLock()
        self.data = {}
        self.received = {}
        self.raw_scatter = collections.deque(maxlen=scatter_points)
        self.actual_path = collections.deque(maxlen=max_path_points)
        self.filtered_path = collections.deque(maxlen=max_path_points)
        self.lidar_points = np.empty((0, 2), dtype=np.float32)
        self.cloud_tf_status = "FAILED"
        self.cloud_tf_fallback_count = 0
        self.cloud_tf_delta = None
        self.cloud_tf_warned = False
        self.lidar_display_frame = lidar_display_frame.lstrip("/") or "body"
        self.lidar_min_z = float(lidar_min_z)
        self.lidar_max_z = float(lidar_max_z)
        self.lidar_min_range = float(lidar_min_range)
        self.lidar_max_range = float(lidar_max_range)
        self.raw_point_count = 0
        self.finite_point_count = 0
        self.height_filtered_count = 0
        self.range_filtered_count = 0
        self.displayed_point_count = 0
        self.odom_stamps = collections.deque(maxlen=50)
        self.lidar_max_points = lidar_max_points
        self.voxel_size = voxel_size
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._subscribe("gps", "/gps/fix", NavSatFix, self._gps_callback)
        self._subscribe("fastlio", "/Odometry", Odometry, self._fastlio_callback)
        self._subscribe("fused", "/open_nav/odom", Odometry, self._fused_callback)
        self._subscribe("heading", "/open_nav/heading/status", HeadingStatus)
        self._subscribe(
            "gps_filtered",
            "/open_nav/gps_filtered_pose",
            PoseStamped,
            self._filtered_callback,
        )
        self._subscribe("actual_path_msg", "/open_nav/path", Path, self._actual_path_callback)
        self._subscribe("pending", "/open_nav/waypoints", Path)
        self._subscribe("mission", "/open_nav/mission_path", Path)
        self._subscribe("mission_status", "/open_nav/mission_status", MissionStatus)
        self._subscribe("local_plan", "/open_nav/local_plan", Path)
        self._subscribe("cmd_raw", "/open_nav/cmd_vel_raw", Twist)
        self._subscribe("cmd_final", "/open_nav/cmd_vel", Twist)
        self._subscribe("safety", "/open_nav/obstacle_status", ObstacleStatus)
        self._subscribe("planner", "/open_nav/local_planner/status", DiagnosticArray)
        self._subscribe("debug", "/open_nav/navigation_debug", DiagnosticArray)
        self._subscribe(
            "candidates", "/open_nav/local_plan_candidates", MarkerArray
        )
        rospy.Subscriber(
            "/livox/lidar_filter1", PointCloud2, self._cloud_callback, queue_size=1
        )

    def _subscribe(self, key, topic, msg_type, callback=None):
        handler = callback or (lambda msg, k=key: self._store(k, msg))
        rospy.Subscriber(topic, msg_type, handler, queue_size=10)

    def _store(self, key, message):
        with self.lock:
            self.data[key] = message
            self.received[key] = rospy.Time.now().to_sec()

    def _gps_callback(self, message):
        self._store("gps", message)
        if (
            message.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(message.latitude)
            and math.isfinite(message.longitude)
        ):
            with self.lock:
                self.raw_scatter.append((message.latitude, message.longitude))

    def _fastlio_callback(self, message):
        self._store("fastlio", message)
        with self.lock:
            self.odom_stamps.append(rospy.Time.now().to_sec())

    def _fused_callback(self, message):
        self._store("fused", message)
        with self.lock:
            self.actual_path.append(
                (message.pose.pose.position.x, message.pose.pose.position.y)
            )

    def _filtered_callback(self, message):
        self._store("gps_filtered", message)
        with self.lock:
            self.filtered_path.append(
                (message.pose.position.x, message.pose.position.y)
            )

    def _actual_path_callback(self, message):
        self._store("actual_path_msg", message)
        with self.lock:
            self.actual_path.clear()
            self.actual_path.extend(
                (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
            )

    def _cloud_callback(self, message):
        tf_status = "DIRECT"
        tf_delta = None
        try:
            rows = np.asarray(
                list(
                    point_cloud2.read_points(
                        message, field_names=("x", "y", "z"), skip_nans=False
                    )
                ),
                dtype=np.float32,
            )
            raw_count = len(rows)
            finite_count = 0
            height_count = 0
            range_count = 0
            if rows.size == 0:
                points = np.empty((0, 2), dtype=np.float32)
            else:
                rows = rows.reshape((-1, 3))
                source_frame = message.header.frame_id.lstrip("/")
                finite = np.isfinite(rows).all(axis=1)
                rows = rows[finite]
                finite_count = len(rows)
                if source_frame == self.lidar_display_frame:
                    tf_status = "DIRECT"
                elif source_frame:
                    cloud_stamp = message.header.stamp
                    try:
                            transform_stamped = self.tf_buffer.lookup_transform(
                                self.lidar_display_frame,
                                source_frame,
                            cloud_stamp if cloud_stamp != rospy.Time(0) else rospy.Time(0),
                            rospy.Duration(0.15),
                        )
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException) as exc:
                        tf_status = "LATEST"
                        self.cloud_tf_fallback_count += 1
                        rospy.loginfo_once(
                            "[nav_dashboard] cloud TF timestamp unavailable; using latest TF fallback"
                        )
                        rospy.logwarn_throttle(
                            5.0,
                            "[nav_dashboard] cloud TF sync failed; latest fallback in use: %s",
                            exc,
                        )
                        try:
                            transform_stamped = self.tf_buffer.lookup_transform(
                                self.lidar_display_frame, source_frame, rospy.Time(0), rospy.Duration(0.05)
                            )
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                                tf2_ros.ExtrapolationException):
                            self.cloud_tf_status = "FAILED"
                            raise
                    transform = transform_stamped.transform
                    if cloud_stamp != rospy.Time(0) and transform_stamped.header.stamp != rospy.Time(0):
                        tf_delta = abs((transform_stamped.header.stamp - cloud_stamp).to_sec())
                    rows = self._transform_points(rows, transform)
                height_mask = (
                    (rows[:, 2] >= self.lidar_min_z)
                    & (rows[:, 2] <= self.lidar_max_z)
                )
                rows = rows[height_mask]
                height_count = len(rows)
                distance = np.hypot(rows[:, 0], rows[:, 1])
                range_mask = (
                    (distance >= self.lidar_min_range)
                    & (distance <= self.lidar_max_range)
                )
                rows = rows[range_mask]
                range_count = len(rows)
                if len(rows):
                    keys = np.floor(rows[:, :2] / self.voxel_size).astype(np.int32)
                    _, indexes = np.unique(keys, axis=0, return_index=True)
                    points = rows[np.sort(indexes), :2]
                    if len(points) > self.lidar_max_points:
                        indexes = np.linspace(
                            0, len(points) - 1, self.lidar_max_points, dtype=np.int32
                        )
                        points = points[indexes]
                else:
                    points = np.empty((0, 2), dtype=np.float32)
            displayed_count = len(points)
        except Exception as exc:
            self.cloud_tf_status = "FAILED"
            rospy.logwarn_throttle(5.0, "[nav_dashboard] cloud decode/TF failed: %s", exc)
            return
        with self.lock:
            self.lidar_points = points
            self.received["cloud"] = rospy.Time.now().to_sec()
            self.cloud_tf_status = tf_status
            self.cloud_tf_delta = tf_delta
            self.raw_point_count = raw_count
            self.finite_point_count = finite_count
            self.height_filtered_count = height_count
            self.range_filtered_count = range_count
            self.displayed_point_count = displayed_count

    @staticmethod
    def _transform_points(points, transform):
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        rotation = np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )
        translation = np.asarray(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            dtype=np.float32,
        )
        return points.dot(rotation.T) + translation

    def snapshot(self):
        with self.lock:
            data = dict(self.data)
            data["raw_scatter"] = list(self.raw_scatter)
            data["actual_path"] = list(self.actual_path)
            data["filtered_path"] = list(self.filtered_path)
            data["lidar_points"] = self.lidar_points.copy()
            data["odom_hz"] = self._frequency(self.odom_stamps)
            data["received"] = dict(self.received)
            data["cloud_tf_status"] = self.cloud_tf_status
            data["cloud_tf_fallback_count"] = self.cloud_tf_fallback_count
            data["cloud_tf_delta"] = self.cloud_tf_delta
            data["raw_point_count"] = self.raw_point_count
            data["finite_point_count"] = self.finite_point_count
            data["height_filtered_count"] = self.height_filtered_count
            data["range_filtered_count"] = self.range_filtered_count
            data["displayed_point_count"] = self.displayed_point_count
            return data

    @staticmethod
    def _frequency(stamps):
        if len(stamps) < 2:
            return 0.0
        elapsed = stamps[-1] - stamps[0]
        return 0.0 if elapsed <= 0.0 else (len(stamps) - 1) / elapsed


class MapCanvas(QtWidgets.QWidget):
    waypoint_requested = QtCore.pyqtSignal(float, float, float, float)
    heading_requested = QtCore.pyqtSignal(float, float)

    def __init__(self, tile_root, drag_threshold=6.0, parent=None):
        super().__init__(parent)
        self.tile_root = tile_root
        self.zoom = 16
        self.min_zoom = 12
        self.max_zoom = 18
        self.center_world = latlon_to_world(30.54715, 114.356, self.zoom)
        self.drag_origin = None
        self.press_origin = None
        self.drag_distance = 0.0
        self.follow_robot = False
        self.raw_trail = False
        self.waypoint_editing = False
        self.heading_calibrating = False
        self.pending_heading_ref = None
        self.drag_threshold = max(1.0, float(drag_threshold))
        self.snapshot = {}
        self.origin = None
        self.tile_cache = collections.OrderedDict()
        self.setMinimumSize(360, 360)
        self.setMouseTracking(True)
        self._load_metadata()

    def _load_metadata(self):
        metadata_path = os.path.join(self.tile_root, "metadata.json")
        try:
            with open(metadata_path, "r", encoding="utf-8") as stream:
                metadata = json.load(stream)
            self.min_zoom = int(metadata.get("minzoom", 12))
            self.max_zoom = int(metadata.get("maxzoom", 18))
            west, south, east, north = map(float, metadata["bounds"].split(","))
            self.zoom = min(max(self.zoom, self.min_zoom), self.max_zoom)
            self.center_world = latlon_to_world(
                0.5 * (south + north), 0.5 * (west + east), self.zoom
            )
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            rospy.logwarn("[nav_dashboard] tile metadata unavailable: %s", exc)

    def set_snapshot(self, snapshot, origin):
        self.snapshot = snapshot
        self.origin = origin
        if self.follow_robot:
            self.center_on_robot()
        self.update()

    def center_on_robot(self):
        position = self._fused_latlon()
        if position is not None:
            self.center_world = latlon_to_world(position[0], position[1], self.zoom)
            self.update()

    def set_waypoint_editing(self, enabled):
        self.waypoint_editing = enabled
        self.setCursor(QtCore.Qt.CrossCursor if enabled else QtCore.Qt.ArrowCursor)
        self.update()

    def set_heading_calibrating(self, enabled):
        self.heading_calibrating = bool(enabled)
        if not enabled:
            self.pending_heading_ref = None
        self.setCursor(QtCore.Qt.CrossCursor if enabled else QtCore.Qt.ArrowCursor)
        self.update()

    def set_pending_heading_ref(self, east, north):
        if east is None or north is None:
            self.pending_heading_ref = None
        else:
            self.pending_heading_ref = (east, north)
        self.update()

    def screen_to_map(self, point):
        if self.origin is None:
            return None
        world_x = self.center_world[0] + point.x() - 0.5 * self.width()
        world_y = self.center_world[1] + point.y() - 0.5 * self.height()
        latitude, longitude = world_to_latlon(world_x, world_y, self.zoom)
        east, north = latlon_to_enu(
            latitude, longitude, self.origin[0], self.origin[1]
        )
        return east, north, latitude, longitude

    def _fused_latlon(self):
        fused = self.snapshot.get("fused")
        if fused is None or self.origin is None:
            return None
        return enu_to_latlon(
            fused.pose.pose.position.x,
            fused.pose.pose.position.y,
            self.origin[0],
            self.origin[1],
        )

    def _screen_from_latlon(self, latitude, longitude):
        world_x, world_y = latlon_to_world(latitude, longitude, self.zoom)
        return QtCore.QPointF(
            0.5 * self.width() + world_x - self.center_world[0],
            0.5 * self.height() + world_y - self.center_world[1],
        )

    def _screen_from_enu(self, east, north):
        if self.origin is None:
            return None
        latitude, longitude = enu_to_latlon(
            east, north, self.origin[0], self.origin[1]
        )
        return self._screen_from_latlon(latitude, longitude)

    def _tile(self, zoom, x, y):
        key = (zoom, x, y)
        if key in self.tile_cache:
            pixmap = self.tile_cache.pop(key)
            self.tile_cache[key] = pixmap
            return pixmap
        path = os.path.join(self.tile_root, str(zoom), str(x), "{}.png".format(y))
        pixmap = QtGui.QPixmap(path)
        self.tile_cache[key] = pixmap
        while len(self.tile_cache) > 100:
            self.tile_cache.popitem(last=False)
        return pixmap

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#161a1d"))
        self._draw_tiles(painter)
        self._draw_map_layers(painter)
        painter.setPen(QtGui.QColor("#d7dde2"))
        mode = tr("follow", "FOLLOW") if self.follow_robot else tr("free_view", "FREE")
        painter.drawText(12, 22, "{}  z{}  {}".format(tr("map", "MAP"), self.zoom, mode))

    def _draw_tiles(self, painter):
        left = self.center_world[0] - 0.5 * self.width()
        top = self.center_world[1] - 0.5 * self.height()
        first_x = int(math.floor(left / TILE_SIZE))
        last_x = int(math.floor((left + self.width()) / TILE_SIZE))
        first_y = int(math.floor(top / TILE_SIZE))
        last_y = int(math.floor((top + self.height()) / TILE_SIZE))
        for tile_x in range(first_x, last_x + 1):
            for tile_y in range(first_y, last_y + 1):
                pixmap = self._tile(self.zoom, tile_x, tile_y)
                x = int(tile_x * TILE_SIZE - left)
                y = int(tile_y * TILE_SIZE - top)
                if pixmap.isNull():
                    painter.fillRect(x, y, 256, 256, QtGui.QColor("#22282d"))
                else:
                    painter.drawPixmap(x, y, pixmap)

    def _draw_polyline(self, painter, points, color, width=2.0):
        screen = [
            self._screen_from_enu(x, y)
            for x, y in points
            if math.isfinite(float(x)) and math.isfinite(float(y))
        ]
        screen = [point for point in screen if point is not None]
        if len(screen) >= 2:
            painter.setPen(QtGui.QPen(QtGui.QColor(color), width))
            painter.drawPolyline(QtGui.QPolygonF(screen))

    def _draw_dashed_polyline(self, painter, points, color, width=2.0):
        screen = [
            self._screen_from_enu(x, y)
            for x, y in points
            if math.isfinite(float(x)) and math.isfinite(float(y))
        ]
        screen = [point for point in screen if point is not None]
        if len(screen) >= 2:
            pen = QtGui.QPen(QtGui.QColor(color), width, QtCore.Qt.DashLine)
            painter.setPen(pen)
            painter.drawPolyline(QtGui.QPolygonF(screen))

    def _path_points(self, key):
        message = self.snapshot.get(key)
        if message is None:
            return []
        return [(p.pose.position.x, p.pose.position.y) for p in message.poses]

    def _draw_map_layers(self, painter):
        scatter = self.snapshot.get("raw_scatter", [])
        scatter = [
            point for point in scatter
            if len(point) >= 2 and math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        ]
        if self.raw_trail and len(scatter) > 1:
            painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 80, 120), 1.0))
            painter.drawPolyline(
                QtGui.QPolygonF([self._screen_from_latlon(*point) for point in scatter])
            )
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(0, 255, 80, 95))
        for point in scatter:
            painter.drawEllipse(self._screen_from_latlon(*point), 2.5, 2.5)

        gps = self.snapshot.get("gps")
        gps_valid = (
            gps is not None
            and getattr(gps.status, "status", -1) >= NavSatStatus.STATUS_FIX
            and math.isfinite(gps.latitude)
            and math.isfinite(gps.longitude)
        )
        if gps_valid:
            current = self._screen_from_latlon(gps.latitude, gps.longitude)
            painter.setBrush(QtGui.QColor("#00ff50"))
            painter.drawEllipse(current, 5.0, 5.0)
            covariance = gps.position_covariance
            if (
                len(covariance) >= 9
                and math.isfinite(covariance[0])
                and math.isfinite(covariance[4])
                and covariance[0] > 0.0
                and covariance[4] > 0.0
            ):
                meters_per_pixel = (
                    math.cos(math.radians(gps.latitude))
                    * 2.0
                    * math.pi
                    * EARTH_RADIUS
                    / (TILE_SIZE * (2 ** self.zoom))
                )
                painter.setBrush(QtGui.QColor(0, 255, 80, 30))
                painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 80, 140), 1.0))
                painter.drawEllipse(
                    current,
                    math.sqrt(covariance[0]) / meters_per_pixel,
                    math.sqrt(covariance[4]) / meters_per_pixel,
                )

        self._draw_polyline(painter, self.snapshot.get("actual_path", []), "#ffd400", 2.5)
        self._draw_polyline(painter, self.snapshot.get("filtered_path", []), "#00e5ff", 1.8)
        mission_points = self._path_points("mission")
        pending_points = self._path_points("pending")
        display_waypoints = mission_points if mission_points else pending_points
        if mission_points:
            self._draw_polyline(painter, mission_points, "#2080ff", 3.0)
        else:
            self._draw_dashed_polyline(painter, pending_points, "#2080ff", 2.5)
        local_plan_message = self.snapshot.get("local_plan")
        if (
            local_plan_message is not None
            and local_plan_message.header.frame_id.lstrip("/") != "body"
        ):
            self._draw_polyline(painter, self._path_points("local_plan"), "#ff33dd", 3.0)

        filtered = self.snapshot.get("gps_filtered")
        if filtered is not None:
            if not all(math.isfinite(v) for v in (
                filtered.pose.position.x, filtered.pose.position.y
            )):
                point = None
            else:
                point = self._screen_from_enu(filtered.pose.position.x, filtered.pose.position.y)
            if point is not None:
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(QtGui.QColor("#00e5ff"))
                painter.drawEllipse(point, 4.0, 4.0)

        painter.setFont(QtGui.QFont("Sans", 9, QtGui.QFont.Bold))
        for index, (east, north) in enumerate(display_waypoints, 1):
            if not all(math.isfinite(v) for v in (east, north)):
                continue
            point = self._screen_from_enu(east, north)
            if point is None:
                continue
            painter.setBrush(QtGui.QColor("#2080ff"))
            painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1.0))
            painter.drawEllipse(point, 7.0, 7.0)
            painter.drawText(point + QtCore.QPointF(9, -8), str(index))

        debug = diagnostic_dict(self.snapshot.get("debug"))
        navigation = debug.get("Navigation", {})
        try:
            target_x = float(navigation.get("target_x", "nan"))
            target_y = float(navigation.get("target_y", "nan"))
            target = self._screen_from_enu(target_x, target_y)
            if target is not None and math.isfinite(target_x) and math.isfinite(target_y):
                painter.setBrush(QtCore.Qt.NoBrush)
                painter.setPen(QtGui.QPen(QtGui.QColor("#ff33dd"), 3.0))
                painter.drawEllipse(target, 11.0, 11.0)
        except ValueError:
            pass

        self._draw_robot_arrow(painter, self.snapshot.get("fastlio"), "#9da5ab", 0.55)
        self._draw_robot_arrow(painter, self.snapshot.get("fused"), "#ff3030", 1.0)
        self._draw_heading_calibration_overlay(painter)

        if self.waypoint_editing:
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2.0))
            painter.setBrush(QtGui.QColor(32, 128, 255, 210))
            banner = QtCore.QRectF(0.5 * self.width() - 85, 10, 170, 30)
            painter.drawRoundedRect(banner, 4, 4)
            painter.drawText(
                banner, int(QtCore.Qt.AlignCenter), tr("editing_mode", "WAYPOINT EDITING")
            )

        if self.heading_calibrating:
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffd400"), 2.0))
            painter.setBrush(QtGui.QColor(255, 212, 0, 210))
            banner = QtCore.QRectF(0.5 * self.width() - 95, 10, 190, 30)
            painter.drawRoundedRect(banner, 4, 4)
            painter.drawText(
                banner,
                int(QtCore.Qt.AlignCenter),
                tr("heading_calibration_mode", "HEADING CALIBRATION"),
            )

    def _draw_heading_calibration_overlay(self, painter):
        if not self.heading_calibrating or self.pending_heading_ref is None:
            return
        fused = self.snapshot.get("fused")
        if fused is None:
            return
        robot_point = self._screen_from_enu(
            fused.pose.pose.position.x,
            fused.pose.pose.position.y,
        )
        ref_point = self._screen_from_enu(*self.pending_heading_ref)
        if robot_point is None or ref_point is None:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("#ffd400"), 4.0))
        painter.drawLine(robot_point, ref_point)
        painter.setBrush(QtGui.QColor("#ff3030"))
        painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1.0))
        painter.drawEllipse(robot_point, 6.0, 6.0)
        painter.setBrush(QtGui.QColor("#ffd400"))
        painter.drawEllipse(ref_point, 7.0, 7.0)
        painter.drawText(ref_point + QtCore.QPointF(10, -8), "x")

    def _draw_robot_arrow(self, painter, odometry, color, alpha):
        if odometry is None:
            return
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        point = self._screen_from_enu(
            x, y
        )
        if point is None:
            return
        length = 22.0
        tip = arrow_tip_screen(point, yaw, length)
        pen_color = QtGui.QColor(color)
        pen_color.setAlphaF(alpha)
        painter.setPen(QtGui.QPen(pen_color, 4.0))
        painter.drawLine(point, tip)
        painter.setBrush(pen_color)
        painter.drawEllipse(point, 5.0, 5.0)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.drag_origin = event.pos()
            self.press_origin = event.pos()
            self.drag_distance = 0.0
            self.follow_robot = False

    def mouseMoveEvent(self, event):
        if self.drag_origin is None:
            return
        delta = event.pos() - self.drag_origin
        self.drag_distance += math.hypot(delta.x(), delta.y())
        self.center_world = (
            self.center_world[0] - delta.x(),
            self.center_world[1] - delta.y(),
        )
        self.drag_origin = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        release_distance = float("inf")
        if self.press_origin is not None:
            release_delta = event.pos() - self.press_origin
            release_distance = math.hypot(release_delta.x(), release_delta.y())
        gesture_distance = max(self.drag_distance, release_distance)
        if (
            self.waypoint_editing
            and event.button() == QtCore.Qt.LeftButton
            and self.press_origin is not None
            and gesture_distance <= self.drag_threshold
        ):
            waypoint = self.screen_to_map(event.pos())
            if waypoint is not None:
                self.waypoint_requested.emit(*waypoint)
        elif (
            self.heading_calibrating
            and event.button() == QtCore.Qt.LeftButton
            and self.press_origin is not None
            and gesture_distance <= self.drag_threshold
        ):
            waypoint = self.screen_to_map(event.pos())
            if waypoint is not None:
                self.heading_requested.emit(waypoint[0], waypoint[1])
        self.drag_origin = None
        self.press_origin = None
        self.drag_distance = 0.0

    def wheelEvent(self, event):
        new_zoom = self.zoom + (1 if event.angleDelta().y() > 0 else -1)
        new_zoom = max(self.min_zoom, min(self.max_zoom, new_zoom))
        if new_zoom == self.zoom:
            return
        factor = 2 ** (new_zoom - self.zoom)
        self.center_world = (
            self.center_world[0] * factor,
            self.center_world[1] * factor,
        )
        self.zoom = new_zoom
        self.update()


class LidarCanvas(QtWidgets.QWidget):
    def __init__(self, cloud_timeout=1.0, parent=None):
        super().__init__(parent)
        self.snapshot = {}
        self.range_x = (-2.0, 8.0)
        self.range_y = (-3.0, 3.0)
        self.cloud_timeout = max(0.1, float(cloud_timeout))
        self.setMinimumWidth(190)

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        self.update()

    def _screen(self, x, y):
        margin = 20.0
        scale = min(
            (self.width() - 2 * margin) / (self.range_y[1] - self.range_y[0]),
            (self.height() - 2 * margin) / (self.range_x[1] - self.range_x[0]),
        )
        return QtCore.QPointF(
            0.5 * self.width() - y * scale,
            self.height() - margin - (x - self.range_x[0]) * scale,
        )

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#101417"))
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self._draw_grid(painter)
        self._draw_boundaries(painter)
        self._draw_cloud(painter)
        self._draw_candidates(painter)
        self._draw_best_plan(painter)
        self._draw_robot(painter)
        painter.setPen(QtGui.QColor("#d7dde2"))
        painter.drawText(12, 22, tr("lidar_planner", "LIDAR / LOCAL PLANNER"))
        self._draw_status(painter)

    def _draw_status(self, painter):
        received = self.snapshot.get("received", {})
        cloud_stamp = received.get("cloud")
        cloud_age = None
        if cloud_stamp is not None:
            cloud_age = max(0.0, rospy.Time.now().to_sec() - cloud_stamp)
        lidar_ok = cloud_age is not None and cloud_age <= self.cloud_timeout
        point_count = self.snapshot.get("displayed_point_count", 0)
        raw_count = self.snapshot.get("raw_point_count", 0)
        finite_count = self.snapshot.get("finite_point_count", 0)
        height_count = self.snapshot.get("height_filtered_count", 0)
        range_count = self.snapshot.get("range_filtered_count", 0)
        tf_status = self.snapshot.get("cloud_tf_status", "FAILED")
        tf_count = self.snapshot.get("cloud_tf_fallback_count", 0)
        tf_delta = self.snapshot.get("cloud_tf_delta")
        planner = diagnostic_dict(self.snapshot.get("debug")).get("Planner", {})
        valid = planner.get("valid_candidates", "-")
        total = planner.get("total_candidates", "-")
        if UI_LANGUAGE == "zh_CN":
            age_text = "-" if cloud_age is None else "{:.0f} 毫秒".format(cloud_age * 1000.0)
            delta_text = "-" if tf_delta is None else "{:.0f} 毫秒".format(tf_delta * 1000.0)
            text = (
                "{}：{}\nRaw：{}\n有限 XYZ：{}\n高度过滤：{}\n距离过滤：{}\n显示：{}\n年龄：{}\nTF：{}\n回退次数：{}\n时间差：{}\n{}：{} {}/{}"
            ).format(
                tr("lidar", "LiDAR"), "正常" if lidar_ok else tr("no_data", "NO DATA"),
                raw_count, finite_count, height_count, range_count, point_count,
                age_text, tf_status, tf_count, delta_text,
                tr("planner", "Planner"), tr("valid", "valid"), valid, total,
            )
        else:
            age_text = "-" if cloud_age is None else "{:.0f} ms".format(cloud_age * 1000.0)
            delta_text = "-" if tf_delta is None else "{:.0f} ms".format(tf_delta * 1000.0)
            text = "LiDAR: {}\nRaw: {}\nFinite XYZ: {}\nHeight: {}\nRange: {}\nDisplayed: {}\nAge: {}\nTF: {}\nFallbacks: {}\nTF delta: {}\nPlanner: valid {}/{}".format(
                "OK" if lidar_ok else "NO DATA", raw_count, finite_count,
                height_count, range_count, point_count, age_text,
                tf_status, tf_count, delta_text, valid, total
            )
        painter.setPen(QtGui.QColor("#d7dde2"))
        painter.drawText(
            QtCore.QRectF(max(8, self.width() - 270), 35, min(260, self.width() - 16), 205),
            int(QtCore.Qt.AlignRight | QtCore.Qt.AlignTop),
            text,
        )

    def _draw_grid(self, painter):
        painter.setPen(QtGui.QPen(QtGui.QColor("#273038"), 1.0))
        for x in range(-2, 9):
            painter.drawLine(self._screen(x, -3), self._screen(x, 3))
        for y in range(-3, 4):
            painter.drawLine(self._screen(-2, y), self._screen(8, y))

    def _polygon(self, painter, points, color, width=2.0, fill=None):
        polygon = QtGui.QPolygonF([self._screen(x, y) for x, y in points])
        painter.setPen(QtGui.QPen(QtGui.QColor(color), width))
        painter.setBrush(QtCore.Qt.NoBrush if fill is None else QtGui.QColor(fill))
        painter.drawPolygon(polygon)

    def _draw_boundaries(self, painter):
        self._polygon(
            painter,
            [(-0.6, -0.25), (-0.6, 0.25), (0.6, 0.25), (0.6, -0.25)],
            "#ffffff",
            2.0,
            QtGui.QColor(255, 255, 255, 20),
        )
        self._polygon(
            painter,
            [(-0.65, -0.30), (-0.65, 0.30), (0.65, 0.30), (0.65, -0.30)],
            "#ff9d20",
            1.5,
        )
        self._polygon(
            painter,
            [(0.6, -0.4), (0.6, 0.4), (2.6, 0.4), (2.6, -0.4)],
            "#ffcf30",
            1.0,
            QtGui.QColor(255, 207, 48, 15),
        )
        safety = self.snapshot.get("safety")
        if (
            safety is not None
            and math.isfinite(float(safety.front_distance))
            and safety.front_distance >= 0.0
        ):
            point = self._screen(0.6 + safety.front_distance, 0.0)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ff4040"), 2.0))
            painter.drawLine(point + QtCore.QPointF(-15, 0), point + QtCore.QPointF(15, 0))

    def _draw_cloud(self, painter):
        painter.setPen(QtGui.QPen(QtGui.QColor(100, 205, 255, 175), 2.0))
        for x, y in self.snapshot.get("lidar_points", []):
            if self.range_x[0] <= x <= self.range_x[1] and self.range_y[0] <= y <= self.range_y[1]:
                painter.drawPoint(self._screen(float(x), float(y)))

    def _draw_candidates(self, painter):
        message = self.snapshot.get("candidates")
        if message is None:
            return
        for marker in message.markers:
            if marker.type != marker.LINE_STRIP or len(marker.points) < 2:
                continue
            color = QtGui.QColor.fromRgbF(
                marker.color.r, marker.color.g, marker.color.b, max(0.15, marker.color.a)
            )
            painter.setPen(QtGui.QPen(color, max(1.0, marker.scale.x * 35.0)))
            marker_points = [
                self._screen(point.x, point.y)
                for point in marker.points
                if all(math.isfinite(value) for value in (point.x, point.y))
            ]
            if len(marker_points) < 2:
                continue
            painter.drawPolyline(
                QtGui.QPolygonF(marker_points)
            )

    def _draw_best_plan(self, painter):
        local_plan = self.snapshot.get("local_plan")
        fused = self.snapshot.get("fused")
        if local_plan is None or len(local_plan.poses) < 2:
            return
        if local_plan.header.frame_id.lstrip("/") != "body" and fused is None:
            return
        points = []
        local_frame = local_plan.header.frame_id.lstrip("/")
        if local_frame == "body":
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in local_plan.poses
                if all(math.isfinite(value) for value in (
                    pose.pose.position.x, pose.pose.position.y
                ))
            ]
        else:
            robot_x = fused.pose.pose.position.x
            robot_y = fused.pose.pose.position.y
            yaw = yaw_from_quaternion(fused.pose.pose.orientation)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            for pose in local_plan.poses:
                dx = pose.pose.position.x - robot_x
                dy = pose.pose.position.y - robot_y
                if not all(math.isfinite(value) for value in (dx, dy)):
                    continue
                points.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))
        if len(points) < 2:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("#ff33dd"), 3.0))
        painter.drawPolyline(QtGui.QPolygonF([self._screen(x, y) for x, y in points]))

    def _draw_robot(self, painter):
        origin = self._screen(0.0, 0.0)
        forward = self._screen(0.8, 0.0)
        painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 3.0))
        painter.drawLine(origin, forward)


class StatusCard(QtWidgets.QFrame):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName("statusCard")
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum
        )
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 7)
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("cardTitle")
        self.body = QtWidgets.QLabel(tr("waiting", "WAITING"))
        self.body.setWordWrap(True)
        self.body.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.title)
        layout.addWidget(self.body)

    def set_text(self, text, state="neutral"):
        self.body.setText(text)
        colors = {
            "ok": "#37d67a",
            "warn": "#ffbf47",
            "error": "#ff5b5b",
            "neutral": "#c8d0d7",
        }
        self.body.setStyleSheet("color: {};".format(colors[state]))


class DashboardWindow(QtWidgets.QMainWindow):
    operation_result = QtCore.pyqtSignal(str, bool)

    def __init__(self, cache):
        super().__init__()
        self.cache = cache
        self.bag_process = None
        self.bag_started_at = None
        self.bag_output = None
        self.last_bag_output = None
        self.editor_state = "IDLE"
        self.heading_calibrating = False
        self.controller_dry_run = True
        self.available_services = set()
        self.service_buttons = {}
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.mission_patrol_mode = rospy.get_param("~mission_patrol_mode", "once")
        self.mission_loop_count = int(rospy.get_param("~mission_loop_count", 0))
        self.waypoint_topic = rospy.get_param(
            "~waypoint_goal_topic", "/open_nav/goal_add"
        )
        self.waypoint_pub = rospy.Publisher(
            self.waypoint_topic, PoseStamped, queue_size=10
        )
        tile_root = rospy.get_param(
            "~tile_root", "/root/catkin_ws/src/tron_open_space_nav/maps/wuhan_tiles"
        )
        self.bag_directory = rospy.get_param("~bag_directory", "/root/catkin_ws/bags")
        self.path_decimation = max(1, int(rospy.get_param("~path_decimation", 2)))
        self.setWindowTitle(tr("window_title", "TRON1 Navigation Dashboard"))
        self.setMinimumSize(900, 600)
        self._initial_width = int(rospy.get_param("~window_width", 1580))
        self._initial_height = int(rospy.get_param("~window_height", 900))
        self._build_ui(tile_root)
        self._apply_style()
        self._setup_timers()
        self.operation_result.connect(self._show_operation_result)

    def fit_to_screen(self):
        """Choose a usable initial size without preventing later resizing."""
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self.resize(self._initial_width, self._initial_height)
            return
        available = screen.availableGeometry()
        width = min(self._initial_width, max(900, available.width() - 24))
        height = min(self._initial_height, max(600, available.height() - 48))
        self.resize(width, height)
        self.move(
            available.x() + max(0, (available.width() - width) // 2),
            available.y() + max(0, (available.height() - height) // 2),
        )

    def _build_ui(self, tile_root):
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(7)
        columns = QtWidgets.QHBoxLayout()
        columns.setSpacing(7)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(7)
        self.cards = {}
        card_titles = {
            "GNSS": tr("card_gnss", "GNSS"),
            "FAST-LIO": tr("card_fastlio", "FAST-LIO"),
            "FUSION": tr("card_fusion", "FUSION"),
            "PLANNER": tr("card_planner", "PLANNER"),
            "SAFETY": tr("card_safety", "SAFETY"),
            "CONTROLLER": tr("card_controller", "CONTROLLER"),
        }
        for key, title in card_titles.items():
            card = StatusCard(title)
            self.cards[key] = card
            left_layout.addWidget(card)
        self._add_controls(left_layout)
        left_layout.addStretch(1)

        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setObjectName("leftScroll")
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left_scroll.setMinimumWidth(300)
        left_scroll.setWidget(left)

        center = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        toolbar = QtWidgets.QHBoxLayout()
        center_button = QtWidgets.QPushButton(tr("center", "Center"))
        center_button.clicked.connect(self._center_map)
        self.follow_button = QtWidgets.QPushButton(tr("follow_robot", "Follow robot"))
        self.follow_button.setCheckable(True)
        self.follow_button.toggled.connect(self._follow_changed)
        self.raw_trail_button = QtWidgets.QCheckBox(tr("raw_gnss_trail", "Raw GNSS trail"))
        self.raw_trail_button.toggled.connect(self._raw_trail_changed)
        toolbar.addWidget(center_button)
        toolbar.addWidget(self.follow_button)
        toolbar.addWidget(self.raw_trail_button)
        toolbar.addStretch(1)
        self.map_canvas = MapCanvas(
            tile_root,
            drag_threshold=float(rospy.get_param("~waypoint_drag_threshold_px", 6.0)),
        )
        self.map_canvas.waypoint_requested.connect(self._publish_waypoint)
        self.map_canvas.heading_requested.connect(self._heading_reference_clicked)
        center_layout.addLayout(toolbar)
        center_layout.addWidget(self.map_canvas, 1)

        self.lidar_canvas = LidarCanvas(
            cloud_timeout=float(rospy.get_param("~lidar_status_timeout", 1.0))
        )
        columns.addWidget(left_scroll, 19)
        columns.addWidget(center, 57)
        columns.addWidget(self.lidar_canvas, 24)
        root.addLayout(columns, 1)

        self.telemetry = QtWidgets.QLabel(
            tr("waiting_debug", "Waiting for /open_nav/navigation_debug")
        )
        self.telemetry.setObjectName("telemetry")
        self.telemetry.setMinimumHeight(44)
        root.addWidget(self.telemetry)
        self.setCentralWidget(central)

    def _add_controls(self, layout):
        label = QtWidgets.QLabel(tr("operations", "OPERATIONS"))
        label.setObjectName("sectionTitle")
        layout.addWidget(label)
        self.editor_state_label = QtWidgets.QLabel(
            tr("mission_state", "Mission state: {}").format(
                translated_state(self.editor_state)
            )
        )
        self.editor_state_label.setObjectName("missionState")
        layout.addWidget(self.editor_state_label)

        grid = QtWidgets.QGridLayout()
        self.start_edit_button = QtWidgets.QPushButton(tr("start_editing", "Start editing"))
        self.finish_edit_button = QtWidgets.QPushButton(tr("finish_editing", "Finish editing"))
        self.undo_button = QtWidgets.QPushButton(tr("undo_waypoint", "Undo last point"))
        self.clear_button = QtWidgets.QPushButton(tr("clear_mission", "Clear mission"))
        self.confirm_button = QtWidgets.QPushButton(tr("confirm_mission", "Confirm mission"))
        self.pause_button = QtWidgets.QPushButton(tr("pause_mission", "Pause mission"))
        self.resume_button = QtWidgets.QPushButton(tr("resume_mission", "Resume mission"))
        self.reset_button = QtWidgets.QPushButton(
            tr("reset_localization", "Reset localization stats")
        )
        self.heading_button = QtWidgets.QPushButton(tr("calibrate_heading", "Calibrate heading"))
        self.clear_heading_button = QtWidgets.QPushButton(tr("clear_heading", "Clear heading"))
        self.heading_button.clicked.connect(self._start_heading_calibration)
        self.clear_heading_button.clicked.connect(self._clear_heading_alignment)

        self.start_edit_button.clicked.connect(self._start_waypoint_editing)
        self.finish_edit_button.clicked.connect(self._finish_waypoint_editing)
        self.undo_button.clicked.connect(self._undo_waypoint)
        self.clear_button.clicked.connect(
            lambda _checked=False: self._call_service("/open_nav/mission/clear", Empty)
        )
        self.confirm_button.clicked.connect(self._confirm_and_start_mission)
        self.pause_button.clicked.connect(
            lambda _checked=False: self._call_service("/open_nav/mission/pause", Empty)
        )
        self.resume_button.clicked.connect(
            lambda _checked=False: self._call_service("/open_nav/mission/resume", Empty)
        )
        self.reset_button.clicked.connect(
            lambda _checked=False: self._call_service(
                "/open_nav/localization/reset_distance", ResetLocalizationDistance
            )
        )

        buttons = [
            self.start_edit_button,
            self.finish_edit_button,
            self.undo_button,
            self.clear_button,
            self.confirm_button,
            self.pause_button,
            self.resume_button,
            self.reset_button,
            self.heading_button,
            self.clear_heading_button,
        ]
        for index, button in enumerate(buttons):
            button.setEnabled(False)
            button.setMinimumHeight(32)
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
            )
            grid.addWidget(button, index // 2, index % 2)
        self.service_buttons = {
            "/open_nav/mission/delete_waypoint": self.undo_button,
            "/open_nav/mission/clear": self.clear_button,
            "/open_nav/mission/pause": self.pause_button,
            "/open_nav/mission/resume": self.resume_button,
            "/open_nav/localization/reset_distance": self.reset_button,
            "/open_nav/heading/set_alignment": self.heading_button,
            "/open_nav/heading/clear_alignment": self.clear_heading_button,
        }
        layout.addLayout(grid)

        self.mission_feedback = QtWidgets.QLabel("")
        self.mission_feedback.setWordWrap(True)
        layout.addWidget(self.mission_feedback)

        bag_row = QtWidgets.QHBoxLayout()
        self.start_bag_button = QtWidgets.QPushButton(tr("start_rosbag", "Start rosbag"))
        self.stop_bag_button = QtWidgets.QPushButton(tr("stop_rosbag", "Stop rosbag"))
        self.stop_bag_button.setEnabled(False)
        self.start_bag_button.clicked.connect(self._start_bag)
        self.stop_bag_button.clicked.connect(self._stop_bag)
        bag_tooltip = tr("bag_tooltip", "Record navigation topics for offline debugging.")
        self.start_bag_button.setToolTip(bag_tooltip)
        self.stop_bag_button.setToolTip(bag_tooltip)
        bag_row.addWidget(self.start_bag_button)
        bag_row.addWidget(self.stop_bag_button)
        layout.addLayout(bag_row)
        self.bag_status = QtWidgets.QLabel(tr("bag_idle", "Not recording"))
        self.bag_status.setWordWrap(True)
        self.bag_status.setToolTip(bag_tooltip)
        layout.addWidget(self.bag_status)

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #171b1f; color: #d9e0e6; font-size: 12px; }
            QScrollArea#leftScroll { background: transparent; border: none; }
            QScrollArea#leftScroll > QWidget > QWidget { background: #171b1f; }
            QFrame#statusCard { background: #22282d; border: 1px solid #343d44; border-radius: 5px; }
            QLabel#cardTitle, QLabel#sectionTitle { color: #8fa3b3; font-weight: bold; }
            QLabel#missionState { color: #62b5ff; font-weight: bold; padding: 3px; }
            QLabel#telemetry { background: #22282d; border: 1px solid #343d44; padding: 8px; }
            QPushButton { background: #303941; border: 1px solid #46525b; border-radius: 4px; padding: 6px; }
            QPushButton:hover { background: #3b4852; }
            QPushButton:disabled { color: #65717a; background: #24292d; }
            QPushButton:checked { background: #176a56; }
            """
        )

    def _setup_timers(self):
        map_hz = float(rospy.get_param("~map_refresh_hz", 8.0))
        lidar_hz = float(rospy.get_param("~lidar_refresh_hz", 5.0))
        status_hz = float(rospy.get_param("~status_refresh_hz", 8.0))
        self.map_timer = QtCore.QTimer(self)
        self.map_timer.timeout.connect(self._refresh_map)
        self.map_timer.start(int(1000.0 / max(map_hz, 1.0)))
        self.lidar_timer = QtCore.QTimer(self)
        self.lidar_timer.timeout.connect(self._refresh_lidar)
        self.lidar_timer.start(int(1000.0 / max(lidar_hz, 1.0)))
        self.status_timer = QtCore.QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(int(1000.0 / max(status_hz, 1.0)))
        self.service_timer = QtCore.QTimer(self)
        self.service_timer.timeout.connect(self._refresh_services)
        self.service_timer.start(2000)

    def _origin(self):
        if not rospy.get_param("/tron_open_space_nav/origin_ready", False):
            return None
        return (
            float(rospy.get_param("/tron_open_space_nav/origin_lat")),
            float(rospy.get_param("/tron_open_space_nav/origin_lon")),
        )

    def _refresh_map(self):
        snapshot = self.cache.snapshot()
        actual = snapshot.get("actual_path", [])
        snapshot["actual_path"] = actual[:: self.path_decimation]
        self.map_canvas.set_snapshot(snapshot, self._origin())

    def _refresh_lidar(self):
        self.lidar_canvas.set_snapshot(self.cache.snapshot())

    @staticmethod
    def _number(section, key, default="nan"):
        return section.get(key, default)

    def _refresh_status(self):
        snapshot = self.cache.snapshot()
        self._sync_mission_state(snapshot)
        debug = diagnostic_dict(snapshot.get("debug"))
        gnss = debug.get("GNSS", {})
        localization = debug.get("Localization", {})
        navigation = debug.get("Navigation", {})
        planner = debug.get("Planner", {})
        safety = debug.get("Safety", {})
        control = debug.get("Control", {})
        gps_ok = gnss.get("fix_status") == "FIX" and float(gnss.get("message_age", 99)) < 2.0
        self.cards["GNSS"].set_text(
            ("{}  数据年龄 {} 秒\n纬度 {}\n经度 {}\n东/北标准差 {} / {} 米" if UI_LANGUAGE == "zh_CN" else
             "{}  age {}s\nlat {}\nlon {}\nsigma E/N {} / {} m").format(
                translated_state(gnss.get("fix_status", "NO_FIX")),
                gnss.get("message_age", "-"),
                gnss.get("latitude", "-"),
                gnss.get("longitude", "-"),
                gnss.get("sigma_east", "-"),
                gnss.get("sigma_north", "-"),
            ),
            "ok" if gps_ok else "warn",
        )
        fast_age = float(localization.get("fastlio_age", 99))
        fast_fresh = 0.0 <= fast_age < 0.5
        self.cards["FAST-LIO"].set_text(
            ("{}  {:.1f} Hz\n位置 x/y {} / {}\n航向角 {} 度" if UI_LANGUAGE == "zh_CN" else
             "{}  {:.1f} Hz\nx/y {} / {}\nyaw {} deg").format(
                translated_state("OK" if fast_fresh else "STALE"),
                snapshot.get("odom_hz", 0.0),
                localization.get("fastlio_x", "-"),
                localization.get("fastlio_y", "-"),
                localization.get("fastlio_yaw_deg", "-"),
            ),
            "ok" if fast_fresh else "warn",
        )
        fused_age = float(localization.get("fused_age", 99))
        fused_fresh = 0.0 <= fused_age < 0.5
        heading_msg = snapshot.get("heading")
        if heading_msg is not None and heading_msg.aligned:
            heading_state = tr("heading_aligned", "ALIGNED")
            heading_source = heading_msg.source or "MANUAL"
            heading_ref = "{:.1f} m".format(heading_msg.calibration_reference_distance)
        else:
            heading_state = tr("heading_unaligned", "UNALIGNED")
            heading_source = "UNALIGNED"
            heading_ref = "-"
        raw_fastlio_yaw = (
            math.degrees(heading_msg.raw_fastlio_yaw)
            if heading_msg is not None else 0.0
        )
        nav_fastlio_yaw = (
            math.degrees(heading_msg.normalized_fastlio_yaw)
            if heading_msg is not None else 0.0
        )
        self.cards["FUSION"].set_text(
            ("{}  数据年龄 {:.2f} 秒\n位置 x/y {} / {}\n航向角 {} 度\n航向状态 {}\nFAST-LIO(raw) {:.1f}°\nFAST-LIO(nav) {:.1f}°  Offset {:.1f}°\n全局 {:.1f}°\n标定来源 {}  参考距离 {}"
             if UI_LANGUAGE == "zh_CN" else
             "{}  age {:.2f}s\nx/y {} / {}\nyaw {} deg\nheading {}\nFAST-LIO(raw) {:.1f}°\nFAST-LIO(nav) {:.1f}° offset {:.1f}°\nglobal {:.1f}°\nsource {} ref {}").format(
                translated_state("OK" if fused_fresh else "STALE"),
                fused_age,
                localization.get("fused_x", "-"),
                localization.get("fused_y", "-"),
                localization.get("fused_yaw_deg", "-"),
                heading_state,
                raw_fastlio_yaw,
                nav_fastlio_yaw,
                math.degrees(heading_msg.yaw_offset) if heading_msg is not None else 0.0,
                math.degrees(heading_msg.global_yaw) if heading_msg is not None else 0.0,
                heading_source,
                heading_ref,
            ),
            "ok" if fused_fresh and heading_msg is not None and heading_msg.aligned else "warn",
        )
        valid = planner.get("valid_candidates", "0")
        total = planner.get("total_candidates", "0")
        self.cards["PLANNER"].set_text(
            ("影子模式  {}\n有效候选 {}/{}\n最优速度 v/w {} / {}\n净空 {}  评分 {}" if UI_LANGUAGE == "zh_CN" else
             "SHADOW  {}\nvalid {}/{}\nbest v/w {} / {}\nclearance {}  score {}").format(
                translated_state(planner.get("state", "WAITING")),
                valid,
                total,
                planner.get("best_v", "-"),
                planner.get("best_w", "-"),
                planner.get("best_clearance", "-"),
                planner.get("best_score", "-"),
            ),
            "ok" if planner.get("state") == "OK" else "warn",
        )
        safety_state = safety.get("state", "UNKNOWN")
        self.cards["SAFETY"].set_text(
            ("{}\n前方距离 {} 米\n碰撞风险 {}\n点云年龄 {} 秒" if UI_LANGUAGE == "zh_CN" else
             "{}\nfront {} m\nrisk {}\ncloud age {}s").format(
                translated_state(safety_state),
                safety.get("front_distance", "-"),
                safety.get("collision_risk", "-"),
                safety.get("pointcloud_age", "-"),
            ),
            "ok" if safety_state == "FREE" else ("warn" if safety_state == "SLOW" else "error"),
        )
        dry_run = control.get("controller_dry_run", "true") == "true"
        self.controller_dry_run = dry_run
        self.cards["CONTROLLER"].set_text(
            ("{}\n仿真输出={}" if UI_LANGUAGE == "zh_CN" else "{}\ndry_run={}").format(
                "测试模式（TEST）" if dry_run and UI_LANGUAGE == "zh_CN" else
                ("实机模式（REAL）" if UI_LANGUAGE == "zh_CN" else ("TEST" if dry_run else "REAL")),
                str(dry_run).lower(),
            ),
            "warn" if dry_run else "error",
        )
        telemetry_format = (
            "目标方位 {bearing} 度 | 融合航向 {yaw} 度 | 航向误差 {error} 度 | "
            "GNSS 航迹角 {course}（{valid}） | 横向误差 {cte} 米 | 目标距离 {goal} 米 | "
            "原始速度 {rv}/{rw} | 最终速度 {fv}/{fw}"
            if UI_LANGUAGE == "zh_CN"
            else
            "bearing {bearing} deg | fused yaw {yaw} deg | heading error {error} deg | "
            "GNSS course {course} ({valid}) | CTE {cte} m | goal {goal} m | "
            "raw cmd {rv}/{rw} | final cmd {fv}/{fw}"
        )
        self.telemetry.setText(
            telemetry_format.format(
                bearing=navigation.get("desired_bearing_deg", "-"),
                yaw=localization.get("fused_yaw_deg", "-"),
                error=navigation.get("heading_error_deg", "-"),
                course=localization.get("gnss_course_deg", "-"),
                valid=("运动中有效" if UI_LANGUAGE == "zh_CN" else "moving")
                if localization.get("gnss_course_valid") == "true"
                else ("无效或速度过低" if UI_LANGUAGE == "zh_CN" else "invalid/slow"),
                cte=navigation.get("cross_track_error", "-"),
                goal=navigation.get("distance_to_goal", "-"),
                rv=control.get("cmd_raw_linear", "-"),
                rw=control.get("cmd_raw_angular", "-"),
                fv=control.get("cmd_final_linear", "-"),
                fw=control.get("cmd_final_angular", "-"),
            )
        )
        self._refresh_bag_status()

    def _pending_count(self, snapshot=None):
        data = self.cache.snapshot() if snapshot is None else snapshot
        pending = data.get("pending")
        return 0 if pending is None else len(pending.poses)

    def _sync_mission_state(self, snapshot):
        status = snapshot.get("mission_status")
        runtime_state = "" if status is None else status.state.upper()
        if runtime_state == "RUNNING":
            self.editor_state = "RUNNING"
        elif runtime_state == "PAUSED":
            self.editor_state = "PAUSED"
        elif self.editor_state != "EDITING":
            self.editor_state = "READY" if self._pending_count(snapshot) > 0 else "IDLE"
        self.map_canvas.set_waypoint_editing(self.editor_state == "EDITING")
        self.editor_state_label.setText(
            tr("mission_state", "Mission state: {}").format(
                translated_state(self.editor_state)
            )
        )
        self._update_operation_buttons(snapshot)

    def _update_operation_buttons(self, snapshot=None):
        pending_count = self._pending_count(snapshot)
        editable = self.editor_state not in ("RUNNING", "PAUSED")
        collector_ready = self.waypoint_pub.get_num_connections() > 0
        origin_ready = self._origin() is not None
        self.start_edit_button.setEnabled(
            editable and self.editor_state != "EDITING" and collector_ready and origin_ready
        )
        self.finish_edit_button.setEnabled(self.editor_state == "EDITING")
        self.undo_button.setEnabled(
            editable
            and pending_count > 0
            and "/open_nav/mission/delete_waypoint" in self.available_services
        )
        self.clear_button.setEnabled(
            "/open_nav/mission/clear" in self.available_services
            and (pending_count > 0 or self.editor_state in ("RUNNING", "PAUSED"))
        )
        self.confirm_button.setEnabled(
            self.editor_state == "READY"
            and pending_count > 0
            and "/open_nav/mission/send" in self.available_services
            and "/open_nav/mission/start_patrol" in self.available_services
        )
        self.pause_button.setEnabled(
            self.editor_state == "RUNNING"
            and "/open_nav/mission/pause" in self.available_services
        )
        self.resume_button.setEnabled(
            self.editor_state == "PAUSED"
            and "/open_nav/mission/resume" in self.available_services
        )
        self.reset_button.setEnabled(
            "/open_nav/localization/reset_distance" in self.available_services
        )

    def _start_waypoint_editing(self):
        if self._origin() is None:
            self._show_operation_result(tr("origin_unavailable", "Navigation origin unavailable"), True)
            return
        if self.editor_state in ("RUNNING", "PAUSED"):
            return
        self.editor_state = "EDITING"
        self.map_canvas.set_waypoint_editing(True)
        self._show_operation_result(tr("editing_mode", "Waypoint editing mode"), False)
        self._sync_mission_state(self.cache.snapshot())

    def _start_heading_calibration(self):
        if self._origin() is None:
            self._show_operation_result("无法标定：导航原点尚未建立", True)
            return
        if "/open_nav/heading/set_alignment" not in self.available_services:
            self._show_operation_result("航向标定服务不可用", True)
            return
        if self.editor_state == "EDITING":
            self._show_operation_result(tr("finish_first", "Finish editing first"), True)
            return
        self.heading_calibrating = True
        self.map_canvas.set_heading_calibrating(True)
        self._show_operation_result(tr("heading_hint", "请点击机器人当前正前方尽可能远的位置，建议20m以上。"), False)

    def _clear_heading_alignment(self):
        if "/open_nav/heading/clear_alignment" not in self.available_services:
            self._show_operation_result("清除航向标定服务不可用", True)
            return
        self._call_service("/open_nav/heading/clear_alignment", Trigger)
        self.heading_calibrating = False
        self.map_canvas.set_heading_calibrating(False)

    def _heading_distance_warning(self, reference_distance):
        if reference_distance < 10.0:
            return "参考点距离过短，航向误差可能较大。"
        if reference_distance < 20.0:
            return "可以使用，但建议选择 20m 以上参考点。"
        return ""

    def _heading_reference_clicked(self, east, north):
        if not self.heading_calibrating:
            return
        snapshot = self.cache.snapshot()
        fused = snapshot.get("fused")
        if fused is None:
            self._show_operation_result("等待融合定位后再标定", True)
            return
        rx = fused.pose.pose.position.x
        ry = fused.pose.pose.position.y
        heading_status = snapshot.get("heading")
        if heading_status is None:
            self._show_operation_result("等待标准化 FAST-LIO 航向后再标定", True)
            return
        reference_distance = math.hypot(east - rx, north - ry)
        global_heading = math.atan2(north - ry, east - rx)
        fastlio_yaw = heading_status.normalized_fastlio_yaw
        offset = wrap_to_pi(global_heading - fastlio_yaw)
        predicted_global_yaw = wrap_to_pi(fastlio_yaw + offset)
        warning = self._heading_distance_warning(reference_distance)
        self.map_canvas.set_pending_heading_ref(east, north)
        text = (
            "FAST-LIO yaw: {:.1f}°\n地图指定全局方向: {:.1f}°\n计算 Yaw Offset: {:.1f}°\n"
            "应用后 Global Yaw: {:.1f}°\n参考点距离: {:.1f} m\n{}"
        ).format(
            math.degrees(fastlio_yaw),
            math.degrees(global_heading),
            math.degrees(offset),
            math.degrees(predicted_global_yaw),
            reference_distance,
            warning,
        )
        answer = QtWidgets.QMessageBox.warning(
            self,
            "确认航向标定",
            text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            self.map_canvas.set_pending_heading_ref(None, None)
            return
        try:
            service = rospy.ServiceProxy("/open_nav/heading/set_alignment", SetHeadingAlignment)
            response = service(offset, reference_distance, "MANUAL")
            self._show_operation_result(response.message, not response.success)
        except Exception as exc:
            self._show_operation_result(str(exc), True)
        finally:
            self.heading_calibrating = False
            self.map_canvas.set_heading_calibrating(False)

    def _finish_waypoint_editing(self):
        if self.editor_state != "EDITING":
            return
        self.editor_state = "READY" if self._pending_count() > 0 else "IDLE"
        self.map_canvas.set_waypoint_editing(False)
        self._sync_mission_state(self.cache.snapshot())

    def _publish_waypoint(self, east, north, latitude, longitude):
        if self.editor_state != "EDITING":
            return
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = east
        goal.pose.position.y = north
        goal.pose.orientation.w = 1.0
        self.waypoint_pub.publish(goal)
        self._show_operation_result(
            "已发送航点：x={:.2f}, y={:.2f}，纬度={:.7f}，经度={:.7f}".format(
                east, north, latitude, longitude
            ) if UI_LANGUAGE == "zh_CN" else
            "Waypoint sent: x={:.2f}, y={:.2f}, lat={:.7f}, lon={:.7f}".format(
                east, north, latitude, longitude
            ),
            False,
        )

    def _undo_waypoint(self):
        count = self._pending_count()
        if count <= 0:
            self._show_operation_result(tr("no_waypoint", "No waypoint available"), True)
            return

        def worker():
            try:
                rospy.wait_for_service("/open_nav/mission/delete_waypoint", timeout=1.0)
                response = rospy.ServiceProxy(
                    "/open_nav/mission/delete_waypoint", DeleteWaypoint
                )(count - 1)
                self.operation_result.emit(response.message, not response.success)
            except Exception as exc:
                self.operation_result.emit(str(exc), True)

        threading.Thread(target=worker, daemon=True).start()

    def _confirm_and_start_mission(self):
        if self.editor_state == "EDITING":
            self._show_operation_result(tr("finish_first", "Finish editing first"), True)
            return
        if self.editor_state != "READY" or self._pending_count() <= 0:
            self._show_operation_result(tr("no_waypoint", "No waypoint available"), True)
            return
        heading = self.cache.snapshot().get("heading")
        if heading is None or not heading.aligned:
            if not self.controller_dry_run:
                QtWidgets.QMessageBox.warning(
                    self,
                    "航向未标定",
                    tr("heading_blocked_real", "Heading is not aligned. Calibrate heading first."),
                )
                return
            answer = QtWidgets.QMessageBox.warning(
                self,
                "航向未标定",
                tr("heading_warn_test", "Heading is not aligned. TEST mode only."),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        answer = QtWidgets.QMessageBox.question(
            self,
            tr("confirm_start_title", "Confirm and start navigation"),
            tr("confirm_start_text", "Confirm the route and start navigation?"),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return

        def worker():
            try:
                rospy.wait_for_service("/open_nav/mission/send", timeout=2.0)
                sent = rospy.ServiceProxy("/open_nav/mission/send", Trigger)()
                if not sent.success:
                    self.operation_result.emit(sent.message, True)
                    return
                rospy.wait_for_service("/open_nav/mission/start_patrol", timeout=2.0)
                started = rospy.ServiceProxy(
                    "/open_nav/mission/start_patrol", MissionPatrolStart
                )(self.mission_patrol_mode, self.mission_loop_count)
                self.operation_result.emit(started.message, not started.success)
            except Exception as exc:
                self.operation_result.emit(str(exc), True)

        threading.Thread(target=worker, daemon=True).start()

    def _show_operation_result(self, message, error=False):
        self.mission_feedback.setText(message)
        self.mission_feedback.setStyleSheet(
            "color: {};".format("#ff5b5b" if error else "#37d67a")
        )

    def _refresh_services(self):
        try:
            services = rosgraph.Master(rospy.get_name()).getSystemState()[2]
            service_names = {name for name, _providers in services}
        except Exception:
            service_names = set()
        self.available_services = service_names
        for service, button in self.service_buttons.items():
            available = service in self.available_services
            button.setToolTip(
                tr("calls_service", "Calls {}").format(service)
                if available
                else tr("disabled_service", "Disabled: service unavailable ({})").format(service)
            )
        self.confirm_button.setToolTip(
            "调用 /open_nav/mission/send，然后调用 /open_nav/mission/start_patrol"
            if UI_LANGUAGE == "zh_CN"
            else "Calls mission/send, then mission/start_patrol"
        )
        self._update_operation_buttons()

    def _call_service(self, service_name, service_type):
        def worker():
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
                response = rospy.ServiceProxy(service_name, service_type)()
                rospy.loginfo("[nav_dashboard] %s -> %s", service_name, response)
                message = getattr(response, "message", "")
                if not message:
                    message = (
                        "服务调用成功：{}".format(service_name)
                        if UI_LANGUAGE == "zh_CN"
                        else "Service call succeeded: {}".format(service_name)
                    )
                success = getattr(response, "success", True)
                self.operation_result.emit(message, not success)
            except Exception as exc:
                rospy.logwarn("[nav_dashboard] service %s failed: %s", service_name, exc)
                self.operation_result.emit(str(exc), True)

        threading.Thread(target=worker, daemon=True).start()

    def _center_map(self):
        self.map_canvas.center_on_robot()

    def _follow_changed(self, enabled):
        self.map_canvas.follow_robot = enabled
        if enabled:
            self.map_canvas.center_on_robot()

    def _raw_trail_changed(self, enabled):
        self.map_canvas.raw_trail = enabled
        self.map_canvas.update()

    def _start_bag(self):
        if self.bag_process is not None and self.bag_process.poll() is None:
            return
        os.makedirs(self.bag_directory, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output = os.path.join(self.bag_directory, "tron_nav_{}".format(stamp))
        command = ["rosbag", "record", "-O", output] + BAG_TOPICS
        try:
            self.bag_process = subprocess.Popen(
                command, preexec_fn=os.setsid, stdout=subprocess.DEVNULL
            )
            self.bag_started_at = datetime.datetime.now()
            self.bag_output = output + ".bag"
            self.last_bag_output = self.bag_output
            self.start_bag_button.setEnabled(False)
            self.stop_bag_button.setEnabled(True)
            rospy.loginfo("[nav_dashboard] recording bag: %s.bag", output)
        except OSError as exc:
            rospy.logerr("[nav_dashboard] unable to start rosbag: %s", exc)

    def _stop_bag(self):
        if self.bag_process is not None and self.bag_process.poll() is None:
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            try:
                self.bag_process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
            rospy.loginfo("[nav_dashboard] rosbag stopped")
        self.bag_process = None
        self.bag_started_at = None
        self.bag_output = None
        self.start_bag_button.setEnabled(True)
        self.stop_bag_button.setEnabled(False)

    def _refresh_bag_status(self):
        running = self.bag_process is not None and self.bag_process.poll() is None
        if running and self.bag_started_at is not None:
            elapsed = max(0, int((datetime.datetime.now() - self.bag_started_at).total_seconds()))
            hours, remainder = divmod(elapsed, 3600)
            minutes, seconds = divmod(remainder, 60)
            filename = os.path.basename(self.bag_output or "-")
            self.bag_status.setText(
                "{}\n{:02d}:{:02d}:{:02d}\n{}".format(
                    tr("bag_recording", "Recording"),
                    hours,
                    minutes,
                    seconds,
                    tr("bag_file", "File: {}").format(filename),
                )
            )
            self.bag_status.setStyleSheet("color: #ff5b5b;")
            return
        if self.bag_process is not None:
            self.bag_process = None
            self.bag_started_at = None
            self.bag_output = None
            self.start_bag_button.setEnabled(True)
            self.stop_bag_button.setEnabled(False)
        text = tr("bag_idle", "Not recording")
        if self.last_bag_output:
            text += "\n" + tr("bag_last_file", "Last file: {}").format(
                os.path.basename(self.last_bag_output)
            )
        self.bag_status.setText(text)
        self.bag_status.setStyleSheet("color: #c8d0d7;")

    def closeEvent(self, event):
        self._stop_bag()
        event.accept()


def main():
    global UI_LANGUAGE
    rospy.init_node("tron_nav_dashboard", disable_signals=True)
    UI_LANGUAGE = str(rospy.get_param("~language", "zh_CN"))
    app = QtWidgets.QApplication(sys.argv)
    available_families = set(QtGui.QFontDatabase().families())
    for family in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei", "Sans"):
        if family in available_families:
            app.setFont(QtGui.QFont(family, 10))
            break
    cache = RosCache(
        max_path_points=int(rospy.get_param("~max_path_points", 3000)),
        scatter_points=int(rospy.get_param("~raw_scatter_points", 200)),
        lidar_max_points=int(rospy.get_param("~lidar_max_points", 3500)),
        voxel_size=float(rospy.get_param("~lidar_voxel_size", 0.10)),
        lidar_display_frame=rospy.get_param("~lidar_display_frame", "body"),
        lidar_min_z=float(rospy.get_param("~lidar_min_z", -0.4)),
        lidar_max_z=float(rospy.get_param("~lidar_max_z", 1.0)),
        lidar_min_range=float(rospy.get_param("~lidar_min_range", 0.1)),
        lidar_max_range=float(rospy.get_param("~lidar_max_range", 8.0)),
    )
    window = DashboardWindow(cache)
    window.fit_to_screen()
    window.show()
    exit_code = app.exec_()
    rospy.signal_shutdown("dashboard closed")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
