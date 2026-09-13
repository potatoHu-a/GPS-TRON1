#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import uuid

import rospy
import websocket
from geometry_msgs.msg import Twist


class TronControllerBridge(object):
    """Step 6: /open_nav/cmd_vel -> TRON WebSocket request_twist (30Hz)."""

    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/open_nav/cmd_vel")
        self.ws_url = rospy.get_param("~ws_url", "ws://10.192.1.2:5000")
        self.accid = rospy.get_param("~accid", "WF_TRON1A_412")
        self.rate_hz = rospy.get_param("~publish_rate", 30.0)
        self.dry_run = rospy.get_param("~dry_run", True)
        self.auto_prepare = rospy.get_param("~auto_prepare", False)

        self._cmd_lock = threading.Lock()
        self._latest_cmd = Twist()
        self._ws = None
        self._ws_thread = None
        self._send_lock = threading.Lock()
        self._connected = False

        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=10)

        rospy.loginfo(
            "[tron_controller_bridge] configuration:\n"
            "  ws_url=%s\n"
            "  accid=%s\n"
            "  dry_run=%s",
            self.ws_url,
            self.accid,
            self.dry_run,
        )

        if not self.dry_run:
            self._start_websocket()
            if self.auto_prepare:
                threading.Thread(target=self._prepare_robot, daemon=True).start()
        else:
            rospy.logwarn("[tron_controller_bridge] dry_run=true, WebSocket disabled")

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_loop)
        rospy.loginfo(
            "[tron_controller_bridge] cmd=%s publish_rate=%.1f",
            self.cmd_topic,
            self.rate_hz,
        )

    def cmd_callback(self, msg):
        with self._cmd_lock:
            self._latest_cmd = msg

    def _generate_guid(self):
        return str(uuid.uuid4())

    def _send_request(self, title, data=None):
        if data is None:
            data = {}
        payload = json.dumps(
            {
                "accid": self.accid,
                "title": title,
                "timestamp": int(time.time() * 1000),
                "guid": self._generate_guid(),
                "data": data,
            }
        )
        with self._send_lock:
            if self._ws and self._ws.sock and self._ws.sock.connected:
                self._ws.send(payload)

    def _prepare_robot(self):
        time.sleep(1.0)
        self._send_request("request_stand_mode")
        time.sleep(2.0)
        self._send_request("request_walk_mode")

    def _start_websocket(self):
        def on_open(_ws):
            self._connected = True
            rospy.loginfo("[tron_controller_bridge] WebSocket connected")

        def on_close(_ws, *_args):
            self._connected = False
            rospy.logwarn("[tron_controller_bridge] WebSocket closed")

        def on_error(_ws, error):
            rospy.logwarn(
                "[tron_controller_bridge] failed to connect %s: %s",
                self.ws_url,
                error,
            )

        self._ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error,
        )

        def run_ws():
            while not rospy.is_shutdown():
                try:
                    self._ws.run_forever(ping_interval=20, ping_timeout=10)
                except Exception as exc:
                    rospy.logwarn("[tron_controller_bridge] ws loop: %s", exc)
                time.sleep(1.0)

        self._ws_thread = threading.Thread(target=run_ws, daemon=True)
        self._ws_thread.start()

    def publish_loop(self, _event):
        with self._cmd_lock:
            cmd = self._latest_cmd

        if self.dry_run:
            if abs(cmd.linear.x) > 1e-3 or abs(cmd.angular.z) > 1e-3:
                rospy.loginfo_throttle(
                    1.0,
                    "[tron_controller_bridge] dry_run cmd x=%.3f z=%.3f",
                    cmd.linear.x,
                    cmd.angular.z,
                )
            return

        self._send_request(
            "request_twist",
            {"x": float(cmd.linear.x), "y": 0.0, "z": float(cmd.angular.z)},
        )


def main():
    rospy.init_node("tron_controller_bridge")
    TronControllerBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
