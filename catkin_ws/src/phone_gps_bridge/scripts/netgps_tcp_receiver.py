#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time

import rospy
from phone_gps_bridge.nmea_ros_publisher import NmeaRosPublisher


class NetGpsReceiver(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "172.18.125.125")
        self.port = int(rospy.get_param("~port", 10110))
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))
        self.connect_timeout = float(rospy.get_param("~connect_timeout", 5.0))
        self.receive_timeout = float(rospy.get_param("~receive_timeout", 5.0))
        self.publisher = NmeaRosPublisher("netgps")
        self.last_receive_timeout_log = 0.0

    def run(self):
        while not rospy.is_shutdown():
            sock = None
            try:
                rospy.loginfo("[netgps] connecting %s:%d", self.host, self.port)
                sock = socket.create_connection(
                    (self.host, self.port), timeout=self.connect_timeout
                )
                sock.settimeout(self.receive_timeout)
                rospy.loginfo("[netgps] connected")
                self.receive_loop(sock)
            except socket.timeout:
                rospy.logwarn("[netgps] connection timeout")
            except (socket.error, OSError) as exc:
                rospy.logwarn("[netgps] connection error: %s", exc)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except socket.error:
                        pass

            if not rospy.is_shutdown():
                rospy.logwarn("[netgps] disconnected, retrying...")
                time.sleep(self.reconnect_interval)

    def receive_loop(self, sock):
        buffer = ""
        while not rospy.is_shutdown():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                now = time.time()
                if now - self.last_receive_timeout_log >= 10.0:
                    rospy.logwarn("[netgps] receive timeout, connection still alive")
                    self.last_receive_timeout_log = now
                continue
            except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, OSError) as exc:
                rospy.logwarn("[netgps] receive error: %s", exc)
                break

            if not data:
                break

            buffer += data.decode("ascii", errors="ignore")
            lines = buffer.split("\n")
            buffer = lines.pop()

            for line in lines:
                sentence = line.strip("\r")
                if sentence:
                    self.handle_sentence(sentence)

            if len(buffer) > 4096:
                buffer = buffer[-4096:]

    def handle_sentence(self, sentence):
        self.publisher.handle_sentence(sentence)


def main():
    rospy.init_node("netgps_tcp_receiver")
    receiver = NetGpsReceiver()
    receiver.run()


if __name__ == "__main__":
    main()
