#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


def nmea_to_decimal(raw, direction):
    """Convert NMEA DDMM.MMMM / DDDMM.MMMM to decimal degrees."""
    if not raw:
        return None
    try:
        raw = raw.strip()
        dot = raw.index(".")
        deg_len = dot - 2
        degrees = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        value = degrees + minutes / 60.0
        if direction in ("S", "W"):
            value = -value
        return value
    except (ValueError, IndexError):
        return None


def nmea_checksum_ok(sentence):
    """Return True when sentence has a valid NMEA checksum."""
    line = sentence.strip()
    if not line.startswith("$") or "*" not in line:
        return False

    body, checksum_text = line[1:].split("*", 1)
    checksum_text = checksum_text[:2]
    if len(checksum_text) != 2:
        return False

    checksum = 0
    for char in body:
        checksum ^= ord(char)

    try:
        expected = int(checksum_text, 16)
    except ValueError:
        return False

    return checksum == expected


def sentence_payload(sentence, validate_checksum):
    """Return comma-split NMEA payload or None."""
    line = sentence.strip()
    if not line.startswith("$"):
        return None
    if validate_checksum and not nmea_checksum_ok(line):
        return None

    return line.split("*", 1)[0].split(",")


def parse_nmea_sentence(sentence, validate_checksum=True):
    """Parse supported NMEA sentences into a small normalized dict."""
    parts = sentence_payload(sentence, validate_checksum)
    if not parts or len(parts[0]) < 6:
        return None

    talker = parts[0][1:3]
    sentence_type = parts[0][3:]
    result = {
        "talker": talker,
        "type": sentence_type,
        "raw_type": parts[0][1:],
        "timestamp": parts[1] if len(parts) > 1 else "",
    }

    try:
        if sentence_type == "GGA" and len(parts) >= 10:
            result["latitude"] = nmea_to_decimal(parts[2], parts[3])
            result["longitude"] = nmea_to_decimal(parts[4], parts[5])
            result["fix_quality"] = int(parts[6]) if parts[6] else 0
            result["satellites"] = int(parts[7]) if parts[7] else 0
            result["hdop"] = float(parts[8]) if parts[8] else None
            result["altitude"] = float(parts[9]) if parts[9] else None
            result["valid"] = result["fix_quality"] > 0
            return result

        if sentence_type == "RMC" and len(parts) >= 10:
            status = parts[2]
            result["latitude"] = nmea_to_decimal(parts[3], parts[4])
            result["longitude"] = nmea_to_decimal(parts[5], parts[6])
            result["valid"] = status == "A"
            result["fix_quality"] = 1 if status == "A" else 0
            result["course"] = float(parts[8]) if parts[8] else None
            return result

        if sentence_type == "HDT" and len(parts) >= 2:
            result["heading"] = float(parts[1]) if parts[1] else None
            return result

        if sentence_type == "HDG" and len(parts) >= 2:
            result["magnetic_heading"] = float(parts[1]) if parts[1] else None
            return result

        if sentence_type == "VTG" and len(parts) >= 2:
            result["course"] = float(parts[1]) if parts[1] else None
            return result
    except (ValueError, IndexError):
        return None

    return None


def build_navsat_fix(parsed, frame_id):
    """Build NavSatFix directly from a parsed GGA/RMC sentence."""
    msg = NavSatFix()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    lat = parsed.get("latitude")
    lon = parsed.get("longitude")
    alt = parsed.get("altitude")
    hdop = parsed.get("hdop")
    valid = parsed.get("valid", False)

    msg.status.service = NavSatStatus.SERVICE_GPS
    if valid and lat is not None and lon is not None:
        msg.status.status = NavSatStatus.STATUS_FIX
    else:
        msg.status.status = NavSatStatus.STATUS_NO_FIX

    msg.latitude = lat if lat is not None else 0.0
    msg.longitude = lon if lon is not None else 0.0
    msg.altitude = alt if alt is not None else 0.0

    if hdop is not None and hdop > 0:
        var = (hdop * 5.0) ** 2
        msg.position_covariance[0] = var
        msg.position_covariance[4] = var
        msg.position_covariance[8] = var * 4.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    return msg


class NetGpsReceiver(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "172.18.125.125")
        self.port = int(rospy.get_param("~port", 10110))
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))
        self.connect_timeout = float(rospy.get_param("~connect_timeout", 5.0))
        self.receive_timeout = float(rospy.get_param("~receive_timeout", 5.0))
        self.frame_id = rospy.get_param("~frame_id", "gps_link")
        self.publish_magnetic_heading = rospy.get_param("~publish_magnetic_heading", True)
        self.validate_checksum = rospy.get_param("~validate_checksum", True)

        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.heading_pub = rospy.Publisher("/gps/heading", Float64, queue_size=10)
        self.magnetic_heading_pub = rospy.Publisher(
            "/gps/magnetic_heading", Float64, queue_size=10
        )
        self.course_pub = rospy.Publisher("/gps/course", Float64, queue_size=10)

        self.last_fix_timestamp = None
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
        parsed = parse_nmea_sentence(sentence, self.validate_checksum)
        if parsed is None:
            return

        sentence_type = parsed["type"]
        if sentence_type in ("GGA", "RMC"):
            self.publish_fix(parsed)
            if sentence_type == "RMC":
                self.publish_course(parsed.get("course"))
        elif sentence_type == "HDT":
            self.publish_heading(parsed.get("heading"))
        elif sentence_type == "HDG":
            self.publish_magnetic_heading_value(parsed.get("magnetic_heading"))
        elif sentence_type == "VTG":
            self.publish_course(parsed.get("course"))

    def publish_fix(self, parsed):
        timestamp = parsed.get("timestamp")

        if timestamp and timestamp == self.last_fix_timestamp:
            return

        if parsed.get("latitude") is None or parsed.get("longitude") is None:
            return

        self.fix_pub.publish(build_navsat_fix(parsed, self.frame_id))
        self.last_fix_timestamp = timestamp

    def publish_heading(self, heading):
        if heading is not None:
            self.heading_pub.publish(Float64(data=heading))

    def publish_magnetic_heading_value(self, magnetic_heading):
        if self.publish_magnetic_heading and magnetic_heading is not None:
            self.magnetic_heading_pub.publish(Float64(data=magnetic_heading))

    def publish_course(self, course):
        if course is not None:
            self.course_pub.publish(Float64(data=course))


def main():
    rospy.init_node("netgps_tcp_receiver")
    receiver = NetGpsReceiver()
    receiver.run()


if __name__ == "__main__":
    main()
