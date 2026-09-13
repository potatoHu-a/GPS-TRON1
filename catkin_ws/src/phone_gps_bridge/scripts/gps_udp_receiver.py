#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import socket
import threading

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


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


def parse_nmea_line(line):
    """Parse GGA/RMC NMEA sentence, return dict or None."""
    line = line.strip()
    if not line.startswith("$"):
        return None

    parts = line.split("*")[0].split(",")
    if len(parts) < 2:
        return None

    sentence_type = parts[0][3:]  # e.g. GPGGA -> GGA
    result = {"type": sentence_type}

    try:
        if sentence_type.endswith("GGA") and len(parts) >= 10:
            result["latitude"] = nmea_to_decimal(parts[2], parts[3])
            result["longitude"] = nmea_to_decimal(parts[4], parts[5])
            result["fix_quality"] = int(parts[6]) if parts[6] else 0
            result["satellites"] = int(parts[7]) if parts[7] else 0
            result["hdop"] = float(parts[8]) if parts[8] else None
            result["altitude"] = float(parts[9]) if parts[9] else None
            result["valid"] = result["fix_quality"] > 0
            return result

        if sentence_type.endswith("RMC") and len(parts) >= 10:
            status = parts[2]
            result["latitude"] = nmea_to_decimal(parts[3], parts[4])
            result["longitude"] = nmea_to_decimal(parts[5], parts[6])
            result["valid"] = status == "A"
            result["fix_quality"] = 1 if status == "A" else 0
            return result
    except (ValueError, IndexError):
        pass

    return None


class GpsState(object):
    """Thread-safe GPS state from latest NMEA data."""

    def __init__(self):
        self._lock = threading.Lock()
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.fix_quality = 0
        self.satellites = 0
        self.hdop = None
        self.valid = False
        self.source_ip = ""

    def update(self, parsed, source_ip):
        with self._lock:
            if parsed.get("latitude") is not None:
                self.latitude = parsed["latitude"]
            if parsed.get("longitude") is not None:
                self.longitude = parsed["longitude"]
            if parsed.get("altitude") is not None:
                self.altitude = parsed["altitude"]
            if "fix_quality" in parsed:
                self.fix_quality = parsed["fix_quality"]
            if "satellites" in parsed:
                self.satellites = parsed["satellites"]
            if parsed.get("hdop") is not None:
                self.hdop = parsed["hdop"]
            if "valid" in parsed:
                self.valid = parsed["valid"]
            self.source_ip = source_ip

    def snapshot(self):
        with self._lock:
            return (
                self.latitude,
                self.longitude,
                self.altitude,
                self.fix_quality,
                self.satellites,
                self.hdop,
                self.valid,
                self.source_ip,
            )


def udp_listener(port, state, debug):
    """Background thread: receive UDP NMEA and update state."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)
    rospy.loginfo("Listening UDP 0.0.0.0:%d", port)

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(4096)
            source_ip = addr[0]
            text = data.decode("utf-8", errors="ignore")
            for line in text.splitlines():
                parsed = parse_nmea_line(line)
                if parsed is None:
                    continue
                state.update(parsed, source_ip)
                if debug:
                    lat, lon, alt, fq, sats, hdop, valid, sip = state.snapshot()
                    rospy.loginfo(
                        "[UDP %s] type=%s lat=%.6f lon=%.6f alt=%s sats=%d fix=%d",
                        sip,
                        parsed["type"],
                        lat or 0.0,
                        lon or 0.0,
                        "%.1f" % alt if alt is not None else "N/A",
                        sats,
                        fq,
                    )
        except socket.timeout:
            continue
        except Exception as exc:
            rospy.logwarn("UDP receive error: %s", exc)

    sock.close()


def build_navsat_fix(state, frame_id):
    """Build NavSatFix message from current state."""
    lat, lon, alt, fix_quality, sats, hdop, valid, _ = state.snapshot()

    msg = NavSatFix()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    msg.status.service = NavSatStatus.SERVICE_GPS
    if valid and lat is not None and lon is not None:
        msg.status.status = NavSatStatus.STATUS_FIX
    else:
        msg.status.status = NavSatStatus.STATUS_NO_FIX

    msg.latitude = lat if lat is not None else 0.0
    msg.longitude = lon if lon is not None else 0.0
    msg.altitude = alt if alt is not None else 0.0

    # Covariance from HDOP rough estimate
    if hdop is not None and hdop > 0:
        var = (hdop * 5.0) ** 2  # ~5m per HDOP unit
        msg.position_covariance[0] = var
        msg.position_covariance[4] = var
        msg.position_covariance[8] = var * 4.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    return msg


def main():
    rospy.init_node("gps_udp_receiver")

    port = rospy.get_param("~port", 10110)
    frame_id = rospy.get_param("~frame_id", "gps_link")
    publish_rate = rospy.get_param("~publish_rate", 5.0)
    debug = rospy.get_param("~debug", True)

    pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
    state = GpsState()

    t = threading.Thread(target=udp_listener, args=(port, state, debug))
    t.daemon = True
    t.start()

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        try:
            msg = build_navsat_fix(state, frame_id)
            pub.publish(msg)
        except Exception as exc:
            rospy.logwarn("Publish error: %s", exc)
        rate.sleep()


if __name__ == "__main__":
    main()
