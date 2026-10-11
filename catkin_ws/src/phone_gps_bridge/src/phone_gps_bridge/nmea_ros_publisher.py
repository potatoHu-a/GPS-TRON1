"""Shared ROS publishers and GST association for NMEA input transports."""

import math
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from phone_gps_bridge.nmea_parser import gst_covariance, parse_nmea_sentence


class NmeaRosPublisher(object):
    def __init__(self, log_prefix):
        self.log_prefix = log_prefix
        self.frame_id = rospy.get_param("~frame_id", "gps_link")
        self.validate_checksum = rospy.get_param("~validate_checksum", True)
        self.publish_magnetic_heading = rospy.get_param(
            "~publish_magnetic_heading", True
        )
        self.gst_max_age = float(rospy.get_param("~gst_max_age", 1.0))
        self.debug_unknown = rospy.get_param("~debug_unknown_sentences", False)

        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.heading_pub = rospy.Publisher("/gps/heading", Float64, queue_size=10)
        self.magnetic_heading_pub = rospy.Publisher(
            "/gps/magnetic_heading", Float64, queue_size=10
        )
        self.course_pub = rospy.Publisher("/gps/course", Float64, queue_size=10)

        self.last_fix_timestamp = None
        self.latest_gst = None
        self.latest_gst_received = None

    def handle_sentence(self, sentence):
        parsed = parse_nmea_sentence(sentence, self.validate_checksum)
        if parsed is None:
            if self.debug_unknown and sentence.startswith("$"):
                rospy.logdebug("[%s] ignored NMEA sentence: %s", self.log_prefix, sentence)
            return

        sentence_type = parsed["type"]
        if sentence_type == "GST":
            if gst_covariance(parsed) is not None:
                self.latest_gst = parsed
                self.latest_gst_received = time.monotonic()
            return
        if sentence_type in ("GGA", "RMC"):
            self.publish_fix(parsed)
            if sentence_type == "RMC":
                self.publish_course(parsed.get("course"))
            return
        if sentence_type == "HDT":
            self.publish_heading(parsed.get("heading"))
            return
        if sentence_type == "HDG":
            self.publish_magnetic_heading_value(parsed.get("magnetic_heading"))
            return
        if sentence_type == "VTG":
            self.publish_course(parsed.get("course"))

    def publish_fix(self, parsed):
        timestamp = parsed.get("timestamp")
        if timestamp and timestamp == self.last_fix_timestamp:
            return

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS

        valid = parsed.get("valid", False)
        latitude = parsed.get("latitude")
        longitude = parsed.get("longitude")
        if valid and latitude is not None and longitude is not None:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude = latitude
            msg.longitude = longitude
            altitude = parsed.get("altitude")
            msg.altitude = altitude if altitude is not None else float("nan")
            self._apply_covariance(msg, parsed.get("hdop"))
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = float("nan")
            msg.longitude = float("nan")
            msg.altitude = float("nan")
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)
        self.last_fix_timestamp = timestamp
        if parsed["type"] == "GGA":
            rospy.logdebug(
                "[%s] GGA quality=%d satellites=%d valid=%s",
                self.log_prefix,
                parsed.get("fix_quality", 0),
                parsed.get("satellites", 0),
                valid,
            )

    def _apply_covariance(self, msg, hdop):
        if (
            self.latest_gst is not None
            and self.latest_gst_received is not None
            and time.monotonic() - self.latest_gst_received <= self.gst_max_age
        ):
            covariance = gst_covariance(self.latest_gst)
            if covariance is not None:
                msg.position_covariance = covariance
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                return

        if hdop is not None and math.isfinite(hdop) and hdop > 0.0:
            variance = (hdop * 5.0) ** 2
            msg.position_covariance[0] = variance
            msg.position_covariance[4] = variance
            msg.position_covariance[8] = variance * 4.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    def publish_heading(self, heading):
        if heading is not None:
            self.heading_pub.publish(Float64(data=heading))

    def publish_magnetic_heading_value(self, heading):
        if self.publish_magnetic_heading and heading is not None:
            self.magnetic_heading_pub.publish(Float64(data=heading))

    def publish_course(self, course):
        if course is not None:
            self.course_pub.publish(Float64(data=course))
