#!/usr/bin/env python3
"""Read NMEA from a reconnecting USB serial GNSS receiver."""

import time

import rospy
import serial

from phone_gps_bridge.nmea_ros_publisher import NmeaRosPublisher


PARITY = {
    "none": serial.PARITY_NONE,
    "even": serial.PARITY_EVEN,
    "odd": serial.PARITY_ODD,
}


class SerialGnssReceiver(object):
    def __init__(self):
        self.device = rospy.get_param("~device")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.bytesize = int(rospy.get_param("~bytesize", 8))
        self.parity_name = str(rospy.get_param("~parity", "none")).lower()
        self.stopbits = float(rospy.get_param("~stopbits", 1))
        self.software_flow_control = rospy.get_param(
            "~software_flow_control", False
        )
        self.hardware_flow_control = rospy.get_param(
            "~hardware_flow_control", False
        )
        self.read_timeout = float(rospy.get_param("~read_timeout", 1.0))
        self.reconnect_interval = float(
            rospy.get_param("~reconnect_interval", 2.0)
        )
        if self.parity_name not in PARITY:
            raise ValueError("parity must be none, even, or odd")
        self.publisher = NmeaRosPublisher("serial_gnss")

    def run(self):
        while not rospy.is_shutdown():
            port = None
            try:
                rospy.loginfo(
                    "[serial_gnss] opening %s at %d %d%s%s",
                    self.device,
                    self.baudrate,
                    self.bytesize,
                    self.parity_name[0].upper(),
                    "%g" % self.stopbits,
                )
                port = serial.Serial(
                    port=self.device,
                    baudrate=self.baudrate,
                    bytesize=self.bytesize,
                    parity=PARITY[self.parity_name],
                    stopbits=self.stopbits,
                    timeout=self.read_timeout,
                    xonxoff=self.software_flow_control,
                    rtscts=self.hardware_flow_control,
                    dsrdtr=self.hardware_flow_control,
                )
                rospy.loginfo("[serial_gnss] connected")
                self.read_loop(port)
            except (serial.SerialException, OSError, ValueError) as exc:
                rospy.logwarn("[serial_gnss] serial error: %s", exc)
            finally:
                if port is not None:
                    try:
                        port.close()
                    except serial.SerialException:
                        pass
            if not rospy.is_shutdown():
                rospy.logwarn(
                    "[serial_gnss] disconnected, retrying in %.1fs",
                    self.reconnect_interval,
                )
                time.sleep(self.reconnect_interval)

    def read_loop(self, port):
        while not rospy.is_shutdown():
            try:
                raw_line = port.readline()
            except (serial.SerialException, OSError) as exc:
                rospy.logwarn("[serial_gnss] read error: %s", exc)
                return
            if not raw_line:
                continue
            sentence = raw_line.decode("ascii", errors="ignore").strip("\r\n ")
            if not sentence or not sentence.startswith("$") or "*" not in sentence:
                continue
            try:
                self.publisher.handle_sentence(sentence)
            except Exception as exc:
                rospy.logwarn_throttle(
                    1.0, "[serial_gnss] bad sentence ignored: %s", exc
                )


def main():
    rospy.init_node("serial_gnss_receiver")
    try:
        SerialGnssReceiver().run()
    except (KeyError, TypeError, ValueError) as exc:
        rospy.logfatal("[serial_gnss] invalid configuration: %s", exc)
        raise


if __name__ == "__main__":
    main()
