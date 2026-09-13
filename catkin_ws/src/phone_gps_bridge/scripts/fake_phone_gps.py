#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulate Android phone sending NMEA GGA/RMC over UDP."""

import math
import socket
import time


def decimal_to_nmea(value, is_lat):
    """Convert decimal degrees to NMEA DDMM.MMMM format."""
    sign = 1 if value >= 0 else -1
    value = abs(value)
    degrees = int(value)
    minutes = (value - degrees) * 60.0
    if is_lat:
        return "%02d%07.4f" % (degrees, minutes)
    return "%03d%07.4f" % (degrees, minutes)


def nmea_checksum(sentence):
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return "%02X" % cs


def make_gga(lat, lon, alt, fix_quality=1, sats=12, hdop=1.0):
    lat_str = decimal_to_nmea(lat, True)
    lon_str = decimal_to_nmea(lon, False)
    lat_dir = "N" if lat >= 0 else "S"
    lon_dir = "E" if lon >= 0 else "W"
    body = "GPGGA,%s,%s,%s,%s,%s,%d,%d,%.1f,%.1f,M,0.0,M,," % (
        time.strftime("%H%M%S.00", time.gmtime()),
        lat_str, lat_dir,
        lon_str, lon_dir,
        fix_quality, sats, hdop, alt,
    )
    return "$%s*%s" % (body, nmea_checksum(body))


def make_rmc(lat, lon, valid=True):
    lat_str = decimal_to_nmea(lat, True)
    lon_str = decimal_to_nmea(lon, False)
    lat_dir = "N" if lat >= 0 else "S"
    lon_dir = "E" if lon >= 0 else "W"
    status = "A" if valid else "V"
    body = "GPRMC,%s,%s,%s,%s,%s,%s,0.0,0.0,%s,0.0,E" % (
        time.strftime("%H%M%S.00", time.gmtime()),
        status,
        lat_str, lat_dir,
        lon_str, lon_dir,
        time.strftime("%d%m%y", time.gmtime()),
    )
    return "$%s*%s" % (body, nmea_checksum(body))


def main():
    target_ip = "127.0.0.1"
    target_port = 10110
    rate_hz = 5.0

    lat = 30.53
    lon = 114.36
    alt = 30.0
    # ~1 m/s eastward at Wuhan latitude
    lon_step = (1.0 / 111320.0) / rate_hz

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Fake phone GPS -> %s:%d @ %.1f Hz" % (target_ip, target_port, rate_hz))
    print("Start: lat=%.6f lon=%.6f, moving east" % (lat, lon))

    try:
        while True:
            gga = make_gga(lat, lon, alt)
            rmc = make_rmc(lat, lon)
            payload = (gga + "\r\n" + rmc + "\r\n").encode("utf-8")
            sock.sendto(payload, (target_ip, target_port))
            lon += lon_step
            time.sleep(1.0 / rate_hz)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
