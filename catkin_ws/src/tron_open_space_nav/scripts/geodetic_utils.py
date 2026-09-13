#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WGS84 geodetic <-> local ENU utilities for tron_open_space_nav."""

from __future__ import division

import math

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def geodetic_to_enu(lat_deg, lon_deg, alt_m, lat0_deg, lon0_deg, alt0_m):
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def is_valid_navsat(msg):
    if msg.status.status < 0:
        return False
    if msg.latitude == 0.0 and msg.longitude == 0.0:
        return False
    return True


def haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Great-circle distance on WGS84 sphere (m). Good for local < few km."""
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    dlat = lat2 - lat1
    dlon = math.radians(lon2_deg - lon1_deg)
    a = (
        math.sin(dlat * 0.5) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon * 0.5) ** 2
    )
    return 2.0 * WGS84_A * math.asin(min(1.0, math.sqrt(a)))


def navsat_horizontal_sigma(msg):
    """
    Estimate horizontal 1-sigma (m) from NavSatFix position_covariance.
    Returns None if covariance is unknown.
    """
    if msg.position_covariance_type == msg.COVARIANCE_TYPE_UNKNOWN:
        return None
    cov = msg.position_covariance
    var_x = max(float(cov[0]), 0.0)
    var_y = max(float(cov[4]), 0.0)
    if var_x <= 0.0 and var_y <= 0.0:
        return None
    return math.sqrt(max(var_x, var_y))


def median_value(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def reject_origin_outlier(lat, lon, samples, max_jump_m=15.0):
    """Reject sample if too far from current sample median (m)."""
    if len(samples) < 3:
        return False
    lats = [s[0] for s in samples]
    lons = [s[1] for s in samples]
    med_lat = median_value(lats)
    med_lon = median_value(lons)
    if med_lat is None:
        return False
    jump = haversine_distance_m(lat, lon, med_lat, med_lon)
    return jump > max_jump_m


def ecef_to_geodetic(x, y, z):
    """ECEF (m) -> WGS84 geodetic degrees/m."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = math.atan2(z + WGS84_E2 * n * sin_lat, p)
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def enu_to_ecef(east, north, up, lat0_deg, lon0_deg, alt0_m):
    """Local ENU (m) at origin -> ECEF (m)."""
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    dx = -sin_lon0 * east - sin_lat0 * cos_lon0 * north + cos_lat0 * cos_lon0 * up
    dy = cos_lon0 * east - sin_lat0 * sin_lon0 * north + cos_lat0 * sin_lon0 * up
    dz = cos_lat0 * north + sin_lat0 * up
    return x0 + dx, y0 + dy, z0 + dz


def enu_to_geodetic(east, north, up, lat0_deg, lon0_deg, alt0_m):
    """Local ENU (m) at WGS84 origin -> lat/lon/alt degrees."""
    x, y, z = enu_to_ecef(east, north, up, lat0_deg, lon0_deg, alt0_m)
    return ecef_to_geodetic(x, y, z)


def geodetic_to_utm(lat_deg, lon_deg, alt_m=0.0):
    """WGS84 -> UTM easting/northing (m). Uses pyproj when available."""
    try:
        import pyproj
    except ImportError:
        return None

    zone = int((lon_deg + 180.0) / 6.0) + 1
    hemisphere = "north" if lat_deg >= 0.0 else "south"
    proj = pyproj.Proj(proj="utm", zone=zone, ellps="WGS84", hemisphere=hemisphere)
    easting, northing = proj(lon_deg, lat_deg)
    return easting, northing, alt_m, zone, hemisphere
