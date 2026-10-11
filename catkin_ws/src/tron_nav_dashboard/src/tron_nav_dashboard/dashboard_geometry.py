"""Pure 2-D dashboard geometry helpers; intentionally has no ROS/Qt imports."""

import math


def arrow_delta_screen(yaw, length):
    """Return (dx, dy) for ENU yaw rendered in Qt screen coordinates."""
    return length * math.cos(yaw), -length * math.sin(yaw)


def arrow_tip_xy(x, y, yaw, length):
    dx, dy = arrow_delta_screen(yaw, length)
    return x + dx, y + dy
