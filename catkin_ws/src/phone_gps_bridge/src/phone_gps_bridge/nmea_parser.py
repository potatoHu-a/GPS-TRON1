"""Pure NMEA parsing helpers shared by TCP and serial receivers."""

import math


def nmea_to_decimal(raw, direction):
    """Convert NMEA DDMM.MMMM / DDDMM.MMMM to decimal degrees."""
    if not raw or direction not in ("N", "S", "E", "W"):
        return None
    try:
        dot = raw.strip().index(".")
        degree_length = dot - 2
        if degree_length not in (2, 3):
            return None
        degrees = float(raw[:degree_length])
        minutes = float(raw[degree_length:])
        if minutes < 0.0 or minutes >= 60.0:
            return None
        value = degrees + minutes / 60.0
        if direction in ("S", "W"):
            value = -value
        return value
    except (ValueError, IndexError):
        return None


def nmea_checksum_ok(sentence):
    """Return True only when a complete NMEA checksum is valid."""
    line = sentence.strip()
    if not line.startswith("$") or "*" not in line:
        return False
    body, checksum_text = line[1:].split("*", 1)
    if len(checksum_text) < 2:
        return False
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    try:
        return checksum == int(checksum_text[:2], 16)
    except ValueError:
        return False


def sentence_payload(sentence, validate_checksum=True):
    """Return the comma-separated payload of one complete NMEA sentence."""
    line = sentence.strip()
    if not line.startswith("$"):
        return None
    if validate_checksum and not nmea_checksum_ok(line):
        return None
    return line.split("*", 1)[0].split(",")


def parse_nmea_sentence(sentence, validate_checksum=True):
    """Parse supported NMEA sentences into a transport-independent dict."""
    parts = sentence_payload(sentence, validate_checksum)
    if not parts or len(parts[0]) < 6:
        return None

    sentence_type = parts[0][3:]
    result = {
        "talker": parts[0][1:3],
        "type": sentence_type,
        "raw_type": parts[0][1:],
        "timestamp": parts[1] if len(parts) > 1 else "",
    }

    try:
        if sentence_type == "GGA" and len(parts) >= 10:
            result.update(
                latitude=nmea_to_decimal(parts[2], parts[3]),
                longitude=nmea_to_decimal(parts[4], parts[5]),
                fix_quality=int(parts[6]) if parts[6] else 0,
                satellites=int(parts[7]) if parts[7] else 0,
                hdop=float(parts[8]) if parts[8] else None,
                altitude=float(parts[9]) if parts[9] else None,
            )
            result["valid"] = (
                result["fix_quality"] > 0
                and result["latitude"] is not None
                and result["longitude"] is not None
            )
            return result

        if sentence_type == "RMC" and len(parts) >= 10:
            result.update(
                status=parts[2],
                latitude=nmea_to_decimal(parts[3], parts[4]),
                longitude=nmea_to_decimal(parts[5], parts[6]),
                speed_knots=float(parts[7]) if parts[7] else None,
                course=float(parts[8]) if parts[8] else None,
                date=parts[9],
            )
            result["valid"] = (
                result["status"] == "A"
                and result["latitude"] is not None
                and result["longitude"] is not None
            )
            return result

        if sentence_type == "GST" and len(parts) >= 9:
            result.update(
                rms=float(parts[2]) if parts[2] else None,
                semi_major_std=float(parts[3]) if parts[3] else None,
                semi_minor_std=float(parts[4]) if parts[4] else None,
                orientation=float(parts[5]) if parts[5] else None,
                latitude_std=float(parts[6]) if parts[6] else None,
                longitude_std=float(parts[7]) if parts[7] else None,
                altitude_std=float(parts[8]) if parts[8] else None,
            )
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


def gst_covariance(parsed):
    """Return ENU diagonal covariance from GST lon/lat/alt 1-sigma errors."""
    if not parsed or parsed.get("type") != "GST":
        return None
    east = parsed.get("longitude_std")
    north = parsed.get("latitude_std")
    up = parsed.get("altitude_std")
    if any(value is None or not math.isfinite(value) or value < 0.0 for value in (east, north, up)):
        return None
    return [east * east, 0.0, 0.0, 0.0, north * north, 0.0, 0.0, 0.0, up * up]
