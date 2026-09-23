#!/usr/bin/env python3

import math
import os
import sys
import unittest

SOURCE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, SOURCE_DIR)

from phone_gps_bridge.nmea_parser import gst_covariance, parse_nmea_sentence


class NmeaParserTest(unittest.TestCase):
    def test_no_fix(self):
        gga = parse_nmea_sentence(
            "$GPGGA,001405.80,,,,,0,00,,,,,,,*40"
        )
        rmc = parse_nmea_sentence(
            "$GPRMC,001405.80,V,,,,,,,010117,0.0,W,N*0A"
        )
        self.assertIsNotNone(gga)
        self.assertFalse(gga["valid"])
        self.assertEqual(gga["fix_quality"], 0)
        self.assertIsNone(gga["latitude"])
        self.assertIsNotNone(rmc)
        self.assertFalse(rmc["valid"])

    def test_valid_gga(self):
        parsed = parse_nmea_sentence(
            "$GPGGA,083654.40,3032.8179125,N,11421.3512529,E,1,13,1.6,10.8678,M,0.000,M,,*5E"
        )
        self.assertTrue(parsed["valid"])
        self.assertAlmostEqual(parsed["latitude"], 30.0 + 32.8179125 / 60.0)
        self.assertAlmostEqual(parsed["longitude"], 114.0 + 21.3512529 / 60.0)
        self.assertAlmostEqual(parsed["altitude"], 10.8678)
        self.assertEqual(parsed["fix_quality"], 1)
        self.assertEqual(parsed["satellites"], 13)
        self.assertAlmostEqual(parsed["hdop"], 1.6)

    def test_rmc_course_is_not_heading(self):
        parsed = parse_nmea_sentence(
            "$GPRMC,083654.40,A,3032.8179125,N,11421.3512529,E,000.049,074.1,230926,0.0,W,A*23"
        )
        self.assertTrue(parsed["valid"])
        self.assertAlmostEqual(parsed["course"], 74.1)
        self.assertNotIn("heading", parsed)

    def test_gst_covariance(self):
        parsed = parse_nmea_sentence(
            "$GPGST,083654.40,0.48,0.65,0.54,48.6609,1.3825,0.5787,3.5201*51"
        )
        covariance = gst_covariance(parsed)
        self.assertAlmostEqual(covariance[0], 0.5787 ** 2)
        self.assertAlmostEqual(covariance[4], 1.3825 ** 2)
        self.assertAlmostEqual(covariance[8], 3.5201 ** 2)
        self.assertTrue(all(math.isfinite(value) for value in covariance))

    def test_private_and_bad_sentences_are_ignored(self):
        self.assertIsNone(parse_nmea_sentence("$PSIC,PST,...", False))
        self.assertIsNone(parse_nmea_sentence("$PSIC,GSI,...", False))
        self.assertIsNone(parse_nmea_sentence("$PSIC,BSI,...", False))
        self.assertIsNone(parse_nmea_sentence("$GPGGA,broken*00"))
        self.assertIsNone(parse_nmea_sentence("partial line"))


if __name__ == "__main__":
    unittest.main()
