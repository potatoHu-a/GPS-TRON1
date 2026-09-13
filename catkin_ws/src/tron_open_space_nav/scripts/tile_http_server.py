#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple HTTP server for offline Mapviz XYZ tiles (z/x/y.png)."""

import os
import threading

import rospy
from http.server import HTTPServer, SimpleHTTPRequestHandler


class TileHttpServer(object):
    def __init__(self):
        self.port = int(rospy.get_param("~port", 8088))
        self.tile_root = rospy.get_param("~tile_root", "")
        if not self.tile_root or not os.path.isdir(self.tile_root):
            rospy.logwarn(
                "[tile_http_server] tile_root missing or not a directory: %s",
                self.tile_root,
            )

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        rospy.loginfo(
            "[tile_http_server] serving %s at http://127.0.0.1:%d/{z}/{x}/{y}.png",
            self.tile_root,
            self.port,
        )

    def _run(self):
        root = self.tile_root if self.tile_root else os.getcwd()
        os.chdir(root)

        class Handler(SimpleHTTPRequestHandler):
            def log_message(self, *_args):
                pass

        httpd = HTTPServer(("0.0.0.0", self.port), Handler)
        httpd.serve_forever()


def main():
    rospy.init_node("tile_http_server")
    TileHttpServer()
    rospy.spin()


if __name__ == "__main__":
    main()
