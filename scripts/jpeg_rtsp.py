#!/usr/bin/env python3
"""JPEG-over-RTSP publisher (GstRtspServer + rtpjpegpay, no x264).

Restreams Flask MJPEG at http://127.0.0.1:5000/video_feed so /dev/video*
stays with Flask. Path /live, port 8554 if free else 8555/8556.
"""
from __future__ import print_function

import json
import os
import socket
import sys
import time

SRC_URL = os.environ.get("UGV_MJPEG_URL", os.environ.get("BEAST_MJPEG_URL", "http://127.0.0.1:5000/video_feed"))
STATE_PATH = os.environ.get("UGV_RTSP_STATE", os.environ.get("BEAST_RTSP_STATE", "/tmp/ugv-jpeg-rtsp.json"))
MOUNT = os.environ.get("UGV_RTSP_PATH", os.environ.get("BEAST_RTSP_PATH", "/live"))
PORTS = [8554, 8555, 8556]


def port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def pick_port():
    env = os.environ.get("BEAST_RTSP_PORT")
    if env:
        p = int(env)
        if port_free(p):
            return p
        raise SystemExit("port %s busy" % p)
    for p in PORTS:
        if port_free(p):
            return p
    raise SystemExit("no free RTSP port in %s" % PORTS)


def _public_host():
    host = os.environ.get("UGV_RTSP_HOST") or os.environ.get("BEAST_RTSP_HOST")
    if host:
        return host
    try:
        return socket.gethostname()
    except OSError:
        return "127.0.0.1"


def write_state(port, path):
    data = {
        "url": "rtsp://%s:%s%s" % (_public_host(), port, path),
        "bind": "0.0.0.0",
        "port": port,
        "path": path,
        "src": SRC_URL,
        "codec": "jpeg",
        "pid": os.getpid(),
    }
    tmp = STATE_PATH + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(data, fh)
        fh.write("\n")
    os.replace(tmp, STATE_PATH)
    print("BEAST_RTSP_URL=%s" % data["url"], flush=True)


def launch_pipeline():
    # JPEG passthrough only. souphttpsrc + jpegparse + rtpjpegpay (no x264).
    return (
        "( souphttpsrc location=%s is-live=true do-timestamp=true timeout=5 "
        "retries=2147483647 ! multipartdemux ! jpegparse ! "
        "rtpjpegpay name=pay0 pt=26 )"
    ) % SRC_URL


def main():
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import Gst, GstRtspServer, GLib

    Gst.init(None)
    port = pick_port()
    path = MOUNT if MOUNT.startswith("/") else "/" + MOUNT
    write_state(port, path)

    server = GstRtspServer.RTSPServer()
    server.set_address("0.0.0.0")
    server.set_service(str(port))
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(launch_pipeline())
    factory.set_shared(True)
    factory.set_latency(200)
    mounts = server.get_mount_points()
    mounts.add_factory(path, factory)
    if server.attach(None) == 0:
        raise SystemExit("GstRtspServer attach failed on %s" % port)
    print("jpeg-rtsp listening on rtsp://0.0.0.0:%s%s" % (port, path), flush=True)
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.remove(STATE_PATH)
        except OSError:
            pass


if __name__ == "__main__":
    main()
