-- Flask/OpenCV owns the Realtek UVC camera on the UGV.
-- PipeWire must not lock /dev/video0 in YUYV 640x480 (that blocks 5MP MJPEG).
-- ALSA audio is unchanged.
v4l2_monitor.enabled = false
libcamera_monitor.enabled = false
