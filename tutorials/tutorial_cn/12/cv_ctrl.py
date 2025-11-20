import cv2
import threading
import datetime, time
import numpy as np
import math
import yaml, os, json, subprocess

log_file_path = "./Mediamtx/mediamtx.log"

with open(log_file_path, "w") as log_file:
    mediamtx_command = [
        './Mediamtx/mediamtx',
        './Mediamtx/mediamtx.yml',
    ]
    mediamtx_process = subprocess.Popen(
        mediamtx_command,
        stdout=log_file,
        stderr=log_file
    )

def is_raspberry_pi5():
    with open('/proc/cpuinfo', 'r') as file:
        for line in file:
            if 'Model' in line:
                if 'Raspberry Pi 5' in line:
                    return True
                else:
                    return False

if is_raspberry_pi5():
   ffmpeg_command = [
    'ffmpeg',
    '-y',
   '-loglevel', 'quiet',    
    '-f', 'rawvideo',         
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',       
    '-s', '640x480',           
    '-r', '30',                
    '-i', '-',                 
    '-c:v', 'libx264',         
    '-b:v', '300k',
    '-crf', '28',             
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-tune', 'zerolatency',
    '-f', 'rtsp',              
    'rtsp://localhost:8554/cam'
]
else:
   ffmpeg_command = [
    'ffmpeg',
    '-y',
    '-loglevel', 'quiet',    
    '-f', 'rawvideo',         
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',       
    '-s', '640x480',           
    '-r', '30',                
    '-i', '-',
    '-vf', 'format=yuv420p',                  
    '-c:v', 'h264_v4l2m2m',         
    '-b:v', '800k',
    '-pix_fmt', 'yuv420p',
    '-fflags', 'nobuffer',    
    '-f', 'rtsp',              
    'rtsp://localhost:8554/cam'
]

class OpencvFuncs():
    """docstring for OpencvFuncs"""
    def __init__(self):
        self.ffmpeg_process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)
        # camera type detection
        self.usb_camera_connected = self.usb_camera_detection()
        self.csi_camera_connected = False
        self.oak_camera_connected = False

        # usb camera init
        if self.usb_camera_connected:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # csi camera init
        if not self.usb_camera_connected:
            print("init csi camera.")
            try:
                # libraries for csi camera
                from picamera2 import Picamera2
                from picamera2.encoders import H264Encoder, Encoder
                from picamera2.outputs import FfmpegOutput

                self.encoder = H264Encoder(1000000)
                self.picam2 = Picamera2()
                self.picam2.configure(self.picam2.create_video_configuration(main={"format": 'XRGB8888', "size": (f['video']['default_res_w'], f['video']['default_res_h'])}))
                self.picam2.start()
                self.csi_camera_connected = True
            except:
                self.csi_camera_connected = False

        #oak camera init 
        if not self.usb_camera_connected and not self.csi_camera_connected:
            try:
                # libraries for oak camera
                import depthai as dai
                self.pipeline = dai.Pipeline()

                self.camRgb = self.pipeline.createColorCamera()
                self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
                self.camRgb.setInterleaved(False)
                # self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_480_P)
                self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)

                self.xout = self.pipeline.createXLinkOut()
                self.xout.setStreamName("video")
                self.camRgb.video.link(self.xout.input)

                self.device = dai.Device(self.pipeline)
                self.output_queue = self.device.getOutputQueue(name="video", maxSize=8, blocking=False)

                self.oak_camera_connected = True
            except Exception as e:
                print(f"[cv_ctrl.frame_process] error: {e}")
                self.oak_camera_connected = False


    def frame_process(self):
        while True:
            try:
                if self.usb_camera_connected:
                    success, input_frame = self.camera.read()
                    if not success:
                        self.camera.release()
                        time.sleep(1)
                        self.camera = cv2.VideoCapture(0)
                elif self.csi_camera_connected:
                    input_frame = self.picam2.capture_array()
                elif self.oak_camera_connected:
                    input_frame = self.output_queue.get().getCvFrame()
                    input_frame = cv2.resize(input_frame, (640, 480))
                else:
                    input_frame = 255 * np.ones((480, 640, 3), dtype=np.uint8)
                    cv2.putText(input_frame, f"camera read failed... \nusb - csi - oak", 
                                (round(0.05*640), round(0.1*640 + 5 * 13)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.369, (0, 0, 0), 1)
            except Exception as e:
                print(f"[cv_ctrl.frame_process] error: {e}")
                input_frame = 255 * np.ones((480, 640, 3), dtype=np.uint8)
                cv2.putText(input_frame, f"camera read failed... \n{e}", 
                            (round(0.05*640), round(0.1*640 + 5 * 13)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.369, (0, 0, 0), 1)
        
            # encode frame
            try:
                self.ffmpeg_process.stdin.write(input_frame.tobytes())
            except:
                pass

            time.sleep(1/30)

    def usb_camera_detection(self):
        lsusb_output = subprocess.check_output(["lsusb"]).decode("utf-8")
        if "Camera" in lsusb_output:
            print("USB Camera connected")
            return True
        else:
            print("USB Camera not connected")
            return False
