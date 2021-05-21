#!/usr/bin/env python 
# -*- coding:utf-8 -*-
import cv2
import pyzed.sl as sl
import time
import sys
import pyzed.sl as sl
import numpy as np
from PIL import Image
from yolo import YOLO
import math
camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1
yolo = YOLO()
# t1 = time.time()
# distance=''
# x1=x2=y1=y2=0
IsDetected=False


def detect():
    print("Running...")
    init = sl.InitParameters()
    cam = sl.Camera()
    init.camera_resolution = sl.RESOLUTION.HD720 # Use HD1080 video mode
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init.camera_fps = 30  # Set fps at 30
    if not cam.is_opened():
        print("Opening ZED Camera...")
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    mat = sl.Mat()
    key=''
    while key != 113:  # for 'q' key
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            frame=mat.get_data()
            frame = cv2.resize(frame, (int(mat.get_width() / 2), int(mat.get_height() / 2)),
                               interpolation=cv2.INTER_AREA)
            frame = Image.fromarray(np.uint8(frame))
            frame,boxes,predicted_class=yolo.detect_image(frame)
            if(len(boxes)>0):
                IsDetected=True
                break

        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()
    cam.close()
    # print("\nFINISH")
if __name__ == '__main__':
    detect()