#!/usr/bin/env python 
# -*- coding:utf-8 -*-
import time
import sys
import pyzed.sl as sl
import cv2
import numpy as np
from PIL import Image
from yolo import YOLO
yolo = YOLO()
file_path='video.svo'
t1 = time.time()
input_type = sl.InputType()
input_type.set_from_svo_file(file_path)
init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
cam = sl.Camera()
status = cam.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()
runtime = sl.RuntimeParameters()
mat = sl.Mat()
key = ''
fps = 10.0
while key != 113:  # for 'q' key
    err = cam.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        cam.retrieve_image(mat)
        frame=mat.get_data()
        frame = Image.fromarray(np.uint8(frame))
        # 进行检测
        frame = np.array(yolo.detect_image(frame))
        fps = (fps + (1. / (time.time() - t1))) / 2
        print("fps= %.2f" % (fps))
        frame = cv2.putText(frame, "fps= %.2f" % (fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("ZED", frame)
        key = cv2.waitKey(1)
    else:
        key = cv2.waitKey(1)
cv2.destroyAllWindows()
