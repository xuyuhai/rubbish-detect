#!/usr/bin/env python 
# -*- coding:utf-8 -*-
import math
import numpy as np
import pyzed.sl as sl
from PIL import Image
from yolo import YOLO
import cv2
import MySQLdb
camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1
yolo = YOLO()
trash={
    '其他垃圾':['modulator_tube','cigarette_butt','toothbrush '],
    '可回收垃圾':['book','can ','carton','cigarette_case','lunch_box','glass_bottle','plastic_bottle','remote_control','shoe','t_shirt','paper_cup','tin_can','trousers','waste_paper','mobilephone','pencil'],
    '有害垃圾':['button_battery','battery','capsule','mercurial_thermometer','mask','buttoncell'],
    '厨余垃圾':['applecore', 'banana_peel','tea_leaf ','watermelon_peel','vegetable_leaf','rice']
    }
def distanceToCamera():
    H=0.405 # 摄像头高度 单位为米
    L=0.23  #摄像头底部到车的距离
    Class=''
    Classes=[]
    Distance=[]
    num=0
    distance=float("nan")
    print("Running...")
    init = sl.InitParameters()
    cam = sl.Camera()
    init.camera_resolution = sl.RESOLUTION.HD720  # Use HD1080 video mode
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
    depth = sl.Mat()
    point_cloud = sl.Mat()
    mat = sl.Mat()
    while math.isnan(distance):
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            frame = mat.get_data()
            frame = cv2.resize(frame, (int(mat.get_width() / 2), int(mat.get_height() / 2)),
                               interpolation=cv2.INTER_AREA)
            cv2.imshow('1',frame)
            cv2.waitKey(5)
            frame = Image.fromarray(np.uint8(frame))
            frame, boxes, predicted_class = yolo.detect_image(frame)

            if len(boxes)>0 :
                num=len(boxes)
                for i in range(num):
                    x1=boxes[i][0]
                    y1=boxes[i][1]
                    x2=boxes[i][2]
                    y2=boxes[i][3]
                    center_x=round((x2-x1)//2+x1)
                    center_y=round((y2-y1)//2+y1)
                    err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                    distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                         point_cloud_value[1] * point_cloud_value[1] +
                                         point_cloud_value[2] * point_cloud_value[2])
                    distance=math.sqrt(distance*distance-H*H)
                    distance=distance-L
                    Distance.append(distance)

                    # distance=distance*100
                    # print("Distance to Camera at ({0}, {1}): {2} m\n".format(center_x, center_y, distance))
                    if predicted_class[i] in trash['其他垃圾']:
                        Class='其他垃圾'
                    elif predicted_class[i] in trash['可回收垃圾']:
                        Class='可回收垃圾'
                    elif predicted_class[i] in trash ['有害垃圾']:
                        Class='有害垃圾'
                    elif predicted_class[i] in trash['厨余垃圾']:
                        Class='厨余垃圾'
                    Classes.append(Class)
    return num ,Distance,Classes
def insertinfo():
    num,Distance,Classes=distanceToCamera()
    db = MySQLdb.connect('localhost', 'root', '123456', 'rubbish_cart', charset='utf8')
    # 使用cursor()方法获取操作游标
    cursor = db.cursor()
    for i in range (num):

        print(Classes[i],Distance[i])
        sql = "insert into  trash( trash_sort, distance) values( %(trash_sort)s, %(distance)s)"
        value = {"trash_sort":Classes[i] , "distance": Distance[i]}
        try:
            # 执行sql语句
            cursor.execute(sql, value)
            db.commit()
        except:
            db.rollback()
    db.close()
if __name__ == '__main__':
    insertinfo()












 