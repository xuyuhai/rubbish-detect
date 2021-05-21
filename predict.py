'''
predict.py有几个注意点
1、无法进行批量预测，如果想要批量预测，可以利用os.listdir()遍历文件夹，利用Image.open打开图片文件进行预测。
2、如果想要保存，利用r_image.save("img.jpg")即可保存。
3、如果想要获得框的坐标，可以进入detect_image函数，读取top,left,bottom,right这四个值。
4、如果想要截取下目标，可以利用获取到的top,left,bottom,right这四个值在原图上利用矩阵的方式进行截取。
'''
from PIL import Image
import math
import cv2
from yolo import YOLO
import numpy as np
yolo = YOLO()

while True:
    img = input('Input image filename:')
    try:
        image = Image.open(img)

    except:
        print('Open Error! Try again!')
        continue
    else:
        # image=cv2.resize(image,(640,360))
        image = image.resize((640, 360), Image.ANTIALIAS)
        # image=np.array(image)
        r_image ,boxes,Class = yolo.detect_image(image)

        print(boxes)
        # if len(boxes) > 0:
        #     x1 = boxes[0][0]
        #     y1 = boxes[0][1]
        #     x2 = boxes[0][2]
        #     y2 = boxes[0][3]
        #     center_x = round(x1 + (x2 - x1) // 2)
        #     center_y = round(y2)
        #     disy = math.fabs(180 - center_y)
        #     disx = math.fabs(320 - center_x)
        #     bate1 = 70 * (disy / 360)
        #     bate1=math.pi*(bate1-30)/180
        #     Distance_Y = 3.5 /math.tan(bate1)
        #     bate2 = 110 * (disx / 640)
        #     bate2=math.pi*(bate2)/180
        #     L = 3.5/math.sin(bate1)
        #     Distance_X = L * math.tan(bate2)
        #     Distance = math.sqrt(Distance_X * Distance_X + Distance_Y * Distance_Y)+23
        #     print(Distance)
        r_image.show()