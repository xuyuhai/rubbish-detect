import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2


# 这里的数据就是之前的代码得到的HSV的上下限
lower_blue = np.array([10, 192, 44])
upper_blue = np.array([179, 255, 255])
font = cv2.FONT_HERSHEY_SIMPLEX


# 调用ZED的代码
def main():
    print("Running...")
    init = sl.InitParameters()
    zed = sl.Camera()
    if not zed.is_opened():
        print("Opening ZED Camera...")
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    runtime = sl.RuntimeParameters()


    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720

    mat = sl.Mat()
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    key = ''
    while key != 113:  # for 'q' key
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            # time t1
            t1 = cv2.getTickCount()
            # threshold
            frame = mat.get_data()
            # 因为ZED是双目相机，这里只使用左侧相机的图像检测物体
            frame = cv2.resize(frame, (int(mat.get_width() / 2), int(mat.get_height() / 2)),
                               interpolation=cv2.INTER_AREA)
            # 基于颜色的目标识别
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            # cv2.imshow("result",result)
            # result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
            result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            blurred = cv2.blur(mask, (5, 5))
            (_, threshold) = cv2.threshold(blurred, 5, 255, cv2.THRESH_BINARY)
            cnts, h = cv2.findContours(threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.imshow("threshold", threshold)
            # morphology
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 50))
            closing = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)
            closed = cv2.erode(closing, None, iterations=4)
            closed = cv2.dilate(closed, None, iterations=4)
            # findcontours

            cnts, h = cv2.findContours(closed.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(cnts) > 0:
                c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
                rect = cv2.minAreaRect(c)
                print(rect)
                box = cv2.boxPoints(rect)
                print(box)
                box = np.int0(box)
                cv2.drawContours(frame, [box], -1, (0, 255, 0), 3)

                # get the coordinate of the contours
                # 计算目标框的坐标以及中心点


                Xs = [i[0] for i in box]
                Ys = [i[1] for i in box]

                x1 = min(Xs)
                x2 = max(Xs)
                y1 = min(Ys)
                y2 = max(Ys)
                # distance
                x = round(x1 + x2)
                y = round(y1 + y2)

                print(x,y)
                # 获取中心点的点云数据并计算距离
                err, point_cloud_value = point_cloud.get_value(x, y)

                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                     point_cloud_value[1] * point_cloud_value[1] +
                                     point_cloud_value[2] * point_cloud_value[2])
                t2 = cv2.getTickCount()
                if not np.isnan(distance) and not np.isinf(distance):
                    distance = round(distance)
                    print("Distance to Camera at ({0}, {1}): {2} mm\n".format(x, y, distance))
                    # time t2

                # Increment the loop
                else:
                    print("Can't estimate distance at this position, move the camera\n")
                sys.stdout.flush()
                t = (t2 - t1) / cv2.getTickFrequency()
                FPS = 1 / t
                fps = "Camera FPS: {0}.".format(FPS)
                cv2.putText(frame, fps, (50, 50), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                Z = "Distance: {} mm".format(distance)
                cv2.putText(frame, Z, (x2, y2), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("ZED", frame)
                key = cv2.waitKey(5)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()

    zed.close()
    print("\nFINISH")


if __name__ == "__main__":
    main()

