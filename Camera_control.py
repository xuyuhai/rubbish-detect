import cv2
import pyzed.sl as sl
camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1


def main():
    print("Running...")
    init = sl.InitParameters()
    cam = sl.Camera()
    init.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init.camera_fps = 30  # Set fps at 30
    if not cam.is_opened():
        print("Opening ZED Camera...")
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    runtime = sl.RuntimeParameters()
    mat = sl.Mat()

    print_camera_information(cam)
    print_help()

    key = ''
    while key != 113:  # for 'q' key
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.RIGHT)
            cv2.imshow("ZED", mat.get_data())
            key = cv2.waitKey(5)
            settings(key, cam, runtime, mat)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()

    cam.close()
    print("\nFINISH")


def print_camera_information(cam):
    print("Resolution: {0}, {1}.".format(round(cam.get_camera_information().camera_resolution.width, 2), cam.get_camera_information().camera_resolution.height))
    print("Camera FPS: {0}.".format(cam.get_camera_information().camera_fps))
    print("Firmware: {0}.".format(cam.get_camera_information().camera_firmware_version))
    print("Serial number: {0}.\n".format(cam.get_camera_information().serial_number))


def print_help():
    print("Help for camera setting controls")
    print("  Increase camera settings value:     +")
    print("  Decrease camera settings value:     -")
    print("  Switch camera settings:             s")
    print("  Reset all parameters:               r")
    print("  Record a video:                     z")
    print("  save a image :                      o")
    print("  Quit:                               q\n")


def settings(key, cam, runtime, mat):
    if key == 115:  # for 's' key
        switch_camera_settings()
    elif key == 43:  # for '+' key
        current_value = cam.get_camera_settings(camera_settings)
        cam.set_camera_settings(camera_settings, current_value + step_camera_settings)
        print(str_camera_settings + ": " + str(current_value + step_camera_settings))
    elif key == 45:  # for '-' key
        current_value = cam.get_camera_settings(camera_settings)
        if current_value >= 1:
            cam.set_camera_settings(camera_settings, current_value - step_camera_settings)
            print(str_camera_settings + ": " + str(current_value - step_camera_settings))
    elif key == 114:  # for 'r' key
        cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
        print("Camera settings: reset")
    elif key == 122:  # for 'z' key
        record(cam, runtime, mat)
    elif key ==111 :
        img = sl.ERROR_CODE.FAILURE
        while img != sl.ERROR_CODE.SUCCESS:
            filepath = input("Enter filepath name: ")
            img = mat.write(filepath)
            print("Saving image : {0}".format(repr(img)))
            if img == sl.ERROR_CODE.SUCCESS:
                break
            else:
                print("Help: you must enter the filepath + filename + PNG extension.")


def switch_camera_settings():
    global camera_settings
    global str_camera_settings
    if camera_settings == sl.VIDEO_SETTINGS.BRIGHTNESS:
        camera_settings = sl.VIDEO_SETTINGS.CONTRAST
        str_camera_settings = "Contrast"
        print("Camera settings: CONTRAST")
    elif camera_settings == sl.VIDEO_SETTINGS.CONTRAST:
        camera_settings = sl.VIDEO_SETTINGS.HUE
        str_camera_settings = "Hue"
        print("Camera settings: HUE")
    elif camera_settings == sl.VIDEO_SETTINGS.HUE:
        camera_settings = sl.VIDEO_SETTINGS.SATURATION
        str_camera_settings = "Saturation"
        print("Camera settings: SATURATION")
    elif camera_settings == sl.VIDEO_SETTINGS.SATURATION:
        camera_settings = sl.VIDEO_SETTINGS.SHARPNESS
        str_camera_settings = "Sharpness"
        print("Camera settings: Sharpness")
    elif camera_settings == sl.VIDEO_SETTINGS.SHARPNESS:
        camera_settings = sl.VIDEO_SETTINGS.GAIN
        str_camera_settings = "Gain"
        print("Camera settings: GAIN")
    elif camera_settings == sl.VIDEO_SETTINGS.GAIN:
        camera_settings = sl.VIDEO_SETTINGS.EXPOSURE
        str_camera_settings = "Exposure"
        print("Camera settings: EXPOSURE")
    elif camera_settings == sl.VIDEO_SETTINGS.EXPOSURE:
        camera_settings = sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE
        str_camera_settings = "White Balance"
        print("Camera settings: WHITEBALANCE")
    elif camera_settings == sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE:
        camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
        str_camera_settings = "Brightness"
        print("Camera settings: BRIGHTNESS")


def record(cam, runtime, mat):
    vid = sl.ERROR_CODE.FAILURE
    out = False
    while vid != sl.ERROR_CODE.SUCCESS and not out:
        filepath = input("Enter filepath name: ")
        record_param = sl.RecordingParameters(filepath)
        vid = cam.enable_recording(record_param)
        print(repr(vid))
        if vid == sl.ERROR_CODE.SUCCESS:
            print("Recording started...")
            out = True
            print("Hit spacebar to stop recording: ")
            key = False
            while key != 32:  # for spacebar
                err = cam.grab(runtime)
                if err == sl.ERROR_CODE.SUCCESS:
                    cam.retrieve_image(mat)
                    cv2.imshow("ZED", mat.get_data())
                    key = cv2.waitKey(5)
        else:
            print("Help: you must enter the filepath + filename + SVO extension.")
            print("Recording not started.")
    cam.disable_recording()
    print("Recording finished.")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()