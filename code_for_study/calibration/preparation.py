import numpy as np
import cv2
import time
import datetime
LEFT_PATH = "/Users/admin/Desktop/Robot/EIIC/left/{:06d}.png"
RIGHT_PATH = "/Users/admin/Desktop/Robot/EIIC/right/{:06d}.png"

CAMERA_WIDTH = 2560
CAMERA_HEIGHT = 720

# TODO: Use more stable identifiers
cam = cv2.VideoCapture(0)


# Increase the resolution
cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
# left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
# right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
# CROP_WIDTH = 960
# def cropHorizontal(image):
#     return image[:,
#             int((CAMERA_WIDTH-CROP_WIDTH)/2):
#             int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

frameId = 1
start_time = datetime.datetime.now()
# Grab both frames first, then retrieve to minimize latency between cameras
try:
    while(True):
        if not (cam.grab()):
            print("No more frames")
            break

        _, camFrame = cam.retrieve()
        leftFrame = camFrame[:, :1280]
        rightFrame = camFrame[:, 1280:]

        if (datetime.datetime.now()-start_time).total_seconds() >= 1:
            cv2.imwrite(LEFT_PATH.format(frameId), leftFrame)
            cv2.imwrite(RIGHT_PATH.format(frameId), rightFrame)
            start_time = datetime.datetime.now()
            print ("Capture", frameId)
            frameId += 1

        w = (datetime.datetime.now()-start_time).total_seconds()/1 * 2560
        camFrame[:8,0:int(w)] = (0,0,255)
        camFrame = cv2.resize(camFrame, (0,0), fx=0.25, fy=0.25)
        cv2.imshow('cam', camFrame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print (e)

cam.release()
cv2.destroyAllWindows()
