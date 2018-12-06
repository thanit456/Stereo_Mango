import sys
import numpy as np
import cv2

REMAP_INTERPOLATION = cv2.INTER_LINEAR

DEPTH_VISUALIZATION_SCALE = 2048

calibration = np.load('output.npz', allow_pickle=None)
print("NP_LOAD is completed")
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])

CAMERA_WIDTH = 2560
CAMERA_HEIGHT = 720

# TODO: Use more stable identifiers
cam = cv2.VideoCapture(0)

# Increase the resolution
cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


# TODO: Why these values in particular?
# TODO: Try applying brightness/contrast/gamma adjustments to the images
window_size = 11
min_disp = 2
num_disp = 130-min_disp
left_matcher = cv2.StereoSGBM_create(minDisparity = min_disp,
    numDisparities = num_disp,
    blockSize = window_size,
    uniquenessRatio = 5,
    speckleWindowSize = 100,
    speckleRange = 32,
    disp12MaxDiff = 1,
    P1 = 8*3*window_size**2,
    P2 = 32*3*window_size**2)



# FILTER Parameters
lmbda = 80000
sigma = 1.2
visual_multiplier = 1.0

print('computing disparity...')

# Grab both frames first, then retrieve to minimize latency between cameras
while(True):
    if not (cam.grab()):
        print("No more frames")
        break

    _, camFrame = cam.retrieve()
    leftFrame = camFrame[:, :1280]
    rightFrame = camFrame[:, 1280:]
    leftHeight, leftWidth = leftFrame.shape[:2]
    rightHeight, rightWidth = rightFrame.shape[:2]

    if (leftWidth, leftHeight) != imageSize:
        print("Left camera has different size than the calibration data")
        break

    if (rightWidth, rightHeight) != imageSize:
        print("Right camera has different size than the calibration data")
        break

    fixedLeft = cv2.remap(leftFrame, leftMapX, leftMapY, REMAP_INTERPOLATION)
    fixedRight = cv2.remap(rightFrame, rightMapX, rightMapY, REMAP_INTERPOLATION)
    imgL = cv2.cvtColor(leftFrame, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(rightFrame, cv2.COLOR_BGR2GRAY)
   
    if 1:
        displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
        displ = np.int16(displ)
        filteredImg = displ
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        filteredImg = np.uint8(filteredImg)
        cv2.imshow('Disparity Map', filteredImg)
    else :
        cv2.imshow('After', cv2.resize(np.concatenate((np.concatenate((leftFrame,fixedLeft)),np.concatenate((rightFrame,fixedRight))),1), (0,0), fx=0.5, fy=0.5))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
