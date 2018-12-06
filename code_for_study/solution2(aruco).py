import numpy as np
import cv2.aruco as aruco
import cv2
import sys

REMAP_INTERPOLATION = cv2.INTER_LINEAR
CAMERA_WIDTH = 2560
CAMERA_HEIGHT = 720


calibration = np.load('output.npz', allow_pickle=None)
print("NP_LOAD is completed")
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])


cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


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

	grayL = cv2.cvtColor(leftFrame, cv2.COLOR_BGR2GRAY)
	grayR = cv2.cvtColor(rightFrame, cv2.COLOR_BGR2GRAY)

	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	parameters = aruco.DetectorParameters_create()

	cornersL, idsL, rejectedImgPointsL = aruco.detectMarkers(grayL, aruco_dict, parameters=parameters)
	print("Left corners",cornersL)
	cornersR, idsR, rejectedImgPointsR = aruco.detectMarkers(grayR, aruco_dict, parameters=parameters)
	print("Right corners",cornersR)
	grayL = aruco.drawDetectedMarkers(grayL, cornersL)
	grayR = aruco.drawDetectedMarkers(grayR, cornersR)

	cv2.imshow('Left gray',grayL)
	cv2.imshow('Right gray',grayR)

	if(len(cornersL) != 0 and len(cornersR) != 0){
		disp = abs(cornersR - cornersL)
		print(disp)
	}
	
	if(cv2.waitKey(1) & 0xff == ord('q')):
		break;
cam.release()
cv2.destroyAllWindows()