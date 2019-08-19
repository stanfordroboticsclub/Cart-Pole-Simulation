


import numpy as np
import cv2
import cv2.aruco as aruco

from imutils.video import VideoStream
import imutils

vs = VideoStream(usePiCamera=1).start()

while (True):
    frame = vs.read()

    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners != []:
        print(ids, angle(corners))


def angle(corners):
    corner = corners[0][0,:,:]
    mean = np.mean(corner,axis=0)
    di = corner[0,:] - mean
    return np.degrees(np.arctan2(di[0],di[1]))

    
