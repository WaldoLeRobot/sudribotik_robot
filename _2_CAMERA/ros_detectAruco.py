import os
import cv2
import numpy as np


FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


"""
Detect ArUco markers.
    frame (numpy.ndArray)  ->  data array of the image.

Return function success, corners positions, their ids.
"""
def detectAruco(frame):

    #Get minimal Aruco ductionnary needed
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    #Create parametors for detection
    aruco_parameters = cv2.aruco.DetectorParameters()

    #Create Aruco detector
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

    #Detect ArUco markers
    corners, ids, rejected_corners = aruco_detector.detectMarkers(frame)

    #Tell if no ids
    if ids is None:
        #print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Aucun tag n'a été détecté.")
        return False, None, None
    
    #Flatten ids
    ids = list(ids.flatten())

    #Reshape corners to facilitate their navigation, perform a cast to int too
    corners = list(map(lambda x : x.astype(int).reshape((4,2)), corners))

    return True, corners, ids
