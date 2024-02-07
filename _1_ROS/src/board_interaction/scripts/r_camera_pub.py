#!/usr/bin/env python3
import os
import sys
import time
import json
import rospy
import cv2
import math

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_ROS")[0]) #add parent folder to python path
from _2_CAMERA import ros_detectAruco, ros_arucoCalc
from balise_msgs.msg import PositionPx, PositionPxRectangle, ArrayPositionPxRectangle


"""
Publish elements seen by the camera robot
"""
def publisher():

    # This node will publish all Aruco seen by the robot
    arucoPos_pub = rospy.Publisher("robot1/position/aruco", ArrayPositionPxRectangle, queue_size=10)

    #Tell node name to rospy
    rospy.init_node("r_camera", anonymous=True)

    #Set publish rate
    rate = rospy.Rate(10) #in hz

    #Open camera for video capturing
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    while not rospy.is_shutdown():

        ret,frame = cap.read() #read an image from camera

        #Go to next loop if there is no image to read
        if not ret:
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Problème avec la caméra.")
            continue

        #Search for Aruco tags
        ret, corners, ids = ros_detectAruco.detectAruco(frame)
        #Go to next loop if Aruco cannot be detected
        if not ret:
            #print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Aucun Aruco détecté.")
            continue

        arucoPos_pub.publish(getPositionArucoMsg(corners, ids))
                             
        rate.sleep() #wait according to publish rate

    cap.release() #free camera



"""
Get message for realtime position of arucos.
    corners (list)      ->      all detected aruco corners.
    ids (list)          ->      all detected aruco ids.

Return an ArrayPositionPxRectangle msg.
"""
def getPositionArucoMsg():
    msg = [] #create msg
    aruco_msg = PositionPxRectangle()
    corner = PositionPx() #all corners will use the same instance of PositionPx
    corner.theta = 0 #no theta

    #Get all Aruco tags
    ids, corners = ros_detectAruco.detectAruco()

    #Fill msg array with all detected aruco tag
    for k in range(len(ids)):
        #Save its id
        aruco_msg.id = ids[k]
        
        #Save bottom-left corner
        corner.x, corner.y = corners[k][0]
        aruco_msg.a_px = corner
        
        #Save top-left corner
        corner.x, corner.y = corners[k][1]
        aruco_msg.b_px = corner
        
        #Save top-right corner
        corner.x, corner.y = corners[k][2]
        aruco_msg.c_px = corner
        
        #Save bottom-right corner
        corner.x, corner.y = corners[k][3]
        aruco_msg.d_px = corner

        #Add one aruco to rectangle list
        msg.append(aruco_msg)
    
    return msg

