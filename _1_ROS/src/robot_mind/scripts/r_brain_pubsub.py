#!/usr/bin/env python3
import os
import sys
import rospy
import traceback
from beacon_msgs.msg import ArrayPositionPx, PositionPx, ArrayPositionPxRectangle, Score

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_ROS")[0]) #add parent folder to python path
from _3_BASE_DE_DONNEES import databaseManager

    
class RBrainNode:
    """
    This ROS centralize all data from other node to insert them into a database.
    """
    
    
    def __init__(self):        
    
        #Initialize  and tell node name to rospy
        rospy.init_node('r_brain', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("robot1/position/self", PositionPx, self.robotPosFromOdoCallback)
        rospy.Subscriber("robot1/position/otherRobots", ArrayPositionPx, self.otherRobotsPosFromLidarCallback)
        rospy.Subscriber("robot1/position/aruco", ArrayPositionPxRectangle, self.arucoPosFromCameraCallback)
        rospy.Subscriber("robot1/score", Score, self.scoreCallback)


    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz
        
        while not rospy.is_shutdown():
            
            rate.sleep() #wait according to publish rate



    def robotPosFromOdoCallback(self, data):
        """
        Callback for robot self position given by odometry.
        """

        data_dict = {}

        #Fill the dict with robot self position
        data_dict["position_x"] = data.x
        data_dict["position_y"] = data.y
        data_dict["position_theta"] = data.theta

        #Update database
        databaseManager.insertData("r_self_robot", data_dict)    


    def otherRobotsPosFromLidarCallback(self, data):
        """
        Callback for position of other robots detectd and calculated by the lidar node.
        """
        
        #Fill the dict with positions of other robot
        data_dict = {}
        for pos in data.array_of_positionspx:
            data_dict["position_x"] = pos.x
            data_dict["position_y"] = pos.y
            data_dict["position_theta"] = pos.theta

            #Update database
            databaseManager.insertData("r_other_robots", data_dict)


    def arucoPosFromCameraCallback(self, data):
        """
        Callback for aruco tags raw position detected by robot front camera.
        """

        #Fill the dict with aruco tag data
        data_dict = {} 
        for tag in data.array_of_rectangles:
            data_dict["tag"] = tag.id

            data_dict["ax"] = tag.a_px.x
            data_dict["ay"] = tag.a_px.y

            data_dict["bx"] = tag.b_px.x
            data_dict["by"] = tag.b_px.y

            data_dict["cx"] = tag.c_px.x
            data_dict["cy"] = tag.c_px.y

            data_dict["dx"] = tag.d_px.x
            data_dict["dy"] = tag.d_px.y
        
            #Update database
            databaseManager.insertData("r_aruco", data_dict)


    def scoreCallback(self, data):
        """
        Callback for score points.
        """

        data_dict = {}

        #Fill the dict with points
        data_dict["points"] = data.score

        #Update database
        databaseManager.insertData("r_score", data_dict)



if __name__ == '__main__':
    #Launch the node
    try:
        databaseManager.init_database_robot()
        brain = RBrainNode()#instantiate it
        brain.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")

