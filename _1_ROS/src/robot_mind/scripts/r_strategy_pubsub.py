#!/usr/bin/env python3
import os
import sys
import rospy
import sqlite3
import traceback
from beacon_msgs.msg import ArrayPositionPx, ArrayPositionPxWithType, ArrayPositionPxRectangle

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_ROS")[0]) #add parent folder to python path
from _4_SERIALUS_M2M import serialusM2M
    
class RStrategyNode:
    """
    This ROS is responsible of direct actions of the robot.
    """
    
    
    def __init__(self):        
    
        #Initialize  and tell node name to rospy
        rospy.init_node('r_strategy', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("robot1/position/aruco", ArrayPositionPxRectangle, self.arucoPosFromRobotCallback)
        self.aruco_pos = None

        rospy.Subscriber("robot1/position/otherRobots", ArrayPositionPx, self.otherRobotsPosFromLidarCallback)
        self.other_robots_pos = None

        #Establish serial connection
        self.serial_asserv = self.setSerialConnection()


    def getSerialConnection():
        """
        Get serial connection between this raspberry and the asserv card.
        """
        serial_port = '/dev/ttyUSB1'
        Baudrate=1000000    
        ttl9600=False
        Timeout=5

        return serialusM2M.init_serial(Baudrate,serial_port,Timeout,ttl9600)


    def run(self):
        """
        OPERATE HERE, ALL ACTIONS MUST BE WROTE IN THE WHILE LOOPNOF THIS FUNCTION
        """
        #Set publish rate
        rate = rospy.Rate(10) #in hz
        
        while not rospy.is_shutdown():
            
            rate.sleep() #wait according to publish rate
        
        #Close serial connection
        self.serial_asserv.close()


    def arucoPosFromRobotCallback(self, data):
        """
        Callback for aruco tags raw position detected by robot front camera.
        """
        self.aruco_pos = data.array_of_rectangles


    def otherRobotsPosFromLidarCallback(self, data):
        """
        Callback for position of other robots detectd and calculated by the lidar node.
        """
        self.other_robots_pos = data.array_of_positionspx







if __name__ == '__main__':
    #Launch the node
    try:
        strategy = RStrategyNode()#instantiate it
        strategy.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")