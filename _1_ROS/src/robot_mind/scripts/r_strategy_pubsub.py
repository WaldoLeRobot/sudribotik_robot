#!/usr/bin/env python3
import os
import sys
import rospy
import traceback
from beacon_msgs.msg import ArrayPositionPx, ArrayPositionPxWithType, ArrayPositionPxRectangle

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

#sys.path.insert(1, FILE_PATH.split("_1_ROS")[0]) #add parent folder to python path
sys.path.insert(1, FILE_PATH.split("_1_ROS")[0] + "_4_SERIALUS_M2M/") #add SerialusM2M path
import serialusM2M as serialus
import fonction_deplacement as fdd
import fonction_Pos_Calage as fcalage
import fonction_PID as fpid
    
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
        self.serial_asserv = None
        while not self.serial_asserv:
             self.serial_asserv = self.setSerialConnection()

        #Set PID parameters
        fpid.set_ENTRAXE_MM("033600", self.serial_asserv)
        fpid.set_PERIMETRE_ROUE_MM("021000", self.serial_asserv)
        fpid.set_PID_VITESSE_DIST("06500", "05500", "99000", self.serial_asserv)
        fpid.set_PID_BREAK("01250", "00600","20000", self.serial_asserv)
        fpid.set_MAX_ERREUR_INTEGRALLE_V("045000", self.serial_asserv)
        fpid.set_MAX_E_INTEGRALLE_BRAKE("000500", self.serial_asserv)


    def setSerialConnection(self):
        """
        Set serial connection between this raspberry and the asserv card.
        """
        serial_port = '/dev/ttyUSB0'
        Baudrate=1000000    
        ttl9600=False
        Timeout=5
        #Try to instanciate a comunication between the Pi and the asserv card
        try:
            serial = serialus.init_serial(Baudrate,serial_port,Timeout,ttl9600)
        except Exception as e:
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Avez-vous bien branché la carte d'asservissement "+\
                  f"sur le port {serial_port} ?")
            serial = None

        return serial

    def run(self):
        """
        OPERATE HERE, ALL ACTIONS MUST BE WROTE IN THE WHILE LOOP OF THIS FUNCTION
        """
        #Set publish rate
        rate = rospy.Rate(10) #in hz        

        while not rospy.is_shutdown():
            fdd.avancer("0200","100", self.serial_asserv))
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
    print("START STRATEGY")
    strategy = RStrategyNode()#instantiate it
    strategy.run()
    #except Exception as e:
        #traceback.print_exc()
        #print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
