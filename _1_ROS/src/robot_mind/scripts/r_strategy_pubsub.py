#!/usr/bin/env python3
import os
import sys
import rospy
import traceback
import subprocess
import math
import json
from beacon_msgs.msg import ArrayPositionPx, ArrayPositionPxRectangle, PositionPx, Score

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_ROS")[0] + "_4_SERIALUS_M2M/") #add SerialusM2M path
import serialusM2M as serialus
import fonction_deplacement as fdd
import fonction_Pos_Calage as fcalage
import fonction_PID as fpid

configuration_FILEPATH = FILE_PATH.split("_1_ROS")[0]+"init/configuration.json"


    
class RStrategyNode:
    """
    This ROS is responsible of direct actions of the robot.
    """
    
    
    def __init__(self):

        #Get configs    
        with open (configuration_FILEPATH, "r") as f:
            self.config = json.load(f)
    
        #Initialize  and tell node name to rospy
        rospy.init_node('r_strategy', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("robot1/position/aruco", ArrayPositionPxRectangle, self.arucoPosFromRobotCallback)
        self.aruco_pos = None

        rospy.Subscriber("robot1/position/otherRobots", ArrayPositionPx, self.otherRobotsPosFromLidarCallback)
        self.other_robots_pos = None

        #This node will publish to these topics
        self.robotPos_pub = rospy.Publisher("robot1/position/self", PositionPx, queue_size=10)
        self.score_pub = rospy.Publisher("robot1/score", Score, queue_size=10)
        self.score = Score() #score initialized to 0
        self.score.score = 0

        #Establish serial connection
        self.serial_asserv = None
        while not self.serial_asserv:
            self.openSerialConnection()

        #Set PID parameters
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Définitition du pid...")
        fpid.set_ENTRAXE_MM("033600", self.serial_asserv)
        fpid.set_PERIMETRE_ROUE_MM("021000", self.serial_asserv)
        fpid.set_PID_VITESSE_DIST("06500", "05500", "99000", self.serial_asserv)
        fpid.set_PID_BREAK("01250", "00600","20000", self.serial_asserv)
        fpid.set_MAX_ERREUR_INTEGRALLE_V("045000", self.serial_asserv)
        fpid.set_MAX_E_INTEGRALLE_BRAKE("000500", self.serial_asserv)

        #Set position
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Début callage...")
        #fcalage.Callage_All(self.serial_asserv)
        fcalage.set_pos("0350", "0450", "270", self.serial_asserv)

        #Publish self position timer callback
        #rospy.Timer(rospy.Duration(1.0/10.0), self.publishSelfPosition) #publish at a 10hz rate


    def openSerialConnection(self):
        """
        Open serial connection between this raspberry and the asserv card.
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

        self.serial_asserv = serial

    def run(self):
        """
        OPERATE HERE, ALL ACTIONS MUST BE WROTE IN THE WHILE LOOP OF THIS FUNCTION
        """
        #Set publish rate
        rate = rospy.Rate(10) #in hz    

        while not rospy.is_shutdown() and self.serial_asserv:
            
            #Actions
            self.publishSelfPosition()

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
        
        #Print position of other detected object for debug purpose
        print(f"\n-----------------------------------\n"+
              f"Log [{os.times().elapsed}] - {FILE_NAME}"+
              f"\nIl y a {len(self.other_robots_pos)} objet(s) sur le plateau.")
        for k,ovni in enumerate(self.other_robots_pos):
            print(f"\tObjet {k+1}:\n\tx: {ovni.x}\n\ty: {ovni.y}\n")





    def publishSelfPosition(self, time_event=None):
        """
        Publish self position of the robot on the board.
        """
        self_pos = PositionPx()

        #Test if serial connection is opened
        if self.serial_asserv :
            try:
                ret_selfpos = fcalage.get_pos(self.serial_asserv)
                
                self_pos.x = int(float(ret_selfpos[0]))
                self_pos.y = int(float(ret_selfpos[1]))

                #Convert theta from range [0; 360] to [0;2pi]
                self_pos.theta = float(ret_selfpos[2])/180*math.pi
                
             
            except:
                self_pos = PositionPx() #reinitialize the pos so we dont publish partial information
                print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Impossible de publish une position.")

        #Publish
        self.robotPos_pub.publish(self_pos)
        


if __name__ == '__main__':
    #Launch the node
    print("START STRATEGY")
    strategy = RStrategyNode()#instantiate it
    strategy.run()
