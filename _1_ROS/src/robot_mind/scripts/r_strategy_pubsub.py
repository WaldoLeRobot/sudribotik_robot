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
        self.robotPos_pub = rospy.Publisher("robot1/position/self", ArrayPositionPxRectangle, queue_size=10)
        self.score_pub = rospy.Publisher("robot1/score", Score, queue_size=10)
        self.score = Score() #score initialized to 0
        self.score.score = 0

        #Establish serial connection
        self.serial_asserv = None
        while not self.serial_asserv:
             self.serial_asserv = self.setSerialConnection()

        #Set PID parameters
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Définitition du pid...")
        fpid.set_ENTRAXE_MM("033600", self.serial_asserv)
        fpid.set_PERIMETRE_ROUE_MM("021000", self.serial_asserv)
        fpid.set_PID_VITESSE_DIST("06500", "05500", "99000", self.serial_asserv)
        fpid.set_PID_BREAK("01250", "00600","20000", self.serial_asserv)
        fpid.set_MAX_ERREUR_INTEGRALLE_V("045000", self.serial_asserv)
        fpid.set_MAX_E_INTEGRALLE_BRAKE("000500", self.serial_asserv)

        #Set position
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Définitition de la position...")
        fcalage.set_pos("2500", "0500", "090", self.serial_asserv)

        #Publish self position timer callback
        rospy.Timer(rospy.Duration(1.0/20.0), self.publishSelfPosition) #publish at a 20hz rate


    def setSerialConnection(self):
        """
        Set serial connection between this raspberry and the asserv card.
        """

        #Set free access to tty directories to everyone (NOT SAFE AT ALL BUT EZ TO DO)
        password_stdin = subprocess.run(["echo", self.config["WHO_AM_I"][2]], stdout=subprocess.PIPE)
        subprocess.run(["sudo", "chmod", "777", "/dev/tty*"], stdin=password_stdin.stdout, stdout=subprocess.PIPE)
        exit(0)
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
            
            #Publish    
            fdd.orienter("100","100", self.serial_asserv)    
            fdd.avancer("100","100", self.serial_asserv)
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
    


    def publishSelfPosition(self):
        """
        Publish self position of the robot on the board.
        """
        self_pos = PositionPx()
        
        #Test if serial connection is already initialized
        if self.serial_asserv :
            ret_selfpos = fcalage.get_pos(self.serial_asserv)
            self_pos.x = int(float(ret_selfpos[0]))
            self_pos.y = int(float(ret_selfpos[1]))

            #Convert theta from range [0; 360] to [0;2pi]
            self_pos.theta = (float(ret_selfpos[0]))/180*math.pi

        #Publish self postition
        self.robotPos_pub.pusblish(self_pos)






if __name__ == '__main__':
    #Launch the node
    print("START STRATEGY")
    strategy = RStrategyNode()#instantiate it
    strategy.run()
    #except Exception as e:
        #traceback.print_exc()
        #print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
