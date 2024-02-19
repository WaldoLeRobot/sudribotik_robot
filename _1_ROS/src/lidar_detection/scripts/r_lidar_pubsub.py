import numpy as np
import math
import os
import rospy
import serial
import sys
import traceback
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from beacon_msgs.msg import ArrayPositionPx, ArrayPositionPxWithType, ArrayPositionPxRectangle

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)



class LidarNode:
    """
    This ROS node fetch raw data provided by the lidar and pusblish detected objects 
    within the board area.
    To accelerate all calcutions, constants can directly be replaced  by their value in the code.
    """

    def __init__(self):

        #Constants board values
        self.BOARD_WIDTH_IN_METER = 0.3
        self.BOARD_HEIGHT_IN_METER = 0.3

        #Initialize  and tell node name to rospy
        rospy.init_node('r_lidar', anonymous=True)

        #This node will listen to these topics
        #rospy.Subscriber("robot1/position/self", Point, self.getSelfPositionCallback)
        self.self_position_x = 150#None #attributes saving self position in mm
        self.self_position_y = 150#None 
        self.self_position_z = 0 #Z position will not be updated
        self.self_position_theta = 0 #None #in radians (60°)

        rospy.Subscriber("robot1/lidar/rawdata", LaserScan, self.getLidarDataCallback)
        self.lidar_ranges = None#attributes saving lidar data
        self.lidar_intensities = None
        self.NB_OF_RANGES = 456 #constant number of different laser angles used by the lidar
        self.ANGLE_INCREMENT = self.NB_OF_RANGES/(2*math.pi) #unitary angle between two consecutives lasers

        #This node will publish to this topic
        self.otherRobotsPos_pub = rospy.Publisher("robot1/position/otherRobots", PoseArray, queue_size=10)

    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz
        
        while not rospy.is_shutdown():

            #Publish only if we know our self position and if the serial port is still open
            if self.self_position_x and self.self_position_y:
                
                self.getPositionOfOtherRobotsMsg()
                #self.otherRobotsPos_pub.publish(self.getPositionOfOtherRobotsMsg()) #publish

                #self.self_position_x = None #current position reset
                #self.self_position_y = None 
                #self.self_position_theta = None 
            
            rate.sleep() #wait according to publish rate



    def getSelfPositionCallback(self, data):
        """
        Callback for self position.
        """
        self.self_position_x = data.x #in milimeter
        self.self_position_y = data.y

        #Convert theta from range [-pi;pi] to [0;2pi]
        # so its still in radians but in a different range.
        self.self_position_theta = (data.theta + 2*math.pi) % (2*math.pi)


    def getLidarDataCallback(self, data):
        """
        Callback for lidar data.
        """
        self.lidar_ranges = data.ranges
        self.lidar_intensities = data.intensities

    def isRangeOnBoard(self, index, range, x_robot, y_robot, theta_robot):
        """
        Calculate whether a range is on board or not.
            index (int)         ->  index of the range based on his angle.
            range (float)       ->  lidar range of a single pulse (in meter).
            x_robot (float)     ->  position x of the robot in meters.
            y_robot (float)     ->  position y of the robot in meters.
            theta_robot (float) ->  angle of the robot in radians [0;2pi].
        
        Return True if the range is on board, False if not.
        """

        #Filter nan values
        if not range :
            return False
        
        #Find laser quadrant
        theta_relative = (theta_robot + (index*self.ANGLE_INCREMENT)) % (2*math.pi) #angle of the laser relative to the board
        quadrant = math.ceil(theta_relative/(math.pi/2)+0.00001) #in case theta_relative=0 we add a tiny float number

        #Get values to compare x and y projection of the range
        if quadrant == 1 :
            max_x_inside = self.BOARD_WIDTH_IN_METER - x_robot
            max_y_inside = y_robot

        elif quadrant == 2 :
            max_x_inside = x_robot
            max_y_inside = y_robot

        elif quadrant == 3 :
            max_x_inside = x_robot
            max_y_inside = self.BOARD_HEIGHT_IN_METER - y_robot

        elif quadrant == 4 :
            max_x_inside = self.BOARD_WIDTH_IN_METER - x_robot
            max_y_inside = self.BOARD_HEIGHT_IN_METER - y_robot
                    
        #Project the range on the x and y axes
        x_projection = abs(range * math.cos(self.ANGLE_INCREMENT*index))
        y_projection = abs(range * math.sin(self.ANGLE_INCREMENT*index))            

        #Compare these values to know if they are inside or not
        return (x_projection < max_x_inside) and (y_projection < max_y_inside)

        



    def getPositionOfOtherRobotsMsg(self):
        """
        Create message containing position of other robots on the board.
        """
        msg = [] #create msg

        #Wait for the callback giving those values
        if self.lidar_ranges and self.self_position_x :
            #We'll make several operations on lidar ranges so we need to freeze them with the current robot postion
            current_lidar_ranges = self.lidar_ranges
            current_robot_x = self.self_position_x/1000 #cast the mm pos to meter
            current_robot_y = self.self_position_y/1000
            current_robot_theta = self.self_position_theta

        else :
            return msg

        #Filter positions outside of the board by creating a mask.
        # this mask will contain 456 boolean values paired with their lidar_range.
        # if the range is on board its corresponding index is set to True.
        mask_on_range = [self.isRangeOnBoard(index, range,
                                             current_robot_x,
                                             current_robot_y,
                                             current_robot_theta) for index, range in enumerate(current_lidar_ranges)]

        #Apply the mask on the ranges
        lidar_ranges_on_board = [value for value, mask_value in zip(current_lidar_ranges, mask_on_range) if mask_value]

        #Create a list tracking index of ranges inside the board
        lidar_ranges_on_board_index = [idx for idx in range(len(current_lidar_ranges)) if mask_on_range[idx]]

        print(lidar_ranges_on_board[:20])
        print(lidar_ranges_on_board_index[:20])

        #Regroup each range with close index then take the middle one


        other_robot_pos = Pose()
        other_robot_point = Point()
        other_robot_point.z = 0

        return msg


if __name__ == '__main__':
    #Launch the node
    try:
        lidar = LidarNode()#instantiate it
        lidar.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")