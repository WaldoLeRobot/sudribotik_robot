#!/usr/bin/env python3
import numpy as np
import math
import os
import rospy
import sys
import traceback
from sensor_msgs.msg import LaserScan
from beacon_msgs.msg import ArrayPositionPx, PositionPx, ArrayPositionPxRectangle

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
        self.BOARD_WIDTH_IN_METER = 3
        self.BOARD_HEIGHT_IN_METER = 2

        #Initialize  and tell node name to rospy
        rospy.init_node('r_lidar', anonymous=True)

        #This node will listen to these topics
        rospy.Subscriber("robot1/position/self", PositionPx, self.getSelfPositionCallback)
        self.self_position_x = None #attributes saving self position in mm
        self.self_position_y = None 
        self.self_position_theta = None
        
        rospy.Subscriber("robot1/lidar/rawdata", LaserScan, self.getLidarDataCallback)
        self.lidar_ranges = None #length in meter between lidar and touched object 
        self.lidar_intensities = None 
        self.lidar_angle_increment = None #unitary angle between two consecutives lasers, somehow this number vary
        self.lidar_point_size = None #number of laser in a unique turn

        #This node will publish to this topic
        self.otherRobotsPos_pub = rospy.Publisher("robot1/position/otherRobots", ArrayPositionPx, queue_size=10)

    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz
        
        while not rospy.is_shutdown():

            #Publish only if we know our self position and if the serial port is still open
            if self.self_position_x and self.self_position_y:
                
                #Publish
                self.otherRobotsPos_pub.publish(self.getPositionOfOtherRobotsMsg())
            
            rate.sleep() #wait according to publish rate



    def getSelfPositionCallback(self, data):
        """
        Callback for self position.
        """
        self.self_position_x = data.x #in milimeter
        self.self_position_y = data.y

        #Convert theta from range [0; 360] to [0;2pi]
        self.self_position_theta = data.theta/180*math.pi


    def getLidarDataCallback(self, data):
        """
        Callback for lidar data.
        """
        self.lidar_ranges = data.ranges
        self.lidar_intensities = data.intensities
        self.lidar_angle_increment = data.angle_increment
        self.lidar_point_size = len(data.ranges)


    def isRangeOnBoard(self, index, range, angle_increment, x_robot, y_robot, theta_robot):
        """
        Calculate whether a range is on board or not.

        Parameters:
            - index (int): index of the range based on his angle.
            - range (float): lidar range of a single pulse (in meter).
            - angle_increment (float): angle between two lasers pulse (in radians [0;2pi]).
            - x_robot (float): position x of the robot in meters.
            - y_robot (float): position y of the robot in meters.
            - theta_robot (float): angle of the robot in radians [0;2pi].
        
        Returns:
            - bool: True if the range in on board False if not.
            - tuple: two float numbers for the position x,y of a touched object or (None, None).
            - int: quadrant of the laser.
        """

        #Filter nan values
        if not range :
            return False, (None, None), None
        
        #Find laser quadrant
        theta_relative = (theta_robot + (index*angle_increment)) % (2*math.pi) #angle of the laser relative to the board
        quadrant = math.ceil(theta_relative/(math.pi/2)+0.000000000000000000000001) #in case theta_relative=0 we add a tiny float number 
        
        #Get values to compare x and y projection of the range
        if   quadrant == 1 :
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
        else:
            print(f"quadrant:{quadrant}\ntheta_robot:{theta_robot}\nindex:{index}\nangle_increment:{angle_increment}\ntheta_relative:{theta_relative}")
        #Project the range on the x and y axes
        x_projection = abs(range * math.cos(theta_relative))
        y_projection = abs(range * math.sin(theta_relative))         

        #Compare these values to know if they are inside or not
        if (x_projection < max_x_inside) and (y_projection < max_y_inside):

            #Only calculate position on an object if its inside the board to limit calculations
            if   quadrant == 1 :return True, (x_robot + x_projection, y_robot - y_projection), 1
            elif quadrant == 2 :return True, (x_robot - x_projection, y_robot - y_projection), 2
            elif quadrant == 3 :return True, (x_robot - x_projection, y_robot + y_projection), 3
            elif quadrant == 4 :return True, (x_robot + x_projection, y_robot + y_projection), 4
        else :
            return False, (None, None), None

        



    def getPositionOfOtherRobotsMsg(self):
        """
        Create message containing position of other robots on the board.
        """
        msg = [] #create msg

        #Wait for the callback giving those values
        if self.lidar_ranges and self.self_position_x :
            #We'll make several operations on lidar ranges so we need to freeze them with the current robot postion
            current_lidar_ranges = self.lidar_ranges
            current_lidar_angle_increment = self.lidar_angle_increment
            current_lidar_point_size = self.lidar_point_size
            current_robot_x = self.self_position_x/1000 #cast the mm pos to meter
            current_robot_y = self.self_position_y/1000
            current_robot_theta = self.self_position_theta

        else :
            return msg
        
        #Filter out positions outside the board.
        lidar_ranges_on_board_index = [] #this will contain index of ranges inside the board
        lidar_ranges_on_board_position = [] #this will contain ranges position, it will follow the index list
        lidar_ranges_on_board_quadrant = [] #this will contain quadrant of laser, it will follow the index list
        for index, range in enumerate(current_lidar_ranges) :
            
            #Get position of ranges inside the board
            is_inside, pos, quadrant = self.isRangeOnBoard(index, range, current_lidar_angle_increment, current_robot_x, current_robot_y, current_robot_theta)

            if is_inside :
                lidar_ranges_on_board_index.append(index) #store index
                lidar_ranges_on_board_position.append(pos) #store position
                lidar_ranges_on_board_quadrant.append(quadrant) #store quadrant

     
        #Regroup each range with close index then take the middle one     
                           
        #This constant set the maximum number of index missing between two index
        MISSING_VALUE_TOLERANCE = 4     
        #This constant set the minimum number of consecutives ranges detected by the lidar to be considered a solid object
        MINIMUM_NB_OF_RANGES = 3  #it's purpose is to eliminate potential noises
        
        previous_index = None
        middle_index_list = [] #this will store the index of position of other robots
        temp_list = [] #this will store a list of following index
        a=None
        for idx,range_index in enumerate(lidar_ranges_on_board_index):
            
            #The first index is always took 
            if idx == 0:
                previous_index = range_index
                temp_list.append(range_index) #don't forget to add it to the following index list
                continue
            
            #Store next index until the difference between the previous and the current is over the tolerance
            # or if it's the last index in the list
            if (range_index - previous_index > MISSING_VALUE_TOLERANCE) or \
                (idx==len(lidar_ranges_on_board_index)-1):

                #Add the middle index to the middle_index_list only if there is enough ranges.
                if len(temp_list) >= MINIMUM_NB_OF_RANGES :
                    middle_index_list.append(temp_list[len(temp_list)//2])

                    #For the leap problem (explained bellow) we need to store some constants on the first and last temp_list
                    if len(middle_index_list)==1:
                        FIRST_INDEX_OF_FIRST_DETECTED_OBJ = temp_list[0]
                    LAST_INDEX_OF_LAST_DETECTED_OBJ = temp_list[-1]
                    
                #Set a new previous index and clear the following index list to start over
                previous_index = range_index
                temp_list.clear()
                temp_list.append(range_index) 
                continue

            else :
                #The index is considered the part of the group so add it to the list
                temp_list.append(range_index)
                previous_index = range_index

        #Finally there is a leap problem : if an object is seen at the front of the lidar
        # the object will be seen as a group for the first ranges [1,2,3,4,.. and for the last ranges ...,453,454,455,456].
        # Whereas it is only one object at the front we end up with two differents objects dectected.
        # We will address this in 4 steps:               
                          
        #1st We compare the position of the first and the last detected objects to see if there is a leap problem
        # the list of positions should obviously be greater than 1, additionally first and last index of ranges detected should follow up.
        # last index must be part of a detected group
        if  len(middle_index_list)>1 and \
            LAST_INDEX_OF_LAST_DETECTED_OBJ + MISSING_VALUE_TOLERANCE > current_lidar_point_size and \
            FIRST_INDEX_OF_FIRST_DETECTED_OBJ - (LAST_INDEX_OF_LAST_DETECTED_OBJ + MISSING_VALUE_TOLERANCE)  % current_lidar_point_size < 0:  

            #2nd Create a posx and posy with the mean of the first and last positions
            posx_first, posy_first = lidar_ranges_on_board_position[lidar_ranges_on_board_index.index(middle_index_list[0])]
            posx_last, posy_last = lidar_ranges_on_board_position[lidar_ranges_on_board_index.index(middle_index_list[-1])]

            posx_mean = (posx_first + posx_last)/2
            posy_mean = (posy_first + posy_last)/2

            #3rd Replace the position of the first detected object by the mean position
            lidar_ranges_on_board_position[lidar_ranges_on_board_index.index(middle_index_list[0])] = (posx_mean, posy_mean)

            #4th Delete the last detected object
            middle_index_list.pop()

        #Create a Position point with all the middle ranges selected
        #print(f"\n\n----------\nIl y a {len(middle_index_list)} objet(s) sur le plateau.")
        for k,idx in enumerate(middle_index_list):

            #Instatnciate a PositionPx msg
            other_robot_pos = PositionPx()
            other_robot_pos.theta = 0

            #Get the popsition of the laser using the index of its middle position
            pos_x, pos_y =  lidar_ranges_on_board_position[lidar_ranges_on_board_index.index(idx)]
            other_robot_pos.x = int(pos_x*1000) #convert to mm and cast to int
            other_robot_pos.y = int(pos_y*1000)

            #Add to msg
            quadrant = lidar_ranges_on_board_quadrant[lidar_ranges_on_board_index.index(idx)]
            #print(f"{k+1}:\tx: {other_robot_pos.x}\n\ty: {other_robot_pos.y}\n\tquadrant: {quadrant}")
            msg.append(other_robot_pos)

        return msg


if __name__ == '__main__':
    #Launch the node
    try:
        lidar = LidarNode()#instantiate it
        lidar.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
