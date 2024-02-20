import numpy as np
import math
import os
import rospy
import serial
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
        self.BOARD_WIDTH_IN_METER = 0.3
        self.BOARD_HEIGHT_IN_METER = 0.3

        #Initialize  and tell node name to rospy
        rospy.init_node('r_lidar', anonymous=True)

        #This node will listen to these topics
        #rospy.Subscriber("robot1/position/self", Point, self.getSelfPositionCallback)
        self.self_position_x = 150#None #attributes saving self position in mm
        self.self_position_y = 150#None 
        self.self_position_z = 0 #z position will not be updated
        self.self_position_theta = 0 #None

        rospy.Subscriber("robot1/lidar/rawdata", LaserScan, self.getLidarDataCallback)
        self.lidar_ranges = None#attributes saving lidar data
        self.lidar_intensities = None
        self.NB_OF_RANGES = 456 #constant number of different laser angles used by the lidar
        self.ANGLE_INCREMENT = self.NB_OF_RANGES/(2*math.pi) #unitary angle between two consecutives lasers

        #This node will publish to this topic
        self.otherRobotsPos_pub = rospy.Publisher("robot1/position/otherRobots", ArrayPositionPx, queue_size=10)

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
        # so its still in radians but in a different range matching the one ues by the lidar raw data
        self.self_position_theta = (data.theta + 2*math.pi) % (2*math.pi)


    def getLidarDataCallback(self, data):
        """
        Callback for lidar data.
        """
        self.lidar_ranges = data.ranges
        self.lidar_intensities = data.intensities

    def isRangeOnBoard(self, index, range, x_robot, y_robot, theta_robot):
        """
        Calculate whether a range is on board or not
            index (int)         ->  index of the range based on his angle.
            range (float)       ->  lidar range of a single pulse (in meter).
            x_robot (float)     ->  position x of the robot in meters.
            y_robot (float)     ->  position y of the robot in meters.
            theta_robot (float) ->  angle of the robot in radians [0;2pi].
        
        Return True and its position (x,y) if the range is on board, False and (None, None) if not.
        """

        #Filter nan values
        if not range :
            return False, (None, None)
        
        #Find laser quadrant
        theta_relative = (theta_robot + (index*self.ANGLE_INCREMENT)) % (2*math.pi) #angle of the laser relative to the board
        quadrant = math.ceil(theta_relative/(math.pi/2)+0.00001) #in case theta_relative=0 we add a tiny float number

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
                    
        #Project the range on the x and y axes
        x_projection = abs(range * math.cos(self.ANGLE_INCREMENT*index))
        y_projection = abs(range * math.sin(self.ANGLE_INCREMENT*index))            

        #Compare these values to know if they are inside or not
        if (x_projection < max_x_inside) and (y_projection < max_y_inside):

            #Only calculate position on an object if its inside the board to limit calculations
            if   quadrant == 1 :return True, (x_robot + x_projection, y_robot - y_projection)
            elif quadrant == 2 :return True, (x_robot - x_projection, y_robot - y_projection)
            elif quadrant == 3 :return True, (x_robot - x_projection, y_robot + y_projection)
            elif quadrant == 4 :return True, (x_robot + x_projection, y_robot + y_projection)
        else :
            return False, (None, None)

        



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

        #Filter out positions outside the board.
        lidar_ranges_on_board_index = [] #this will contain index of ranges inside the board
        lidar_ranges_on_board_position = [] #this will contain ranges position, it will follow the index list
        for index, range in enumerate(current_lidar_ranges) :
            
            #Get position of ranges inside the board
            is_inside, pos = self.isRangeOnBoard(index, range,current_robot_x, current_robot_y, current_robot_theta)

            if is_inside :
                lidar_ranges_on_board_index.append(index) #store index
                lidar_ranges_on_board_position.append(pos) #store position


        #Regroup each range with close index then take the middle one       
        #This constant set the max number of index missing between two index
        MISSING_VALUE_TOLERANCE = 2      
        #This constant set the minimum number of consecutives ranges detected by the lidar to be considered a solid object
        MINIMUM_NB_OF_RANGES = 3  #it's purpose is to eliminate potential noises
        previous_index = None
        middle_index_list = [] #this will store the index of the position of the other robot
        temp_list = [] #this will store a list of following index

        for idx,range_index in enumerate(lidar_ranges_on_board_index):
            
            #The first index is always took 
            if idx == 0:
                previous_index = range_index
                temp_list.append(range_index) #don't forget to add it to the following index list
                continue
            
            #Store next index until the difference between the previous and the current is over the tolerance
            # OR if it's the last index in the list
            if (range_index - previous_index > MISSING_VALUE_TOLERANCE) or (idx==len(lidar_ranges_on_board_index)-1):

                #Add the middle index to the middle_index_list only if there is enough ranges.
                if len(temp_list) >= MINIMUM_NB_OF_RANGES :
                    middle_index_list.append(temp_list[len(temp_list)//2])

                #Set a new previous index and clear the following index list to start over
                previous_index = range_index
                temp_list.clear()
                temp_list.append(range_index) 
                continue

            else :
                #The index is not big enough to start a new group so add it to the list
                temp_list.append(range_index)
                previous_index = range_index


        #Create a Position with all the middle ranges selected
        print(f"\n\n----------\nIl y a {len(middle_index_list)} objet(s) sur le plateau.")
        for k,idx in enumerate(middle_index_list):

            #Instatnciate a PositionPx msg
            other_robot_pos = PositionPx()
            other_robot_pos.theta = 0

            #Get the popsition of the laser using the index of its middle position
            pos_x, pos_y =  lidar_ranges_on_board_position[lidar_ranges_on_board_index.index(idx)]
            other_robot_pos.x = int(pos_x*1000) #convert to mm and cast to int
            other_robot_pos.y = int(pos_y*1000)

            #Add to msg
            print(f"{k+1}:\tx: {other_robot_pos.x}\n\ty: {other_robot_pos.y}")
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