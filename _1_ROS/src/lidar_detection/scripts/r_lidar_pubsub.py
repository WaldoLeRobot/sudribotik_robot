import os
import rospy
import traceback
import numpy as np
import serial
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from beacon_msgs.msg import ArrayPositionPx, ArrayPositionPxWithType, ArrayPositionPxRectangle

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)



class LidarNode:
    """
    This ROS node fetch raw data provided by the lidar and pusblish detected objects 
    within the board area.
    """

    def __init__(self):

        #Initialize  and tell node name to rospy
        rospy.init_node('r_lidar', anonymous=True)

        #This node will listen to these topics
        #rospy.Subscriber("robot1/position/self", Point, self.getSelfPositionCallback)
        self.self_position_x = 10#None #attributes saving self position 
        self.self_position_y = 10#None 
        self.self_position_z = 0 #Z position will not be updated
        self.self_position_theta = 1.0472 #None #in radians (60°)

        rospy.Subscriber("robot1/lidar/rawdata", LaserScan, self.getLidarDataCallback)
        self.lidar_ranges = None#attributes saving lidar data
        self.lidar_intensities = None
        self.NB_OF_RANGES = 456 #number of different laser angles used by the lidar

        #This node will publish to this topic
        self.otherRobotsPos_pub = rospy.Publisher("robot1/position/otherRobots", PoseArray, queue_size=10)

    def run(self):

        #Set publish rate
        rate = rospy.Rate(10) #in hz
        
        while not rospy.is_shutdown():

            #Publish only if we know our self position and if the serial port is still open
            if self.self_position_x and self.self_position_y and self.serial.is_open :

                self.otherRobotsPos_pub(self.getPositionOfOtherRobotsMsg) #publish

                #self.self_position_x = None #current position reset
                #self.self_position_y = None 
            
            rate.sleep() #wait according to publish rate



    def getSelfPositionCallback(self, data):
        """
        Callback for self position.
        """
        self.self_position_x = data.x
        self.self_position_y = data.y


    def getLidarDataCallback(self, data):
        """
        Callback for lidar data.
        """
        self.lidar_ranges = data.ranges
        self.lidar_intensities = data.intensities

    def isRangeOnBoard(self, range, index):
        """
        Calculate either a range is on board or not.
            range (int) ->  lidar range of a single pulse.
            index (int) ->  index of the range based on his angle.
        
        Return True if the range is on board, False if not.
        """

        #Filter nan values
        if not range :
            return False
        
        

    def getPositionOfOtherRobotsMsg(self):
        """
        Create message containing position of other robots on the board.
        """
        msg = [] #create msg

        #We'll make several operations on lidar ranges so we need to freeze them
        current_lidar_ranges = self.lidar_ranges

        #Filter positions outside of the board by creating a mask.
        # this mask will contain 456 boolean values paired with their lidar_range.
        # if the range is on board its corresponding index is set to True.
        mask_on_range = np.array([self.isRangeOnBoard(r, i) for i, r in enumerate(current_lidar_ranges)], dtype=bool)

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