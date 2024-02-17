import os
import rospy
import traceback
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseArray
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
        rospy.Subscriber("robot1/position/self", Point, self.getSelfPositionCallback)
        self.self_position_x = None #attribute saving self position 
        self.self_position_y = None 
        self.self_position_z = 0 #Z position will not be updated

        #This node will publish to these topics
        self.arucoPos_pub = rospy.Publisher("robot1/position/otherRobots", PoseArray, queue_size=10)
    

    def run(self):
        pass



    def getSelfPositionCallback(self, data):
        """
        Callback for self position.
        """
        self.self_position_x = data.x
        self.self_position_y = data.y





if __name__ == '__main__':
    #Launch the node
    try:
        lidar = LidarNode()#instantiate it
        lidar.run()
    except Exception as e:
        traceback.print_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")