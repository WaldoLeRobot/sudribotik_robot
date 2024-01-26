#!/usr/bin/env python3
import rospy
from balise_msgs.msg import PositionPx, PositionPxWithType, ArrayPositionPx, ArrayPositionPxWithType



"""
Callback for main robots position
"""
def robotPosCallback(data):
    for robot in data.array_of_positionspx_with_type:
        rospy.loginfo(f"{robot.type} on ({robot.x}, {robot.y})")
    
"""
Callback for pamis position
"""
def pamiPosCallback(data):
    for pami in data.array_of_positionspx_with_type:
        rospy.loginfo(f"{pami.type} on ({pami.x}, {pami.y})")
    
"""
Callback for plants position
"""
def plantPosCallback(data):
    for plant in data.array_of_positionspx:
        rospy.loginfo(f"{plant.type} on ({plant.x}, {plant.y})")
    

"""
Fetch and parse data from ros on ihm sub topics
"""
def listener():

    # Tell node name to rospy
    rospy.init_node('listener', anonymous=True)

    # This node will listen to these topics
    rospy.Subscriber("balise/position/robots", ArrayPositionPxWithType, robotPosCallback)
    rospy.Subscriber("balise/position/pamis", ArrayPositionPxWithType, pamiPosCallback)
    rospy.Subscriber("balise/position/plants", ArrayPositionPx, plantPosCallback)
    #rospy.Subscriber("balise/position/pots", ArrayPositionPx, callback)

    #Keeps python from exiting until this node is stopped
    # also it permits to this node to listen to new messages on mentioned topics
    # and to run specified callbacks
    rospy.spin()

if __name__ == '__main__':
    listener()