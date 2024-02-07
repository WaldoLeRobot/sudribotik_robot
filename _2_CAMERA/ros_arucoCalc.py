import math

"""
This file is used to store aruco related function used by our ROS system.
"""




def getCenterArucoTag(corners):
    """
    Calculate center of an Aruco tag.
        corner (list)    ->      aruco corner.

    Return a tuple (int, int).
    """
    #Get x pos (these could be inversed but not important here)
    low_x = corners[0][0]
    high_x = corners[2][0]

    #Get y pos
    low_y = corners[0][1]
    high_y = corners[2][1]

    return ((low_x+high_x)//2 , (low_y+high_y)//2)



def getAngle(corner):
    """
    Get angle rotation of an aruco tag given its corners.
        corner(list)    ->    corner of a aruco tag.

    Return an angle in degrees. From 0° to 360°.
    """
    #Get points
    top_left, top_right, bot_right, bot_left = corner

    #Get parameters for atan func
    adjacent = top_right[0] - top_left[0]
    opposite = top_left[1] - top_right[1]

    #Get angle
    angle = math.atan2(adjacent, opposite)

    #Cast from radians to degrees 
    angle = angle * 180/math.pi
    
    #And add pi/2 (i fucking dont know why)
    angle+=90

    #Force range from 0° to 360°
    if angle < 0 :
        angle+=360

    return int(angle)