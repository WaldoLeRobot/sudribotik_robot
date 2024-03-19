import math

"""
This file is used to store aruco related function used by our ROS system.
This file is a copy of the one in the beacon.
"""

def getCenterArucoTag(corners):
    """
    Calculate center of an Aruco tag.
    
    Parameters:
        - corner (list): aruco corner.

    Returns
        - tuple: x and y postions in int.
    """
    #Get x pos (these could be inversed but not important here)
    low_x = corners[0][0]
    high_x = corners[2][0]

    #Get y pos
    low_y = corners[0][1]
    high_y = corners[2][1]

    return ((low_x+high_x)//2 , (low_y+high_y)//2)



def getAngle(corner, unit="radians"):
    """
    Get angle rotation of an aruco tag given its corners.

    Parameters:
        - corner (list): corner of a aruco tag.
        - unit (str): unit cast (radians [0;2pi] or degrees [0;360])

    Returns:
        - float: angle in the choosen unit.
    """
    #Get points
    top_left, top_right, bot_right, bot_left = corner

    #Get parameters for atan func
    adjacent = top_right[0] - top_left[0]
    opposite = top_left[1] - top_right[1]

    #Get angle
    angle = math.atan2(adjacent, opposite)

    if unit[0] == "d":
        #Cast from radians to degrees 
        angle = angle * 180/math.pi 
        
        #And add pi/2
        angle+=90

        #Force range from 0° to 360°
        if angle < 0 :
            angle+=360
        return angle

    #Supress negatives
    if angle<0:
        angle+=2*math.pi

    return angle
