#!/usr/bin/env python3
import os
import sys
import json
import subprocess

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
sys.path.insert(1, FILE_PATH.split("_1_ROS")[0]) #add parent folder to python path
from init import network_manager

#Load config file   
CONFIG_FILEPATH = FILE_PATH.split("_1_ROS")[0]+"init/configuration.json"
with open (CONFIG_FILEPATH, "r") as f:
    config = json.load(f)



def start_verification():
    """
    Verify that Waldo is on the TP Link network and Louise roscore is runing.
    """

    #1st : Robot Waldo wifi must be connected to TP-Link network
    if not network_manager.get_ipv4("wlan0") == config["SELF_IP_ADDRESS_ON_TP_LINK"]:
        #Condition not satisfied
        print(f"Log [{FILE_NAME}]: Impossible de se connecter au Louise roscore. Vous devez vous connecter"+\
            f" au TP Link afin d'avoir l'addresse IP suivante : {config["SELF_IP_ADDRESS_ON_TP_LINK"]}")
        return False

    #2nd : Beacon roscore must already run on the TP-Link network
    #Run netcat command to ping beacon specific ROS port
    nc_string_bytes = subprocess.run(["nc","-zv", config["LOUISE_IP_ON_TPLINK"], "11311", "-w", "0.6"],
                                    stdout=subprocess.PIPE)
    nc_string = nc_string_bytes.stdout.decode('ascii')
    if not nc_string.split()[-1] == "succeeded!":
        #Condition not satisfied
        print(f"Log [{FILE_NAME}]: Louise roscore non détecté. Vérifiez que Louise à bien lancer"+\
            f" son roscore sur son addresse IP TP Link ({config["LOUISE_IP_ON_TPLINK"]}).")
        return False


    #All condition are satisfied
    return True


if __name__ == "__main__":
    start_verification()