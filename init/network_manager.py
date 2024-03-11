import subprocess
import os
import json

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

#Load config file   
CONFIG_FILEPATH = FILE_PATH.split("init")[0]+"init/configuration.json"
with open (CONFIG_FILEPATH, "r") as f:
    config = json.load(f)

def get_network_status():
    """
    Dynamically get all your IPv4 addresses and hotspot status.

    Returns:
        - dict: eth0: ip address or None
                wlan0: ip address or None
                localhost: ip address
                hotspot: bool

    Note:
        Only 2 interfaces are scaned: eth0 and wlan0
        localhost will always be 127.0.0.1
    """
    
    net_status = {"eth0": None,
                  "wlan0": None,
                  "localhost": "127.0.0.1",
                  "hotspot": False}

    #Get eth0
    net_status["eth0"] = get_ipv4("eth0")

    #Get wlan0
    net_status["wlan0"] = get_ipv4("wlan0")

    #Get hotspot status
    net_status["hotspot"] = net_status["wlan0"] == config["SELF_IP_ADDRESS_ON_SELF_HOTSPOT"]

    return net_status



def get_ipv4(interface):
    """
    Get ipv4 address of an interface.

    Parameters:
        - interface: interface name (either eth0 or)

    Returns:
        - str : ipv4 address of the interface or None
    """
    #Run ifconfig command to get interface infos
    interface_string_bytes = subprocess.run(["ifconfig",interface],
                                            stdout=subprocess.PIPE)
    #Cast bytes to str and del newlines
    interface_string = interface_string_bytes.stdout.decode('ascii').split("\n")
    
    #Slice to get ipv4 address
    ip = interface_string[1].split()[1] 

    #Control that this is an ip and not something else (it must have 3 '.')
    if ip.count('.') != 3:
        return None
    return ip

