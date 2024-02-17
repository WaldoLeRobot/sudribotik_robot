import serial
import io
import time
import os

if __name__ == '__main__' :
    try :
        # configure the serial connections 
        with serial.Serial(port='/dev/ttyUSB0',
                           baudrate=9600,
                           timeout=1,
                           xonxoff=False,
                           rtscts=False,
                           dsrdtr=True) as s:
            for line in s:
                print(line)

    except :
        print('Program exit !')
        pass