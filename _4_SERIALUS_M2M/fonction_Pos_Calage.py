"""
Created on Mon Sep 30 16:16:22 2023 (in Bangkok Baby)
@author: Antoine Jreissati
code Pos,calage SerialusM2M
"""

import serialusM2M as s


'''
    reponse du style  cf fonction envoi_reponse @serialusM2M.py:

    Returns:
        list: Une liste contenant les éléments suivants dans l'ordre :
            - La donnée reçue en sortie (data_out)
            - La donnée reçue en entrée (data_in)
            - Indique si une erreur de lecture s'est produite (err_read)
            - Indique si une erreur de décodage s'est produite (err_decode)
            - Indique si une erreur de réception du message attendu s'est produite (err_recep_msg)
            - Le temps écoulé pour l'opération reponse (timer)
            - Le temps écoulé pour l'opération complete (timer)
    
'''


def get_pos(ser):
    code_function = '000'
    reponse = s.Envoi_reponse(code_function, ser)
    XYT=reponse[1]
    X,Y,T=XYT.split("_")
    return X,Y,T

def set_pos(x,y,t,ser):
    code_function = '010'
    if len(x) != 4:
        return False
    if len(y) != 4:
        return False
    if len(t)!=3:
        return False
    reponse = s.Envoi_reponse(code_function + x + y, ser)
    return reponse[1]

def Callage_All(distance, vitesse, ser):
    """
    Fonction pour envoyer une commande pour avancer le robot.

    Args:
        distance (str, 4): Distance de déplacement en millimètres.
        vitesse (str, 3): Vitesse de déplacement en %.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot
    """
    code_function = '120'
    if len(distance) != 4:
        return False
    else:
        d = distance
    if len(vitesse) != 3:
        return False
    else:
        v = vitesse
    reponse = s.Envoi_reponse(code_function + d + v, ser)
    return reponse[1]

"""   
serial_port = '/dev/ttyUSB0'
Baudrate=1000000    
ttl9600=False
Timeout=2
ser=s.init_serial(Baudrate,serial_port,Timeout,False)

print(get_pos(ser))

ser.close()
"""