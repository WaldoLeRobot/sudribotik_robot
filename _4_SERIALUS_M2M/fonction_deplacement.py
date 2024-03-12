
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 20:30:23 2023(in Bangkok Baby)
@author: Antoine Jreissati 
code deplacement Serialus M2M

"""

import serialusM2M as s
#import matplotlib.pyplot as plt
import numpy as np

'''
sserial_port = '/dev/ttyUSB0'
Baudrate=1000000    
ttl9600=False
Timeout=5
msg_SCR="101204"
'''



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


def avancer(distance, vitesse, ser):
    """
    Fonction pour envoyer une commande pour avancer le robot.

    Args:
        distance (str, 4): Distance de déplacement en millimètres.
        vitesse (str, 3): Vitesse de déplacement en %.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '230'
    if len(distance) != 4:
        return False
    else:
        d = distance
    if len(vitesse) != 3:
        return False
    else:
        v = vitesse
    reponse = s.Envoi_reponse(code_function + d + v, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def reculer(distance, vitesse, ser):
    """
    Fonction pour envoyer une commande pour reculer le robot.

    Args:
        distance (str, 4): Distance de déplacement en millimètres.
        vitesse (str, 3): Vitesse de déplacement en %
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '240'
    if len(distance) != 4:
        return False
    else:
        d = distance
    if len(vitesse) != 3:
        return False
    else:
        v = vitesse
    reponse = s.Envoi_reponse(code_function + d + v, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def orienter(angle, vitesse, ser):
    """
    Fonction pour envoyer une commande pour orienter le robot.

    Args:
        angle (str, 3): Angle de rotation en degrés.
        vitesse (str, 3): Vitesse de rotation en %
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
            -code finction
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '210'
    if len(angle) != 3:
        return False
    else:
        a = angle
    if len(vitesse) != 3:
        return False
    else:
        v = vitesse
    reponse = s.Envoi_reponse(code_function + a + v, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def cibler(x, y, vitesse, ser):
    """
    Fonction pour envoyer une commande pour cibler une position spécifique.

    Args:
        x (str, 4): Coordonnée x de la position cible en millimètres.
        y (str, 4): Coordonnée y de la position cible en millimètres.
        vitesse (str, 3): Vitesse de déplacement %.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '220'
    if len(x) != 4:
        return False
    if len(y) != 4:
        return False
    if len(vitesse) != 3:
        return False
    else:
        v = vitesse
    reponse = s.Envoi_reponse(code_function + x + y + v, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def rejoindre(x, y, sens, vitesse, ser):
    """
    Fonction pour envoyer une commande pour rejoindre une position avec un sens spécifié.

    Args:
        x (str, 4): Coordonnée x de la position à rejoindre en millimètres.
        y (str, 4): Coordonnée y de la position à rejoindre en millimètres.
        sens (str, 1): Sens de rotation (0 ou 1). 
        vitesse (str, 3): Vitesse de déplacement %
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '250'
    if len(x) != 4:
        return False
    if len(y) != 4:
        return False
    if len(vitesse) != 3:
        return False
    if len(sens) != 1:
        return False
    reponse = s.Envoi_reponse(code_function + x + y + sens + vitesse, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def passe_par(x, y, sens, vitesse, parametre, ser):
    """
    Fonction pour envoyer une commande pour passer par une position avec un sens spécifié et des paramètres.

    Args:
        x (str, 4): Coordonnée x de la position à passer par en millimètres.
        y (str, 4): Coordonnée y de la position à passer par en millimètres.
        sens (str, 1): Sens de rotation (0 ou 1).
        vitesse (str, 3): Vitesse de déplacement %.
        parametre (str, 1): Paramètre :
                                        - 0 : debut
                                        - 1 : miliieu
                                        - 3 : fin.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot.
            - Erreur d'évitement.
            - Erreur de blockage.
    """
    erreur_Evitement = False
    erreur_Blockage = False
    code_function = '260'
    if len(x) != 4:
        return False
    if len(y) != 4:
        return False
    if len(vitesse) != 3:
        return False
    if len(sens) != 1:
        return False
    if len(parametre) != 1:
        return False
    reponse = s.Envoi_reponse(code_function + x + y + sens + vitesse + parametre, ser)
    if reponse[1] == '1':
        erreur_Evitement = True
    elif reponse[1] == '2':
        erreur_Blockage = True
    return reponse, erreur_Evitement, erreur_Blockage

def set_Break(ser):
    code_function = '270'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse


