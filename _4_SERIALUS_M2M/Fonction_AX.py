# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 14:57:43 2024
@author: Theophile Baranes, based on Antoine Jreissati's job
code AX SerialusM2M
"""

import serialusM2M as s

'''
serial_port = '/dev/ttyUSB0'
Baudrate=1000000    
ttl9600=False
Timeout=5
msg_SCR="101204"
'''

def AX_set_ang(ID, Angle, vitesse, ser):
    """
    Fonction pour envoyer une commande de déplacement angulaire à un moteur AX.

    Args:
        ID (str, 2): Identifiant du moteur.
        Angle (str, 3): Angle de déplacement sur 360.
        vitesse (str, 4): Vitesse de déplacement sur 1024.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Erreur de timeout.
            - Erreur de checksum.
            - Erreur de limitation de courant.
            - Erreur d'identification.
            - Autres erreurs.
            - Code de fonction.
    """
    erreur_TimeOut = False
    erreur_CheckSum = False
    erreur_LimitationCourant = False
    erreur_CheckID = False
    erreur_Autres = False
    code_function = '400'
    if len(ID) != 2:
        return False
    if len(Angle) != 3:
        return False
    if len(vitesse) != 4:
        return False
    reponse = s.Envoi_reponse(code_function + ID + Angle + vitesse, ser)
    #print (reponse)
    if reponse[1] == '1':
        #print ('erreur timeout')
        erreur_TimeOut = True
    elif reponse[1] == '2':
         #   print ('erreur checksum')
        erreur_CheckSum = True
    elif reponse[1] == '3':
         #   print ('erreur LimitCouran')
        erreur_LimitationCourant = True
    elif reponse[1] == '4':
         #   print ('erreur CheckID')
        erreur_CheckID = True
    elif reponse[1] == '5':
         #   print ('erreur Autres')
        erreur_Autres = True
    return reponse, erreur_TimeOut, erreur_CheckSum, erreur_LimitationCourant, erreur_CheckID, erreur_Autres, code_function

def AX_set_Pos(ID, pos, vitesse, ser):
    """
    Fonction pour envoyer une commande de déplacement de position à un moteur AX.

    Args:
        ID (str, 2): Identifiant du moteur.
        pos (str, 4): Position de déplacement sur 1024.
        vitesse (str, 4): Vitesse de déplacement sur 1024.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Erreur de timeout.
            - Erreur de checksum.
            - Erreur de limitation de courant.
            - Erreur d'identification.
            - Autres erreurs.
            - Code de fonction.
    """
    erreur_TimeOut = False
    erreur_CheckSum = False
    erreur_LimitationCourant = False
    erreur_CheckID = False
    erreur_Autres = False
    code_function = '401'
    if len(ID) != 2:
        return False
    if len(pos) != 4:
        return False
    if len(vitesse) != 4:
        return False
    reponse = s.Envoi_reponse(code_function + ID + pos + vitesse, ser)
    #print (reponse)
    if reponse[1] == '1':
        #print ('erreur timeout')
        erreur_TimeOut = True
    elif reponse[1] == '2':
         #   print ('erreur checksum')
        erreur_CheckSum = True
    elif reponse[1] == '3':
         #   print ('erreur LimitCouran')
        erreur_LimitationCourant = True
    elif reponse[1] == '4':
         #   print ('erreur CheckID')
        erreur_CheckID = True
    elif reponse[1] == '5':
         #   print ('erreur Autres')
        erreur_Autres = True
    return reponse, erreur_TimeOut, erreur_CheckSum, erreur_LimitationCourant, erreur_CheckID, erreur_Autres, code_function

def AX_set_led(ID, State, ser): 
    """
    Fonction pour envoyer une commande pour régler l'état de la LED d'un moteur AX.

    Args:
        ID (str, 2): Identifiant du moteur.
        State (str, 1): État de la LED (0 pour éteint, 1 pour allumé).
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Erreur de timeout.
            - Erreur de checksum.
            - Erreur de limitation de courant.
            - Erreur d'identification.
            - Autres erreurs.
            - Code de fonction.
    """
    erreur_TimeOut = False
    erreur_CheckSum = False
    erreur_LimitationCourant = False
    erreur_CheckID = False
    erreur_Autres = False
    code_function = '402'
    if len(ID) != 2:
        return False
    if len(State) != 1:
        return False
    reponse = s.Envoi_reponse(code_function + ID + State, ser)
    #print (reponse)
    if reponse[1] == '1':
        #print ('erreur timeout')
        erreur_TimeOut = True
    elif reponse[1] == '2':
         #   print ('erreur checksum')
        erreur_CheckSum = True
    elif reponse[1] == '3':
         #   print ('erreur LimitCouran')
        erreur_LimitationCourant = True
    elif reponse[1] == '4':
         #   print ('erreur CheckID')
        erreur_CheckID = True
    elif reponse[1] == '5':
         #   print ('erreur Autres')
        erreur_Autres = True
    return reponse, erreur_TimeOut, erreur_CheckSum, erreur_LimitationCourant, erreur_CheckID, erreur_Autres, code_function

def AX_set_torque(ID, State, ser): 
    """
    Fonction pour envoyer une commande pour régler l'état du couple d'un moteur AX.

    Args:
        ID (str, 2): Identifiant du moteur.
        State (str, 1): État du couple (0 pour désactivé, 1 pour activé).
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Erreur de timeout.
            - Erreur de checksum.
            - Erreur de limitation de courant.
            - Erreur d'identification.
            - Autres erreurs.
            - Code de fonction.
    """
    erreur_TimeOut = False
    erreur_CheckSum = False
    erreur_LimitationCourant = False
    erreur_CheckID = False
    erreur_Autres = False
    code_function = '403'
    if len(ID) != 2:
        return False
    if len(State) != 1:
        return False
    reponse = s.Envoi_reponse(code_function + ID + State, ser)
    #print (reponse)
    if reponse[1] == '1':
        #print ('erreur timeout')
        erreur_TimeOut = True
    elif reponse[1] == '2':
         #   print ('erreur checksum')
        erreur_CheckSum = True
    elif reponse[1] == '3':
         #   print ('erreur LimitCouran')
        erreur_LimitationCourant = True
    elif reponse[1] == '4':
         #   print ('erreur CheckID')
        erreur_CheckID = True
    elif reponse[1] == '5':
         #   print ('erreur Autres')
        erreur_Autres = True
    return reponse, erreur_TimeOut, erreur_CheckSum, erreur_LimitationCourant, erreur_CheckID, erreur_Autres, code_function

def AX_set_alim(State, ser):
    """
    Fonction pour envoyer une commande pour régler l'état de l'alimentation des moteurs AX.

    Args:
        State (str, 1): État de l'alimentation (0 pour éteinte, 1 pour allumée).
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Code de fonction.
    """
    code_function = '404'   
    if len(State) != 1:
        return False
    reponse = s.Envoi_reponse(code_function + State, ser)
    return reponse, code_function

def AX_get_pos(Id, ser):
    """
    Fonction pour envoyer une commande pour obtenir la position d'un moteur AX.

    Args:
        Id (str, 2): Identifiant du moteur.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Code de fonction.
    """
    code_function = '410'   
    if len(Id) != 2:
        return False
    reponse = s.Envoi_reponse(code_function + Id, ser)
    return reponse, code_function

def AX_get_ping(Id, ser):
    """
    Fonction pour envoyer une commande pour obtenir un ping d'un moteur AX.

    Args:
        Id (str, 2): Identifiant du moteur.
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du moteur.
            - Code de fonction.
    """
    # id99= tout les AX
    code_function = '411'   
    if len(Id) != 2:
        return False
    reponse = s.Envoi_reponse(code_function + Id, ser)
    return reponse, code_function


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
"""
serial_port = '/dev/ttyUSB0'
Baudrate=1000000    
ttl9600=False
Timeout=5
ser=s.init_serial(Baudrate,serial_port,Timeout,False)

print(AX_set_led('06','1', ser))

ser.close()
"""