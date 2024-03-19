# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 14:57:43 2024
@author: Theophile Baranes, based on Antoine Jreissati's job (for the arms)
Code Uart Compatible avec Serialus M2M
"""

# Importation des modules nécessaires
import serial  # Module pour la communication série
import time    # Module pour gérer les délais

def init_serial(Baudrate, serial_port, Timeout, ttl9600):

    """
    Initialise la communication série avec les paramètres spécifiés.

    Args:
        Baudrate (int): Taux de bauds (bits par seconde) pour la communication. developper sur 1Mb
        serial_port (str): Nom du port série (par exemple, 'COM3' sur Windows).// raspy: 'ls /dev/ttyUSBx' (x etant le numero du ttl peut etre checker en fesant $ls /dev/ttyUSB**)
        Timeout (float): Durée maximale d'attente pour les opérations de lecture. 
        ttl9600 (bool): Optionnel, détermine si le port doit être ouvert à 9600 bauds avant d'être fermé et reouvert au bon baudrate bug de certain ttl. (utile pour Windows)

    Returns:
        serial.Serial: Objet de communication série initialisé.
        
    lancer la fonction dde cette facon ser=init_serial(...) et utiliser le meme ser pour le reste de la comunication. Ser.close a la fin
    """
    if ttl9600:
        # Si ttl9600 est vrai, ouvre le port série à 9600 bauds pendant un court instant.
        ser = serial.Serial(serial_port, 9600, timeout=0.1)
        time.sleep(0.1)
        ser.close()
        time.sleep(0.1)
    
    # Initialise la communication série avec les paramètres spécifiés.
    ser = serial.Serial(serial_port, Baudrate, timeout=Timeout)
    time.sleep(1)  # Attente d'une seconde pour permettre l'initialisation du port.
    return ser

def timer_s(s):
    """
    Attends pendant la durée spécifiée en secondes.
    precis au ms (plus precis que Time.sleep de python)
    Args:
        s (float): Durée d'attente en secondes.
        
    """
    target_time = time.time() + s
    while time.time() < target_time:
        pass

def send_msg_4bits(msg_SCR, ser):
    """
    Envoie un message en le divisant en morceaux de 4 caractères. pour ne pas mettre le pic en overflow

    Args:
        msg_SCR (str): Message à envoyer.
        ser (serial.Serial): Objet de communication série.

    Returns:
        bool: True si l'envoi est réussi, False sinon.
    """
    while (ser.isOpen())==False:
        pass
    deconposed_msg = [msg_SCR[i:i + 4] for i in range(0, len(msg_SCR), 4)]
    for i in deconposed_msg:
        ser.write(i.encode('utf-8'))  # Envoie la partie du message encodée en UTF-8.
        ser.flush()  # Vide le tampon de sortie.
        timer_s(0.009)  # Attend brièvement.
    ser.write(('\r').encode('utf-8'))  # Envoie un retour chariot ('\r') pour indiquer la fin du message.
    return True

def send_msg_AC(msg_SCR, ser):
    """
    Envoie un message caractère par caractère avec attente de confirmation de réception.

    Args:
        msg_SCR (str): Message à envoyer.
        ser (serial.Serial): Objet de communication série.

    Returns:
        bool: True si l'envoi est réussi, False sinon.
    """
    counter = 0
    for i in msg_SCR:
        counter += 1
        ser.write(i.encode('utf-8'))  # Envoie un caractère encodé en UTF-8.
        ser.flush()  # Vide le tampon de sortie.
        while ser.inWaiting() != counter:
            pass  # Attend la confirmation de réception de chaque caractère.
    ser.write(('\r').encode('utf-8'))  # Envoie un retour chariot ('\r') pour indiquer la fin du message.
    return True

def reception(msg_SCR, ser):
    """
    Reçoit et traite les données de l'UART.

    Args:
        msg_SCR (str): Message à attendre en réception.
        ser (serial.Serial): Objet de communication série.

    Returns:
        list: Une liste contenant les éléments suivants dans l'ordre :
            - La donnée reçue en sortie (data_out)
            - La donnée reçue en entrée (data_in)
            - Indique si une erreur de lecture s'est produite (err_read)
            - Indique si une erreur de décodage s'est produite (err_decode)
            - Indique si une erreur de réception du message attendu s'est produite (err_recep_msg)
            - Le temps écoulé pour la réception (timer)
    """
    starttime = time.time()  # Enregistre le temps de début de la réception.
    try:
        data_out_en = ser.readline()  # Lit la sortie de l'UART.
        data_in_en = ser.readline()   # Lit l'entrée de l'UART.
        err_read = False  # Aucune erreur de lecture.
    except:
        data_out_en = ''  # Les données lues sont vides en cas d'erreur.
        data_in_en = ''
        err_read = True   # Indique qu'une erreur de lecture s'est produite.

    try:

        data_out = data_out_en.decode('utf-8').strip()  # Décode la sortie en UTF-8.
        data_in = data_in_en.decode('utf-8').strip()    # Décode l'entrée en UTF-8.
        err_decode = False  # Aucune erreur de décodage.
    except:
        data_out = data_out_en  # Les données décodées sont vides en cas d'erreur.
        data_in = data_in_en
        err_decode = True      # Indique qu'une erreur de décodage s'est produite.

    if data_out != msg_SCR:
        err_recep_msg = True  # Indique qu'une erreur de réception du message attendu s'est produite.
    else:
        err_recep_msg = False

    endtime = time.time()  # Enregistre le temps de fin de la réception.
    timer = endtime - starttime  # Calcule la durée de la réception.
    RESPONSE = [data_out, data_in, err_read, err_decode, err_recep_msg, timer]  # Crée une liste de résultats.
    return RESPONSE

def Envoi_reponse(msg_SCR, ser):
    """
    Envoie un message et attend la réception d'une réponse.

    Args:
        msg_SCR (str): Message à envoyer.
        ser (serial.Serial): Objet de communication série.

    Returns:
        list: Une liste contenant les éléments suivants dans l'ordre :
            - La donnée reçue en sortie (data_out)
            - La donnée reçue en entrée (data_in)
            - Indique si une erreur de lecture s'est produite (err_read)
            - Indique si une erreur de décodage s'est produite (err_decode)
            - Indique si une erreur de réception du message attendu s'est produite (err_recep_msg)
            - Le temps écoulé pour l'opération reponse (timer)
            -Le temps écoulé pour l'opération complete (timer)
    """
    msg_SCR=str(msg_SCR)
    starttime = time.time()  # Enregistre le temps de début de l'opération.
    send_msg_4bits(msg_SCR, ser)  # Envoie le message.
    recep = reception(msg_SCR, ser)  # Attend la réception de la réponse.
    endtime = time.time()  # Enregistre le temps de fin de l'opération.
    timer = endtime - starttime  # Calcule la durée de l'opération.
    return (recep + [timer])  # Renvoie les résultats de l'opération avec le temps écoulé.

def reinit_err(ser):
    timer_s(0.01) 
    send_msg_4bits('\r', ser)
    ser.read(100)

'''
serial_port = '/dev/ttyUSB0'
Baudrate=1000000
ttl9600=False
Timeout=5
msg_SCR="00"

ser=init_serial(Baudrate,serial_port,Timeout,False)

print(Envoi_reponse(msg_SCR,ser))

#send_msg_4bits(msg_SCR,ser)
#print(reception(msg_SCR,ser))

'''


    
   