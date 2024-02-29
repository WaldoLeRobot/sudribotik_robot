
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 20:30:23 2023(in Bangkok Baby)
@author: Antoine Jreissati 
code deplacement Serialus M2M

"""

import serialusM2M as s
import matplotlib.pyplot as plt
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


'''
utiliser la fonction debug depuis le package fonction_PID et pas cella 
'''

def debug(deplacement):

    #repo=avancer('0200', '010', ser)
    repo= deplacement
    Data=repo[0][1].split('\r')
    Data.pop(0)
    Data.pop(len(Data)-1)
    Data_traiter=[]
    for i in Data:
        Data_traiter.append(i.split('\t'))

    """
    for i in Data_traiter :
        theta consigne                                1
        theta actuel                                  2
        dist cons                                     3
        dist act                                      4
        vitesse theorique                             5
        vitesse r g cons                              6
        vitesse r g act                               7
        vitesse r d cons                              8 
        vitesse r d act                               9  
        erreur vitesse r g actuel                     10 
        erreur vitesse r g integralle                 11
        erreur vitesse r d actuel                     12 
        erreur vitesse r d integralle                 13
        commande d                                    14 
        commande g                                    15 
        erreur ori actuelle                           16   
        erruer dist actuel                            17
        flag assrv imo                                18
        pi                                            19
    """
    theta_cons = []
    theta_act = []
    dist_cons = []
    dist_act = []
    vitesse_theorique = []
    vitesse_rg_cons = []
    vitesse_rg_act = []
    vitesse_rd_cons = []
    vitesse_rd_act = []
    erreur_vitesse_rg_actuel = []
    erreur_vitesse_rg_integrale = []
    erreur_vitesse_rd_actuel = []
    erreur_vitesse_rd_integrale = []
    commande_d = []
    commande_g = []
    erreur_ori_actuelle = []
    erreur_dist_actuel = []
    flag_assrv_imo = []
    pi = []
    time=[]
    count=0
    for packet in Data_traiter:
        time.append(count*20)
        theta_cons.append(float(packet[1]))
        theta_act.append(float(packet[2]))
        dist_cons.append(float(packet[3]))
        dist_act.append(float(packet[4]))
        vitesse_theorique.append(float(packet[5]))
        vitesse_rg_cons.append(float(packet[6]))
        vitesse_rg_act.append(float(packet[7]))
        vitesse_rd_cons.append(float(packet[8]))
        vitesse_rd_act.append(float(packet[9]))
        erreur_vitesse_rg_actuel.append(float(packet[10]))
        erreur_vitesse_rg_integrale.append(float(packet[11]))
        erreur_vitesse_rd_actuel.append(float(packet[12]))
        erreur_vitesse_rd_integrale.append(float(packet[13]))
        commande_d.append(float(packet[14]))
        commande_g.append(float(packet[15]))
        erreur_ori_actuelle.append(float(packet[16]))
        erreur_dist_actuel.append(float(packet[17]))
        flag_assrv_imo.append(packet[18])
        pi.append(float(packet[19]))
        count+=1
        
        
    # Create a dark mode plot
    plt.style.use('dark_background')

    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2,figsize=(10, 8),sharey='row',sharex='row')

    # Customize plot appearance for theta
    #ax1.invert_yaxis()
    ax1.plot(time, theta_cons, label='Theta Consigne', linewidth=2, color='cyan')
    ax1.plot(time, theta_act, label='Theta Actuel', linewidth=2, color='lime')
    ax1.set_xlabel('Time (ms)', color='white')
    ax1.set_ylabel('Angle', color='white')
    ax1.set_title('THETA', color='white')
    ax1.grid(True, color='gray')
    ax1.legend(loc='upper right', fontsize='medium', frameon=True)
    ax1.set_facecolor('#1e1e1e')
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)
    

    # Customize plot appearance for distance

    ax2.plot(time, dist_cons, label='Distance Consigne', linewidth=2, color='cyan')
    ax2.plot(time, dist_act, label='Distance Actuel', linewidth=2, color='lime')
    ax2.set_xlabel('Time (ms)', color='white')
    ax2.set_ylabel('mm', color='white')
    ax2.set_title('Distance', color='white')
    ax2.grid(True, color='gray')
    ax2.legend(loc='upper right', fontsize='medium', frameon=True)
    ax2.set_facecolor('#1e1e1e')
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)
    
    # Display the subplots
    plt.tight_layout()
    plt.show()

    fig2, (ax3, ax4) = plt.subplots(2, figsize=(10, 8), sharey='row', sharex='row')

    # Customize plot appearance for vitesse_rg
    # ax3.invert_yaxis()

    ax3.plot(time, vitesse_rg_cons, label='Vitesse RG Consigne', linewidth=2, color='red')
    ax3.plot(time, vitesse_rg_act, label='Vitesse RG Actuelle', linewidth=2, color='blue')
    ax3.plot(time, commande_g, label='Commande G', linewidth=2, color='purple')  # Add Commande D curve
    ax3.set_xlabel('Time (ms)', color='white')
    ax3.set_ylabel('Velocity', color='white')
    ax3.set_title('Vitesse RG', color='white')
    ax3.grid(True, color='gray')
    ax3.legend(loc='upper right', fontsize='medium', frameon=True)
    ax3.set_facecolor('#1e1e1e')
    ax3.spines['top'].set_visible(False)
    ax3.spines['right'].set_visible(False)

    # Customize plot appearance for vitesse_rd
    # ax4.invert_yaxis()
    ax4.plot(time, vitesse_rd_cons, label='Vitesse RD Consigne', linewidth=2, color='orange')
    ax4.plot(time, vitesse_rd_act, label='Vitesse RD Actuelle', linewidth=2, color='green')
    ax4.plot(time, commande_d, label='Commande D', linewidth=2, color='brown')  # Add Commande G curve
    ax4.set_xlabel('Time (ms)', color='white')
    ax4.set_ylabel('Velocity', color='white')
    ax4.set_title('Vitesse RD', color='white')
    ax4.grid(True, color='gray')
    ax4.legend(loc='upper right', fontsize='medium', frameon=True)
    ax4.set_facecolor('#1e1e1e')
    ax4.spines['top'].set_visible(False)
    ax4.spines['right'].set_visible(False)

    # Display the subplots for velocity-related data
    plt.tight_layout()
    plt.show()

    fig3, (ax5, ax6) = plt.subplots(2, figsize=(10, 8), sharey='row', sharex=True)

    # Customize plot appearance for erreur_vitesse_rg_actuel
    ax5.plot(time, erreur_vitesse_rg_actuel, label='Erreur Vitesse RG Actuelle', linewidth=2, color='purple')
    ax5.plot(time, erreur_vitesse_rg_integrale, label='Erreur Vitesse RG Integrale', linewidth=2, color='pink')
    #ax5.plot(time, commande_g, label='Commande G', linewidth=2, color='blue')  # Add Commande D curve
    ax5.set_xlabel('Time (ms)', color='white')
    ax5.set_ylabel('Error', color='white')
    ax5.set_title('Erreur Vitesse RG', color='white')
    ax5.grid(True, color='gray')
    ax5.legend(loc='upper right', fontsize='medium', frameon=True)
    ax5.set_facecolor('#1e1e1e')
    ax5.spines['top'].set_visible(False)
    ax5.spines['right'].set_visible(False)

    # Customize plot appearance for erreur_vitesse_rd_actuel
    ax6.plot(time, erreur_vitesse_rd_actuel, label='Erreur Vitesse RD Actuelle', linewidth=2, color='orange')
    ax6.plot(time, erreur_vitesse_rd_integrale, label='Erreur Vitesse RD Integrale', linewidth=2, color='green')
    #ax6.plot(time, commande_d, label='Commande D', linewidth=2, color='red')  # Add Commande G curve
    ax6.set_xlabel('Time (ms)', color='white')
    ax6.set_ylabel('Error', color='white')
    ax6.set_title('Erreur Vitesse RD', color='white')
    ax6.grid(True, color='gray')
    ax6.legend(loc='upper right', fontsize='medium', frameon=True)
    ax6.set_facecolor('#1e1e1e')
    ax6.spines['top'].set_visible(False)
    ax6.spines['right'].set_visible(False)

    # Display the subplots for error-related data
    plt.tight_layout()
    plt.show()

    # Create Figure 4 with two subplots ax7 and ax8
    fig4, (ax7, ax8) = plt.subplots(2, figsize=(10, 8), sharey='row', sharex=True)

    # Customize plot appearance for erreur_ori_actuelle
    ax7.plot(time, erreur_ori_actuelle, label='Erreur Orientation Actuelle', linewidth=2, color='cyan')
    ax7.set_xlabel('Time (ms)', color='white')
    ax7.set_ylabel('Error', color='white')
    ax7.set_title('Erreur Orientation', color='white')
    ax7.grid(True, color='gray')
    ax7.legend(loc='upper right', fontsize='medium', frameon=True)
    ax7.set_facecolor('#1e1e1e')
    ax7.spines['top'].set_visible(False)
    ax7.spines['right'].set_visible(False)

    # Customize plot appearance for erreur_dist_actuel
    ax8.plot(time, erreur_dist_actuel, label='Erreur Distance Actuelle', linewidth=2, color='magenta')
    ax8.set_xlabel('Time (ms)', color='white')
    ax8.set_ylabel('Error', color='white')
    ax8.set_title('Erreur Distance', color='white')
    ax8.grid(True, color='gray')
    ax8.legend(loc='upper right', fontsize='medium', frameon=True)
    ax8.set_facecolor('#1e1e1e')
    ax8.spines['top'].set_visible(False)
    ax8.spines['right'].set_visible(False)

    # Display the subplots for error-related data
    plt.tight_layout()
    plt.show()



