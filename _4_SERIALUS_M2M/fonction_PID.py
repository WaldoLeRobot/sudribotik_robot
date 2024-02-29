"""
Created on Mon Sep 30 16:16:22 2023 (in Bangkok Baby)
@author: Antoine Jreissati
Code Reglage PID SerialusM2M
"""

import serialusM2M as s
import fonction_deplacement as fdd


def DEBUG_OFF(ser):
    code_function = '350'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def DEBUG_ON(ser):
    code_function = '351'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_asserve(ser):
    code_function = '300'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_VITESSE_CONSIGNE_MAX_MM(ser):
    code_function = '301'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_VITESSE_DISTANCE_MIN(ser):
    code_function = '302'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_VITESSE_MAX_MM_TENSION(ser):
    code_function = '303'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_DISTANCE_CONSIGNE_MM(ser):
    code_function = '304'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_VITESSE_ANGLE_MAX(ser):
    code_function = '305'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_VITESSE_ANGLE_MIN(ser):
    code_function = '306'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_ORIENTATION_CONSIGNE_DEG(ser):
    code_function = '307'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_PID_VITESSE_DIST(ser):
    code_function = '308'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_PID_BREAK(ser):
    code_function = '309'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_ENTRAXE_MM(ser):
    code_function = '310'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_DIAMETRE_ROUE_CODEUSE(ser):
    code_function = '311'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_PERIMETRE_ROUE_MM(ser):
    code_function = '312'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_coef_D(ser):
    code_function = '313'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def reset_coef_G(ser):
    code_function = '314'
    reponse = s.Envoi_reponse(code_function, ser)
    return reponse[1]


def set_ENTRAXE_MM(_ENTRAXE_MM, ser):
    code_function = '315'
    if len(_ENTRAXE_MM) != 6:
        return False
    else:
        ENTRAXE_MM = _ENTRAXE_MM
    reponse = s.Envoi_reponse(code_function+ENTRAXE_MM, ser)
    return reponse[1]


def set_DIAMETRE_ROUE_CODEUSE(_DIAMETRE_ROUE_CODEUSE, ser):
    code_function = '316'
    if len(_DIAMETRE_ROUE_CODEUSE) != 6:
        return False
    else:
        DIAMETRE_ROUE_CODEUSE = _DIAMETRE_ROUE_CODEUSE
    reponse = s.Envoi_reponse(code_function+DIAMETRE_ROUE_CODEUSE, ser)
    return reponse[1]


def set_PERIMETRE_ROUE_MM(_PERIMETRE_ROUE_MM, ser):
    code_function = '317'
    if len(_PERIMETRE_ROUE_MM) != 6:
        return False
    else:
        PERIMETRE_ROUE_MM = _PERIMETRE_ROUE_MM
    reponse = s.Envoi_reponse(code_function+PERIMETRE_ROUE_MM, ser)
    return reponse[1]


def set_coef_D(_coef_D, ser):
    code_function = '318'
    if len(_coef_D) != 5:
        return False
    else:
        coef_D = _coef_D
    reponse = s.Envoi_reponse(code_function+coef_D, ser)
    return reponse[1]


def set_coef_G(_coef_G, ser):
    code_function = '319'
    if len(_coef_G) != 5:
        return False
    else:
        coef_G = _coef_G
    reponse = s.Envoi_reponse(code_function+coef_G, ser)
    return reponse[1]


def set_MAX_ERREUR_INTEGRALLE_V(_MAX_ERREUR_INTEGRALLE_V, ser):
    code_function = '322'
    if len(_MAX_ERREUR_INTEGRALLE_V) != 6:
        return False
    else:
        MAX_ERREUR_INTEGRALLE_V = _MAX_ERREUR_INTEGRALLE_V
    reponse = s.Envoi_reponse(code_function+MAX_ERREUR_INTEGRALLE_V, ser)
    return reponse[1]


def set_MAX_E_INTEGRALLE_BRAKE(_MAX_E_INTEGRALLE_BRAKE, ser):
    code_function = '323'
    if len(_MAX_E_INTEGRALLE_BRAKE) != 6:
        return False
    else:
        MAX_E_INTEGRALLE_BRAKE = _MAX_E_INTEGRALLE_BRAKE
    reponse = s.Envoi_reponse(code_function+MAX_E_INTEGRALLE_BRAKE, ser)
    return reponse[1]


def set_SEUIL_IMMOBILITE(_SEUIL_IMMOBILITE, ser):
    code_function = '324'
    if len(_SEUIL_IMMOBILITE) != 6:
        return False
    else:
        SEUIL_IMMOBILITE = _SEUIL_IMMOBILITE
    reponse = s.Envoi_reponse(code_function+SEUIL_IMMOBILITE, ser)
    return reponse[1]


def set_VITESSE_CONSIGNE_MAX_MM(_VITESSE_CONSIGNE_MAX_MM, ser):
    code_function = '330'
    if len(_VITESSE_CONSIGNE_MAX_MM) != 2:
        return False
    else:
        VITESSE_CONSIGNE_MAX_MM = _VITESSE_CONSIGNE_MAX_MM
    reponse = s.Envoi_reponse(code_function+VITESSE_CONSIGNE_MAX_MM, ser)
    return reponse[1]


def set_VITESSE_DISTANCE_MIN(_VITESSE_DISTANCE_MIN, ser):
    code_function = '331'
    if len(_VITESSE_DISTANCE_MIN) != 6:
        return False
    else:
        VITESSE_DISTANCE_MIN = _VITESSE_DISTANCE_MIN
    reponse = s.Envoi_reponse(code_function+VITESSE_DISTANCE_MIN, ser)
    return reponse[1]


def set_VITESSE_MAX_MM_TENSION(_VITESSE_MAX_MM_TENSION, ser):
    code_function = '332'
    if len(_VITESSE_MAX_MM_TENSION) != 4:
        return False
    else:
        VITESSE_MAX_MM_TENSION = _VITESSE_MAX_MM_TENSION
    reponse = s.Envoi_reponse(code_function+VITESSE_MAX_MM_TENSION, ser)
    return reponse[1]


def set_DISTANCE_CONSIGNE_MM(_DISTANCE_CONSIGNE_MM, ser):
    code_function = '333'
    if len(_DISTANCE_CONSIGNE_MM) != 6:
        return False
    else:
        DISTANCE_CONSIGNE_MM = _DISTANCE_CONSIGNE_MM
    reponse = s.Envoi_reponse(code_function+DISTANCE_CONSIGNE_MM, ser)
    return reponse[1]


def set_VITESSE_ANGLE_MAX(_VITESSE_ANGLE_MAX, ser):
    code_function = '334'
    if len(_VITESSE_ANGLE_MAX) != 4:
        return False
    else:
        VITESSE_ANGLE_MAX = _VITESSE_ANGLE_MAX
    reponse = s.Envoi_reponse(code_function+VITESSE_ANGLE_MAX, ser)
    return reponse[1]


def set_VITESSE_ANGLE_MIN(_VITESSE_ANGLE_MIN, ser):
    code_function = '335'
    if len(_VITESSE_ANGLE_MIN) != 4:
        return False
    else:
        VITESSE_ANGLE_MIN = _VITESSE_ANGLE_MIN
    reponse = s.Envoi_reponse(code_function+VITESSE_ANGLE_MIN, ser)
    return reponse[1]


def set_ORIENTATION_CONSIGNE_DEG(_ORIENTATION_CONSIGNE_DEG, ser):
    code_function = '336'
    if len(_ORIENTATION_CONSIGNE_DEG) != 5:
        return False
    else:
        ORIENTATION_CONSIGNE_DEG = _ORIENTATION_CONSIGNE_DEG
    reponse = s.Envoi_reponse(code_function+ORIENTATION_CONSIGNE_DEG, ser)
    return reponse[1]


def set_ACC_ORIENTATION_CONSIGNE(_ACC_ORIENTATION_CONSIGNE, ser):
    code_function = '340'
    if len(_ACC_ORIENTATION_CONSIGNE) != 4:
        return False
    else:
        ACC_ORIENTATION_CONSIGNE = _ACC_ORIENTATION_CONSIGNE
    reponse = s.Envoi_reponse(code_function+ACC_ORIENTATION_CONSIGNE, ser)
    return reponse[1]


def set_DCC_ORIENTATION_CONSIGNE(_DCC_ORIENTATION_CONSIGNE, ser):
    code_function = '341'
    if len(_DCC_ORIENTATION_CONSIGNE) != 4:
        return False
    else:
        DCC_ORIENTATION_CONSIGNE = _DCC_ORIENTATION_CONSIGNE
    reponse = s.Envoi_reponse(code_function+DCC_ORIENTATION_CONSIGNE, ser)
    return reponse[1]


def set_ACC_ORIENTATION_MIN(_ACC_ORIENTATION_MIN, ser):
    code_function = '342'
    if len(_ACC_ORIENTATION_MIN) != 4:
        return False
    else:
        ACC_ORIENTATION_MIN = _ACC_ORIENTATION_MIN
    reponse = s.Envoi_reponse(code_function+ACC_ORIENTATION_MIN, ser)
    return reponse[1]


def set_DCC_ORIENTATION_MIN(_DCC_ORIENTATION_MIN, ser):
    code_function = '343'
    if len(_DCC_ORIENTATION_MIN) != 4:
        return False
    else:
        DCC_ORIENTATION_MIN = _DCC_ORIENTATION_MIN
    reponse = s.Envoi_reponse(code_function+DCC_ORIENTATION_MIN, ser)
    return reponse[1]


def set_ACC_POSITION_CONSIGNE(_ACC_POSITION_CONSIGNE, ser):
    code_function = '344'
    if len(_ACC_POSITION_CONSIGNE) != 4:
        return False
    else:
        ACC_POSITION_CONSIGNE = _ACC_POSITION_CONSIGNE
    reponse = s.Envoi_reponse(code_function+ACC_POSITION_CONSIGNE, ser)
    return reponse[1]


def set_DCC_POSITION_CONSIGNE(_DCC_POSITION_CONSIGNE, ser):
    code_function = '345'
    if len(_DCC_POSITION_CONSIGNE) != 4:
        return False
    else:
        DCC_POSITION_CONSIGNE = _DCC_POSITION_CONSIGNE
    reponse = s.Envoi_reponse(code_function+DCC_POSITION_CONSIGNE, ser)
    return reponse[1]


def set_ACC_POSITION_MIN(_ACC_POSITION_MIN, ser):
    code_function = '346'
    if len(_ACC_POSITION_MIN) != 4:
        return False
    else:
        ACC_POSITION_MIN = _ACC_POSITION_MIN
    reponse = s.Envoi_reponse(code_function+ACC_POSITION_MIN, ser)
    return reponse[1]


def set_DCC_POSITION_MIN(_DCC_POSITION_MIN, ser):
    code_function = '347'
    if len(_DCC_POSITION_MIN) != 4:
        return False
    else:
        DCC_POSITION_MIN = _DCC_POSITION_MIN
    reponse = s.Envoi_reponse(code_function+DCC_POSITION_MIN, ser)
    return reponse[1]


def set_PID_VITESSE_DIST(kp, ki, kd, ser):

    code_function = '320'
    if len(kp) != 5:
        return False
    else:
        KP = kp

    if len(ki) != 5:
        return False
    else:
        KI = ki

    if len(kd) != 5:
        return False
    else:
        KD = kd
    reponse = s.Envoi_reponse(code_function+KP+KI+KD, ser)
    # print (reponse)
    return reponse[1]


def set_PID_BREAK(kp, ki, kd, ser):

    code_function = '321'
    if len(kp) != 5:
        return False
    else:
        KP = kp

    if len(ki) != 5:
        return False
    else:
        KI = ki

    if len(kd) != 5:
        return False
    else:
        KD = kd
    reponse = s.Envoi_reponse(code_function+KP+KI+KD, ser)
    # print (reponse)
    return reponse[1]


def debug(deplacement, ser):

    DEBUG_ON(ser)
    repo = deplacement
    Data = repo[0][1].split('\r')
    DEBUG_OFF(ser)
    Data.pop(0)
    Data.pop(len(Data)-1)
    Data_traiter = []
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
    time = []
    count = 0
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
        count += 1

    # Create a dark mode plot
    plt.style.use('dark_background')

    # Create subplots
    fig, (ax1, ax2) = plt.subplots(
        2, figsize=(10, 8), sharey='row', sharex='row')

    # Customize plot appearance for theta
    # ax1.invert_yaxis()
    ax1.plot(time, theta_cons, label='Theta Consigne',
             linewidth=2, color='cyan')
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

    ax2.plot(time, dist_cons, label='Distance Consigne',
             linewidth=2, color='cyan')
    ax2.plot(time, dist_act, label='Distance Actuel',
             linewidth=2, color='lime')
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

    fig2, (ax3, ax4) = plt.subplots(
        2, figsize=(10, 8), sharey='row', sharex='row')

    # Customize plot appearance for vitesse_rg
    # ax3.invert_yaxis()

    ax3.plot(time, vitesse_rg_cons, label='Vitesse RG Consigne',
             linewidth=2, color='red')
    ax3.plot(time, vitesse_rg_act, label='Vitesse RG Actuelle',
             linewidth=2, color='blue')
    ax3.plot(time, commande_g, label='Commande G', linewidth=2,
             color='purple')  # Add Commande D curve
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
    ax4.plot(time, vitesse_rd_cons, label='Vitesse RD Consigne',
             linewidth=2, color='orange')
    ax4.plot(time, vitesse_rd_act, label='Vitesse RD Actuelle',
             linewidth=2, color='green')
    ax4.plot(time, commande_d, label='Commande D', linewidth=2,
             color='brown')  # Add Commande G curve
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

    fig3, (ax5, ax6) = plt.subplots(
        2, figsize=(10, 8), sharey='row', sharex=True)

    # Customize plot appearance for erreur_vitesse_rg_actuel
    ax5.plot(time, erreur_vitesse_rg_actuel,
             label='Erreur Vitesse RG Actuelle', linewidth=2, color='purple')
    ax5.plot(time, erreur_vitesse_rg_integrale,
             label='Erreur Vitesse RG Integrale', linewidth=2, color='pink')
    # ax5.plot(time, commande_g, label='Commande G', linewidth=2, color='blue')  # Add Commande D curve
    ax5.set_xlabel('Time (ms)', color='white')
    ax5.set_ylabel('Error', color='white')
    ax5.set_title('Erreur Vitesse RG', color='white')
    ax5.grid(True, color='gray')
    ax5.legend(loc='upper right', fontsize='medium', frameon=True)
    ax5.set_facecolor('#1e1e1e')
    ax5.spines['top'].set_visible(False)
    ax5.spines['right'].set_visible(False)

    # Customize plot appearance for erreur_vitesse_rd_actuel
    ax6.plot(time, erreur_vitesse_rd_actuel,
             label='Erreur Vitesse RD Actuelle', linewidth=2, color='orange')
    ax6.plot(time, erreur_vitesse_rd_integrale,
             label='Erreur Vitesse RD Integrale', linewidth=2, color='green')
    # ax6.plot(time, commande_d, label='Commande D', linewidth=2, color='red')  # Add Commande G curve
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
    fig4, (ax7, ax8) = plt.subplots(
        2, figsize=(10, 8), sharey='row', sharex=True)

    # Customize plot appearance for erreur_ori_actuelle
    ax7.plot(time, erreur_ori_actuelle,
             label='Erreur Orientation Actuelle', linewidth=2, color='cyan')
    ax7.set_xlabel('Time (ms)', color='white')
    ax7.set_ylabel('Error', color='white')
    ax7.set_title('Erreur Orientation', color='white')
    ax7.grid(True, color='gray')
    ax7.legend(loc='upper right', fontsize='medium', frameon=True)
    ax7.set_facecolor('#1e1e1e')
    ax7.spines['top'].set_visible(False)
    ax7.spines['right'].set_visible(False)

    # Customize plot appearance for erreur_dist_actuel
    ax8.plot(time, erreur_dist_actuel,
             label='Erreur Distance Actuelle', linewidth=2, color='magenta')
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
