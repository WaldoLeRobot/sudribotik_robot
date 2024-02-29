# SerialusM2M

serialus M2M Botik fonctionaliter AX et PID encore sous test et developpement 

#pour utiliser 
- mettre le code c sur le pic
- installer pyserial (pip3 install pyserial)
- etablire une conexion serial physique avec un ttl et initialiser une conexion serial en utilison serialusm2m.py , l objet serial cree va etre utiliser pour tout le programe, ne pas oublier de fermer le serial a la fin du code avec ser.close():
  ```
  import serialusM2M as s
  serial_port = '/dev/ttyUSB0'
  Baudrate=1000000    
  ttl9600=False
  Timeout=5
  ser=s.init_serial(Baudrate,serial_port,Timeout,False)

  # inserer code
  ser.close()

  
  ```
- utiliser la fonction envoi_reponse(msg,ser) de serialusM2M.py pour envoyer des msg au pic en serial :
  ```
  reponse = s.Envoi_reponse(msg, ser)
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
  
- pour utiliser les ficher qui commence avec fonction (les fonction deja coder):
-  chaqun aurai un exemple dedan de comment utiliser ses fonctions,
-  importer les ficher et utiliser directement exemple:
-  pour une liste de tout les fonction coder verifier l excel :  [Excel Serialus M2M](https://esmefr-my.sharepoint.com/:x:/g/personal/antoine_jreissati_esme_fr/ERTKWq9vIw1HvLDK_HuzZbMBeyH93Q8PnuKP7CFvykoTIQ?e=ufe7K2  'View only') 
-  ![image](https://github.com/AntoineJreissati/SerialusM2M/assets/103125833/e8fa5ae4-c689-40b6-9810-a1050ea7036d)
```
import fonction_deplacement as fd
import fonction_PID as pid
import fonction_Pos_Calage as fpc
import Fonction_AX as AX


import serialusM2M as s
serial_port = '/dev/ttyUSB0'
Baudrate=1000000    
ttl9600=False
Timeout=5
ser=s.init_serial(Baudrate,serial_port,Timeout,False)



AX.AX_set_alim(1,ser)
pid.debug(fd.avancer("0100","010",ser),ser)

```
#pour developper: 

- cree ses propres fonction faut les coder en C et en python, en c faut les mettre dans le switchcase et cree la fonction elle meme prenent l exemple de orienter :
- choisir un id pas pris pas une autre fonction et le mettan dans le switch case en utiliser les fonction check sont des nombre si il prend des parametre et check taille serialus  :
```
   case '2': // de
                    id1 = serialusM2M.buffer[1];
                    switch (id1)
                    {
                    case '1': // ori
                        if ((taille_buffer) != 10)
                        {
                            printf("erreur taille char %d \n", taille_buffer);
                            break;
                        }
                        else if (!sontdesdigits(serialusM2M.buffer))
                        {
                            printf("erreur Sont des digits \n");
                            break;
                        }
                        ori();
                        break;
  ```
-cree la fonction ori qui a besoin d aller piocher dans le buffer l angle et la vitesse on le fait ce cette facon : 
```
void ori()
{
    double a = 0.0;
    double v = 0.0;

    char chara[4];
    char charv[4];

    // Extract a et v substrings depuis le buffer
    strncpy(chara, serialusM2M.buffer + 3, 3);
    strncpy(charv, serialusM2M.buffer + 6, 3);

    // Null-terminate les substrings
    chara[3] = '\0';
    charv[3] = '\0';
    // Convert the substrings to double
    a = (double)atof(chara);
    v = (double)atof(charv);
    if (abs(a) > 180 || abs(v) > 200)
    {
        printf("data inchoerante a : %f, v : %f\n ", (float)a, (float)v);
    }
    else
    {

        init_clignotement();
        print_erreur_deplacementM2M(_orienter(a, v));
    }
}
```

apres il faut cree la fonction en pythoon voici une version simplifier de la fonction orientation() depui le fichier fonction_deplacement.py:
```

def orienter(angle, vitesse, ser):
    """
    Fonction pour envoyer une commande pour orienter le robot.

    Args:
        angle (str, 3): Angle de rotation en degrés.
        vitesse (str, 3): Vitesse de rotation en %
        ser (serial.Serial): Objet de la connexion série.

    Returns:
        tuple:
            - Réponse du robot type cf serialusm2m.py envoi_reponse().
    """
  
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
  
    return reponse
```






  
