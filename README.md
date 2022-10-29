*************************************************
Cette brache de test permet d'effectuer des tests
*************************************************


# MySensors-sonde-si7021
Ce projet permet de réaliser une sonde température pour gèrer le chauffage des pièces d'une maison par domotique.

Sonde de température et humidité basé sur Arduino pro mini + sonde Si7021 + RFM69HW + pile CR123A
C'est un projet commencé en septembre 2018.

Sketch with Si7021 and battery monitoring
  
  cible
  - ARDUINO PRO MINI 3.3v 8Mhz modifié (retrait régulateur et led alim)
  - Optiboot external 1MHz RC BOD1.8V 4800baud
  
  si VALUE_DEBUG est défini
    - mesure temp et hum toutes les 2 sec
    - envoi temp et hum toutes les 6 sec max
    - envoi temp si écart > 0.2°C
    - envoi hum si écart > 3%
    - envoi niveau pile toutes les 10 sec
  
  sinon
    - mesure temp et hum toutes les 4 min
    - envoi temp et hum toutes les 4 minutes min et 12 minutes max
    - envoi temp si écart > 0.1°C
    - envoi hum si écart > 3%
    - envoi niveau pile toutes les 6 heures  
  
  Signification de la LED (pin 3)
    - 1 pulse : aucun envoi
    - 2 pulse : envoi temp 
    - 3 pulse : envoi hum
    - 4 pulse : envoi temp et hum
    
    - au reset : 1 pulse pour valeurs normales ; 2 pulse pour indiquer valeurs de DEBUG
  
  Gestion du MySensors Node ID Static par switch
    - bit 4 à 9 ==> poid binaire 0 à 5 ==> valeurs 0 à 63
    - pin en l'air : bit à 1
    - pin à la masse : bit à 0
    - si valeurs 63 (aucun pont de soudure), alors nodeID automatique par MySensors
  

**Changelog**

V1.8  13/10/2021
- essai nouvelle frequence puis retour F par défaut
- augmentation puissance
- ajout libellés aux capteurs (température, humidité et pile)

V1.7  16/07/2021
- délai entre 2 envois Temp. et Hum. : 4 minutes mini, 12 minutes maxi
- envoi niveau de batterie toutes les 6h
- algoritme pour ne pas avoir d'alarme sonde dans jeedom si la température ne change pas (ajoute 0,1 °C à la température précédente)
  (Sonde, Délai max entre 2 messages : 12 minutes), (Thermostat, Délai max entre 2 changements de température de la sonde : 12 minutes)
- Si sonde HS (lecture température toujours la même), envoi de la même température au bout de 2h (pour avoir une erreur dans jeedom)

V1.6  28/11/2020
- Vmin passe de 1900 à 2100
- seuil diff température pour envoi passe de 0.2 à 0.1 °c
- envoi d'un compteur à chaque envoi de battery

V1.5  03/06/2020
- ajout delai après chaque émission

V1.4  09/05/2020
- ajout envoi tension pile
- fréquence envoie niveau pile et tension passe de toutes les 6h à toutes les 1h
- update MySensors : 2.3.2

V1.3  28/03/19
- suppression MY_RFM69_ENABLE_ENCRYPTION
- ajout MY_RFM69_IRQ_PIN
- limitation à 100 du % batterie
