# MySensors-sonde-si7021
Sonde de température et humidité basé sur Arduino pro mini + sonde Si7021 + RFM69HW
C'est un projet comencé en septembre 2018.

Modification en cours :

    délai entre 2 envois
    algoritme pour ne pas avoir d'alarme sonde dans jeedom si la température ne change pas



**Changelog**

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
