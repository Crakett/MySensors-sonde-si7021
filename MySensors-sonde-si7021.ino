/************************************ 
 Zone de définition pour DEBUG
*************************************/
//#define   MY_DEBUG_PRINT
//#define   MY_DEBUG_VERBOSE_RFM69
//#define   VALUE_DEBUG

#define   MY_BAUD_RATE 4800

// For RFM69
#define   MY_RADIO_RFM69
#define   MY_RFM69_NEW_DRIVER
#define   MY_RFM69_FREQUENCY RFM69_868MHZ
//#define   MY_RFM69_FREQUENCY (867345000ul)
#define   MY_IS_RFM69HW
#define   MY_RFM69_IRQ_PIN   2
#define   MY_RFM69_TX_POWER_DBM (10)
#define   MY_RFM69_MAX_POWER_LEVEL_DBM (20)

#define   SKETCH_NAME "Sonde T et H - Si7021"
#define   SKETCH_VERSION "1.8"

#include <MyConfig.h>
#include <MySensors.h>

#include <Wire.h>
#include <SI7021.h>
#include <SPI.h>


// pour la gestion de la LED
#define LED_PIN   3
#define DELAI_ON  20    // en ms
#define DELAI_OFF 280   // en ms

// bits pour la lecture de l'ID
#define SWITCH_FOR_ID
#define POID_FAIBLE  4
#define POID_FORT    9

#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM  1
#define CHILD_ID_BATT 2

#ifdef VALUE_DEBUG
#define SLEEP_TIME 2000              // 2000 ms ==> 2 sec
#define FORCE_TRANSMIT_CYCLE 3       // 3*2 ==> 6 sec ; force l'envoi de la température et humudité toutes les 6 sec max 
#define BATTERY_REPORT_CYCLE 30      // 30*2 ==> 60 sec ; une fois toutes les 60 sec
#define HUMI_TRANSMIT_THRESHOLD 3.0  // seuil pour envoi humidité
#define TEMP_TRANSMIT_THRESHOLD 0.2  // seuil pour envoi température
#else
#define SLEEP_TIME 240000             // 240000 ms ==> 120 sec ==> 4 min ; mesure de la température et humidité toutes les 4 min
#define FORCE_TRANSMIT_CYCLE 3       // 3*4 ==> 12 min ; force l'envoi de la température et humudité toutes les 12 min max (si n'a pas changé)
#define BATTERY_REPORT_CYCLE 90      // 90*4 ==> 360 min ; 360/60 ==> 6 h ; envoi niveau batterie toutes les 6 h
#define HUMI_TRANSMIT_THRESHOLD 3.0  // seuil d'écart humidité pour envoi humidité
#define TEMP_TRANSMIT_THRESHOLD 0.1  // seuil d'écart température pour envoi température
#define SAME_TEMP_THRESHOLD 30       // 30*4 ==> 120 min ; nombre de mesure de température identique pour déclencher panne sonde par envoi temp identique
#endif

#ifdef MY_DEBUG_PRINT
#define DEBUG_SERIAL(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_SERIAL(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

const int DELAI_TRANS = 200;       // 200 ms d'attente après chaque send()

// tensions min max pile
#define VMIN 2100
#define VMAX 3000

int batteryReportCounter = BATTERY_REPORT_CYCLE;  // to make it report the first time.
int measureCount = FORCE_TRANSMIT_CYCLE;  // to make it report the first time.

SI7021 tempEtHumSensor;

float temperature;
int humidity;
float lastTemperature = 0; 
int lastHumidity = 0;
float lastTemperatureForCounter = 0;   // mesure précédente pour compter les mesure de température identique de la sonde 
int sameTemperatureCounter=0;
float deltaTemperature=0;              // 0.1 normalement. Si pendant SAME_TEMP_THRESHOLD la mesure de la température reste identique, alors 0
float nextDeltaTemperature=0.1;        // 0.1 ou -0,1

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP); // Initialize temperature, humidité message et pile message
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgBatt(CHILD_ID_BATT, V_VOLTAGE);


/***************************************************************************************************
 * Setup code
 ***************************************************************************************************/

void setup()
{
  //DEBUG_SERIAL(4800);    // <<<<<<<< Note BAUD_RATE in MySensors.h

  delay(4000);    // délai à la mise sous tension

  DEBUG_PRINTLN(F("Serial started"));
  DEBUG_PRINT(F("Voltage: "));
  DEBUG_PRINT(readVcc());
  DEBUG_PRINTLN(" mV");
  
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

#ifdef VALUE_DEBUG
  pulseLed(2);
#else
  pulseLed(1);
#endif

 delay(1000); // attente avant 1er envoi temp/hum/batt

}


/***************************************************************************************************
 * Lecture de la valeur de l'ID de la sonde pour le réseau MySensors
 ***************************************************************************************************/

#ifdef SWITCH_FOR_ID
void before()
{
  uint8_t i;
  uint8_t valueID=0;

  delay(4000);    // délai à la mise sous tension

  for(i=POID_FAIBLE;i<POID_FORT;i++)
  {
    pinMode(i,INPUT_PULLUP);
    if(digitalRead(i) == HIGH)  valueID += 1 << (i-POID_FAIBLE);
    pinMode(i,INPUT);   // to reduce consommation : no pullup
  }
  DEBUG_PRINT(F("nodeID by switch : ")); DEBUG_PRINTLN(valueID);
  if(valueID != 63)  transportAssignNodeID(valueID);  // set switch nodeID (1 to 62) only if different of 63 decimal (not all bit to 1)
}
#endif


/***************************************************************************************************
 * Présentation MySensors
 ***************************************************************************************************/

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  present(CHILD_ID_TEMP, S_TEMP,F("Temperature")); delay(DELAI_TRANS);
  present(CHILD_ID_HUM, S_HUM,F("Humidite")); delay(DELAI_TRANS);
  present(CHILD_ID_BATT, S_MULTIMETER,F("Pile")); delay(DELAI_TRANS);
}


/***************************************************************************************************
 * Boucle principale
 ***************************************************************************************************/
void loop()
{
  ReadTempAndHumidity();

  // 1 pulse LED si pas d'envoi (ni Temp ni Hum)
  int nbPulse = 1;
  bool forceTransmit = IsForceTransmit();

  DEBUG_PRINT(F("forceTransmit : ")); DEBUG_PRINTLN(forceTransmit);

  nbPulse += SendTemperature(forceTransmit);  
  nbPulse += SendHumidity(forceTransmit);

  // pulse la LED
  pulseLed(nbPulse);

  sendBatteryPercent();

  sleep(SLEEP_TIME);   // mise en mode très basse consommation

}

/***************************************************************************************************
 * Mesure temperature and humidity from Si7021 sensor
 ***************************************************************************************************/
 
void ReadTempAndHumidity()
{
  // lecture sonde température et humidité   
  si7021_thc data = tempEtHumSensor.getTempAndRH();

  temperature = data.celsiusHundredths / 100.0;
  DEBUG_PRINT("T: "); DEBUG_PRINTLN(temperature);

  humidity = data.humidityPercent;
  DEBUG_PRINT("H: "); DEBUG_PRINTLN(humidity);

  // temperature = 28.2;


  DEBUG_PRINT("sameTemperatureCounter "); DEBUG_PRINTLN(sameTemperatureCounter);

  // si la sonde est en panne, on imagine que les températures mesurées seront toujours les mêmes
  if(temperature == lastTemperatureForCounter)
  {
    // si le compteur atteind le seuil, delta température est 0 pour envoyer toujours la même valeur est créer une alarme du thermostat de Jeedom
    if(sameTemperatureCounter>=SAME_TEMP_THRESHOLD)
    {      
      DEBUG_PRINTLN("Seuil sameTemperatureCounter atteind");
      deltaTemperature = 0;
    }
    else
    {
      sameTemperatureCounter++;
    }
  }
  else
  {
    sameTemperatureCounter = 0;
    deltaTemperature = nextDeltaTemperature;
  }
  // on mémorise la température mesurée
  lastTemperatureForCounter = temperature;
  
  DEBUG_PRINT("deltaTemperature "); DEBUG_PRINTLN(deltaTemperature);
}



/***************************************************************************************************
 * Sends temperature only if necessary
 * forceTransmit = true --> force l'envoi
 * retour :
 * 0 : aucun envoi effectué
 * 1 : envoi effectué
 ***************************************************************************************************/
 
int SendTemperature(bool forceTransmit)
{
  float diffTemp = abs(lastTemperature - temperature);
  DEBUG_PRINT(F("Diff. Temp. :")); DEBUG_PRINTLN(diffTemp);
  
  if (diffTemp >= TEMP_TRANSMIT_THRESHOLD || forceTransmit)
  {
    // transmission forcée et pas de changement de température, alors on ajoute offset 0,1°C pour que le thermostat de jeedom ne déclare pas la sonde en panne
    if(diffTemp<TEMP_TRANSMIT_THRESHOLD)
    {
      temperature = lastTemperature + deltaTemperature;
      nextDeltaTemperature *= -1;
      DEBUG_PRINT(F("Diff Temp. < 0,1. nextDeltaTemperature : ")); DEBUG_PRINTLN(nextDeltaTemperature);
    }
    send(msgTemp.set(temperature, 1),false); delay(DELAI_TRANS);
    lastTemperature = temperature;    
    measureCount = 0;
    DEBUG_PRINT(F("Temp. sent! : ")); DEBUG_PRINTLN(temperature);
    // ajoute 1 au nombre de pulse LED à effectuer
    return 1;
  }
  return 0;
}

/***************************************************************************************************
 * Sends humidity only if necessary
 * forceTransmit = true --> force l'envoi
 * retour :
 * 0 : aucun envoi effectué
 * 2 : envoi effectué
 ***************************************************************************************************/
 
int SendHumidity(bool forceTransmit)
{
  float diffHum = abs(lastHumidity - humidity);
  DEBUG_PRINT(F("Diff. Hum.  :")); DEBUG_PRINTLN(diffHum);
  
  if (diffHum >= HUMI_TRANSMIT_THRESHOLD || forceTransmit)
  {
    send(msgHum.set(humidity),false); delay(DELAI_TRANS);
    lastHumidity = humidity;
    measureCount = 0;
    DEBUG_PRINT("Hum. sent! : "); DEBUG_PRINTLN(humidity);
    // ajoute 2 au nombre de pulse LED à effectuer
    return 2;
  }
  return 0;
}


/***************************************************************************************************
 * envoi le niveau de la pile
 ***************************************************************************************************/
 
void sendBatteryPercent()
{
  batteryReportCounter ++;

  if (batteryReportCounter >= BATTERY_REPORT_CYCLE)
  {
    uint16_t batteryVolt = readVcc();
    DEBUG_PRINT("Battery voltage: "); DEBUG_PRINT(batteryVolt); DEBUG_PRINTLN(" mV");
    uint8_t batteryPcnt = constrain(map(batteryVolt, VMIN, VMAX, 0, 100), 0, 100);
    DEBUG_PRINT("Battery percent: "); DEBUG_PRINT(batteryPcnt); DEBUG_PRINTLN(" %");
    sendBatteryLevel(batteryPcnt,false); delay(DELAI_TRANS);
    send(msgBatt.set(batteryVolt),false); delay(DELAI_TRANS);
    DEBUG_PRINTLN(F("Battery sent!"));
    batteryReportCounter = 0;

  }  
}

/***************************************************************************************************
 * function for reading Vcc by reading 1.1V reference against AVcc.
 * Based from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 * To calibrate reading replace 1125300L with scale_constant = internal1.1Ref * 1023 * 1000,
 * where internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
 ***************************************************************************************************/

uint16_t readVcc()
{
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  uint16_t result = (high << 8) | low;
  result = 1125300 / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/***************************************************************************************************
 * Impulsion d'allumage de la LED
 * paramètre :
 * - nombre : nombre de pulse
 ***************************************************************************************************/

void pulseLed(int nombre)
{
  for(int i=0;i<nombre;i++)
  {
    digitalWrite(LED_PIN,HIGH);
    delay(DELAI_ON);
    digitalWrite(LED_PIN,LOW);
    if(nombre>1)  delay(DELAI_OFF);
  }
  
}


/***************************************************************************************************
 * determine si l'envoi T. et H. doit être forcée
 ***************************************************************************************************/

bool IsForceTransmit()
{
  measureCount ++;
  if (measureCount >= FORCE_TRANSMIT_CYCLE)  
  {
    measureCount = 0;
    return true;     // force la transmission
  }
  return false;
}