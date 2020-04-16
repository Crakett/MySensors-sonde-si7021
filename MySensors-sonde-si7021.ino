/* Sketch with Si7021 and battery monitoring
 *  
 *  si VALUE_DEBUG est défini
 *    - mesure temp et hum toutes les 10 sec
 *    - envoi temp et hum toutes les 30 sec max
 *    - envoi temp si écart > 0.2°C
 *    - envoi hum si écart > 3%
 *    - envoi niveau pile toutes les 30 sec
 *  
 *  sinon
 *    - mesure temp et hum toutes les 2 min
 *    - envoi temp et hum toutes les 10 min max
 *    - envoi temp si écart > 0.2°C
 *    - envoi hum si écart > 3%
 *    - envoi niveau pile toutes les 6 heures  
 *  
 *  
 *  Gestion de la LED (pin 3)
 *    - 1 pulse : aucun envoi
 *    - 2 pulse : envoi temp 
 *    - 3 pulse : envoi hum
 *    - 4 pulse : envoi temp et hum
 *    
 *    - au reset : 1 pulse pour valeur normal ; 2 pulse pour indiquer valeur de DEBUG
 *  
 *  Gestion du MySensors Node ID Static par switch
 *    - bit 4 à 9 ==> poid binaire 0 à 5 ==> valeurs 0 à 63
 *    - pin en l'air : bit à 1
 *    - pin à la masse : bit à 0
 *    - si valeurs 63 (aucun pont de soudure), alors nodeID automatique
 *  
 *    
*/
//#define   MY_DEBUG
//#define   MY_DEBUG_VERBOSE_RFM69
#define MY_BAUD_RATE 4800
//#define DEBUG_ON            // Rain gauge specific debug messages.

//#define VALUE_DEBUG OUI

// For RFM69
#define   MY_RADIO_RFM69
#define   MY_RFM69_FREQUENCY RFM69_868MHZ
#define   MY_IS_RFM69HW
#define   MY_RFM69_NEW_DRIVER

// chiffrement 
#define   MY_RFM69_ENABLE_ENCRYPTION
#define   MY_SECURITY_SIMPLE_PASSWD "x8Ig.R76FjKL;e"

#define SKETCH_NAME "Crakett - Si7021"
#define SKETCH_VERSION "1.2"

// ID static
//#define NODE_ID 101             // <<<<<<<<<<<<<<<<<<<<<<<<<<<   Enter Node_ID

#include <MySensors.h>
#include <Wire.h>
#include <SI7021.h>
#include <SPI.h>
#include <RunningAverage.h>


#ifdef MY_DEBUG
#define DEBUG_SERIAL(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_SERIAL(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// pour la gestion de la LED
#define LED_PIN   3
#define DELAI_ON  20    // en ms
#define DELAI_OFF 280   // en ms

// bits pour la lecture de l'ID
#define SWITCH_FOR_ID
#define POID_FAIBLE  4
#define POID_FORT    9


#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1


#ifdef VALUE_DEBUG
#define SLEEP_TIME 10000             // 10000 ms ==> 10 sec
#define FORCE_TRANSMIT_CYCLE 3       // 3*10 ==> 30 sec ; force l'envoi de la température et humudité toutes les 60 sec max 
#define BATTERY_REPORT_CYCLE 3       // 3*10 ==> 30 sec ; une fois toutes les 30 sec
#define HUMI_TRANSMIT_THRESHOLD 3.0  // seuil pour envoi humidité
#define TEMP_TRANSMIT_THRESHOLD 0.2  // seuil pour envoi température
#else
#define SLEEP_TIME 120000            // 120000 ms ==> 120 sec ==> 2 min
#define FORCE_TRANSMIT_CYCLE 5       // 5*2 ==> 10 min ; force l'envoi de la température et humudité toutes les 10 min max 
#define BATTERY_REPORT_CYCLE 180     // 180*2 ==> 360 min ; 360/60 ==> 6 h ; une fois toutes les 6h
#define HUMI_TRANSMIT_THRESHOLD 3.0  // seuil pour envoi humidité
#define TEMP_TRANSMIT_THRESHOLD 0.2  // seuil pour envoi température
#endif

// tensions min max pile
#define VMIN 1900
#define VMAX 3000

// moyenne humidité
#define AVERAGES  2     

int batteryReportCounter = BATTERY_REPORT_CYCLE - 1;  // to make it report the first time.
int measureCount = 0;
float lastTemperature = -100;        // to make it report the first time.
int lastHumidity = -100;             // to make it report the first time.

RunningAverage raHum(AVERAGES);
SI7021 humiditySensor;


MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP); // Initialize temperature message
MyMessage msgHum(CHILD_ID_HUM, V_HUM);

void setup() {
  //DEBUG_SERIAL(4800);    // <<<<<<<<<<<<<<<<<<<<<<<<<< Note BAUD_RATE in MySensors.h
  DEBUG_PRINTLN("Serial started");

  DEBUG_PRINT("Voltage: ");
  DEBUG_PRINT(readVcc());
  DEBUG_PRINTLN(" mV");

  delay(500); // Allow time for radio if power useed as reset

  DEBUG_PRINT("Node and ");
  DEBUG_PRINTLN("2 children presented.");

  raHum.clear();

  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);


#ifdef VALUE_DEBUG
  pulseLed(2);
#else
  pulseLed(1);
#endif

}

#ifdef SWITCH_FOR_ID
void before()
{
  uint8_t i;
  uint8_t valueID=0;

  for(i=POID_FAIBLE;i<POID_FORT;i++)
  {
    pinMode(i,INPUT_PULLUP);
    if(digitalRead(i) == HIGH)  valueID += 1 << (i-POID_FAIBLE);
    pinMode(i,INPUT);   // to reduce consommation : no pullup
  }
  DEBUG_PRINT("nodeID by switch : "); DEBUG_PRINTLN(valueID);
  if(valueID != 63)  transportAssignNodeID(valueID);  // set switch nodeID (1 to 62) only if different of 63 decimal (not all bit to 1)
}
#endif

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  present(CHILD_ID_TEMP, S_TEMP);   // Present sensor to controller
  present(CHILD_ID_HUM, S_HUM);
}

void loop() {

  sendTempHumidityMeasurements();

  sendBatteryPercent();

  sleep(SLEEP_TIME);
}


/***************************************************************************************************
 * Sends temperature and humidity from Si7021 sensor
 ***************************************************************************************************/
 
void sendTempHumidityMeasurements() {
  
  bool forceTransmit = false;
  int nbPulse = 1;     // 1 pulse si pas d'envoi (ni Temp ni Hum)
  
  measureCount ++;
  if (measureCount > FORCE_TRANSMIT_CYCLE)   forceTransmit = true;  // force la transmission
  
  si7021_thc data = humiditySensor.getTempAndRH();

  float temperature = data.celsiusHundredths / 100.0;
  DEBUG_PRINT("T: "); DEBUG_PRINTLN(temperature);
  float diffTemp = abs(lastTemperature - temperature);
  DEBUG_PRINT(F("TempDiff :")); DEBUG_PRINTLN(diffTemp);
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD || forceTransmit) {
    send(msgTemp.set(temperature, 1));
    lastTemperature = temperature;
    measureCount = 0;
    nbPulse += 1;                    // 2 pulse si envoi Temp
    DEBUG_PRINTLN("T sent!");
  }

  int humidity = data.humidityPercent;
  DEBUG_PRINT("H: "); DEBUG_PRINTLN(humidity);
  raHum.addValue(humidity);
  humidity = raHum.getAverage();  // MA sample imply reasonable fast sample frequency
  float diffHum = abs(lastHumidity - humidity);
  DEBUG_PRINT(F("HumDiff  :")); DEBUG_PRINTLN(diffHum);
  if (diffHum > HUMI_TRANSMIT_THRESHOLD || forceTransmit) {
    send(msgHum.set(humidity));
    lastHumidity = humidity;
    measureCount = 0;
    nbPulse += 2;                    // 3 pulse si envoi Hum, 4 pulse si envoi Temp + Hum
    DEBUG_PRINTLN("H sent!");
  }

  pulseLed(nbPulse);   // pulse la LED

}




/***************************************************************************************************
 * envoi le niveau de la pile
 ***************************************************************************************************/
 
void sendBatteryPercent() {

  batteryReportCounter ++;

  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    long batteryVolt = readVcc();
    DEBUG_PRINT("Battery voltage: ");
    DEBUG_PRINT(batteryVolt);
    DEBUG_PRINTLN(" mV");
    uint8_t batteryPcnt = constrain(map(batteryVolt, VMIN, VMAX, 0, 100), 0, 255);
    DEBUG_PRINT("Battery percent: ");
    DEBUG_PRINT(batteryPcnt);
    DEBUG_PRINTLN(" %");
    sendBatteryLevel(batteryPcnt);
    batteryReportCounter = 0;
  }
  
  
}

/***************************************************************************************************
 * function for reading Vcc by reading 1.1V reference against AVcc.
 * Based from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 * To calibrate reading replace 1125300L with scale_constant = internal1.1Ref * 1023 * 1000,
 * where internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
 ***************************************************************************************************/

long readVcc() {
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/***************************************************************************************************
 * Impulsion d'allumage de la LED
 * paramètre :
 * - nombre : nombre de pulse
 ***************************************************************************************************/

void pulseLed(int nombre) {

  for(int i=0;i<nombre;i++) {
    digitalWrite(LED_PIN,HIGH);
    delay(DELAI_ON);
    digitalWrite(LED_PIN,LOW);
    if(nombre>1)  delay(DELAI_OFF);
  }
  
}
