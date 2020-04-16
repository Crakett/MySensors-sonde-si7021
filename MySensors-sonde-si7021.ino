/* Sketch with Si7021 and battery monitoring
 *  Version 1.1
*/
//#define   MY_DEBUG
//#define   MY_DEBUG_VERBOSE_RFM69
#define MY_BAUD_RATE 4800
//#define DEBUG_ON            // Rain gauge specific debug messages. 

// For RFM69
#define   MY_RADIO_RFM69
#define   MY_RFM69_FREQUENCY RFM69_868MHZ
#define   MY_IS_RFM69HW
#define   MY_RFM69_NEW_DRIVER

// chiffrement 
#define   MY_RFM69_ENABLE_ENCRYPTION
#define   MY_SECURITY_SIMPLE_PASSWD "x8Ig.R76FjKL;e"

#define SKETCH_NAME "Si7021"
#define SKETCH_VERSION "1.1"

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

#define LED  A2

// bits pour l'ID
#define POID_FAIBLE  4
#define POID_FORT    9


#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define SLEEP_TIME 15000            // 15s for DEBUG
//#define SLEEP_TIME 300000         // 5 min
#define FORCE_TRANSMIT_CYCLE 36     // 5min*12=1/hour, 5min*36=1/3hour 
#define BATTERY_REPORT_CYCLE 2880   // Once per 5min   =>   12*24*7 = 2016 (one report/week)
#define VMIN 1900
#define VMAX 3300
#define HUMI_TRANSMIT_THRESHOLD 3.0  // THRESHOLD tells how much the value should have changed since last time it was transmitted.
#define TEMP_TRANSMIT_THRESHOLD 0.2
#define AVERAGES 2

int batteryReportCounter = BATTERY_REPORT_CYCLE - 1;  // to make it report the first time.
int measureCount = 0;
float lastTemperature = -100;
int lastHumidity = -100;

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
  /*
    delay(500);
    DEBUG_PRINT("Internal temp: ");
    DEBUG_PRINT(GetInternalTemp()); // Probably not calibrated. Just to print something.
    DEBUG_PRINTLN(" *C");
  */
  delay(500); // Allow time for radio if power useed as reset


  DEBUG_PRINT("Node and ");
  DEBUG_PRINTLN("2 children presented.");

  raHum.clear();

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

}

void before()
{
  uint8_t i;
  uint8_t valueID=0;

  for(i=POID_FAIBLE;i<POID_FORT;i++)
  {
    pinMode(i,INPUT_PULLUP);
    if(digitalRead(i) == HIGH)  valueID += (i-POID_FAIBLE)^2;
    pinMode(i,INPUT);   // to reduce consommation : no pullup
  }
  DEBUG_PRINT("nodeID by switch : "); DEBUG_PRINTLN(valueID);
  if(valueID != 63)  transportAssignNodeID(valueID);  // set switch nodeID (1 to 62) only if different of 63 decimal (not all bit to 1)
}

void presentation()
{
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  present(CHILD_ID_TEMP, S_TEMP);   // Present sensor to controller
  present(CHILD_ID_HUM, S_HUM);
}

void loop() {


  digitalWrite(LED,HIGH);
  delay(20);
  digitalWrite(LED,LOW);

  measureCount ++;
  batteryReportCounter ++;
  bool forceTransmit = false;

  if (measureCount > FORCE_TRANSMIT_CYCLE) {
    forceTransmit = true;
  }
  sendTempHumidityMeasurements(forceTransmit);
  /*
    // Read and print internal temp
    float temperature0 = static_cast<float>(static_cast<int>((GetInternalTemp()+0.5) * 10.)) / 10.;
    DEBUG_PRINT("Internal Temp: "); DEBUG_PRINT(temperature0); DEBUG_PRINTLN(" *C");
  */
  // Check battery
  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    long batteryVolt = readVcc();
    DEBUG_PRINT("Battery voltage: "); DEBUG_PRINT(batteryVolt); DEBUG_PRINTLN(" mV");
    uint8_t batteryPcnt = constrain(map(batteryVolt, VMIN, VMAX, 0, 100), 0, 255);
    DEBUG_PRINT("Battery percent: "); DEBUG_PRINT(batteryPcnt); DEBUG_PRINTLN(" %");
    sendBatteryLevel(batteryPcnt);
    batteryReportCounter = 0;
  }

  sleep(SLEEP_TIME);
}

// function for reading Vcc by reading 1.1V reference against AVcc. Based from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// To calibrate reading replace 1125300L with scale_constant = internal1.1Ref * 1023 * 1000, where internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
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
// function for reading internal temp. From http://playground.arduino.cc/Main/InternalTemperatureSensor
double GetInternalTemp(void) {  // (Both double and float are 4 byte in most arduino implementation)
  unsigned int wADC;
  double t;
  // The internal temperature has to be used with the internal reference of 1.1V. Channel 8 can not be selected with the analogRead function yet.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));   // Set the internal reference and mux.
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC
  while (bit_is_set(ADCSRA, ADSC));  // Detect end-of-conversion
  wADC = ADCW;   // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  t = (wADC - 88.0 ) / 1.0;   // The default offset is 324.31.
  return (t);   // The returned temperature in degrees Celcius.
}

/*********************************************
 * * Sends temperature and humidity from Si7021 sensor
   Parameters
   - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHumidityMeasurements(bool force) {
  bool tx = force;

  //si7021_env data = humiditySensor.getHumidityAndTemperature();
  si7021_thc data = humiditySensor.getTempAndRH();

  float temperature = data.celsiusHundredths / 100.0;
  DEBUG_PRINT("T: "); DEBUG_PRINTLN(temperature);
  float diffTemp = abs(lastTemperature - temperature);
  DEBUG_PRINT(F("TempDiff :")); DEBUG_PRINTLN(diffTemp);
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD || tx) {
    send(msgTemp.set(temperature, 1));
    lastTemperature = temperature;
    measureCount = 0;
    DEBUG_PRINTLN("T sent!");
  }

  int humidity = data.humidityPercent;
  DEBUG_PRINT("H: "); DEBUG_PRINTLN(humidity);
  raHum.addValue(humidity);
  humidity = raHum.getAverage();  // MA sample imply reasonable fast sample frequency
  float diffHum = abs(lastHumidity - humidity);
  DEBUG_PRINT(F("HumDiff  :")); DEBUG_PRINTLN(diffHum);
  if (diffHum > HUMI_TRANSMIT_THRESHOLD || tx) {
    send(msgHum.set(humidity));
    lastHumidity = humidity;
    measureCount = 0;
    DEBUG_PRINTLN("H sent!");
  }

}