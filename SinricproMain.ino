#include <Arduino.h>
#include <WiFi.h>
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <SinricProDimSwitch.h>
#include "SinricProTemperaturesensor.h"
#include "DHT.h" // https://github.com/markruys/arduino-DHT
#include <map>
#include "SinricProDevice.h" 
#include <RBDdimmerESP32.h>

#define WIFI_SSID         "Bamu"
#define WIFI_PASS         "12345678"
#define APP_KEY           "e2bf4750-3c3a-401e-941b-8565cb98ae72"
#define APP_SECRET        "ddb356de-43ad-482c-85e6-3761e5788317-9df1b8c0-4edb-40db-9338-03337d88eebd"
#define TEMP_SENSOR_ID    "6813d77dbddfc53e33e9119b"
#define DIMSWITCH_ID      "6813d5bedc4a25d5c3c13578"
//#define SMOKE_DEVICE_ID   "681401a1bddfc53e33e92ff4"

// comment the following line if you use a toggle switches instead of tactile buttons
#define TACTILE_BUTTON 1
#define BAUD_RATE   115200
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds
#define DEBOUNCE_TIME 250
  
#define RELAYPIN_1 13
#define RELAYPIN_2 32
#define RELAYPIN_3 14
#define DHT_PIN    4

const int ZC_PIN  = 18;
const int PWM_PIN  = 5;
//const int MQ7_PIN = 34;  
DHT dht;   
int MIN_POWER  = 0;
int MAX_POWER  = 80;
int POWER_STEP  = 2;
int power  = 0;

dimmerLampESP32 acd(PWM_PIN, ZC_PIN);
                                   // DHT sensor
typedef struct {      // struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
  bool activeLow;
} deviceConfig_t;
const int GAS_DETECTED_THRESHOLD = 200;

std::map<String, deviceConfig_t> devices = {
    //{deviceId, {relayPIN,  flipSwitchPIN, activeLow}}
    {"6813d591bddfc53e33e90f9b", {  13, 26, false }},
    {"6813d80fdc4a25d5c3c137ec", {  32, 25, false }},
    {"6813d448dc4a25d5c3c1341a", {  14, 33, false }},
    {"6813d63a947cbabd200e796f", {  27, 32, false}},
};
//SinricProDevice &gasSensor = SinricPro[SMOKE_DEVICE_ID];
//int sensorVal;
//String sensorState = "";
//String lastSensorState = "";

float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent
typedef struct {      // struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
  bool activeLow;
} flipSwitchConfig_t;

struct {
  bool powerState = false;
  int powerLevel = 0;
} device_state;

std::map<int, flipSwitchConfig_t> flipSwitches;    // this map is used to map flipSwitch PINs to deviceId and handling debounce and last flipSwitch state checks
                                                  // it will be setup in "setupFlipSwitches" function, using informations from devices map

void setPWM(int powerLevel) {
   int power = map(powerLevel, 0, 100, MIN_POWER, MAX_POWER); // Map power level from 0 to 100, to a value between 0 to 80
   acd.setPower(power);     
   delay(30);
} 

bool onPowerStatedim(const String &deviceId, bool &state) {
  Serial.printf("Device %s power turned %s \r\n", deviceId.c_str(), state?"on":"off");
  device_state.powerState = state;  
  setPWM(state ? 100: 0);
  return true; // request handled properly
}

bool onPowerLevel(const String &deviceId, int &powerLevel) {
  device_state.powerLevel = powerLevel; 
  setPWM(powerLevel);
  Serial.printf("Device %s power level changed to %d\r\n", deviceId.c_str(), device_state.powerLevel);
  return true;
}

bool onAdjustPowerLevel(const String &deviceId, int &levelDelta) {
  device_state.powerLevel += levelDelta;
  Serial.printf("Device %s power level changed about %i to %d\r\n", deviceId.c_str(), levelDelta, device_state.powerLevel);
  levelDelta = device_state.powerLevel;
  setPWM(levelDelta);
  return true;
}

void setupRBDdimmer(){
   acd.begin(NORMAL_MODE, ON);
} 

void setupRelays() { 
  for (auto &device : devices) {           // for each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN; // get the relay pin
    pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT
  }
}

void setupFlipSwitches() {
  for (auto &device : devices)  {                     // for each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;         // set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time
    flipSwitchConfig.lastFlipSwitchState = false;     // set lastFlipSwitchState to false (LOW)
    int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN
    bool activeLow = device.second.activeLow;         // set the activeLow
    flipSwitchConfig.activeLow = activeLow;           
    flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map
    
    if(activeLow) {
      pinMode(flipSwitchPIN, INPUT_PULLUP);           // set the flipSwitch pin to INPUT_PULLUP
    }
    else {
      pinMode(flipSwitchPIN, INPUT);                  // set the flipSwitch pin to INPUT  
    } 
  }
}

bool onPowerState(String deviceId, bool &state) {
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device
  digitalWrite(relayPIN, state);             // set the new relay state
 return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();                                          // get actual millis
  for (auto &flipSwitch : flipSwitches) {                                         // for each flipSwitch in flipSwitches map
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {                    // if time is > debounce time...

      int flipSwitchPIN = flipSwitch.first;                                       // get the flipSwitch pin from configuration
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;           // get the lastFlipSwitchState
      bool activeLow = flipSwitch.second.activeLow; 
      bool flipSwitchState = digitalRead(flipSwitchPIN);                          // read the current flipSwitch state
      if(activeLow) flipSwitchState = !flipSwitchState;

      if (flipSwitchState != lastFlipSwitchState) {                               // if the flipSwitchState has changed...
#ifdef TACTILE_BUTTON
        if (flipSwitchState) {                                                    // if the tactile button is pressed 
#endif      
          flipSwitch.second.lastFlipSwitchChange = actualMillis;                  // update lastFlipSwitchChange time
          String deviceId = flipSwitch.second.deviceId;                           // get the deviceId from config
          int relayPIN = devices[deviceId].relayPIN;                              // get the relayPIN from config
          bool newRelayState = !digitalRead(relayPIN);                            // set the new relay State
          digitalWrite(relayPIN, newRelayState);                                  // set the trelay to the new state

          SinricProSwitch &mySwitch = SinricPro[deviceId];                        // get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(newRelayState);                            // send the event
#ifdef TACTILE_BUTTON
        }
#endif      
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;                  // update lastFlipSwitchState
      }
    }
  }
}
void handleTemperaturesensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  if (SinricPro.isConnected() == false) {
    Serial.printf("Not connected to Sinric Pro...!\r\n");
    return; 
  }

temperature = dht.getTemperature();
humidity = dht.getHumidity();
             // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature || humidity == lastHumidity) return; // if no values changed do nothing...

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    delay(250);
  }
  
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}

void setupSinricPro() {
  for (auto &device : devices) {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  SinricProDimSwitch &myDimSwitch = SinricPro[DIMSWITCH_ID];

  // set callback function to device
  myDimSwitch.onPowerState(onPowerStatedim);
  myDimSwitch.onPowerLevel(onPowerLevel);
  myDimSwitch.onAdjustPowerLevel(onAdjustPowerLevel);
  
  SinricPro.begin(APP_KEY, APP_SECRET);  
}

void setup() {
  Serial.begin(BAUD_RATE);
  setupRelays();
  setupFlipSwitches();
  setupWiFi();
  setupSinricPro();
  setupRBDdimmer();
  dht.setup(DHT_PIN);
}

void loop() {
  SinricPro.handle();
  handleFlipSwitches();
  handleTemperaturesensor();
}
