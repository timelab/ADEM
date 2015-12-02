  /**********************************************/
/* Mobile Arduino Dust Sensor                 */
/*      - SparkFun ESP8266 Thing Wi-Fi        */
/*      - Shinyei PPD42NS dust sensor         */
/**********************************************/
/* !!! THIS CODE IS STILL IN ALPHA STAGE !!!  */
/**********************************************/
/* Based on these open source projects:       */
/* * Arduino Dust Sensor code by Shadowandy   */
/*   www.shadowandy.net                       */
/* * TinyGPS:                                 */
/*   https://github.com/mikalhart/TinyGPSPlus */
/* * Arduino core for ESP8266 WiFi chip:      */
/*   https://github.com/esp8266/Arduino       */
/**********************************************/

#include <ESP8266WiFi.h>                 
#include <TinyGPS++.h>
#include "config.h"

// hardware specific values
#define PPD_PM25_PIN 12
#define PPD_PM10_PIN 0 // not 13 because that is the ping where we receive serial GPS data when Serial.swap() is called.
#define SparkFun_ESP8266_LED_PIN 5
#define GPS_BAUD 9600
//#define GPS_BAUD 115200

// create a config.h file in the same folder with the following contents,
// and fill in your WiFi and thingspeak credentials.
/*
#define WIFISSID "your wifi network name"
#define WIFIPW "your wifi network password"
#define THINGADDR "api.thingspeak.com"
#define THINGKEY "your thingspeak API key"
#define DEBUGADDR ""
*/

const char esp_ssid[] = WIFISSID;
const char esp_pass[] = WIFIPW;
int esp_ChipId = 0;

const char thingSpeak_Address[] = THINGADDR;
const char debugAddress[] = DEBUGADDR;
const char thingSpeak_APIKey[] = THINGKEY;

TinyGPSPlus gps;

// The dataset consists of dust particle data and location, date and time from GPS
struct dataset_struct {
  float ppd_count_PM10;
  float ppd_count_PM25;
  float ppd_con_PM10;
  float ppd_con_PM25;
  float longitude;
  float latitude;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t year;
  uint8_t month;
  uint8_t day;
};
const uint8_t dataset_buffer_size = 128; // adjust this according to the available RAM
dataset_struct dataset_buffer[dataset_buffer_size]; // array for storing the dataset at different time intervals
uint8_t dataset_buffer_current = 0;   // array position where we should save the data
uint8_t dataset_buffer_last = 0;  // last saved array position
uint8_t dataset_buffer_lastsent = dataset_buffer_size-1; // array position of the last succesfully sent data
String dataset_string =""; //string that will be sent to the API

// Variables needed for PPD dust sensor sample counting
unsigned long ppd_starttime;
unsigned long ppd_saveperiod_ms = 10000; // every 10 seconds
//unsigned long ppd_sleeptime_ms = 255000000;
unsigned long ppd_trigger_on_PM10;
unsigned long ppd_trigger_on_PM25;
unsigned long ppd_trigger_off_PM10;
unsigned long ppd_trigger_off_PM25;
volatile unsigned long ppd_lowpulseoccupancy_PM10 = 0;
volatile unsigned long ppd_lowpulseoccupancy_PM25 = 0;
float ppd_ratio_PM10 = 0;
float ppd_ratio_PM25 = 0;
float ppd_count_PM10 = 0;
float ppd_count_PM25 = 0;
boolean ppd_value_PM10 = HIGH;
boolean ppd_value_PM25 = HIGH;
boolean ppd_trigger_PM10 = false;
boolean ppd_trigger_PM25 = false;
unsigned long sampledtime = 0;

// predeclare all functions
void LEDon();
void LEDoff();
boolean connectWiFi();
void uploadThingSpeak(String dataset_string);
void uploadDebug(String dataset_string);
void intrLOPM25();
void intrLOPM10();
void gpsInBuffer();
void ppdInBuffer(float ppd_con_PM10, float ppd_con_PM25, float ppd_count_PM10, float ppd_count_PM25);
void sendBuffer();
String formatDate(uint8_t year, uint8_t month, uint8_t day);
String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
// use DebugPrint() to output data to the serial port
void DebugPrint(String DebugData, boolean linefeed = false);
void DebugPrint(int DebugData, boolean linefeed = false);
void DebugPrint(uint16_t DebugData, boolean linefeed = false);
void DebugPrint(float DebugData, boolean linefeed = false);

void setup() {
  // We can use the LED on the SparkFun board for debugging
  // and turn it on during setup and when we store the sampled data
  pinMode(SparkFun_ESP8266_LED_PIN, OUTPUT);
  LEDon();

  // initialization for the GPS
  Serial.begin(GPS_BAUD);
  Serial.swap(); // SWAP Serial to the GPIO connected to the GPS module. We will call Serial.swap just before and after debug logging

  // initialization for the ESP Wi-Fi
  DebugPrint("\r\n\r\n\r\nStartup", true);
  delay(3000);
  esp_ChipId=ESP.getChipId();
  DebugPrint("  esp_ChipId=" + String(esp_ChipId), true);
  delay(2000);

  // initialization for the dust sensor
  pinMode(PPD_PM25_PIN, FUNCTION_3); //Set TX PIN to GPIO
  pinMode(PPD_PM10_PIN, FUNCTION_3); //Set RX PIN to GPIO
  pinMode(PPD_PM25_PIN, INPUT_PULLUP); //Listen at the designated PIN
  attachInterrupt(PPD_PM25_PIN, intrLOPM25, CHANGE); // Attaching interrupt to PIN
  pinMode(PPD_PM10_PIN, INPUT_PULLUP); //Listen at the designated PIN
  attachInterrupt(PPD_PM10_PIN, intrLOPM10, CHANGE); // Attaching interrupt to PIN
  ppd_starttime = millis(); //Fetching the current time
  
  DebugPrint("Startup done", true);
  LEDoff();
}

void loop() {
  // read all available serial data and decode the GPS information
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  // Dust sensor sample counting happens continuously via the interupt functions.
  // Check if it is time to store the samples as a new data set...
  sampledtime = millis() - ppd_starttime;
  if (sampledtime > ppd_saveperiod_ms) {
    LEDon(); // only for debugging
    
    DebugPrint("GPS=" + formatDate(gps.date.year(), gps.date.month(), gps.date.day()) +" " + formatTime(gps.time.hour(), gps.time.minute(), gps.time.second()) +" "+ String(gps.location.lat())+","+String(gps.location.lng()) + " " + String(gps.satellites.value()) + " sattelites", true);
    DebugPrint("Calculating sample data", true);
    
    ppd_ratio_PM25 = ppd_lowpulseoccupancy_PM25 / (sampledtime * 10.0);
    ppd_count_PM25 = 1.1 * pow(ppd_ratio_PM25, 3) - 3.8 * pow(ppd_ratio_PM25, 2) + 520 * ppd_ratio_PM25 + 0.62;
    ppd_ratio_PM10 = ppd_lowpulseoccupancy_PM10 / (sampledtime * 10.0);
    ppd_count_PM10 = 1.1 * pow(ppd_ratio_PM10, 3) - 3.8 * pow(ppd_ratio_PM10, 2) + 520 * ppd_ratio_PM10 + 0.62;
    ppd_count_PM25 -= ppd_count_PM10;
    
    // Begin mass concentppd_ration calculation
    float ppd_concentration_PM10 = 0;
    float ppd_concentration_PM25 = 0;
    double pi = 3.14159;
    double density = 1.65 * pow(10, 12);
    double K = 3531.5;
    
    // PM10
    double r10 = 2.6 * pow(10, -6);
    double vol10 = (4 / 3) * pi * pow(r10, 3);
    double mass10 = density * vol10;
    ppd_concentration_PM10 = (ppd_count_PM10) * K * mass10;
    
    // PM2.5
    double r25 = 0.44 * pow(10, -6);
    double vol25 = (4 / 3) * pi * pow(r25, 3);
    double mass25 = density * vol25;
    ppd_concentration_PM25 = (ppd_count_PM25) * K * mass25;
    // End of mass concentration calculation
    
    ppd_lowpulseoccupancy_PM25 = 0;
    ppd_lowpulseoccupancy_PM10 = 0;
    ppd_starttime = millis();
    
    // Only store the data in the buffer when there is valid GPS data
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
      // Fill in GPS and dust sensor data to current buffer position
      gpsInBuffer();
      ppdInBuffer(ppd_concentration_PM10, ppd_concentration_PM25, ppd_count_PM10, ppd_count_PM25);

      DebugPrint("Store data in buffer["+String(dataset_buffer_current)+"]", true);
      DebugPrint("  countPM10=" + String (dataset_buffer[dataset_buffer_current].ppd_count_PM10) + ",countPM25=" + String(dataset_buffer[dataset_buffer_current].ppd_count_PM25) + ",ppd_con_PM10=" + String(dataset_buffer[dataset_buffer_current].ppd_con_PM10) + ",ppd_con_PM25=" + String(dataset_buffer[dataset_buffer_current].ppd_con_PM25), true);
      
      dataset_buffer_last = dataset_buffer_current;
      dataset_buffer_current = (dataset_buffer_current+1) % dataset_buffer_size; // we use % (modulo) to make a ring buffer
      
      // Only upload the buffer data when there is Wi-Fi connection
      if (connectWiFi()) sendBuffer();
    }

    LEDoff();
  }
}

void LEDon() {
  digitalWrite(SparkFun_ESP8266_LED_PIN, HIGH);
}

void LEDoff() {
  digitalWrite(SparkFun_ESP8266_LED_PIN, LOW);
}

boolean connectWiFi() {
  char attempts=0;
  if (WiFi.status() == WL_CONNECTED) {
    DebugPrint("Wi-Fi is connected", true);
    return true;
  }
  WiFi.begin(esp_ssid, esp_pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    attempts++;
    if(WiFi.status() == WL_CONNECTED)
    return true;
    if(attempts>10)
    return false;
  }
}

void uploadThingSpeak(String dataset_string) {
  WiFiClient client;
  if (!client.connect(thingSpeak_Address, 80)) {
    return;
  }
  client.print(F("GET /adem?"));
  client.print(dataset_string);
  client.print(F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n"));
  client.println();
  DebugPrint("uploadThingSpeak:", true);
  DebugPrint("  " + dataset_string, true);
}

void uploadDebug(String dataset_string) {
  WiFiClient client;
  if (!client.connect(debugAddress, 3000)) {
    return;
  }
  client.print(F("GET /adem?"));
  client.print(dataset_string);
  client.print(F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n"));
  client.println();
  DebugPrint("uploadDebug:", true);
  DebugPrint("  " + dataset_string, true);
}

void intrLOPM25() {
  ppd_value_PM25 = digitalRead(PPD_PM25_PIN);
  if (ppd_value_PM25 == LOW && ppd_trigger_PM25 == false) {
    ppd_trigger_PM25 = true;
    ppd_trigger_on_PM25 = micros();
  }
  if (ppd_value_PM25 == HIGH && ppd_trigger_PM25 == true) {
    ppd_trigger_off_PM25 = micros();
    ppd_lowpulseoccupancy_PM25 += (ppd_trigger_off_PM25 - ppd_trigger_on_PM25);
    ppd_trigger_PM25 = false;
  }
}

void intrLOPM10() {
  ppd_value_PM10 = digitalRead(PPD_PM10_PIN);
  if (ppd_value_PM10 == LOW && ppd_trigger_PM10 == false) {
    ppd_trigger_PM10 = true;
    ppd_trigger_on_PM10 = micros();
  }
  if (ppd_value_PM10 == HIGH && ppd_trigger_PM10 == true) {
    ppd_trigger_off_PM10 = micros();
    ppd_lowpulseoccupancy_PM10 += (ppd_trigger_off_PM10 - ppd_trigger_on_PM10);
    ppd_trigger_PM10 = false;
  }
}

void gpsInBuffer() {
  dataset_buffer[dataset_buffer_current].latitude = gps.location.lat();
  dataset_buffer[dataset_buffer_current].longitude = gps.location.lng();
  dataset_buffer[dataset_buffer_current].month = gps.date.month();
  dataset_buffer[dataset_buffer_current].day = gps.date.day();
  dataset_buffer[dataset_buffer_current].year = gps.date.year();
  dataset_buffer[dataset_buffer_current].hours = gps.time.hour();
  dataset_buffer[dataset_buffer_current].minutes = gps.time.minute();
  dataset_buffer[dataset_buffer_current].seconds = gps.time.second();
}

void ppdInBuffer(float ppd_con_PM10, float ppd_con_PM25, float ppd_count_PM10, float ppd_count_PM25){
  dataset_buffer[dataset_buffer_current].ppd_count_PM10 = ppd_count_PM10;
  dataset_buffer[dataset_buffer_current].ppd_count_PM25 = ppd_count_PM25;
  dataset_buffer[dataset_buffer_current].ppd_con_PM10 = ppd_con_PM10;
  dataset_buffer[dataset_buffer_current].ppd_con_PM25 = ppd_con_PM25;
}

void sendBuffer(){
  DebugPrint("sendBuffer:", true);
  DebugPrint("  dataset_buffer_lastsent=" + String(dataset_buffer_lastsent), true);
  DebugPrint("  dataset_buffer_last=" + String(dataset_buffer_last), true);
  int i;
  //i= (dataset_buffer_lastsent+1) % dataset_buffer_size;
  i = dataset_buffer_lastsent;
  do {
    i = (i+1) % dataset_buffer_size;
    dataset_string = "key="+String(esp_ChipId)+"&ppd_con_PM10=" + String(dataset_buffer[i].ppd_con_PM10, DEC) + "&ppd_count_PM10=" + String(dataset_buffer[i].ppd_count_PM10, DEC) + "&ppd_con_PM25=" + 
    String(dataset_buffer[i].ppd_con_PM25, DEC) + "&ppd_count_PM25=" + String(dataset_buffer[i].ppd_count_PM25, DEC) + "&latitude=" + String(dataset_buffer[i].latitude, DEC) + 
    "&longitude=" + String(dataset_buffer[i].longitude, DEC) + "&gpsDate=" +formatDate(dataset_buffer[i].year,dataset_buffer[i].month,dataset_buffer[i].day) + 
    "&gpsTime=" + formatTime(dataset_buffer[i].hours,dataset_buffer[i].minutes,dataset_buffer[i].seconds);
    uploadDebug(dataset_string);
    dataset_buffer_lastsent = i;
    DebugPrint("  ["+String(i)+"] "+ dataset_string, true);
  } while (i!= ((dataset_buffer_last+1) % dataset_buffer_size));
}

String formatDate(uint8_t year, uint8_t month, uint8_t day) {
  char gpsDate[11];
  sprintf(gpsDate, "%04d-%02d-%02d", year, month, day);
  return gpsDate;
}

String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
  char gpsTime[9];
  sprintf(gpsTime, "%02d:%02d:%02d", hours, minutes, seconds);
  return gpsTime;
}

void DebugPrint(String DebugData, boolean linefeed) {
  Serial.flush();
  delay (10);Serial.swap();
  if (linefeed) {
    Serial.println(DebugData);Serial.println("");
  }
  else
    Serial.print(DebugData);
  Serial.flush();
  Serial.swap();
}

void DebugPrint(int DebugData, boolean linefeed) {
  Serial.flush();
  delay (10);Serial.swap();
  if (linefeed) {
    Serial.println(DebugData);Serial.println("");
  }
  else
    Serial.print(DebugData);
  Serial.flush();
  Serial.swap();
}

void DebugPrint(uint16_t DebugData, boolean linefeed) {
  Serial.flush();
  delay (10);Serial.swap();
  if (linefeed) {
    Serial.println(DebugData);Serial.println("");
  }
  else
    Serial.print(DebugData);
  Serial.flush();
  Serial.swap();
}

void DebugPrint(float DebugData, boolean linefeed) {
  Serial.flush();
  delay (10);Serial.swap();
  if (linefeed) {
    Serial.println(DebugData);Serial.println("");
  }
  else
    Serial.print(DebugData);
  Serial.flush();
  Serial.swap();
}

