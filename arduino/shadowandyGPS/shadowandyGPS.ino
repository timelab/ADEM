/**********************************************/
/* Building Arduino Dust Sensor using:        */
/*      - ESP8266 ESP-01                      */
/*      - 3.3-to-5v Logic Level Converter     */
/*      - Shinyei PPD42NS                     */
/* http://www.sca-shinyei.com/pdf/PPD42NS.pdf */
/*                                            */
/* Author: shadowandy[dot]sg[at]gmail[dot]com */
/* Web: www.shadowandy.net                    */
/*                                            */         
/* Wiring Instruction:                        */
/*      - PPD42NS Pin 1 => GND                */
/*      - PPD42NS Pin 2 => TX                 */
/*      - PPD42NS Pin 3 => 5V                 */
/*      - PPD42NS Pin 4 => RX                 */
/**********************************************/
/* TinyGPS:                                   */
/* https://github.com/mikalhart/TinyGPSPlus   */
/**********************************************/
#include <ESP8266WiFi.h>                 
#include <TinyGPS++.h>
#include "config.h"

#define PPD_PM25_PIN 18
#define PPD_PM10_PIN 17
#define FSM_BUFFER_SIZE 128

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

static const uint32_t gps_Baud = 9600;
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(13,14); //RX,TX

TinyGPSPlus gps;


struct fsm_struct {
  float ppd_countPM10;
  float ppd_countPM25;
  float conPM10;
  float conPM25;
  float longitude;
  float latitude;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t year;
  uint8_t month;
  uint8_t day;
};
fsm_struct fsm_temp;
fsm_struct fsm_arr[FSM_BUFFER_SIZE];
String fsm_data =""; //string die uiteindelijk naar de api gestuurd zal worden.
uint8_t fsm_next = 0;   //positie voor volgende data in buffer
uint8_t fsm_current = FSM_BUFFER_SIZE;  //postitie laatst bewaarde data in buffer
uint8_t fsm_last = 0; //laatst doorgestuurde waarde

unsigned long ppd_starttime;
unsigned long ppd_sampletime_ms = 10000;
unsigned long ppd_sleeptime_ms = 255000000;
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

boolean connectWiFi();
void updateThingSpeak(String fsm_data);
void updateDebug(String fsm_data);
void intrLOPM25();
void intrLOPM10();
void returnGpsInfo();
void storeBuffer(float ppd_con_PM10, float ppd_con_PM25, float ppd_count_PM10, float ppd_count_PM25);
void sendBuffer();
String formatDate(uint8_t year, uint8_t month, uint8_t day);
String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

void setup() {
  delay(3000);
  Serial.begin(gps_Baud);
  esp_ChipId=ESP.getChipId();
  Serial.print("start esp_ChipId:");
  Serial.println(esp_ChipId);
  delay(2000);
  pinMode(PPD_PM25_PIN, FUNCTION_3); //Set TX PIN to GPIO
  pinMode(PPD_PM10_PIN, FUNCTION_3); //Set RX PIN to GPIO
  pinMode(PPD_PM25_PIN, INPUT_PULLUP); //Listen at the designated PIN
  attachInterrupt(PPD_PM25_PIN, intrLOPM25, CHANGE); // Attaching interrupt to PIN
  pinMode(PPD_PM10_PIN, INPUT_PULLUP); //Listen at the designated PIN
  attachInterrupt(PPD_PM10_PIN, intrLOPM10, CHANGE); // Attaching interrupt to PIN
  ppd_starttime = millis(); //Fetching the current time
}

void loop() {
    
  if ((millis() - ppd_starttime) > ppd_sampletime_ms) //Checking if it is time to sample
  {
    Serial.println("sample loop");
    unsigned long sampledtime = millis() - ppd_starttime;
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

      while (gpsSerial.available() > 0)
        if (gps.encode(gpsSerial.read())){
          returnGpsInfo();
          Serial.println("storeBuffer");
          storeBuffer(ppd_concentration_PM10, ppd_concentration_PM25, ppd_count_PM10, ppd_count_PM25);
          sendBuffer();
        }
  }
  
}
/////////////////////////////////:

boolean connectWiFi() {
  char attempts=0;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("wifi is connected");
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

void updateThingSpeak(String fsm_data) {
  WiFiClient client;
  if (!client.connect(thingSpeak_Address, 80)) {
    return;
  }
  client.print(F("GET /adem?"));
  client.print(fsm_data);
  client.print(F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n"));
  client.println();
  Serial.println(fsm_data);
}

void updateDebug(String fsm_data) {
  WiFiClient client;
  if (!client.connect(debugAddress, 3000)) {
    return;
  }
  client.print(F("GET /adem?"));
  client.print(fsm_data);
  client.print(F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n"));
  client.println();
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

void returnGpsInfo()
{
  if (gps.location.isValid())
  {
    fsm_temp.latitude = gps.location.lat();
    fsm_temp.longitude = gps.location.lng();
  }
  else
  {
    fsm_temp.latitude = 0.0;
    fsm_temp.longitude = 0.0;
  }

  if (gps.date.isValid())
  {
    fsm_temp.month = gps.date.month();
    fsm_temp.day = gps.date.day();
    fsm_temp.year = gps.date.year();
  }
  else
  {
    fsm_temp.month = 0;
    fsm_temp.day = 0;
    fsm_temp.year = 0;
  }
  if (gps.time.isValid())
  {
    fsm_temp.hours = gps.time.hour();
    fsm_temp.minutes = gps.time.minute();
    fsm_temp.seconds = gps.time.second();
  }
  else
  {
    fsm_temp.hours = 0;
    fsm_temp.minutes = 0;
    fsm_temp.seconds = 0;
  }
}

void storeBuffer(float ppd_con_PM10, float ppd_con_PM25, float ppd_count_PM10, float ppd_count_PM25){
  fsm_arr[fsm_next].ppd_countPM10 = ppd_count_PM10;
  fsm_arr[fsm_next].ppd_countPM25 = ppd_count_PM25;
  fsm_arr[fsm_next].conPM10 = ppd_con_PM10;
  fsm_arr[fsm_next].conPM25 = ppd_con_PM25;
  fsm_arr[fsm_next].longitude = fsm_temp.longitude;
  fsm_arr[fsm_next].latitude = fsm_temp.latitude;
  fsm_arr[fsm_next].hours = fsm_temp.hours;
  fsm_arr[fsm_next].minutes = fsm_temp.minutes;
  fsm_arr[fsm_next].seconds = fsm_temp.seconds;
  fsm_arr[fsm_next].year = fsm_temp.year;
  fsm_arr[fsm_next].month = fsm_temp.month;
  fsm_arr[fsm_next].day = fsm_temp.day;
  Serial.println("----------------");  
  Serial.print("day:");
  Serial.print(fsm_temp.day);
  Serial.print(" month:");
  Serial.print(fsm_temp.month);
  Serial.print(" year:");
  Serial.print(fsm_temp.year);
  Serial.print(" hours:");
  Serial.print(fsm_temp.hours);
  Serial.print(" minutes:");
  Serial.print(fsm_temp.minutes);
  Serial.print(" seconds:");
  Serial.print(fsm_temp.seconds);
  Serial.print(" countPM10:");
  Serial.print(ppd_count_PM10);
  Serial.print(" countPM25:");
  Serial.print(ppd_count_PM25);
  Serial.print(" conPM10:");
  Serial.print(ppd_con_PM10);
  Serial.print(" conPM25:");
  Serial.print(ppd_con_PM25);
  Serial.print(" fsm_next:");
  Serial.println(fsm_next);
  fsm_current = fsm_next;
  fsm_next = (fsm_next+1) % FSM_BUFFER_SIZE;
}

void sendBuffer(){
Serial.print(" fsm_last:");
Serial.println(fsm_last);
Serial.print(" fsm_current:");
Serial.println(fsm_current);
  if(((fsm_current+FSM_BUFFER_SIZE)-fsm_last) > 3){
    if(connectWiFi()){
      int i = fsm_last;
      do{
         fsm_data = "key="+String(esp_ChipId)+"&conPM10=" + String(fsm_arr[i].conPM10, DEC) + "&ppd_countPM10=" + String(fsm_arr[i].ppd_countPM10, DEC) + "&conPM25=" + 
         String(fsm_arr[i].conPM25, DEC) + "&ppd_countPM25=" + String(fsm_arr[i].ppd_countPM25, DEC) + "&latitude=" + String(fsm_arr[i].latitude, DEC) + 
         "&longitude=" + String(fsm_arr[i].longitude, DEC) + "&gpsDate=" +formatDate(fsm_arr[i].year,fsm_arr[i].month,fsm_arr[i].day) + 
         "&gpsTime=" + formatTime(fsm_arr[i].hours,fsm_arr[i].minutes,fsm_arr[i].seconds);
          updateDebug(fsm_data);
          i = (i+1) + FSM_BUFFER_SIZE;
          fsm_last = i;
          Serial.println(fsm_data);
      }while(i!=fsm_current);
    }
    return;
  }
}

String formatDate(uint8_t year, uint8_t month, uint8_t day){
  char gpsDate[11];
  sprintf(gpsDate, "%04d-%02d-%02d", year, month, day);
  return gpsDate;
}

String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds){
  char gpsTime[9];
  sprintf(gpsTime, "%02d:%02d:%02d", hours, minutes, seconds);
  return gpsTime;
}

