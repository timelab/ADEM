/*************************************************/
/* Mobile Arduino Dust Sensor                    */
/*      - SparkFun ESP8266 Thing Wi-Fi           */
/*      - Shinyei PPD42NS dust sensor            */
/*************************************************/
/* !!! THIS CODE IS STILL IN ALPHA STAGE !!!     */
/*************************************************/
/* Based on these open source projects:          */
/* * Arduino Dust Sensor code by Shadowandy      */
/*   www.shadowandy.net                          */
/* * TinyGPS:                                    */
/*   https://github.com/mikalhart/TinyGPSPlus    */
/* * Arduino core for ESP8266 WiFi chip:         */
/*   https://github.com/esp8266/Arduino          */
/* * ESP softwareserial, Peter Lerup             */
/*   https://github.com/plerup/espsoftwareserial */
/*************************************************/

#include <Adafruit_NeoPixel.h>
#include <particulate_PPD42.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

// hardware specific values
#define SparkFun_ESP8266_LED_PIN 5      // We can later use GPIO5 for driving a buzzer
#define SparkFun_SERIAL_BAUD 74880

#define GPS_RX_PIN 4                    // to GPS module RX pin. Does not work on GPIO 16 (XPD).
#define GPS_TX_PIN SW_SERIAL_UNUSED_PIN // we send no data to the GPS module
//#define PPD_PM10_PIN 12                 // to PPD42NS pin 4, do not use GPIO16, does not seem to work: 12
//#define PPD_PM25_PIN 13                 // to PPD42NS pin 2, do not use GPIO16
#define NEOPIXEL_PIN 0
#define GPS_BAUD 9600

int esp_ChipId = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

PPD42Sensor particulate;
SoftwareSerial gps_Serial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// predeclare all functions
String formatDate(uint16_t year, uint8_t month, uint8_t day);
String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

void setup() {

  pixels.begin();
  pixels.setBrightness(64);
  
  // We can use the LED on the SparkFun board for debugging
  // and turn it on during setup and when we store the sampled data
//  pinMode(SparkFun_ESP8266_LED_PIN, OUTPUT);
  
  // Initialization for debug output  
  // Don't open serial yet, this makes reprogramming more likely to be successful
  delay(5000);
  
  // Start serial communication
  Serial.begin(SparkFun_SERIAL_BAUD);
  Serial.println("Initializing serial communication... done");

  Serial.print("Initializing PPD42...");
  particulate.begin();
  Serial.println(" done");
  
  // initialization of GPS serial
/*
  Serial.print("Initializing GPS...");
  gps_Serial.begin(GPS_BAUD);
  Serial.println(" done");
*/
  
  // initialization for the ESP Wi-Fi
/*
  Serial.println("Initializing Wi-Fi...");
  esp_ChipId = ESP.getChipId();
  Serial.println(" done - esp_ChipId: " + String(esp_ChipId));
*/

//  NEOred();
  Serial.println("Setup done.");
}

void loop() {

  // read all available serial data and decode the GPS information
/*
  unsigned long start = millis();
  do {
    NEOyellow();
    NEOoff();
    while (gps_Serial.available() > 0) {
      gps.encode(gps_Serial.read());
    }
  } while (millis() - start < 1000);
*/

  delay(30000);

//  unsigned long timing = millis();
//  Serial.println(timing);
//  if (( timing / 1000 ) % 10 == 0) {
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid()) {
      Serial.println("Time: " + formatDate(gps.date.year(), gps.date.month(), gps.date.day()) +" " + formatTime(gps.time.hour(), gps.time.minute(), gps.time.second()));
      Serial.println("Location: " + String(gps.location.lat())+","+String(gps.location.lng()));
      Serial.println("Satellites: " + String(gps.satellites.value()));
    }

    unsigned long PM10 = particulate.readPM10Ppm();
    unsigned long PM25 = particulate.readPM25Ppm();
        
    Serial.print("PM10 ppm: ");
    Serial.println(PM10);
    Serial.print("PM25 ppm: ");
    Serial.println(PM25);

/*
    if (PM10 > 0 || PM25 > 0) {    
      NEOgreen();
    }
*/
//   }
}

/*
void NEOblue() {
    pixels.setPixelColor(0, 0, 0, 70);
    pixels.show();
}

void NEOgreen() {
    pixels.setPixelColor(0, 0, 70, 0);
    pixels.show();
}

void NEOyellow() {
    pixels.setPixelColor(0, 70, 70, 0);
    pixels.show();
}

void NEOred() {
    pixels.setPixelColor(0, 70, 0, 0);
    pixels.show();
}

void NEOwhite() {
    pixels.setPixelColor(0, 70, 70, 70);
    pixels.show();
}

void NEOorange() {
    pixels.setPixelColor(0, 70, 35, 70);
    pixels.show();
}

void NEOoff() {
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
}
*/

String formatDate(uint16_t year, uint8_t month, uint8_t day) {
  char gpsDate[11] = "";
  sprintf(gpsDate, "%04d-%02d-%02d", year, month, day);
  return gpsDate;
}

String formatTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
  char gpsTime[9]= "";
  sprintf(gpsTime, "%02d:%02d:%02d", hours, minutes, seconds);
  return gpsTime;
}
