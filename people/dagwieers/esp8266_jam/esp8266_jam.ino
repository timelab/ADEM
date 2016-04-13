// Serial debugging off
#define DEBUGSERIAL true

#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <particulate_PPD42.h>
#include <TinyGPS++.h>

#ifdef DEBUGSERIAL
#include <SoftwareSerial.h>
#endif

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define WIFI_SSID "timelab-wifi"
#define WIFI_WPA2 "timelab09"

#define NEOPIXEL_PIN 5
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 14
#define GPS_RX_PIN 0                    // to GPS module RX pin. Does not work on GPIO 16 (XPD).
#define GPS_TX_PIN 4                    // we send no data to the GPS module
#define SERIAL_RX_PIN 8
#define SERIAL_TX_PIN 7

#define GPS_BAUD 9600
#define SERIAL_BAUD 74880
#define SERIAL1_BAUD 74880

#if DEBUGSERIAL
#define __LOG(msg) Serial.print(msg); if (client) client.print(msg);
#define __LOGLN(msg) Serial.println(msg); if (client) client.println(msg);
#else
#define __LOG(msg) Serial1.print(msg); if (client) client.print(msg);
#define __LOGLN(msg) Serial1.print(msg); if (client) client.println(msg);
#endif      

// Neopixel needs to be outside of setup()
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PPD42Sensor particulate;
#ifdef DEBUGSERIAL
SoftwareSerial gps_Serial(GPS_RX_PIN, GPS_TX_PIN);
#endif
TinyGPSPlus gps;
WiFiServer server(80);
WiFiClient client;

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

void setup() {

  neopixel.begin();
  neopixel.setBrightness(64);

  // Blue
  neopixel.setPixelColor(0, 0, 0, 70);
  neopixel.show();

//  delay(2000);
#ifdef DEBUGSERIAL
//  pinMode(SERIAL_RX_PIN, INPUT);
//  pinMode(SERIAL_TX_PIN, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  gps_Serial.begin(GPS_BAUD);  
#else
//  pinMode(SERIAL1_RX_PIN, INPUT);
//  pinMode(SERIAL1_TX_PIN, OUTPUT);
  Serial1.begin(SERIAL1_BAUD)
//  pinMode(GPS_RX_PIN, INPUT);
  Serial.begin(GPS_BAUD);  
#endif

  __LOGLN("Boot started.");
  __LOGLN("NeoPixel initialized...");
  __LOGLN("SoftwareSerial initialized...");
  __LOGLN("GPS initialized...");
  __LOGLN("Serial communication initialized...");

/*
  __LOGLN("Initializing ESP8266...");
  if (esp8266.begin())
    __LOGLN(" done");
  else
    __LOGLN(" failed to communicate");
*/

  __LOG("Initializing PPD42...");
  particulate.begin();
  __LOGLN(" done");

/*
  __LOG("Initializing WiFi access point...");
  setupWiFiAP();
  __LOGLN(" done");
*/

  __LOG("Initializing WiFi client...");
  if (setupWiFiClient()) {
    __LOGLN(" done");
    __LOG("WiFi local IP: ");
    __LOGLN(WiFi.localIP());
  } else {
    __LOGLN(" failed");
  }

  __LOG("Initializing Server...");
  server.begin();
  __LOGLN(" done");

  __LOGLN("Boot finished.");

  // Red
  neopixel.setPixelColor(0, 70, 0, 0);
  neopixel.show();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (client) {
//    __LOGLN("Sending data to web client.");
    client.println("Time: " + formatDate(gps.date.year(), gps.date.month(), gps.date.day()) +" " + formatTime(gps.time.hour(), gps.time.minute(), gps.time.second()));
    client.println("Location: " + String(gps.location.lat())+","+String(gps.location.lng()));
    client.println("Satellites: " + String(gps.satellites.value()));
    client.flush();
    
    delay(500);
  } else {
    client = server.available();
    if (client) {
//      __LOGLN("Web connection received.");
      String req = client.readStringUntil('\r');
//      __LOGLN(req);
      client.flush();  

/*
      // Match the request
      int val = -1; // We'll use 'val' to keep track of both the
                    // request type (read/set) and value if set.
      if (req.indexOf("/neopixel/blue") != -1) {
        neopixel.setPixelColor(0, 0, 0, 70);
        neopixel.show();
      } else if (req.indexOf("/neopixel/yellow") != -1) {
        neopixel.setPixelColor(0, 70, 70, 0);
        neopixel.show();
      } else {
        neopixel.setPixelColor(0, 0, 70, 0);
        neopixel.show();    
      }
*/

      client.print("HTTP/1.1 200 OK\r\n");
      client.print("Content-Type: text/plain\r\n\r\n");
      client.flush();
      __LOGLN("Answering web request.");
      
//    } else {
//      Serial.println("No web client.");
    }
  }

  unsigned long start = millis();
  do {
#ifdef DEBUGSERIAL
    while (gps_Serial.available() > 0) {
      gps.encode(gps_Serial.read());
#else
    while (Serial.available() > 0) {
      gps.encode(Serial.read());
#endif
    }
    delay(5);
  } while (millis() - start < 100);

/*
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid()) {
    Serial.println("Time: " + formatDate(gps.date.year(), gps.date.month(), gps.date.day()) +" " + formatTime(gps.time.hour(), gps.time.minute(), gps.time.second()));
    Serial.println("Location: " + String(gps.location.lat())+","+String(gps.location.lng()));
    Serial.println("Satellites: " + String(gps.satellites.value()));
  }
*/

  delay(20);

  unsigned long PM10 = particulate.readPM10Ppm();
  unsigned long PM25 = particulate.readPM25Ppm();

  if (PM10 > 0 || PM25 > 0) {
    if (PM10 > 0) {
      __LOG("PM10 ppm: ");
      __LOGLN(PM10);
      neopixel.setPixelColor(0, 0, 70, 0);
      neopixel.show();
    }
    if (PM25 > 0) {
      __LOG("PM25 ppm: ");
      __LOGLN(PM25);
      neopixel.setPixelColor(0, 0, 70, 0);
      neopixel.show();
    }
  }
}

void setupWiFiAP() {
  WiFi.mode(WIFI_AP);
  String AP_NameString = "Timelab ADEM - Dag";

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, "timelab09");
}

bool setupWiFiClient() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_WPA2);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    return false;
  } else {
    return true;
  }
//  WiFi.printDiag(Serial);
}
