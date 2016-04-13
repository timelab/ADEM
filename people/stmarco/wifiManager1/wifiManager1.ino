#include <ESP8266WiFi.h>   
//needed for wifimanager library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#define TRIGGER_PIN 12

    WiFiManager wifiManager;


void setup() {
//WiFiManager wifiManager;
//wifiManager.autoConnect("ESPWIFI", "connect");
pinMode(TRIGGER_PIN, INPUT_PULLUP);
Serial.begin(74880);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    wifiManager.startConfigPortal("OnDemandAP");
    Serial.println("connected...yeey :)");
  }
}

