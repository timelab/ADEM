#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "BufferedResponse.h"

ESP8266WebServer server(80);

#define WIFI_WAIT 5

const char *ssid = "";
const char *password = "";


void loop() {
  server.handleClient();
};


void handleRoot() {
  BufferedResponse response(server, "text/plain", 200, 1024);

  for (uint32_t i = 0; i < 50000; i++)
    response.printf("%10i", i);
}
void setupWifi() {
  WiFi.hostname("webtest");
  WiFi.begin(ssid, password);

  // Wait for connection - didn't do so???
  int s = WIFI_WAIT;
  while (s > 0 && WiFi.status() != WL_CONNECTED) {
    delay(1000);
    s--;
  }
}


void setup() {
  Serial.begin(9600);
  setupWifi();

  server.on("/", handleRoot);

  server.begin();
  Serial.println("HTTP server started");
};