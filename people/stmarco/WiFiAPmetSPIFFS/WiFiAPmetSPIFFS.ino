/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Create a WiFi access point and provide a web server on it. */

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include "FS.h"

//Global variables
const char *ssid = "ESPap";
const char *password = "thereisnospoon";
const int led = 13;
String wifiList = "";

//function prototypes
String getHead(void);
String getFoot(void);
String getWifi(void);
void initAp(void);
void handleRoot(void);
void handleScan(void);
void handleSetup(void);
void handleAdem(void);
void handleStyle(void);
void handleNotFound(void);
void handleSave(void);

ESP8266WebServer server(80);


/* Just a little test message.  Go to http://192.168.4.1 in a web browser
 * connected to this access point to see it.
 */

void setup() {
	delay(1000);
	Serial.begin(115200);
	Serial.println();
  //wifiList = getWifi();
	Serial.print("Configuring access point...");
	/* You can remove the password parameter if you want the AP to be open. */
  initAp();
	server.on("/", handleRoot);
 server.on("/index.html", handleRoot);
 server.on("/scan.html", handleScan);
 server.on("/setup", handleSetup);
 server.on("/adem.svg", handleAdem);
 server.on("/style.css", handleStyle);
 server.on("/save", handleSave);
  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });


  server.onNotFound(handleNotFound);

	server.begin();
	Serial.println("HTTP server started");
}

void loop() {
	server.handleClient();
}

void initAp() {
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

}


/* begin en einde van elke webpagina */

void handleRoot() {
  FILE i = SPIFFS.open("/index.html", "r");

      String index ="";
      while(i.available()) {
         index += i.readStringUntil('\n');
    }
  server.send(200, "text/html", index);
}

void handleAdem() {
  FILE a = SPIFFS.open("/adem.svg", "r");
  String adem = "";
  while(a.available()) {
    adem += a.readStringUntil('\n');
  }
  server.send(200, "text/html", adem);
}

void handleStyle() {
  FILE s = SPIFFS.open("/style.css", "r");
  String style = "";
  while(s.available()) {
    style += s.readStringUntil('\n');
  }
  server.send(200, "text/html", style);
}


void handleScan() {
  String scanMessage="";
    scanMessage += getHead();
    scanMessage += wifiList;
    scanMessage += getFoot();
  server.send(200, "text/html", scanMessage);
}


void handleSetup(){
  String setupMessage = "<h1>Setup page</h1>\n";
  setupMessage += "<div><form method=\"post\" action=\"/save\">\n";
  setupMessage += "<input type=\"text\" name=\"SSID\" placeholder=\"SSID\"><br>\n";
  setupMessage += "<input type=\"text\" name=\"Username\" placeholder=\"username (optional)\"><br>\n";
  setupMessage += "<input type=\"text\" name=\"network key\" placeholder=\"network key\"><br>\n";
  setupMessage += "<button type=\"submit\">Save</button>\n";
  server.send(200, "text/html", getHead() + setupMessage + getFoot());
}
void handleNotFound(){
  digitalWrite(led, 1);
  String notFoundMessage = "File Not Found\n\n";
  notFoundMessage += "URI: ";
  notFoundMessage += server.uri();
  notFoundMessage += "\nMethod: ";
  notFoundMessage += (server.method() == HTTP_GET)?"GET":"POST";
  notFoundMessage += "\nArguments: ";
  notFoundMessage += server.args();
  notFoundMessage += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    notFoundMessage += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", getHead() + notFoundMessage + getFoot());
  digitalWrite(led, 0);
}
void handleSave(){
  String saveMessage= "";
  saveMessage += getHead();
  saveMessage += "<h1>Connection data saved</h1>";
  String ssid = server.arg(0);
  String user = server.arg(1);
  String key = server.arg(2);
  saveMessage += "<p>SSID: <em>\n";
  saveMessage += ssid;
  saveMessage += "</em></p>\n";
  saveMessage += "<p>Username: <em>\n";
  saveMessage += user;
  saveMessage += "</em></p>\n";
  saveMessage += "<p>network key: <em>\n";
  saveMessage += key;
  saveMessage += "</em></p>\n";
  saveMessage += getFoot();
  server.send(200, "text/html", saveMessage);
}

String getWifi() {
  boolean hasWifi=false;
  String wifiList="";
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  while (!hasWifi){
  
 // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  if (n == 0)
    wifiList ="no networks found";
  else
  {
    wifiList += " networks found";
    for (int i = 0; i < n; ++i)
    {
      
      // Print SSID and RSSI for each network found
      wifiList += "<li>";
      wifiList += i;
      wifiList += ": ";
      wifiList += WiFi.SSID(i);
      wifiList += " (";
      wifiList += WiFi.RSSI(i);
      wifiList += ")";
      wifiList += (WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*";
      wifiList += "</li>";
      }
    }
  }
  return wifiList;
  // Wait a bit before scanning again
}


