/*
 *  This sketch demonstrates how to scan WiFi networks. 
 *  The API is almost the same as with the WiFi Shield library, 
 *  the most obvious difference being the different file you need to include:
 */
#include "ESP8266WiFi.h"

boolean isWifi = false;
boolean isConnected = false;
int mywifi = -1;
String mywifi1 = "timelab-wifi";
String mywifi2 = "TelenetWiFree";
String wifiAP ="";

void setup() {
  Serial.begin(9600);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Setup done");
}

void loop() {
 if(!isWifi){ 
 wifiAP = getWifi();
 Serial.print("AP:");
 Serial.println(wifiAP);
 isWifi = true;
 }
 else if(! isConnected){
  isConnected = connectWifi(wifiAP);
 }
 }

String getWifi() {
  boolean hasWifi=false;
  while (!hasWifi){
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      
      // Print SSID and RSSI for each network found
      Serial.print(i);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.print((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      if(mywifi1.compareTo(String(WiFi.SSID(i)))==0){
      Serial.print(" mywifi ");
      mywifi = i;
      }
      if(mywifi2.compareTo(String(WiFi.SSID(i)))==0){
        Serial.print(" mywifi");
        if(mywifi>=0){
          if(int(WiFi.RSSI(i))>int(WiFi.RSSI(mywifi))){
            mywifi = i;
          }
        }
        else{
          mywifi = i;
        }
      }
      Serial.print((mywifi2.compareTo(String(WiFi.SSID(i)))==0)?" mywifi ":"");
      Serial.println("");
      delay(10);
    }
    Serial.println("mywifi:"+ String(mywifi));
  }
  Serial.println("");
  return WiFi.SSID(mywifi);
  // Wait a bit before scanning again
  delay(5000);
  }
}

boolean connectWifi(String wifiAP){
  
}


