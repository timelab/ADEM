#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#define LED 5
#define TRIGGER_PIN 12
WiFiManager wifiManager;
void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(LED,OUTPUT);
    pinMode(TRIGGER_PIN,INPUT_PULLUP);
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);
    delay(100);
    digitalWrite(LED,HIGH);
      
    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    
    //reset saved settings
    //wifiManager.resetSettings();
    
    //set custom ip for portal
    //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

    //fetches ssid and pass from eeprom and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if(!wifiManager.autoConnect("AutoConnectAP")){
         Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
    }
    //or use this for auto generated name ESP + ChipID
    //wifiManager.autoConnect();
    
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);
    delay(100);
    digitalWrite(LED,HIGH);
    delay(100);
    digitalWrite(LED,LOW);
 //   digitalWrite(LED,HIGH);

}

void loop() {
  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    Serial.println("reset");
    wifiManager.resetSettings();
    ESP.reset();
    delay(2000);

  }
    
}
