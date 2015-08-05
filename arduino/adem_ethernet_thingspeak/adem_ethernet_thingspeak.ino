/*

 Arduino --> ThingSpeak Channel via Ethernet
 
 The ThingSpeak Client sketch is designed for the Arduino and Ethernet.
 This sketch updates a channel feed with an analog input reading via the
 ThingSpeak API (https://thingspeak.com/docs)
 using HTTP POST. The Arduino uses DHCP and DNS for a simpler network setup.
 The sketch also includes a Watchdog / Reset function to make sure the
 Arduino stays connected and/or regains connectivity after a network outage.
 Use the Serial Monitor on the Arduino IDE to see verbose network feedback
 and ThingSpeak connectivity status.
 
 Getting Started with ThingSpeak:
 
   * Sign Up for New User Account - https://thingspeak.com/users/new
   * Create a new Channel by selecting Channels and then Create New Channel
   * Enter the Write API Key in this sketch under "ThingSpeak Settings"
 
 Arduino Requirements:
 
   * Arduino with Ethernet Shield or Arduino Ethernet
   * Arduino 1.0+ IDE
   
  Network Requirements:

   * Ethernet port on Router    
   * DHCP enabled on Router
   * Unique MAC Address for Arduino
 
 Created: October 17, 2011 by Hans Scharler (http://www.nothans.com)
 
 Additional Credits:
 Example sketches from Arduino team, Ethernet by Adrian McEwen
 
*/

/* 
 Lieven Blancke, 2015-08-01    
  - Do not put your writeAPIKey here, put it in /home/lieven/thingspeak_credentials.h
    and do not check in this separate file 
    #define TSWRITEAPIKEY  "ABCDEFHIJKLMNOP" // place your ThinkSpeak Write API Key here

  - PPD60V analog output or TP 7 on the PCB of PPD42N connected to Analog PIN 0

*/

#include <SPI.h>
#include <Ethernet.h>
#include "/home/lieven/thingspeak_credentials.h"

// Local Network Settings
byte mac[] = { 0xD4, 0x28, 0xB2, 0xFF, 0xA0, 0xA2 }; // Must be unique on local network

// ThingSpeak Settings
char thingSpeakAddress[] = "api.thingspeak.com";
String writeAPIKey = TSWRITEAPIKEY;

const int updateThingSpeakInterval = 16 * 1000;      // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)

// Thingspeak Variable Setup
long lastConnectionTime = 0; 
boolean lastConnected = false;
int failedCounter = 0;
String analogValue0 = "";

// PPD Variable Setup
unsigned long analogcount = 0;
double analogtotal;
double analogsend;
unsigned long analogsendcount = 0;
unsigned long DISCARTED_SAMPLES = 4;
unsigned long starttime = 0;
unsigned long sampletime_ms = 1000;
unsigned long analogvalue = 0;
double LconAV = 0;
double Lugm3_2 = 0;


// Initialize Arduino Ethernet Client
EthernetClient client;

void setup()
{
  // Start Serial for debugging on the Serial Monitor
  Serial.begin(9600);
  
  // Start Ethernet on Arduino
  startEthernet();

  starttime = millis();
}

void loop()
{ 
  // Read PPD value from Analog Input Pin 0
  analogvalue = analogvalue + analogRead(0);
  analogcount++;
  if ((millis()-starttime) > sampletime_ms) // every second
  {
    starttime = millis(); // moved to create more regular periods.
    analogvalue = analogvalue/analogcount;
    analogtotal = (analogvalue*5.0)/1024;
    LconAV = (240.0*pow(analogtotal,6) - 2491.3*pow(analogtotal,5) + 9448.7*pow(analogtotal,4) - 14840.0*pow(analogtotal,3) + 10684.0*pow(analogtotal,2) + 2211.8*(analogtotal) + 7.9623);
    Lugm3_2 = (0.0000000495*pow(LconAV,2) + 0.0015247767*(LconAV));//Cardboard
    //y = 0.0000000495x2 + 0.0015247767x - 0.7700757363 //Cardboard
    
    //Lugm3 = (0.000000000000000000000000124084*pow(LconAV,6) - 0.000000000000000000021601485060*pow(LconAV,5) + 0.00000000000000143560109703371*pow(LconAV,4) - 0.0000000000441691662383661*pow(LconAV,3) + 0.000000633178466830333*pow(LconAV,2) - 0.00127531308452014*(LconAV) + 6.46046830008057); //-1E-16x4 + 1E-11x3 - 4E-07x2 + 0.0055x - 8.1472         
    //0.000000000000000000000000124084x6 - 0.000000000000000000021601485060x5 + 0.000000000000001435601097033710x4 - 0.000000000044169166238366100000x3 + 0.000000633178466830333000000000x2 - 0.001275313084520140000000000000x + 6.460468300080570000000000000000
    //0.000000000000000000000000107376x6 - 0.000000000000000000017973858956x5 + 0.000000000000001131107063421510x4 - 0.000000000031786851160857600000x3 + 0.000000383869228359063000000000x2 + 0.000966287521677600000000000000x

    //if(LconAV < 0){
    //  LconAV = 0;
    //}
    if(Lugm3_2 < 0){
      Lugm3_2 = 0;
    }
 
    Serial.println();
    Serial.print(F(" Analog Total: "));
    Serial.print(analogtotal);
    Serial.print(F(" Lower Concentration: "));
    Serial.print(LconAV);
    Serial.print(F(" Lower ugm3_2: "));
    Serial.print(Lugm3_2);
    Serial.println();

    analogsendcount++;
    // de eerste drie samples (seconden) na upload naar ThingSpeak lijken af te wijken, die tellen we niet mee
    if (analogsendcount>DISCARTED_SAMPLES) {
      analogsend+= Lugm3_2;
    }
    else {
      Serial.println("...discarted sample!");
    }
    
    analogvalue = 0;
    analogcount = 0;
  }

  // Print Update Response to Serial Monitor
  if (client.available())
  {
    char c = client.read();
    Serial.print(c);
  }

  // Disconnect from ThingSpeak
  if (!client.connected() && lastConnected)
  {
    Serial.println("...disconnected");
    Serial.println();
    
    //millis1=millis();
    client.stop();
    debugit("client.stop");
  }
  
  // Update ThingSpeak
  if(!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval))
  {
    analogsend=analogsend/(analogsendcount - DISCARTED_SAMPLES);
    analogsendcount=0;
    analogValue0 = String(analogsend, DEC);
    updateThingSpeak("field1="+analogValue0); // temporarily we only send the last measured value
  }
  
  // Check if Arduino Ethernet needs to be restarted
  if (failedCounter > 3 ) {startEthernet();}
  
  lastConnected = client.connected();
}

void debugit (String debugdata)
{
  Serial.print ("###DEBUG ");
  Serial.print (millis()/100);
  Serial.println (":" + debugdata);
}


void updateThingSpeak(String tsData)
{
  if (client.connect(thingSpeakAddress, 80))
  {         
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+writeAPIKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");

    client.print(tsData);
    
    lastConnectionTime = millis();
    
    if (client.connected())
    {
      Serial.println("Connecting to ThingSpeak...");
      Serial.println();
      
      failedCounter = 0;
    }
    else
    {
      failedCounter++;
  
      Serial.println("Connection to ThingSpeak failed ("+String(failedCounter, DEC)+")");   
      Serial.println();
    }
    
  }
  else
  {
    failedCounter++;
    
    Serial.println("Connection to ThingSpeak Failed ("+String(failedCounter, DEC)+")");   
    Serial.println();
    
    lastConnectionTime = millis(); 
  }
}

void startEthernet()
{
  
  client.stop();

  Serial.println("Connecting Arduino to network...");
  Serial.println();  

  delay(1000);
  
  // Connect to network amd obtain an IP address using DHCP
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("DHCP Failed, reset Arduino to try again");
    Serial.println();
  }
  else
  {
    Serial.println("Arduino connected to network using DHCP");
    Serial.println();
  }
  
  delay(1000);
}
