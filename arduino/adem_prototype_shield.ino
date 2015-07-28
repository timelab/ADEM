/*

 Interface to Interface to Shinyei Model PPD42NS and Samyoung DSM501A Particle Sensors
 with GPS and logging to SD
 
 Based on code by Christopher Nafis April 2012
 Modified by David Holstius May 2012
 DSM501A pinout and interrupt by Lieven Blancke April 2014
 
 http://www.seeedstudio.com/depot/grove-dust-sensor-p-1050.html
 http://www.sca-shinyei.com/pdf/PPD42NS.pdf
 http://www.samyoungsnc.com/eng/m3/down.php?type=1&no=21 (pdf)
 
 For the PPD42NS:
  JST Pin 1 (Black wire)  => Arduino GND
  JST Pin 3 (Red wire)    => Arduino 5VDC
  JST Pin 4 (Yellow wire) => Arduino Leonardo: Digital Pin 3 / Arduino Uno,Ethernet,Mega: Digital Pin 2
 For the Samyoung DSM201A:
  JST Pin 5 (Orange wire) => Arduino GND
  JST Pin 3 (White wire)  => Arduino 5VDC
  JST Pin 2 (Red wire)    => Arduino Leonardo: Digital Pin 3 / Arduino Uno,Ethernet,Mega: Digital Pin 2
 For the Sharp GPY1010
  JST PIN 1 (White wire)  => VLED: voeden met ontkoppeling door 150ohm en 220µF uit Vcc)
  JST PIN 2 (Blue wire)   => GND (GND for LED)
  JST PIN 3 (Green wire)  => Arduino Leonardo: Digital Pin 4 (LEDin)
  JST PIN 4 (Yellow wire) => GND (GND signal)
  JST PIN 5 (Black wire)  => Arduino Leonardo: Analog Pin 0
  JST PIN 6 (Red wire)    => Arduino 5VDC
 
 2014-04-22: Added SD logging
  5V      => Arduino Leonardo ICSP Pin 2
  3V3     => Not Connected
  CS      => Arduino Leonardo Digital Pin 10 (you could change this to 4)
  MOSI    => Arduino Leonardo ICSP Pin 4
  MISO    => Arduino Leonardo ICSP Pin 1
  SCK/CLK => Arduino Leonardo ICSP Pin 3
  GND     => Arduino Leonardo ICSP Pin 6
  
  2014-04-25: Added GPS
   TX     => Arduino Leonardo Digital Pin 0 (RX)
   RX     => Arduino Leonardo Digital Pin 1 (TX)
   
  TO DO:
   - DS1820 temperature sensor
   - PWM drive the exhaust 40mm fan to a fixed speed
   - vergeet niet op te vangen dat micros() elke 70 minuten overloopt! -> reset elke vol uur
   - brownout detectie (sluit file af voor stroom uit gaat)
   - file naam = y.csv:
       a: year: A=2014, B=2015...Z= (ascii value year (4 chars) -1949)
       m: month: 
  */

// include the SD library:
#include <SPI.h>
#include <SD.h>

// serial to PC for debugging
#define USB_BAUD        9600
// serial to GPS
#define GPS_BAUD        115200
// Define pins
#define Samyoung_intPin    3         // Digital input pin used for the Samyoung DSM501A dust sensor (need hardware interrupt)
#define Samyoung_Int       0         // Hardware interrupt bound to the above pin
#define Samyoung_debugLED  13        // digital pin used to indicate dust detected interrupt
#define SD_CSpin           5         // SD Card CS (SPI Chip Select)
#define Sharp_dustPin      0         // Analog input pin connected to pin 5 on the sensor the Sharp GP2Y1010AU0F dust sensor
#define Sharp_ledEnable    4         // Digital output pin connected to Pin 3 on the Sharp GP2Y1010AU0F dust sensor
// sampling
#define T_INTERVAL    5000         // sample duration (60000ms for 1 minute stationary application)
// how long are max NMEA lines to parse?
#define GPS_MAXLINELENGTH 120
#define SD_filename     "datalog3.txt"

unsigned long Samyoung_t0;                // time of last serial dump
volatile unsigned long Samyoung_t1;       // time of LOW start
unsigned long T_elapsed;           // time since last serial dump (ms)
volatile unsigned long Samyoung_acc;      // sum of time spent LOW
unsigned long Samyoung_raw;               // fraction of time spent LOW

// needed for GPS string reception
char GPS_c;
char GPS_line1[GPS_MAXLINELENGTH] = "";     // data from GPS
char GPS_line2[GPS_MAXLINELENGTH] = "";     // full data line from GPS
volatile char *GPS_currentline;
volatile char *GPS_lastline;
int GPS_lineidx = 0;
bool GPS_recvdflag = false;

// output
String dataString = "";

// Sharp sensor
unsigned int Sharp_dustVal=0;
int Sharp_delayTime=280;//280ms
int Sharp_delayTime2=40;//was 40ms
unsigned int Sharp_offTime=9680;
unsigned long Sharp_dustValSum=0;
unsigned int Sharp_dustValCount=0;
unsigned long Sharp_dustValAvg;

void setup() {
  // initialization for DSM501A
  attachInterrupt(Samyoung_Int, Samyoung_int, CHANGE);
  pinMode(Samyoung_intPin, INPUT);
  pinMode(Samyoung_debugLED,OUTPUT); // we use an LED to monitor dust sensor interrupts
  Samyoung_acc = 0;
  Samyoung_t0 = millis();
  // initialization for Sharp
  pinMode(Sharp_ledEnable,OUTPUT);
  
  GPS_currentline = GPS_line1;
  GPS_lastline    = GPS_line2;
  
  Serial1.begin(GPS_BAUD);
  delay (5000);
  Serial1.print("$PMTK220,1000*1F\r\n"); // Send NMEA command to change update rate to 1Hz
  Serial1.print("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); // Send NMEA command to only enable RMC status

  Serial.begin(USB_BAUD);
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_CSpin, OUTPUT);
  // add this if running Arduino version earlier than 1.5.6-r2
  // https://code.google.com/p/arduino/issues/detail?id=931
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CSpin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop () {
  // read char from GPS
  if (Serial1.available()) {
    GPS_c = Serial1.read();
    if (GPS_c=='$') {
      // the start of the line
      GPS_currentline[GPS_lineidx] = 0; // zero-terminate the empty string
      GPS_lineidx = 0;
    }
    if (GPS_c == '\n') {
      GPS_currentline[GPS_lineidx] = 0; // zero-terminate the fully received GPS NMEA string
      if (GPS_currentline == GPS_line1) {
        GPS_currentline = GPS_line2;
        GPS_lastline = GPS_line1;
      } else {
        GPS_currentline = GPS_line1;
        GPS_lastline = GPS_line2;
      }
      GPS_lineidx = 0;
      GPS_recvdflag = true;
    }
    GPS_currentline[GPS_lineidx++] = GPS_c;
    if (GPS_lineidx >= GPS_MAXLINELENGTH) GPS_lineidx = GPS_MAXLINELENGTH-1;
  }
  
  // do a Sharp GP2Y1010AU0 measurement
  digitalWrite(Sharp_ledEnable,LOW); // power on the LED
  delayMicroseconds(280); //280µs
  Sharp_dustVal=analogRead(Sharp_dustPin); // read the dust value
  // the analog read takes about 120µs to execute
  delayMicroseconds(40); //40µs
  digitalWrite(Sharp_ledEnable,HIGH); // turn the LED off
  delayMicroseconds(9680-120); //9680µs
  // sum it up
  Sharp_dustValSum+=Sharp_dustVal;
  Sharp_dustValCount++;

  T_elapsed = millis() - Samyoung_t0;
  if ((T_elapsed > T_INTERVAL) && GPS_recvdflag) {
    // calculate Samyoung percentage low time
    Samyoung_raw = Samyoung_acc / T_elapsed;  // expressed as percentage * 10
    Samyoung_acc = 0;
    Samyoung_t0 = millis();

    // Calculate Sharp average
    Sharp_dustValAvg=5000*Sharp_dustValSum/Sharp_dustValCount/1024; // expressed in mV (5V=5000mV=1024 analog read)
    Sharp_dustValSum=0;
    Sharp_dustValCount=0;

    dataString = String(Sharp_dustValAvg) + "," + String(Samyoung_raw) + "," + (char *)GPS_lastline;
    Serial.println(dataString);
 
    File dataFile = SD.open(SD_filename, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }  
    // if the file isn't open, pop up an error:
    else {
      Serial.print("error opening ");
      Serial.println(SD_filename);
    } 
    dataString = "";
    GPS_recvdflag = false;
  }
}

void Samyoung_int()
{
  // the sensor pin goes low when dust detected
  // the amount of low time is proportional to the total amount of dust
  // whe measure the time the pin is low and accumulate it
  if (digitalRead(Samyoung_intPin) == LOW)
  { // note start of the LOW period
    Samyoung_t1=micros();
    digitalWrite(Samyoung_debugLED,HIGH);
  }
  else
  { // end of the LOW period
    Samyoung_acc+=micros() - Samyoung_t1;
    digitalWrite(Samyoung_debugLED,LOW);
  }
  // vergeet niet op te vangen dat micros() elke 70 minuten overloopt!
}

