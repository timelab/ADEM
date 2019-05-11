/*
 * This file is part of the ADEM project.
 *
 * ADEM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, 
 * (at your option) any later version.
 *
 * ADEM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ADEM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 Dag Wieers, Lieven Blancke, Koen Verstringe
 *
 */

//#define DEMO 1
 
#include <TickerSchedlr.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <accelero_MPU6050.h>
#include <barometer_BMP085.h>
#include <battery_lipo.h>
#include <buzzer_passive.h>
//#include <gps_SwSerial.h>
#include <gps_I2C.h>
#include <humidity_HTU21D.h>
#include <led_NeoPixel.h>
#include <particulate_PPD42.h>
#include <wifi_WiFiManager.h>
#include "store_and_forward.h"


#define PM1_PIN 12
#define PM25_PIN 13	// The PM25 we use in naming everywhere actually means PM2.5!
#define GPS_TX_PIN 16
#define GPS_RX_PIN 4
#define GPS_BAUD 9600
#define NEOPIXEL_PIN 0
#define SERIAL_BAUD 74880

// defines for debugging purposes (un)comment where needed
// #define TELNET
#define DEBUG
#define set_breakmode 1
#undef TELNET

#ifdef TELNET

#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
extern void TelnetSetup();

RemoteDebug Telnet;

// Host name

#define HOST_NAME "rem-debug" // PLEASE CHANGE IT

// Time

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;
#endif

#ifdef DEBUG
  #ifdef TELNET
    #define __LOG(msg) Telnet.print(msg)
    #define __LOGLN(msg) Telnet.println(msg)
    #define __LOGHEX(msg) Telnet.print("0x");Telnet.print(msg,HEX);Telnet.print(" ")
  #else
    #define __LOG(msg) Serial.print(msg)
    #define __LOGLN(msg) Serial.println(msg)
    #define __LOGHEX(msg) Serial.print("0x");Serial.print(msg,HEX);Serial.print(" ")
  #endif
#else
#define __LOG(msg)
#define __LOGLN(msg)
#define __LOGHEX(msg) 
#endif

// Different states of the program
enum state_t {
  STATE_START,
  STATE_DEMO,
  STATE_SLEEP,
  STATE_CONFIG,
  STATE_GPSTEST,
  STATE_COLLECT,
  STATE_WIFITEST,
  STATE_UPLOAD,
  STATE_RESET,
  STATE_LOW_POWER,
};
char *states[] {
  "START",
  "DEMO",
  "SLEEP",
  "CONFIG",
  "GPSTEST",
  "COLLECT",
  "WIFITEST",
  "UPLOAD",
  "RESET",
  "LOW POWER",
};
state_t prev_state = STATE_START;
state_t state = STATE_START;
bool wififail = false;
bool demoflag = false;
uint16_t sensors_detected = 0;
uint16_t sensors_config = -1;

/*#ifdef DEMO
const char *mdnsname = "adem";  // Used by mDNS. Try http://adem.local/
DNSServer dnsserver;
ESP8266WebServer webserver(80);
IPAddress apIP(192, 168, 4, 1);
state_t next_state = STATE_DEMO;
#else // DEMO*/
state_t next_state = STATE_START;
//#endif // DEMO

// DEBUG input
class Debug {
public:
  boolean gpsready = false;
  boolean moving   = false;
  boolean shaken   = false;
  boolean wifi     = false;
#ifdef set_breakmode
  boolean breakmode = true ;
#else
  boolean breakmode = false;
#endif  
  boolean breakcont = false;
} debug;

volatile bool interrupt_flag;

storeAndForwardBuf buffer(10000);

// Objects for all the sensor libraries
MPU6050Sensor accelerometer;
BMP085Sensor barometer;
LipoBattery battery;
PassiveBuzzer buzzer;
//SwSerialGPS gps = SwSerialGPS(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
I2CGps gps = I2CGps();
HTU21DFSensor humidity;
NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PPD42Sensor particulate(PM1_PIN, PM25_PIN);
char SSID[20];
const int SCHED_MAX_TASKS = 100;
TickerSchedlr *schedule = NULL;
WiFiManagerWifi wifimngr;
myWiFiManagerParameter * custom_config;
myWiFiManagerParameter * demo;
myWiFiManagerParameter * sensors_parm;

String page = "";

// FIXME: Since brightness does not seem to work, use "darker" colors
// Colors
uint32_t black = led.Color(0, 0, 0);
uint32_t white = led.Color(127, 127, 127);
uint32_t red = led.Color(127, 0, 0);
uint32_t orange = led.Color(127, 63, 0);
uint32_t yellow = led.Color(127, 127, 0);
uint32_t green = led.Color(0, 127, 0);
uint32_t light_blue = led.Color(0, 127, 127);
uint32_t blue = led.Color(0, 0, 127);
uint32_t purple = led.Color(63, 0, 127);

uint32_t _lst_msg = 0;
uint32_t ts =0;
uint32_t i2c_ts = 0;
uint32_t i2c_status = 0;

unsigned char twi_sda, twi_scl;

uint32_t colors[] = {
  red,
  light_blue,
  black,
  yellow,
  orange,
  green,
  purple,
  blue,
  white,
  black
};

uint16_t led_state = 0;

// Tasks
TickerTask *accelerometer_task = NULL;
TickerTask *barometer_task = NULL;
TickerTask *battery_task = NULL;
TickerTask *gps_task = NULL;
TickerTask *humidity_task = NULL;
TickerTask *particulate_task = NULL;
TickerTask *upload_task = NULL;

// TASKS
void accelerometer_run(void *) {
//  __LOGLN(":accelerometer: -> Check moving/shaken");
  //accelerometer.read();
  //Serial.println(accelerometer.report());

  // process normal accel data in logging.
  // MPU will store data internally in its FIFO so we have to loop
  while (accelerometer.hasData()) {
    accelerometer.read();
    buffer.write((char *)accelerometer.dataToBuffer(), accelerometer.dataBufferSize());
    yield(); // we can have lots of data stpp WDT crashes
   __LOGLN(accelerometer.report());
  }
}

void barometer_run(void *) {
  barometer.read();
  buffer.write((char *)barometer.dataToBuffer(), barometer.dataBufferSize());
  __LOGLN(barometer.report());
}

void battery_run(void *) {
  battery.read();
  buffer.write((char *)battery.dataToBuffer(), battery.dataBufferSize());
//  __LOGLN(battery.report());
}

void gps_run(void *) {
  gps.read();
  if (gps.ready) {
    buffer.write((char *)gps.dataToBuffer(), gps.dataBufferSize());
    __LOGLN(gps.report());
  }
}

void humidity_run(void *) {
  humidity.read();
  buffer.write((char *)humidity.dataToBuffer(), humidity.dataBufferSize());
  __LOGLN(humidity.report());
}

void particulate_run(void *) {
  particulate.read();
  buffer.write((char *)particulate.dataToBuffer(), particulate.dataBufferSize());
  __LOGLN(particulate.report());
}

void upload_run(void *){
  // here we have to peek the sensor ID and nack the peek
  // depending on the sensor ID we peek the buffer for databufferSize bytes and send the off to the sensor::bufferReport
  // which returns the json string
  // then we send off this data to the server over the WIFI
  // when succesfully posted we can acknowledge the buffer and move on to the next buffered item
  // by running this as an idle task it will almost constantly process the buffer as long as there is buffered data
  // for debug purposes we could log the WIFI postings
  // if done sending data we suspend this task. The state machine will resume the task when needed

  char lbuf[40];
  int lID;
  int s = 0;
  String strReport;
  Sensor * pSens = NULL;
  
  if (not buffer.empty()){
    __LOG("buffered report ");
    lID = buffer.peek();
    __LOG(" for sensor ID : ");__LOG(lID);
    switch (lID){
      case BAROMETER_BMP085:
        __LOGLN(" BAROMETER");
        pSens = &barometer;
        break;
      case HUMIDITY_HTU21D:
        __LOGLN(" HUMIDITY");
        pSens = &humidity;
        break;
      case PARTICULATE_PPD42:
        __LOGLN(" PPD42");
        pSens = &particulate;
        break;
      case GPS_SWSERIAL:
        __LOGLN(" GPS SERIAL");
        pSens = &gps;
        break;
      case GPS_I2C:
        __LOGLN(" GPS I2C");
        pSens = &gps;
        break;
      case ACCELERO_MPU6050:
        __LOGLN(" ACCELEROMETER");
        pSens = &accelerometer;
        break;
      default :
        return;
/// TODO : what if ID is not recognized ? How to recover from corrupt data ?
    }
    buffer.ack();
    
//    __LOG("get buffer for sensor for nrbytes = ");__LOGLN(pSens->dataBufferSize());
    s = buffer.peek(&lbuf[0], pSens->dataBufferSize());
    if (s > 0 ) {
 /*     __LOGLN("");
      __LOG(lbuf[0]);
      for (int i = 1; i < s;i++){
        __LOG(lbuf[i]); __LOG(",");
      }
      __LOGLN("");*/
      strReport = pSens->bufferedReport((uint8_t *)&lbuf[0]);
      __LOG(strReport);
    }  
    
    __LOGLN("#");
    // send off strReport to datastore
    // if ACK of storage then ack buffer
    boolean success = true;
    if (success){
        __LOGLN("acknowledge buffer");
        buffer.ack();
    }
    else {
        __LOGLN("nack buffer");
        buffer.nack();
    }
  }
  else{
    yield();
  }

}

void demo_run(void *){

}

// STATES
void start_state() {
  // TODO
  // I2C scanner based on address we call the proper sensor begin
  // address should be identical to the device ID in the sensorIDs.h file
  scan_I2C();
  //read config
  //// TODO read sensor config from json file
  sensors_config = -1;
  battery_task = TickerTask::createPeriodic(&battery_run, 300000);
  battery_task->name = "battery";
  next_state = STATE_SLEEP;
}

void demo_state() {
  delay(50);
//  dnsserver.processNextRequest();
//  webserver.handleClient();
}

void sleep_state() {
  delay(500);

  if (sensors_detected != sensors_config){
    next_state = STATE_CONFIG;
  }
  else
  if (accelerometer.isMoving() or debug.moving) {
    next_state = STATE_GPSTEST;
    if (accelerometer.isMoving()) __LOGLN("accelerometer.isMoving(), next_state = STATE_GPSTEST");
    if (debug.moving) __LOGLN("debug.moving, next_state = STATE_GPSTEST");
  } 
  else {
    if (not buffer.empty() and not wififail) {
      next_state = STATE_WIFITEST;
    } 
    else if (accelerometer.isShaken() or debug.shaken) {
      next_state = STATE_CONFIG;
    }
  }
}

void config_state() {

  // Temporarily use debug.shaken to go in and out config state
  //if (finished or timeout or canceled) {
  if (not debug.shaken) {
    next_state = STATE_SLEEP;
  }
  if (demoflag) {
    next_state = STATE_DEMO;
  }
}

void gpstest_state() {

  if (accelerometer.isMoving() or debug.moving) {
    gps.read();
    if (gps.ready or debug.gpsready) {
      next_state = STATE_COLLECT;
      if (gps.ready) __LOGLN("gps.ready, next_state = STATE_COLLECT");
      if (debug.gpsready) __LOGLN("debug.gpsready, next_state = STATE_COLLECT");
    }
    else
      delay(500); // to overcome quick looping and WDT timeoutc
  } 
  else {
    next_state = STATE_SLEEP;
  }
}

void collect_state() {

  // Sensor tasks should be reporting on their own
  //LOGLN("Collecting...");

  if ( not (accelerometer.isMoving() or debug.moving) or not (gps.ready or debug.gpsready) ) {
    next_state = STATE_GPSTEST;
  }
}

void wifitest_state() {
  __LOG("in wifi test ");
  //ETS_GPIO_INTR_DISABLE();
  
    wifimngr.start_client();
  __LOG("wifi client started ");
  // ETS_GPIO_INTR_ENABLE();
  if (wifimngr.connected or debug.wifi) {
    next_state = STATE_UPLOAD;
  }
  else
  {
    wififail = true;
  }

  if (accelerometer.isMoving() or debug.moving or buffer.empty() or wififail) {
    next_state = STATE_SLEEP;
  }
}

void upload_state() {

  // Upload action finishes successfully or times out
  // Create JSON of X records
  // Send to server
  // Empty datastore

  if (buffer.empty()) {
    next_state = STATE_LOW_POWER;
  }
}

void lowpower_state() {

  // Upload action finishes successfully or times out
  // Create JSON of X records
  // Send to server
  // Empty datastore

  if ( accelerometer.isShaken()) {
    next_state = STATE_SLEEP;
  }
}

void reset_state() {
  ESP.reset();
}


// TRANSITIONS
void config_to_demo() {
  barometer.begin();
  barometer_task = TickerTask::createPeriodic(&barometer_run, 15000);
  barometer_task->name = "barometer";
  humidity.begin();
  humidity_task = TickerTask::createPeriodic(&humidity_run, 15000);
  humidity_task->name = "humidity";
  particulate.begin();
  particulate_task = TickerTask::createPeriodic(&particulate_run, 60000);
  particulate_task->name = "particulate";

/*  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(SSID);

  Serial.print("Initializing dns server...");
  dnsserver.setErrorReplyCode(DNSReplyCode::NoError);
  dnsserver.start(53, "*", apIP);
  Serial.println(" OK");

  Serial.print("Initializing mdns responder...");
  MDNS.begin(mdnsname);
  MDNS.addService("http", "tcp", 80);
  Serial.println(" OK");

  Serial.print("Initializing web server...");
  wifimngr.begin();
  /*wifimngr.start_ap(SSID);
  //webserver.on("/", website);
  wifimngr.registerExternalPage("/demo", &website);
  //webserver.on("/sensors.json", sensors);
  wifimngr.registerExternalPage("/sensors.json", &sensors);
  */
  Serial.println(" OK");

  Serial.println();
  Serial.print("=> Please connect to SSID: ");
  Serial.println(SSID);
  Serial.println("=> And surf to: http://adem.local/");
  Serial.println();
}

void demo_to_config(){
  demoflag = false;
}

void start_to_sleep() {
//  buzzer.start_sound();
  accelerometer.begin();
  accelerometer_task = TickerTask::createPeriodic(&accelerometer_run, 1000);
  accelerometer_task->name = "accelerometer";

  wifimngr.sleep();

  __LOGLN("Device entered sleep state, waiting for action !");
}

void sleep_to_config() {

//  buzzer.config_sound();

  __LOG("Starting WiFi in AP mode using SSID "); __LOG(SSID); __LOG("... ");
  wifimngr.begin();
  page += "<table>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<0 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='1'/>";
  page += "</td><td>Gps</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<1 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='2'/>";
  page += "</td><td>barometer</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<2 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='4'/>";
  page += "</td><td>accelero</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<3 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='8'/>";
  page += "</td><td>humidity</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<4 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='16'/>";
  page += "</td><td>neopixel</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<5 & sensors_config) != 0 ? "checked": "unchecked";
  page += " value='32'/>";
  page += "</td><td>buzzer </td></tr>";
  page += "</table>";

  const char * ptr = page.c_str();

  custom_config = new myWiFiManagerParameter( "config","config","0",2,ptr );

  //Local intialization. Once its business is done, there is no need to keep it around

  //set config save notify callback
  wifimngr.setSaveConfigCallback(saveConfigCallback);

  demo = new myWiFiManagerParameter("Demo","DEMO", &SetupDemoPage);
  sensors_parm = new myWiFiManagerParameter("sensors.json", &sensors);
  //add all your parameters here
  wifimngr.addParameter(custom_config);
  wifimngr.addParameter(demo);
  wifimngr.addParameter(sensors_parm);

  wifimngr.start_ap(SSID);

  // Disable for now, it's our temporary switch to go in and out of config
  debug.shaken = false;
}

void sleep_to_lowpower(){
  // bring all sensors to low power state.
  // wake up is done by interrupt handling

}

void lowpower_to_sleep() {
  // should not happen. 
  // wake up will bring us in start state ????!!!!

}

void config_to_sleep() {

  // Suspend wifiap task
///  wifimngr.sleep();

  __LOGLN("Device entered sleep state, waiting for action !");
}

void sleep_to_gpstest() {

  // Resume gps task
  gps.begin();
  gps_task = TickerTask::createPeriodic(&gps_run, 1000);
  gps_task->name = "gps";
  wififail = false; // clear wifi timeout failure
}

void gpstest_to_sleep() {

  // Suspend gps task
  gps_task->clear();
  gps.end();
  __LOGLN("Device entered sleep state, waiting for action !");
}

void gpstest_to_collect() {

//  buzzer.collect_sound();

  // Resume sensor tasks
  barometer.begin();
  barometer_task = TickerTask::createPeriodic(&barometer_run, 10000);
  barometer_task->name = "barometer";
  humidity.begin();
  humidity_task = TickerTask::createPeriodic(&humidity_run, 10000);
  humidity_task->name = "humidity";
//  particulate.begin();
//  particulate_task = TickerTask::createPeriodic(&particulate_run, 10000);
//  particulate_task->name = "particulate";
}

void collect_to_gpstest() {

  // Suspend sensor tasks
  barometer_task->clear();
  barometer.end();
  humidity_task->clear();
  humidity.end();
//  particulate_task->clear();
//  particulate.end();
}

void sleep_to_wifitest() {

  // Resume wificlient tas
  // disable the accelero interrupts while connecting to wifi
  
}

void wifitest_to_sleep() {

  // Suspend wificlient task
///  wifimngr.sleep();

  __LOGLN("Device entered sleep state, waiting for action !");
}

void wifitest_to_upload() {
  // TODO KOV enable accelero interrupts
  
  if (not upload_task) {
    upload_task = TickerTask::createIdle(&upload_run);
  }
  upload_task->resume();
}

void upload_to_wifitest() {
  upload_task->suspend();
}

void debug_help() {
  __LOGLN();
  __LOGLN("=> Press \"g\" for gpsfix, \"m\" for moving, \"r\" to restart,");
  __LOGLN("=>       \"s\" to shake, \"w\" for wifi and \"h\" for help.");
  __LOGLN("=>       \"b\" to break mode, \"c\" for continue in break mode.");
  __LOGLN();
}

void scan_I2C(){
   int error = 0;
  int nDevices = 0;
  for(int address = 0;address < 127 ; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      __LOG("I2C - ");
      __LOGHEX(address);
      switch (address){
      case 0x20:
        sensors_detected |= 1 << GPS_I2C; 
        __LOGLN(" GPS I2C");
        break;
      case 0x40:
        sensors_detected |= 1 << HUMIDITY_HTU21D; 
        __LOGLN(" HUMIDITY");
        break;
      case 0x50:
        sensors_detected |= 1 << BUFFER_MEMORY; 
        __LOGLN(" I2C MEMORY");
        break;
      case 0x68:
       sensors_detected |= 1 << ACCELERO_MPU6050; 
        __LOGLN(" ACCELEROMETER");
        break;
      case 0x77:
        sensors_detected |= 1 << BAROMETER_BMP085; 
        __LOGLN(" BAROMETER");
        break;
      default :
        __LOGLN(" ????? Unknown device");
      }
      nDevices++;
    }
    else if (error==4) {
      __LOG("ERROR on address : ");
      __LOGHEX(address);__LOGLN();
    } 
  }
  if (nDevices == 0) {
    __LOGLN("No I2C devices found");
  }
  else {
    __LOG("I2C scan signature : "); __LOGHEX(sensors_detected);__LOGLN("");
    __LOGLN("done");
  }
}

void check_I2C(){
// I2C status check and reset if needed
  int tmp = Wire.status();
  if  (tmp != I2C_OK){
    i2c_status = tmp;
  }
  else{
      i2c_ts = ts;
  }
  if ((ts - i2c_ts) > 1000) { // no I2C_OK status for more than 1 sec
    switch (Wire.status()){
      case  I2C_SCL_HELD_LOW:
        __LOG("I2C ::::: SCL :::: CLOCK HELD LOW");
        break;
      case I2C_SCL_HELD_LOW_AFTER_READ:
        __LOG("I2C ::::: SCL :::: CLOCK HELD LOW AFTER READ");
        break;
      case I2C_SDA_HELD_LOW:
        __LOG("I2C ::::: SDA :::: DATA HELD LOW");
        break;
      case I2C_SDA_HELD_LOW_AFTER_INIT:
        __LOG("I2C ::::: SDA :::: DATA HELD LOW AFTER INIT");
        break;
      case I2C_OK:
        i2c_ts = ts;
        break;
    }
    Serial.println("Starting I2C bus recovery");
    twi_sda = SDA; // comming from pins_arduino.h
    twi_scl = SCL; // comming from pins_arduino.h

    //try i2c bus recovery at 100kHz = 5uS high, 5uS low
    pinMode(twi_sda, OUTPUT);//keeping SDA high during recovery
    digitalWrite(twi_sda, HIGH);
    pinMode(twi_scl, OUTPUT);
    for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
      digitalWrite(twi_scl, HIGH);
      delayMicroseconds(5);
      digitalWrite(twi_scl, LOW);
      delayMicroseconds(5);
    }

    //a STOP signal (SDA from low to high while CLK is high)
    digitalWrite(twi_sda, LOW);
    delayMicroseconds(5);
    digitalWrite(twi_scl, HIGH);
    delayMicroseconds(2);
    digitalWrite(twi_sda, HIGH);
    delayMicroseconds(2);
    //bus status is now : FREE

    Serial.println("bus recovery done, starting scan in 2 secs");
    //return to power up mode
    twi_stop();
    delay(2000);
    //pins + begin advised in https://github.com/esp8266/Arduino/issues/452
    Wire.begin();
    switch (Wire.status()){
      case  I2C_SCL_HELD_LOW:
        __LOG("I2C ::::: SCL :::: CLOCK HELD LOW");
        break;
      case I2C_SCL_HELD_LOW_AFTER_READ:
        __LOG("I2C ::::: SCL :::: CLOCK HELD LOW AFTER READ");
        break;
      case I2C_SDA_HELD_LOW:
        __LOG("I2C ::::: SDA :::: DATA HELD LOW");
        break;
      case I2C_SDA_HELD_LOW_AFTER_INIT:
        __LOG("I2C ::::: SDA :::: DATA HELD LOW AFTER INIT");
        break;
      case I2C_OK:
        i2c_ts = ts;
        break;
    }
    __LOGLN("");
  }
// I2C status check
}

void handle_debug_cmd(char c){
  if (c != '\n' and c != '\r') {
      __LOG("Key "); __LOG(c); __LOG(" is pressed. ");
      switch (c) {
        case 'g':
          debug.gpsready = not debug.gpsready;
          __LOG("debug.gpsready is "); __LOGLN(debug.gpsready?"on.":"off.");
          break;
        case 'm':
          debug.moving = not debug.moving;
          __LOG("debug.moving is "); __LOGLN(debug.moving?"on.":"off.");
          break;
        case 'r':
          __LOGLN("Restarting system.");
          next_state = STATE_RESET;
          break;
        case 's':
          debug.shaken = not debug.shaken;
          __LOG("debug.shaken is "); __LOGLN(debug.shaken?"on.":"off.");
          break;
        case 'w':
          debug.wifi = not debug.wifi;
          __LOG("debug.wifi is "); __LOGLN(debug.wifi?"on.":"off.");
          break;
        case 'b':
            debug.breakmode = not debug.breakmode;
            debug.breakcont = false;
            __LOG("debug.break is "); __LOGLN(debug.wifi?"on.":"off. <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            break;
        case 'c':
            if (debug.breakmode ){
              debug.breakcont = true;
            }
            __LOGLN("debug.breakcontinue set ");
            break;
          case 'e':
            __LOGLN("Trying to connect to wifi access point");
            yield();
            ETS_GPIO_INTR_DISABLE();
            delay(1000);
///            wifimngr.start_client();
            ETS_GPIO_INTR_ENABLE();
            break; 
        case 'd':
            __LOG(" >>> >>> >>> current state is : "); __LOG(states[state]);
            break;
        case '0' : // STATE_START
                  state = STATE_START;next_state = STATE_START;
                  break;
        case '1' : //STATE_DEMO,
                  state = STATE_START;next_state = STATE_START;
                  break;
        case '2' : //STATE_SLEEP,
                  state = STATE_SLEEP;next_state = STATE_SLEEP;
                  break;
        case '3' : //STATE_CONFIG,
                  state = STATE_SLEEP;next_state = STATE_CONFIG;
                  break;
        case '4' : //STATE_GPSTEST,
                  state = STATE_GPSTEST;next_state = STATE_GPSTEST;
                  break;
        case '5' : //STATE_COLLECT,
                  state = STATE_COLLECT;next_state = STATE_COLLECT;
                  break;
        case '6' : //STATE_WIFITEST,
                  state = STATE_WIFITEST;next_state = STATE_WIFITEST;
                  break;
        case '7' : //STATE_UPLOAD,
                  state = STATE_UPLOAD;next_state = STATE_UPLOAD;
                  break;
        case '8' : //STATE_RESET,
                  state = STATE_RESET;next_state = STATE_RESET;
                  break;
        case 'h':
        case '?':
          __LOG(" >>> >>> >>> current state is : "); __LOG(states[state]);
          debug_help();
          break;
        default:
          __LOGLN("No action.");
      }
  }
}

void setup() {
  
  state = STATE_START;
  Wire.begin();
  Wire.setClock(100000);
  Wire.setClockStretchLimit(50000);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  delay(1000);
  Serial.println("Serial communication... OK");
#ifdef TELNET  
  TelnetSetup();
#endif

  __LOGLN();
  __LOGLN("Setup started in DEBUG mode.");
  debug_help();

  led.begin();
  led.setcolor(led_state, colors[state]);

//  buzzer.begin();
  if (buffer.full()) {
    Serial.println("ERROR no memory to buffer data");
  }

  Serial.print("Initializing scheduler... ");
  schedule = TickerSchedlr::Instance(SCHED_MAX_TASKS);
  Serial.println("OK");

  sprintf(SSID, "ADEM-%d", ESP.getChipId());
}

// Main loop takes care of state and transition management
void loop() {
  ESP.wdtFeed();
  ts = millis();

  check_I2C();

  if( state == next_state){
    switch(state) {
      case STATE_START:     start_state(); break;
      case STATE_DEMO:      demo_state(); break;
      case STATE_SLEEP:     sleep_state(); break;
      case STATE_CONFIG:    config_state(); break;
      case STATE_GPSTEST:   gpstest_state(); break;
      case STATE_COLLECT:   collect_state(); break;
      case STATE_WIFITEST:  wifitest_state(); break;
      case STATE_UPLOAD:    upload_state(); break;
      case STATE_LOW_POWER: lowpower_state(); break;
      case STATE_RESET:     reset_state(); break;
      default:              reset_state(); break;
    }
  }
  
  if (state != next_state)  {
    if (ts - _lst_msg > 1000){
      _lst_msg = ts;
      __LOG("Transition from "); __LOG(states[state]); __LOG(" to "); __LOG(states[next_state]);
      if( debug.breakmode && not debug.breakcont) {
        __LOGLN(" >>>wait for 'c' = continue flag set<<<");
      }
    }
    if( not debug.breakmode || debug.breakcont) {
    //{  
      _lst_msg = 0;
      debug.breakcont = false;  // clear continue flag in break mode
      __LOG(" >>> >>> >>> moving to next state : "); __LOG(states[state]);__LOG("  -->  ");__LOG(states[next_state]); __LOGLN("<<< <<< <<<");
      switch(state) {

        case STATE_START:
          switch(next_state) {
            case STATE_SLEEP:     start_to_sleep(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_DEMO:
          switch(next_state) {
            case STATE_CONFIG:    demo_to_config(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_SLEEP:
          switch(next_state) {
            case STATE_CONFIG:    sleep_to_config(); break;
            case STATE_GPSTEST:   sleep_to_gpstest(); break;
            case STATE_WIFITEST:  sleep_to_wifitest(); break;
            case STATE_LOW_POWER: sleep_to_lowpower(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_CONFIG:
          switch(next_state) {
            case STATE_SLEEP:     config_to_sleep(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_GPSTEST:
          switch(next_state) {
            case STATE_SLEEP:     gpstest_to_sleep(); break;
            case STATE_COLLECT:   gpstest_to_collect(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_COLLECT:
          switch(next_state) {
            case STATE_GPSTEST:   collect_to_gpstest(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_WIFITEST:
          switch(next_state) {
            case STATE_SLEEP:     wifitest_to_sleep(); break;
            case STATE_UPLOAD:    wifitest_to_upload(); break;
            default:              next_state = STATE_RESET;
          }; break;

        case STATE_UPLOAD:
          switch(next_state) {
            case STATE_WIFITEST:  upload_to_wifitest(); break;
            default:              next_state = STATE_RESET;
          }; break;
          case STATE_LOW_POWER:
          switch(next_state) {
            case STATE_SLEEP:     lowpower_to_sleep(); break;
            default:              next_state = STATE_RESET;
          }; break;
      }

      prev_state = state;
      state = next_state;

      led.setcolor(led_state, colors[next_state]);

//    __LOG("Transition to ");  __LOG(states[next_state]); __LOGLN(" ended.");
    }
  }

  schedule->tick();
  
#ifdef TELNET
    Telnet.handle();
#endif  

#ifdef DEBUG
  if (Serial.available() > 0) {
    char c = Serial.read();
    handle_debug_cmd(c);
  }
#endif // DEBUG
}

//#ifdef DEMO
const char demo_html[] PROGMEM =
"<!DOCTYPE html><html><head><title>ADEM Demo</title>\n"
"<meta charset=\"UTF-8\"><meta name=\"apple-mobile-web-app-capable\" content=\"yes\">\n"
"<script type=\"application/javascript\">\n"
"var density = 1.5 * Math.pow(10, 12);var pi = 3.14159627;var K = 3531.5;var r1 = 0.15 * Math.pow(10, -6);var vol1 = (4.0 / 3.0) * pi * Math.pow(r1, 3);var mass1 = density * vol1;var r25 = 0.44 * Math.pow(10, -6);var vol25 = (4.0 / 3.0) * pi * Math.pow(r25, 3);var mass25 = density * vol25;"
"function PMcount(PPM) {var ratioPPM = PPM / 60000 * 10.0;return 1.1 * Math.pow(ratioPPM, 3) - 3.8 * Math.pow(ratioPPM, 2) + 520 * ratioPPM + 0.62;}"
"function PM1mass(PM1) {return Math.round(PMcount(PM1) * K * mass1 * 100) / 100;}\n"
"function PM25mass(PM25) {return Math.round(PMcount(PM25) * K * mass25 * 100) / 100;}\n"
"function update_all() {var request = new XMLHttpRequest();request.open('GET', '/sensors.json', true);request.setRequestHeader('X-Requested-With', 'XMLHttpRequest');\n"
"request.onload = function () {var json = JSON.parse(this.response);document.getElementById('bar').innerHTML = json['barometer']['Pressure'];document.getElementById('tmp').innerHTML = ((json['barometer']['Temperature'] + json['humidity']['Temperature']) / 2).toFixed(2);\n"
"document.getElementById('hum').innerHTML = json['humidity']['Humidity'];document.getElementById('pm1').innerHTML = (json['particulate']['PM1'] / 100).toFixed(2);document.getElementById('pm2').innerHTML = (json['particulate']['PM2.5'] / 100).toFixed(2);\n"
"document.getElementById('pm1c').innerHTML = (PMcount(json['particulate']['PM1']) / 100).toFixed(2);document.getElementById('pm2c').innerHTML = (PMcount(json['particulate']['PM2.5']) / 100).toFixed(2);document.getElementById('pm1m').innerHTML = (PM1mass(json['particulate']['PM1'] - json['particulate']['PM2.5']) + PM25mass(json['particulate']['PM2.5'])).toFixed(2);\n"
"document.getElementById('pm2m').innerHTML = (PM25mass(json['particulate']['PM2.5'])).toFixed(2);};request.send();};\n"
"setInterval(update_all, 20000);\n"
"</script>\n"
"<style>\n"
"*{ -webkit-box-sizing:border-box; -moz-box-sizing:border-box; box-sizing:border-box }\n"
"html,body { height:100%; margin:0 }\n"
"body { padding-top:20px; font-family:sans-serif; font-weight:700; color:#30a5e7; font-size:28px; line-height:1.5 }\n"
"table.outer { width:100%; height: 100%; }\n"
"td.outer { align: center; vertical-align: middle; }\n"
"table.inner { width:100%; }\n"
"td { display:table-cell; vertical-align:top; width:33%; text-align:right }\n"
"td.col2 { border-bottom:1px solid #30a5e7; padding-right:10px }\n"
"td.col3 { text-align:left; padding-left:10px }\n"
"a.top { border-bottom:1px solid #30a5e7 }\n"
".sub { font-size:.65em }\n"
".top { position:absolute }\n"
"svg.top { top:5%; left:6% }\n"
"a.top { font-size:.75em; color:#30a5e7; bottom:7%; left:50%; -webkit-transform:translateX(-50%); transform:translateX(-50%); text-decoration:none }\n"
"@media(max-height:700px) {\n"
"body { font-size:22px }\n"
"svg.top { top:3%; width:64px }\n"
"}\n"
"@media(max-height:450px) {\n"
".top { display:none }\n"
"}\n"
"@media(max-width:750px) {\n"
"svg.top { left:50%; -webkit-transform:translateX(-50%); transform:translateX(-50%) }\n"
"}\n"
"</style>\n"
"</head>\n"
"<body onload=\"update_all();\"><a href=\"http://ik-adem.be/\" target=\"_blank\" class=\"top\">ik-adem.be</a><table class=\"outer\"><tr><td class=\"outer\"><table class=\"inner\">"
"<tbody><tr><td rowspan=\"2\">&#128684;</td><td class=\"col2\" id=\"pm1\">N/A</div></td> <td class=\"col3\">PM<span class=\"sub\">1.0</span></td></tr><tr><td class=\"col2\" id=\"pm1c\">N/A</div></td><td class=\"col3\">pc/.01ft³</td></tr><tr><td>&#128168;</td><td class=\"col2\" id=\"pm1m\">N/A</div></td><td class=\"col3\">µg/m³</td></tr>\n"
"<tr><td rowspan=\"2\">&#127981;</td><td class=\"col2\" id=\"pm2\">N/A</div></td> <td class=\"col3\">PM<span class=\"sub\">2.5</span></td></tr><tr><td class=\"col2\" id=\"pm2c\">N/A</div></td><td class=\"col3\">pc/.01ft³</td></tr><tr><td>&#128168;</td><td class=\"col2\" id=\"pm2m\">N/A</div></td><td class=\"col3\">µg/m³</td></tr>\n"
"<tr><td>☀️</td><td class=\"col2\" id=\"tmp\">N/A</div></td> <td class=\"col3\">°C</td></tr><tr><td>&#128167;</td><td class=\"col2\" id=\"hum\">N/A</div></td> <td class=\"col3\">%</td></tr><tr><td>&#9729;</td><td class=\"col2\" id=\"bar\">N/A</div></td> <td class=\"col3\">mbar</td></tr>\n"
"</tbody></table></td></tr></table>\n"
"<svg class=\"top\" xmlns=\"http://www.w3.org/2000/svg\" x=\"0\" y=\"0\" width=\"94px\" height=\"97px\" viewBox=\"0 0 94 97\"><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M59.707,38.168c0,1.609-1.303,2.912-2.91,2.912 s-2.909-1.303-2.909-2.912c0-1.605,1.302-2.908,2.909-2.908S59.707,36.562,59.707,38.168z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M46.91,91.997c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S46.91,90.392,46.91,91.997z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M67.66,38.711 74.08,42.564 66.034,47.178\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M47,92 c24.855,0,45-20.145,45-45.002C92,22.146,71.855,2,47,2C22.146,2,2,22.146,2,46.998c0,7.953,2.312,18.233,8.667,25.995 c4.848,5.908,15.083,9.917,25.167,5.25c6.155-2.849,12.924-9.339,15.916-12.503c3.741-3.955,10.948-12.77,13.088-16.424 c7.185-12.273-0.257-23.352-14.627-22.324c-7.94,0.566-12.831,7.312-16.68,16.422c-2.546,6.025-3.522,13.453-2.061,18.061\"/><path fill=\"#FFF\" stroke=\"#30A5E7\" stroke-width=\"2\" stroke-linecap=\"round\" d=\"M35.848,47.939 c0,0-7.844,0.533-11.19-2.15c-1.387-1.112-1.408-2.963-1.123-3.996c0.575-2.083,4.444-2.963,4.444-2.963s-4.574-3.471-3.21-6.623 c0.917-2.121,2.964-2.918,4.728-2.906c4.31,0.028,11.14,6.617,12.1,7.938\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M35.707,64.168c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S35.707,62.562,35.707,64.168z\"/></svg>\n"
"</body></html>";

/*boolean isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}*/

/*void website() {
  // If we get another request, redirect to http://adem.local/
  if (!isIp(webserver.hostHeader()) && webserver.hostHeader() != String("adem.local")) {
//    webserver.sendHeader("Location", String("http://") + toStringIp(webserver.client().localIP()), true);
    webserver.sendHeader("Location", "http://adem.local/", true);
    webserver.send ( 302, "text/plain", "Redirecting to http://adem.local/");
    __LOG("DEBUG: Accessed http://"); __LOG(webserver.hostHeader()); __LOG(webserver.uri()); __LOG(" from "); __LOG(webserver.client().remoteIP()); __LOGLN();
  } else {
    webserver.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webserver.sendHeader("Pragma", "no-cache");
    webserver.sendHeader("Expires", "-1");
    webserver.send(200, "text/html", demo_html);
  }
}*/

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  //// TODO save config to json file
}

String SetupDemoPage(void){
  __LOGLN(demo_html);
  Serial.printf("client heap size: %u\n", ESP.getFreeHeap());
  return FPSTR(demo_html);
}

String sensors(void) {
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  StaticJsonBuffer<256> jsonBufferBar;
  root["barometer"] = jsonBufferBar.parseObject(barometer.report());
  StaticJsonBuffer<256> jsonBufferHum;
  root["humidity"] = jsonBufferHum.parseObject(humidity.report());
  StaticJsonBuffer<256> jsonBufferPar;
  root["particulate"] = jsonBufferPar.parseObject(particulate.report());
  char data[256];
  root.printTo(data, sizeof(data));
  //webserver.send(200, "application/json", data);
  Serial.println("{\"barometer\":{\"Sensor\":\"BMP085/BMP180\",\"Temperature\":23.2,\"Pressuse\":1021.34},\"humidity\":{\"Sensor\":\"HTU21DF\",\"Temperature\":21.3,\"Humidity\":53.2},\"PPparticulateD42\":{\"Sensor\":\"PPD42\",\"PM1\":2103,\"PM2.5\":5302}}");

  return FPSTR("{\"barometer\":{\"Sensor\":\"BMP085/BMP180\",\"Temperature\":23.2,\"Pressuse\":1021.34},\"humidity\":{\"Sensor\":\"HTU21DF\",\"Temperature\":21.3,\"Humidity\":53.2},\"particulate\":{\"Sensor\":\"PPD42\",\"PM1\":2103,\"PM2.5\":5302}}");
}    

//#endif

// vim:syntax=cpp
#ifdef TELNET // Not in PRODUCTION

void TelnetSetup(){

#ifdef HOSTNAME
    if (MDNS.begin(HOST_NAME)) {
        Serial.print("* MDNS responder started. Hostname -> ");
        Serial.println(HOST_NAME);
    }
    // Register the services

    // MDNS.addService("http", "tcp", 80);   // Web server - discomment if you need this

    MDNS.addService("telnet", "tcp", 23); // Telnet server RemoteDebug
#endif
Telnet.begin(HOST_NAME); // Initiaze the telnet server

    Telnet.setResetCmdEnabled(true); // Enable the reset command

    //Debug.showDebugLevel(false); // To not show debug levels
    //Debug.showTime(true); // To show time
    //Debug.showProfiler(true); // To show profiler - time between messages of Debug
                                // Good to "begin ...." and "end ...." messages

    Telnet.showProfiler(true); // Profiler
    Telnet.showColors(true); // Colors

    // Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP8266 is plugged in USB

    String helpCmd =  "gps for gpsfix\nmove for moving\nshake to shake\nwifi for wifi";
    
    Telnet.setHelpProjectsCmds(helpCmd);
    Telnet.setCallBackProjectCmds(&processCmdRemoteDebug);

    // This sample

    Serial.println("* Arduino RemoteDebug Library");
    Serial.println("*");
    Serial.print("* WiFI connected. IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("*");
    Serial.println("* Please use the telnet client (telnet for Mac/Unix or putty and others for Windows)");
    Serial.println("*");
    Serial.println("* This sample will send messages of debug in all levels.");
    Serial.println("*");
    Serial.println("* Please try change debug level in telnet, to see how it works");
    Serial.println("*");

}



// Process commands from RemoteDebug

void processCmdRemoteDebug() {

    String lastCmd = Telnet.getLastCommand();
    lastCmd.toLowerCase();
    handle_debug_cmd(lastCmd[0]) ;
}
#endif

/* SDA hangs low
  Serial.println("Starting I2C bus recovery");
  delay(2000);
  //try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
  digitalWrite(SDAPIN, HIGH);
  pinMode(CLKPIN, OUTPUT);
  for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
    digitalWrite(CLKPIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLKPIN, LOW);
    delayMicroseconds(5);
  }

  //a STOP signal (SDA from low to high while CLK is high)
  digitalWrite(SDAPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(CLKPIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SDAPIN, HIGH);
  delayMicroseconds(2);
  //bus status is now : FREE

  Serial.println("bus recovery done, starting scan in 2 secs");
  //return to power up mode
  pinMode(SDAPIN, INPUT);
  pinMode(CLKPIN, INPUT);
  delay(2000);
  //pins + begin advised in https://github.com/esp8266/Arduino/issues/452
  Wire.pins(SDAPIN, CLKPIN); //this changes default values for sda and clock as well
  Wire.begin(SDAPIN, CLKPIN);
  //only pins: no signal on clk and sda
  //only begin: no signal on clk, no signal on sda

  */