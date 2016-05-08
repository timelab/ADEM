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

#include <ESP8266WiFi.h>
#include <TickerSchedlr.h>

//#include <accelerator_MPU6050.h>
#include <barometer_BMP085.h>
#include <battery_lipo.h>
#include <buzzer_passive.h>
#include <gps_SwSerial.h>
#include <humidity_HTU21D.h>
#include <led_NeoPixel.h>
#include <particulate_PPD42.h>
#include "store_And_Forward.h"

#define GPS_TX_PIN 0
#define GPS_RX_PIN 4
#define GPS_BAUD 9600
#define NEOPIXEL_PIN 5
#define SERIAL_BAUD 74880

#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

// Different states of the program
enum state_t {
  STATE_START,
  STATE_SLEEP,
  STATE_CONFIG,
  STATE_GPSTEST,
  STATE_COLLECT,
  STATE_WIFITEST,
  STATE_UPLOAD,
  STATE_RESET,
};
char *states[] {
  "START",
  "SLEEP",
  "CONFIG",
  "GPSTEST",
  "COLLECT",
  "WIFITEST",
  "UPLOAD",
  "RESET",
};
state_t prev_state = STATE_START;
state_t state = STATE_START;
state_t next_state = STATE_SLEEP;

// DEBUG input
class Debug {
public:
  boolean gpsready = false;
  boolean moving = false;
  boolean shaken = false;
  boolean wifi = false;
} debug;

class StubAccelerometer {
public:
  boolean moving = false;
  boolean shaken = false;
} accelerometer;

//class StubBuffer {
//public:
//  boolean empty = true;
//} buffer;

storeAndForwardBuf buffer(10000);

class StubWifiClient {
public:
  boolean fix = false;
  boolean timeout = true;
} wificlient;

// Objects for all the sensor libraries
//MPU6050Sensor accelerometer;
BMP085Sensor barometer;
LipoBattery battery;
PassiveBuzzer buzzer;
SwSerialGPS gps = SwSerialGPS(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
HTU21DFSensor humidity;
NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PPD42Sensor particulate;
//WIFIap wifiap;
//WIFIcl wificl;
const int SCHED_MAX_TASKS = 200;
TickerSchedlr *schedule = NULL;

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

uint32_t colors[] = {
  red,
  black,
  yellow,
  orange,
  green,
  purple,
  blue,
  white,
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
  //LOGLN(":accelerometer: -> Check moving/shaken");
  //accelerometer.read();
  // Serial.println(accelerometer.report());
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
  buffer.write((char *)gps.dataToBuffer(), gps.dataBufferSize());
  __LOGLN(gps.report());
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

void upload(void *){
	// here we have to peek the sensor ID and nack the peek
	// depending on the sensor ID we peek the buffer for databufferSize bytes and send the off to the sensor::bufferReport
	// which returns the json string
	// then we send off this data to the server over the WIFI
	// when succesfully posted we can acknowledge the buffer and move on to the next buffered item
	// by running this as an idle task it will almost constantly process the buffer as long as there is buffered data
	// for debug purposes we could log the WIFI postings
	// if done sending data we suspend this task. The state machine will resume the task when needed
}


// STATES
void start_state() {
//  accelerator.begin()
  battery.begin();
  humidity.begin();
  // FIXME: Moving barometer.begin() up, calibration hangs forever ??
  barometer.begin();
  particulate.begin();

  accelerometer_task = TickerTask::createPeriodic(&accelerometer_run, 1000);
  accelerometer_task->name = "accelerometer";

  battery_task = TickerTask::createPeriodic(&battery_run, 60000);
  battery_task->name = "battery";

  next_state = STATE_SLEEP;
}

void sleep_state() {

  if (accelerometer.moving or debug.moving) {
    next_state = STATE_GPSTEST;
  } else {
    if (not buffer.empty()) {
      next_state = STATE_WIFITEST;
    } else if (accelerometer.shaken or debug.shaken) {
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
}

void gpstest_state() {

  //gps.read();

  if (accelerometer.moving or debug.moving) {
    if (gps.ready or debug.gpsready) {
      next_state = STATE_COLLECT;
    }
  } else {
    next_state = STATE_SLEEP;
  }
}

void collect_state() {

  // Sensor tasks should be reporting on their own
  //LOGLN("Collecting...");

  if ( not (accelerometer.moving or debug.moving) or not (gps.ready or debug.gpsready) ) {
    next_state = STATE_GPSTEST;
  }
}

void wifitest_state() {

  if (wificlient.fix or debug.wifi) {
    next_state = STATE_UPLOAD;
  }

  if (accelerometer.moving or debug.moving or buffer.empty() or wificlient.timeout) {
    next_state = STATE_SLEEP;
  }
}

void upload_state() {

	if (not upload_task)
		upload_task = TickerTask::createIdle(&upload);
	else
	upload_task->resume();

  // Upload action finishes successfully or times out
  // Create JSON of X records
  // Send to server
  // Empty datastore

  if (buffer.empty()) {
	upload_task->suspend();
    next_state = STATE_WIFITEST;
  }
}

void reset_state() {
  ESP.reset();
}


// TRANSITIONS
void start_to_sleep() {

//  buzzer.start_sound();

  Serial.print("Turning WiFi off... ");
//  wifi_set_sleep_type(LIGHT_SLEEP_T);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  Serial.println("OK");
}

void sleep_to_config() {

//  buzzer.config_sound();

  // Resume wifiap task
  //wifiap.begin();

  Serial.print("Starting WiFi as Access Point... ");
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_AP);
  WiFi.softAP("IK-ADEM");
  delay(50);
  Serial.println("OK");

  // Disable for now, it's our temporary switch to go in and out of config
  //debug.shaken = false;

}

void config_to_sleep() {

  // Suspend wifiap task
  //wifiap.end();
  Serial.print("Turning WiFi off... ");
//  wifi_set_sleep_type(LIGHT_SLEEP_T);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  Serial.println("OK");
}

void sleep_to_gpstest() {

  // Resume gps task
  gps.begin();
  gps_task = TickerTask::createPeriodic(&gps_run, 1000);
  gps_task->name = "gps";
}

void gpstest_to_sleep() {

  // Suspend gps task
  gps_task->clear();
  gps.end();
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
  particulate.begin();
  particulate_task = TickerTask::createPeriodic(&particulate_run, 10000);
  particulate_task->name = "particulate";
}

void collect_to_gpstest() {

  // Suspend sensor tasks
  barometer_task->clear();
  barometer.end();
  humidity_task->clear();
  humidity.end();
  particulate_task->clear();
  particulate.end();
}

void sleep_to_wifitest() {

  //wificlient.begin();
  // Resume wificlient task
}

void wifitest_to_sleep() {

  // Suspend wificlient task
  //wificlient_task->clear();
  //wificlient.end();
}

void wifitest_to_upload() {
}

void upload_to_wifitest() {
}


void setup() {
  state = STATE_START;

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Serial communication... OK");

  __LOGLN();
  __LOGLN("Setup started in DEBUG mode.");
  __LOGLN("Press \"g\" for gpsfix, \"m\" for moving, \"r\" to restart, \"s\" to shake and \"w\" for wifi.");
  __LOGLN();

  led.begin();
  led.setcolor(led_state, colors[state]);

//  buzzer.begin();
  if (buffer.full())
	  Serial.println("ERROR no memory to buffer data");

  Serial.print("Initializing scheduler... ");
  schedule = TickerSchedlr::Instance(SCHED_MAX_TASKS);
  Serial.println("OK");
}


// Main loop takes care of state and transition management
void loop() {

  switch(state) {
    case STATE_START:     start_state(); break;
    case STATE_SLEEP:     sleep_state(); break;
    case STATE_CONFIG:    config_state(); break;
    case STATE_GPSTEST:   gpstest_state(); break;
    case STATE_COLLECT:   collect_state(); break;
    case STATE_WIFITEST:  wifitest_state(); break;
    case STATE_UPLOAD:    upload_state(); break;
    case STATE_RESET:     reset_state(); break;
    default:              start_state(); break;
  }

  if (state != next_state) {
    __LOG("Transition from "); __LOG(states[state]); __LOG(" to "); __LOGLN(states[next_state]);

    switch(state) {

      case STATE_START:
        switch(next_state) {
          case STATE_SLEEP:     start_to_sleep(); break;
          default:              next_state = STATE_RESET;
        }; break;

      case STATE_SLEEP:
        switch(next_state) {
          case STATE_CONFIG:    sleep_to_config(); break;
          case STATE_GPSTEST:   sleep_to_gpstest(); break;
          case STATE_WIFITEST:  sleep_to_wifitest(); break;
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

    }

    prev_state = state;
    state = next_state;

    led.setcolor(led_state, colors[next_state]);

//    __LOG("Transition to ");  __LOG(states[next_state]); __LOGLN(" ended.");
  }

  schedule->tick();

#ifdef DEBUG
  if (Serial.available() > 0) {
    char c = Serial.read();
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
        default:
          __LOGLN("No action.");
      }
    }
  }
#endif

}

// vim:syntax=cpp
