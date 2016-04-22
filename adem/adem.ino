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
#include <gps_SwSerial.h>
#include <humidity_HTU21D.h>
#include <led_NeoPixel.h>
#include <particulate_PPD42.h>

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
enum state_t { START, SLEEP, CONFIG, GPSTEST, COLLECT, WIFITEST, UPLOAD, RESET };
char *states[] { "START", "SLEEP", "CONFIG", "GPSTEST", "COLLECT", "WIFITEST", "UPLOAD", "RESET" };
state_t prev_state = START;
state_t state = START;
state_t next_state = SLEEP;

// DEBUG input
class Debug {
public:
  boolean gpsready = false;
  boolean moving = false;
  boolean shaken = false;
  boolean wifi = false;
} debug;

// Objects for all the sensor libraries
//MPU6050Sensor accelerometer;
BMP085Sensor barometer;
//PassiveBuzzer buzzer;
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
uint32_t orange = led.Color(127, 31, 0);
uint32_t yellow = led.Color(127, 127, 0);
uint32_t green = led.Color(0, 127, 0);
uint32_t light_blue = led.Color(0, 127, 127);
uint32_t blue = led.Color(0, 0, 127);
uint32_t purple = led.Color(31, 0, 127);

uint32_t colors[] = { red, black, yellow, orange, green, purple, blue, white };

uint16_t led_state = 0;

// Tasks
TickerTask *accelerometer_task = NULL;
TickerTask *barometer_task = NULL;
TickerTask *gps_task = NULL;
TickerTask *humidity_task = NULL;
TickerTask *particulate_task = NULL;

// TASKS
void accelerometer_run(void *) {
  //LOGLN(":accelerometer: -> Check moving/shaken");
  //accelerometer.read();
  // Serial.println(accelerometer.report());
}

void barometer_run(void *) {
  barometer.read();
  __LOGLN(barometer.report());
}

void gps_run(void *) {
  gps.read();
  __LOGLN(gps.report());
}

void humidity_run(void *) {
  humidity.read();
  __LOGLN(humidity.report());
}

void particulate_run(void *) {
  particulate.read();
  __LOGLN(particulate.report());
}


// STATES
void start_state() {
//  accelerator.begin()
  humidity.begin();
  // FIXME: Moving barometer.begin() up, calibration hangs forever ??
  barometer.begin();
  particulate.begin();

  accelerometer_task = TickerTask::createPeriodic(&accelerometer_run, 1000);
  accelerometer_task->name = "accelerometer";

  next_state = SLEEP;
}

void sleep_state() {

  // if (accelerometer.moving or debug.moving) {
  if (debug.moving) {
    next_state = GPSTEST;
  } else {
    // if (! buffer.empty) {
    if (false) {
      next_state = WIFITEST;
    //} else if (accelerometer.shaken or debug.shaken) {
    } else if (debug.shaken) {
      next_state = CONFIG;
    }
  }
}

void config_state() {

  //if (finished or timeout or canceled) {
  if (true) {
    next_state = SLEEP;
  }
}

void gpstest_state() {

  //gps.read();

  //if (accelerometer.moving or debug.moving) {
  if (debug.moving) {
    if (gps.ready or debug.gpsready) {
      next_state = COLLECT;
    }
  } else {
    next_state = SLEEP;
  }
}

void collect_state() {

  // Sensor tasks should be reporting on their own
  //LOGLN("Collecting...");

  //if ( not (accelerometer.moving or debug.moving) or not (gps.ready or debug.gpsready) ) {
  if ( not (debug.moving) or not (gps.ready or debug.gpsready)) {
    next_state = GPSTEST;
  }
}

void wifitest_state() {

  //if (wificlient.fix or debug.wifi) {
  if (debug.wifi) {
    state = UPLOAD;
  }

  //if (accelerometer.moving or debug.moving or buffer.empty or wificlient.timeout) {
  if (debug.moving) {
    next_state = SLEEP;
  }
}

void upload_state() {

  // Upload action finishes successfully or times out
  // Create JSON of X records
  // Send to server
  // Empty datastore

//  if (buffer.empty) {
  if (true) {
    next_state = WIFITEST;
  }
}


// TRANSITIONS
void start_to_sleep() {
}

void sleep_to_config() {

  // Resume wifiap task
  //wifiap.begin();

  debug.shaken = false;
}

void sleep_to_gpstest() {

  // Resume gps task
  gps.begin();
  gps_task = TickerTask::createPeriodic(&gps_run, 1000);
  gps_task->name = "gps";
}

void sleep_to_wifitest() {

  //wificlient.begin();
  // Resume wificlient task
}

void config_to_sleep() {

  // Suspend wifiap task
  //wifiap.end();
}

void gpstest_to_sleep() {

  // Suspend gps task
  gps_task->clear();
  gps.end();
}

void gpstest_to_collect() {

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

void wifitest_to_sleep() {

  // Suspend wificlient task
  //wificlient_task-kill();
  //wificlient.end();
}

void wifitest_to_upload() {
}

void upload_to_wifitest() {
}


void setup() {
  state = START;
  led.setcolor(led_state, colors[state]);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Serial communication... OK");

  __LOGLN();
  __LOGLN("Setup started in DEBUG mode.");
  __LOGLN("Press \"g\" for gpsfix, \"m\" for moving, \"r\" to restart, \"s\" to shake and \"w\" for wifi.");
  __LOGLN();

  led.begin();

  Serial.print("Initializing scheduler... ");
  schedule = TickerSchedlr::Instance(SCHED_MAX_TASKS);
  Serial.println("OK");

  Serial.print("Turning WiFi off... ");
  WiFi.forceSleepBegin();
  Serial.println("OK");
}


// Main loop takes care of state and transition management
void loop() {

  switch(state) {
    case START:     start_state(); break;
    case SLEEP:     sleep_state(); break;
    case CONFIG:    config_state(); break;
    case GPSTEST:   gpstest_state(); break;
    case COLLECT:   collect_state(); break;
    case WIFITEST:  wifitest_state(); break;
    case UPLOAD:    upload_state(); break;
    default:        start_state(); break;
  }

  if (state != next_state) {
    __LOG("Transition from "); __LOG(states[state]); __LOG(" to "); __LOGLN(states[next_state]);

    switch(state) {

      case START:
        switch(next_state) {
          case SLEEP:     start_to_sleep(); break;
          default:        next_state = RESET;
        }; break;

      case SLEEP:
        switch(next_state) {
          case CONFIG:    sleep_to_config(); break;
          case GPSTEST:   sleep_to_gpstest(); break;
          case WIFITEST:  sleep_to_wifitest(); break;
          default:        next_state = RESET;
        }; break;

      case CONFIG:
        switch(next_state) {
          case SLEEP:     config_to_sleep(); break;
          default:        next_state = RESET;
        }; break;

      case GPSTEST:
        switch(next_state) {
          case SLEEP:     gpstest_to_sleep(); break;
          case COLLECT:   gpstest_to_collect(); break;
          default:        next_state = RESET;
        }; break;

      case COLLECT:
        switch(next_state) {
          case GPSTEST:   collect_to_gpstest(); break;
          default:        next_state = RESET;
        }; break;

      case WIFITEST:
        switch(next_state) {
          case SLEEP:     wifitest_to_sleep(); break;
          case UPLOAD:    wifitest_to_upload(); break;
          default:        next_state = RESET;
        }; break;

      case UPLOAD:
        switch(next_state) {
          case WIFITEST:  upload_to_wifitest(); break;
          default:        next_state = RESET;
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
          debug.gpsready = ! debug.gpsready;
          __LOG("debug.gpsready is "); __LOGLN(debug.gpsready?"on.":"off.");
          break;
        case 'm':
          debug.moving = ! debug.moving;
          __LOG("debug.moving is "); __LOGLN(debug.moving?"on.":"off.");
          break;
        case 'r':
          __LOGLN("Restarting system.");
          ESP.restart();
          break;
        case 's':
          debug.shaken = ! debug.shaken;
          __LOG("debug.shaken is "); __LOGLN(debug.shaken?"on.":"off.");
          break;
        case 'w':
          debug.wifi = ! debug.wifi;
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
