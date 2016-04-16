//#include <accelerator_MPU6050.h>
#include <barometer_BMP085.h>
#include <gps_SwSerial.h>
#include <humidity_HTU21D.h>
#include <led_NeoPixel.h>
#include <particulate_PPD42.h>
#include <TickerSchedlr.h>

#define NEOPIXEL_PIN 5
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 14
#define SERIAL_RX_PIN 8
#define SERIAL_TX_PIN 7

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

// Objects for all the sensor libraries
//MPU6050Sensor accelerometer;
BMP085Sensor barometer;
//PassiveBuzzer buzzer;
SwSerialGPS gps = SwSerialGPS(0, 4, 9600);
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
uint32_t white = led.Color(63, 63, 63);
uint32_t red = led.Color(63, 0, 0);
uint32_t orange = led.Color(63, 31, 0);
uint32_t yellow = led.Color(63, 63, 0);
uint32_t green = led.Color(0, 63, 0);
uint32_t light_blue = led.Color(0, 63, 63);
uint32_t blue = led.Color(0, 0, 63);
uint32_t purple = led.Color(31, 0, 63);

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
//  __LOGLN(":accelerometer:");
}

void barometer_run(void *) {
  barometer.read();
  Serial.println(barometer.report());
}

void gps_run(void *) {
  gps.read();
  Serial.println(gps.report());
}

void humidity_run(void *) {
  humidity.read();
  Serial.println(humidity.report());
}

void particulate_run(void *) {
  Serial.println(":particulate: -> Dump record");
  //Serial.println(particulate.report());
}


// STATES
void start_state() {
//  Serial.print("Initializing accelerator sensor... ");
//  accelerator.begin()
//  Serial.println("OK");

  Serial.print("Initializing humidity sensor... ");
  humidity.begin();
  Serial.println("OK");

  Serial.print("Initializing particulate sensor... ");
  particulate.begin();
  Serial.println("OK");

  Serial.print("Initializing barometer sensor... ");
  barometer.begin();
  Serial.println("OK");

  accelerometer_task = TickerTask::createPeriodic(&accelerometer_run, 500);
  accelerometer_task->name = "accelerometer";
  barometer_task = TickerTask::createPeriodic(&barometer_run, 60000);
  barometer_task->name = "barometer";
  gps_task = TickerTask::createPeriodic(&gps_run, 1000);
  gps_task->name = "gps";
  humidity_task = TickerTask::createPeriodic(&humidity_run, 60000);
  humidity_task->name = "humidity";
  particulate_task = TickerTask::createPeriodic(&particulate_run, 30000);
  particulate_task->name = "particulate";

  Serial.println("Periodic tasks initialized...");

  next_state = SLEEP;
}

void sleep_state() {

  // if (accelerometer.moving) {
  if (true) {
    next_state = GPSTEST;
  } else {
    // if (! buffer.empty) {
    if (false) {
      next_state = WIFITEST;
    //} else if (accelerometer.shaking) {
    } else if (false) {
      next_state = CONFIG;
    }
  }
}

void config_state() {

  //if (finished || timeout || canceled) {
  if (true) {
    next_state = SLEEP;
  }
}

void gpstest_state() {

  //if (accelerometer.moving) {
  if (true) {
    if (gps.ready) {
      next_state = COLLECT;
    }
  } else {
    next_state = SLEEP;
  }
}

void collect_state() {

  // sensor tasks should be reporting on their own
//  __LOGLN("Collecting...");

  //if (! accelerometer.moving || ! gps.ready )) {
  if (! gps.ready) {
    next_state = GPSTEST;
  }
}

void wifitest_state() {

  //if (wificlient.fix) {
  if (false) {
    state = UPLOAD;
  }

  //if (accelerometer.moving || buffer.empty || wificlient.timeout) {
  if (false) {
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

  //wifiap.begin();
  // Resume wifiap task
}

void sleep_to_gpstest() {

  gps.begin();
  // Resume gps task
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

  // Suspend barometer task
  // Suspend humidity task
  // Suspend particulate task
  // Suspend gps task
  barometer.end();
  humidity.end();
  particulate.end();
  //gps.end();
}

void gpstest_to_collect() {

  barometer.begin();
  humidity.begin();
  particulate.begin();
  // Resume barometer task
  // Resume humidity task
  // Resume particulate task
}

void collect_to_gpstest() {
}

void wifitest_to_sleep() {

  // Suspend wificlient task
  //wificlient.end();
}

void wifitest_to_upload() {
}

void upload_to_wifitest() {
}

void setup() {
  state = START;

  Serial.begin(SERIAL_BAUD);
  __LOGLN("Setup started in DEBUG mode.");

  Serial.println("Serial communication... OK");

  Serial.print("Initializing LED... ");
  led.begin();
  Serial.println("OK");

  Serial.print("Initializing scheduler... ");
  schedule = TickerSchedlr::Instance(SCHED_MAX_TASKS);
  Serial.println("OK");
}

// the loop function runs over and over again until power down or reset
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

    __LOG("End transition to ");  __LOGLN(states[next_state]);
  }

  schedule->tick();

}

// vim:syntax=cpp
