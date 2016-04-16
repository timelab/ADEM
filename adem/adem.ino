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
//SwSerialGPS gps = SwSerialGPS(8, 7, 9600);
HTU21DFSensor humidity;
NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
PPD42Sensor particulate;
//WIFIap wifiap;
//WIFIcl wificl;
const int SCHED_MAX_TASKS = 200;
TickerSchedlr *schedule = NULL;


// Tasks
TickerTask *accelerometer_task = NULL;
TickerTask *barometer_task = NULL;
TickerTask *buzzer_task = NULL;
TickerTask *gps_task = NULL;
TickerTask *humidity_task = NULL;
TickerTask *led_task = NULL;
TickerTask *particulate_task = NULL;
TickerTask *wifiap_task = NULL;
TickerTask *wificl_task = NULL;

// TASKS
void accelerometer_run(void *) {
  Serial.println(":accelerometer:");
}

void barometer_run(void *) {
  Serial.println(":barometer: -> Dump record");
  Serial.println(barometer.report());
}

void buzzer_run(void *) {
//  Serial.println(":buzzer_run:");
}

void gps_run(void *) {
  Serial.println(":gps: -> Dump time and location record");
  //Serial.println(gps.report());
}

void humidity_run(void *) {
  Serial.println(":humidity: -> Dump record");
  Serial.println(humidity.report());
}

//void led_run(void *) {
//  Serial.println(":led_run:");
//}

void particulate_run(void *) {
  Serial.println(":particulate: -> Dump record");
  //Serial.print(particulate.report());
}

//void wifiap_run(void *) {
//  Serial.println(":wifiap_run:");
//}

//void wificlient_run(void *) {
//  Serial.println(":wificlient_run:");
//}


// STATES
void start_state() {
  led.setcolor(0, 63, 0, 0); // Red

  //accelerator.begin()
  //Serial.println("Accelerator sensor initialized...");

//  barometer.begin();
  Serial.println("Barometer sensor initialized...");

  //buzzer.begin()
  //Serial.println("Buzzer initialized...");

  humidity.begin();
  Serial.println("Humidity sensor initialized...");

  particulate.begin();
  Serial.println("Particulate sensor initialized...");

  //accelerometer_task = TickerTask::createPeriodic(&accelerometer_run, 500);
  //accelerometer_task->name = "accelerometer";
  barometer_task = TickerTask::createPeriodic(&barometer_run, 60000);
  barometer_task->name = "barometer";
  //buzzer_task = TickerTask::createPeriodic(&buzzer_run, 100);
  //buzzer_task->name = "buzzer";
  gps_task = TickerTask::createPeriodic(&gps_run, 1000);
  gps_task->name = "gps";
  humidity_task = TickerTask::createPeriodic(&humidity_run, 60000);
  humidity_task->name = "humidity";
  //led_task = TickerTask::createPeriodic(&led_run, 1000);
  //led_task->name = "led";
  particulate_task = TickerTask::createPeriodic(&particulate_run, 30000);
  particulate_task->name = "particulate";
  //wifiap_task = TickerTask::createPeriodic(&wifiap_run, 5000);
  //wifiap_task->name = "wifiap";
  //wificlient_task = TickerTask::createPeriodic(&wificlient_run, 5000);
  //wificlient_task->name = "wificlient";

  Serial.println("All tasks created...");

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
      //if (gps.ready) {
      if (true) {
        next_state = COLLECT;
      }
    } else {
      next_state = SLEEP;
    }
}

void collect_state() {

  // sensor tasks should be reporting on their own
//  Serial.println("Collecting...");

  //if (! accelerometer.moving || buffer.empty || wifi.timeout )) {
  if (false) {
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

  next_state = WIFITEST;
}


// TRANSITIONS
void start_to_sleep() {
  Serial.println("start_to_sleep begins");
  led.setcolor(0, 0, 0, 0); // Black
  Serial.println("start_to_sleep ends");
}

void sleep_to_config() {

  led.setcolor(0, 0, 63, 63); // Yellow
  //wifiap.begin();
  // Resume wifiap task
}

void sleep_to_gpstest() {

  led.setcolor(0, 63, 31, 0); // Orange
  //gps.begin();
  // Resume gps task
}

void sleep_to_wifitest() {

  led.setcolor(0, 63, 0, 63); // Purple
  //wificlient.begin();
  // Resume wificlient task
}

void config_to_sleep() {

  led.setcolor(0, 0, 0, 0); // Black
  // Suspend wifiap task
  //wifiap.end();
}

void gpstest_to_sleep() {

  led.setcolor(0, 0, 0, 0); // Black
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

  led.setcolor(0, 0, 63, 0); // Green
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
  led.setcolor(0, 0, 0, 0); // Black

  // Suspend wificlient task
  //wificlient.end();
}

void wifitest_to_upload() {
  led.setcolor(0, 0, 0, 63); // Blue
}

void upload_to_wifitest() {
  led.setcolor(0, 63, 0, 63); // Purple
}

void setup() {
  state = START;

  Serial.begin(SERIAL_BAUD);
  Serial.println("Setup started.");
  Serial.println("Serial communication initialized...");

  led.begin();
  Serial.println("LED initialized...");

  schedule = TickerSchedlr::Instance(SCHED_MAX_TASKS);
  Serial.println("Scheduler started...");
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
    Serial.print("Transition from "); Serial.print(states[state]);
    Serial.print(" to "); Serial.println(states[next_state]);

    switch(state) {

      case START:
        switch(next_state) {
          case SLEEP:     start_to_sleep(); break;
          default:        next_state = RESET;
        }; Serial.println("During case."); delay(100); break;

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

    Serial.println("End of transition."); 

    prev_state = state;
    state = next_state;

    // FIXME: Set the led here based on state
  }

  schedule->tick();

}

// vim: syntax=c
