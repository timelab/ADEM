#include <TickerSchedlr.h>
#include <Adafruit_NeoPixel.h>
#include <particulate_PPD42.h>
#include <barometer_BMP085.h>
#include <humidity_HTU21D.h>
//#include <gps_SoftwareSerial.h>

#define NEOPIXEL_PIN 5
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 14
#define SERIAL_RX_PIN 8
#define SERIAL_TX_PIN 7

#define SERIAL_BAUD 74880

// Different states of the program
enum state_t {START, SLEEP, CONFIG, GPS_TEST, COLLECT, WIFI_TEST, UPLOAD, GPS_START, GPS_STOP, WIFI_START, WIFI_STOP};
state_t state = START;

// Objects for all the sensor libraries
//MPU6050Sensor accelerometer;
BMP085Sensor barometer;
//PassiveBuzzer buzzer;
//GPSSensor gps;
HTU21DFSensor humidity;
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
//LEDSensor led;
PPD42Sensor particulate;
//WIFIap wifiap;
//WIFIcl wificl;

// Scheduler
TickerSchedlr *schedule = 0; //TickerSchedlr::Instance(200); HERE OR IN SETUP?

// Tasks
TickerTask *accelerometer_task = NULL;
TickerTask *barometer_task = NULL;
TickerTask *buzzer_task = NULL;
TickerTask *gps_task = NULL;
TickerTask *humidity_task = NULL;
TickerTask *led_task = NULL;
TickerTask *particulate_task = NULL;
//TickerTask *wifiap_task = NULL;
//TickerTask *wificl_task = NULL;

// Task code blocks

void accelerometer_block(void *) {
  __LOG(" :accelerometer_block: ");
  __LOGLN("  ");
}

void barometer_block(void *) {
  __LOGLN("  ");
}

void buzzer_block(void *) {
  __LOGLN("  ");
}

void gps_block(void *) {
  __LOGLN("  ");
}

void humidity_block(void *) {
  __LOGLN("  ");
}

void led_block(void *) {
  __LOGLN("  ");
}

void particulate_block(void *) {
  __LOGLN("  ");
}

//void wifiap_block(void *) {
//  __LOGLN("  ");
//}

//void wificl_block(void *) {
//  __LOGLN("  ");
//}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Setup started.");
  Serial.println("Serial communication initialized...");

  neopixel.begin();
  neopixel.setBrightness(63);
  neopixel.setPixelColor(0, 63, 0, 63);
  neopixel.show();
  Serial.println("NeoPixel initialized...");

//  gps_Serial.begin(GPS_BAUD);  
//  Serial.println("GPS initialized...");

//  Serial.print("Initializing PPD42...");
//  particulate.begin();
//  Serial.println(" done");

  schedule = TickerSchedlr::Instance(200); // HERE OR IN SETUP?
  accelerometer_task = TickerTask::createPeriodic(&accelerometer_block, 5000);
  accelerometer_task->name = "accelerometer";
  barometer_task = TickerTask::createPeriodic(&barometer_block, 5000);
  barometer_task->name = "barometer";
  buzzer_task = TickerTask::createPeriodic(&buzzer_block, 5000);
  buzzer_task->name = "buzzer";
  gps_task = TickerTask::createPeriodic(&gps_block, 5000);
  gps_task->name = "gps";
  humidity_task = TickerTask::createPeriodic(&humidity_block, 5000);
  humidity_task->name = "humidity";
  led_task = TickerTask::createPeriodic(&led_block, 5000);
  led_task->name = "led";
  particulate_task = TickerTask::createPeriodic(&particulate_block, 5000);
  particulate_task->name = "particulate";
  //wifiap_task = TickerTask::createPeriodic(&wifiap_block, 5000);
  //wifiap_task->name = "wifiap";
  //wificl_task = TickerTask::createPeriodic(&wificl_block, 5000);
  //wificl_task->name = "wificl";

  schedule = TickerSchedlr::Instance();  // HERE OR IN SETUP? OR IS THIS DOUBLE
  __LOG("Task accelerometer next time is "); __LOGLN(accelerometer_task->getNextTime());
  __LOG("Task accelerometer period is "); __LOGLN(accelerometer_task->interval);
  __LOG("Task accelerometer type is "); __LOGLN(accelerometer_task->tasktype);
  __LOG("Schedule time "); __LOGLN(schedule->getTickTime());


  __LOGLN("schedule added");
  Serial.println("Tasks created...");

  humidity.begin();
  Serial.println("Humidity sensor initialized...");

  barometer.begin();
  Serial.println("Barometer sensor initialized...");

  Serial.println("Setup finished.");
}

// the loop function runs over and over again until power down or reset
void loop() {
  schedule->tick();
}
