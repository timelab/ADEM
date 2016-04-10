//#include <SPI.h>
//#include <ByteBuffer.h>
//#include <BufferedSerial.h>


#include <Adafruit_NeoPixel.h>
#include <Ppd42.h>
#include <Wire.h>
#include <Sensor.h>
#include <ArduinoJson.h>
#include <TickerSchedlr.h>
#include <BMP085Sensor.h>
#include <HTU21DFSensor.h>

TickerSchedlr *schedule = 0; //TickerSchedlr::Instance(200);


int count = 0;
int count2 = 0;
int count3 = 0;
TickerTask * T1 = NULL;
TickerTask * T2 = NULL;
TickerTask * T3 = NULL;
int pin1 = 0;
int pin2 = 14;
int pin3 = 13;
BMP085Sensor druksensor;
HTU21DFSensor vochtsensor;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, pin1, NEO_GRB + NEO_KHZ800);
//test test2;

void flip(void *)
{
//	int state = digitalRead(12);  // get the current state of GPIO1 pin
//	digitalWrite(12, !state);     // set pin to the opposite state
	__LOG(count);
	__LOG(" :flip1: ");
	__LOGLN(state);
	++count;
	Serial.println(vochtsensor.report());
	Serial.println(druksensor.report());
}

void flip2(void *ptr)
{
	int pin = 14;
	//int *iptr = (int *)ptr;
	//int state = digitalRead(*iptr);  // get the current state of GPIO1 pin
	int state = digitalRead(pin);  // get the current state of GPIO1 pin
	digitalWrite(pin, !state);     // set pin to the opposite state
	__LOG(count2);
	__LOG(" :flip2: ");
	__LOGLN(state);
	++count2;

}

void flip3(void *)
{
	int state = digitalRead(13);  // get the current state of GPIO1 pin
	digitalWrite(13, !state);     // set pin to the opposite state
	__LOG(count3);
	__LOG(" :flip3: ");
	__LOGLN(state);
	__LOG("last "); __LOG(T3->getLastTime()); __LOG(" next ");    __LOG(T3->getNextTime()); __LOG("  period "); __LOGLN(T3->getNextTime() - T3->getLastTime());
	++count3;

	vochtsensor.read();
	druksensor.read();

}

void setup() {
	Serial.begin(74880);
	//pinMode(12, OUTPUT);
	//digitalWrite(12, LOW);
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'
	//strip.setBrightness(64);
	strip.setPixelColor(0,255, 0, 255);
	pinMode(14, OUTPUT);
	digitalWrite(14, LOW);
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

	schedule = TickerSchedlr::Instance(200);
	T1 = TickerTask::createPeriodic(&flip, 5000);
	T2 = TickerTask::createIdle(&flip2);
	T3 = TickerTask::createPeriodic(&flip3, 1000);
	T1->name = "Task 0";
	T2->name = "Task 1";
	T3->name = "Task 2";
	schedule = TickerSchedlr::Instance();
	__LOG("Task 0 next time is "); __LOGLN(T1->getNextTime());
	__LOG("Task 0 period is "); __LOGLN(T1->interval);
	__LOG("Task 0 type is "); __LOGLN(T1->tasktype);
	__LOG("Schedule time "); __LOGLN(schedule->getTickTime());
	__LOG("Task 1 next time is "); __LOGLN(T2->getNextTime());
	__LOG("Task 1 period is "); __LOGLN(T2->interval);
	__LOG("Task 1 type is "); __LOGLN(T2->tasktype);
	//__LOG("Schedule time "); __LOGLN(schedule->getTickTime());
	__LOG("Task 2 next time is "); __LOGLN(T3->getNextTime());
	__LOG("Task 2 period is "); __LOGLN(T3->interval);
	__LOG("Schedule time "); __LOGLN(schedule->getTickTime());

	__LOGLN("schedule added");
	
	vochtsensor.begin();
	druksensor.begin();

}

// the loop function runs over and over again until power down or reset
void loop() {
	schedule->tick();
}
