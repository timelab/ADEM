/*
  Ppd42.cpp - Library for ADEM PPD42 dust sensor.
  Created by Lieven Blancke.
*/

#include <Arduino.h>
#include <Ppd42.h>
/*
	instantiate and initialize the static variables that are used in 
	the static interrupt functions that handle the hardware interrupts 
	triggered by the PPD42 dust sensor
*/
int Ppd42::PPD_PM10_PIN = 12;
volatile boolean Ppd42::triggerStartedPM10 = false;
volatile unsigned long Ppd42::triggerStartMicrosPM10 = 0;
volatile unsigned long Ppd42::triggeredTotalMicrosPM10 = 0;
int Ppd42::PPD_PM25_PIN = 13;
volatile boolean Ppd42::triggerStartedPM25 = false;
volatile unsigned long Ppd42::triggerStartMicrosPM25 = 0;
volatile unsigned long Ppd42::triggeredTotalMicrosPM25 = 0;

Ppd42::Ppd42(){

}

void Ppd42::begin() {
	// initialization for the dust sensor
	readMillisPM10 = millis(); //Fetch the current time
	readMillisPM25 = millis(); //Fetch the current time
	triggeredTotalMicrosPM10 = 0;
	triggeredTotalMicrosPM25 = 0;
	debug="";

	pinMode(PPD_PM10_PIN, INPUT);
	pinMode(PPD_PM25_PIN, INPUT);
	activated = true;
	attachInterrupt(PPD_PM10_PIN, handleInterruptPM10, CHANGE); // Attaching interrupt to PIN
	attachInterrupt(PPD_PM25_PIN, handleInterruptPM25, CHANGE); // Attaching interrupt to PIN
}

void Ppd42::end() {
	activated = true;
	// detach the interrupts
	detachInterrupt(PPD_PM10_PIN);
	detachInterrupt(PPD_PM25_PIN);	
}

void Ppd42::read() {
}

unsigned long Ppd42::readPM10Ppm() {
	if (activated) {
		unsigned long currentMillis=millis(); 
		unsigned long sampledMillis = currentMillis - readMillisPM10;
		unsigned long triggeredTotalMicros = triggeredTotalMicrosPM10;
		readMillisPM10 = currentMillis;
		triggeredTotalMicrosPM10 = 0;
		return 1000 * triggeredTotalMicros / sampledMillis;
	}
	else
		return 0;
}

unsigned long Ppd42::readPM25Ppm() {
	if (activated) {
		unsigned long currentMillis=millis(); 
		unsigned long sampledMillis = currentMillis - readMillisPM25;
		unsigned long triggeredTotalMicros = triggeredTotalMicrosPM25;
		readMillisPM25 = currentMillis;
		triggeredTotalMicrosPM25 = 0;
		return 1000 * triggeredTotalMicros / sampledMillis;
	}
	else
		return 0;
}

void Ppd42::write() {
}

void Ppd42::interrupt() {
}

void Ppd42::handleInterruptPM10() {
	if (digitalRead(PPD_PM10_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		if (!triggerStartedPM10) {
			triggerStartMicrosPM10 = micros();
			triggerStartedPM10 = true;
		}
		else {
		}
	else	// not LOW, thus end of the trigger
		if (triggerStartedPM10) {
			triggeredTotalMicrosPM10 += (micros() - triggerStartMicrosPM10);
			triggerStartedPM10 = false;
		}
}

void Ppd42::handleInterruptPM25() {
	if (digitalRead(PPD_PM25_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		if (!triggerStartedPM25) {
			triggerStartMicrosPM25 = micros();
			triggerStartedPM25 = true;
		}
		else {
		}
	else	// not LOW, thus end of the trigger
		if (triggerStartedPM25) {
			triggeredTotalMicrosPM25 += (micros() - triggerStartMicrosPM25);
			triggerStartedPM25 = false;
		}
}

void Ppd42::process() {
}

String Ppd42::report()  {
	// should report a JSON string
	return "";
}
