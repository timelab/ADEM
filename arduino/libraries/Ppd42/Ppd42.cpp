/*
  Ppd42.cpp - Library for ADEM PPD42 sensor.
  Created by Lieven Blancke.
*/

#include <Arduino.h>
#include <Ppd42.h>

// instantiate and initialize the static variables that are used in the static interrupt functions that handle the hardware interrupts triggered by the PPD42 dust sensor
int Ppd42::PPD_PM10_PIN = 12;
volatile boolean Ppd42::triggerStartedPM10 = false;
volatile unsigned long Ppd42::triggerStartTimePM10 = 0;
volatile unsigned long Ppd42::triggeredTotalTimePM10 = 0;
int Ppd42::PPD_PM25_PIN = 13;
volatile boolean Ppd42::triggerStartedPM25 = false;
volatile unsigned long Ppd42::triggerStartTimePM25 = 0;
volatile unsigned long Ppd42::triggeredTotalTimePM25 = 0;

Ppd42::Ppd42(){

}
// eventueel overloading, bv met INPUT pins, OUTPUT pins...

void Ppd42::begin() {
	// initialization for the dust sensor
	readTimePM10 = millis(); //Fetch the current time
	readTimePM25 = millis(); //Fetch the current time
	debug="";

	//pinMode(PPD_PM25_PIN, FUNCTION_3); //Set TX PIN to GPIO
	//pinMode(PPD_PM10_PIN, FUNCTION_3); //Set RX PIN to GPIO
	//pinMode(PPD_PM25_PIN, INPUT_PULLUP); //Listen at the designated PIN
	pinMode(PPD_PM10_PIN, INPUT);
	pinMode(PPD_PM25_PIN, INPUT);
	activated = true;
	attachInterrupt(PPD_PM10_PIN, handleInterruptPM10, CHANGE); // Attaching interrupt to PIN
	attachInterrupt(PPD_PM25_PIN, handleInterruptPM25, CHANGE); // Attaching interrupt to PIN
	//pinMode(PPD_PM10_PIN, INPUT_PULLUP); //Listen at the designated PIN
	//attachInterrupt(PPD_PM10_PIN, intrLOPM10, CHANGE); // Attaching interrupt to PIN
}

void Ppd42::end() {
}

void Ppd42::read() {
}

double Ppd42::readPM10() {
	if (activated) {
		unsigned long currentTime=millis(); 
		unsigned long sampledTime = currentTime - readTimePM10;
		unsigned long triggeredTotalTime = triggeredTotalTimePM10;
		readTimePM10 = currentTime;
		triggeredTotalTimePM10 = 0;
		return 1.0 * triggeredTotalTime / sampledTime;
	}
	else
		return 0;
}

double Ppd42::readPM25() {
	if (activated) {
		unsigned long currentTime=millis(); 
		unsigned long sampledTime = currentTime - readTimePM25;
		unsigned long triggeredTotalTime = triggeredTotalTimePM25;
		readTimePM25 = currentTime;
		triggeredTotalTimePM25 = 0;
		return 1.0 * triggeredTotalTime / sampledTime;
	}
	else
		return 0;
}

void Ppd42::write() {
}

void Ppd42::interrupt() {
}

void Ppd42::handleInterruptPM10() {
/*	boolean ppd_value_PM10 = digitalRead(PPD_PM10_PIN);
	if (ppd_value_PM10 == LOW && triggerStartedPM10 == false) {
		triggerStartedPM10 = true;
		triggerStartTimePM10 = micros();
	}
	if (ppd_value_PM10 == HIGH && triggerStartedPM10 == true) {
		triggeredTotalTimePM10 += (micros() - triggerStartTimePM10);
		 triggerStartedPM10 = false;
	}
*/
	if (digitalRead(PPD_PM10_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		if (!triggerStartedPM10) {
			triggerStartedPM10 = true;
			triggerStartTimePM10 = micros();
		}
	else	// not LOW, thus end of the trigger
		if (triggerStartedPM10) {
			triggeredTotalTimePM10 += (micros() - triggerStartTimePM10);
			triggerStartedPM10 = false;
		}
}

void Ppd42::handleInterruptPM25() {
	boolean ppd_value_PM25 = digitalRead(PPD_PM25_PIN);
	if (ppd_value_PM25 == LOW && triggerStartedPM25 == false) {
		triggerStartedPM25 = true;
		triggerStartTimePM25 = micros();
	}
	if (ppd_value_PM25 == HIGH && triggerStartedPM25 == true) {
		triggeredTotalTimePM25 += (micros() - triggerStartTimePM25);
		 triggerStartedPM25 = false;
	}
}

void Ppd42::process() {
}

String Ppd42::report()  {
	// should report a JSON string
	return "";
}
