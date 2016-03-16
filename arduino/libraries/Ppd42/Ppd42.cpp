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
int Ppd42::_PPD_PM10_PIN = 12;
volatile unsigned long Ppd42::_triggerStartMicrosPM10 = 0;
volatile unsigned long Ppd42::_triggeredTotalMicrosPM10 = 0;
int Ppd42::_PPD_PM25_PIN = 13;
volatile unsigned long Ppd42::_triggerStartMicrosPM25 = 0;
volatile unsigned long Ppd42::_triggeredTotalMicrosPM25 = 0;

Ppd42::Ppd42(){

}

void Ppd42::begin() {
}

void Ppd42::begin(int PpdPM10Pin, int PpdPM25Pin) {
	// initialization for the dust sensor
	_readMillisPM10 = millis(); //Fetch the current time
	_readMillisPM25 = millis(); //Fetch the current time
	_triggeredTotalMicrosPM10 = 0;
	_triggeredTotalMicrosPM25 = 0;
	debug="";

	//_PPD_PM10_PIN=PpdPM10Pin;
	//_PPD_PM25_PIN=PpdPM25Pin;
	pinMode(_PPD_PM10_PIN, INPUT);
	pinMode(_PPD_PM25_PIN, INPUT);
	_activated = true;
	
	// Attach interrupts handlers to hardware pins
	attachInterrupt(digitalPinToInterrupt(_PPD_PM10_PIN), _handleInterruptPM10, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_PPD_PM25_PIN), _handleInterruptPM25, CHANGE);
}

void Ppd42::end() {
	_activated = true;
	// detach the interrupts
	detachInterrupt(_PPD_PM10_PIN);
	detachInterrupt(_PPD_PM25_PIN);	
}

void Ppd42::read() {
}

unsigned long Ppd42::readPM10Ppm() {
	if (_activated) {
		unsigned long _currentMillis=millis(); 
		unsigned long _sampledMillis = _currentMillis - _readMillisPM10;
		unsigned long _triggeredTotalMicros = _triggeredTotalMicrosPM10;
		_readMillisPM10 = _currentMillis;
		_triggeredTotalMicrosPM10 = 0;
		return 1000 * _triggeredTotalMicros / _sampledMillis;
	}
	else
		return 0;
}

unsigned long Ppd42::readPM25Ppm() {
	if (_activated) {
		unsigned long _currentMillis=millis(); 
		unsigned long _sampledMillis = _currentMillis - _readMillisPM25;
		unsigned long _triggeredTotalMicros = _triggeredTotalMicrosPM25;
		_readMillisPM25 = _currentMillis;
		_triggeredTotalMicrosPM25 = 0;
		return 1000 * _triggeredTotalMicros / _sampledMillis;
	}
	else
		return 0;
}

void Ppd42::write() {
}

void Ppd42::interrupt() {
}

void Ppd42::_handleInterruptPM10() {
	if (digitalRead(_PPD_PM10_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		_triggerStartMicrosPM10 = micros();
	else	// not LOW, thus end of the trigger
		_triggeredTotalMicrosPM10 += (micros() - _triggerStartMicrosPM10);
}

void Ppd42::_handleInterruptPM25() {
	if (digitalRead(_PPD_PM25_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		_triggerStartMicrosPM25 = micros();
	else	// not LOW, thus end of the trigger
		_triggeredTotalMicrosPM25 += (micros() - _triggerStartMicrosPM25);
}

void Ppd42::process() {
}

String Ppd42::report()  {
	// should report a JSON string
	return "";
}
