#include <Ppd42.h>
/* 
 *  ESP8266 mobile dust sensor
 */


void setup() {
  // read sensor configuration from eeprom and call setup for the configured sensors
  Ppd42 test;
  test.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
