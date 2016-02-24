#include <Ppd42.h>
/* 
 *  ESP8266 mobile dust sensor
 */

Ppd42 Ppd42Sensor;

void setup() {
  Ppd42Sensor.begin();
}

void loop() {
  Ppd42Sensor.process();
}
