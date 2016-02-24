#include <Ppd42.h>
/* 
 *  ESP8266 mobile dust sensor
 */

Ppd42 Ppd42Sensor;

void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println("START");
  Ppd42Sensor.begin();
}

void loop() {
  Ppd42Sensor.process();
  delay (5000);
  Serial.print("PM10%=");
  Serial.println(Ppd42Sensor.readPM10()*100);
  Serial.print("PM25%=");
  Serial.println(Ppd42Sensor.readPM25()*100);  
}
