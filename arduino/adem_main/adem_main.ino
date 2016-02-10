/* 
  ESP8266 mobile dust sensor

*/

#include "Ppd42/Ppd42.h"
/*
  .h files per sensor containing
    <sensor>_setup
      setup the sensor, initialize variables
    <sensor>_sample
      take sample(s) and move data to temporary storage
    <sensor>_interrupt
      make sure you define variables as volatile
    <sensor>_process
      process temporary storage and massage to representable data
    <sensor>_report
      report data to officially log
  any of these functions can be empty and should take no more than 1 second
  <sensor> can by anything: ppd42, wifi, gps, temp, hygro, gyro, led, beep, ...

  data structure:
    variables for each sensor are kept in the sensor file, or better: class!
    
*/



void setup() {
  // read sensor configuration from eeprom and call setup for the configured sensors
  
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
