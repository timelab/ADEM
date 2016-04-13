/*
 * EEPROM Write
 *
 * Stores values read from analog input 0 into the EEPROM.
 * These values will stay in the EEPROM when the board is
 * turned off and may be retrieved later by another sketch.
 */

#include <EEPROM.h>

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int addr = 0;

void setup() {
  delay(1000);
  /***
    Need to divide by 4 because analog inputs range from
    0 to 1023 and each byte of the EEPROM can only hold a
    value from 0 to 255.
  ***/

  int val = 99;

  /***
    Write the value to the appropriate byte of the EEPROM.
    these values will remain there when the board is
    turned off.
  ***/

  EEPROM.write(addr, val);

Serial.begin(9600);
delay(300);
Serial.print("val:");
Serial.println(val);
    /***
    As the EEPROM sizes are powers of two, wrapping (preventing overflow) of an
    EEPROM address is also doable by a bitwise and of the length - 1.

    ++addr &= EEPROM.length() - 1;
  ***/
}

void loop() {


  delay(100);
}
