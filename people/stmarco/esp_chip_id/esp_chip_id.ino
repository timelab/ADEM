#include <ESP8266WiFi.h>                 
ADC_MODE(ADC_VCC);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial.println("start uitlezen verschillende parameters");
delay(100);

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print("getChipId:");
Serial.println(ESP.getChipId());
delay(100);
Serial.print("getFlashChipId:");
Serial.println(ESP.getFlashChipId());
delay(100);
Serial.print("getFreeHeap:");
Serial.println(ESP.getFreeHeap());
delay(100);
Serial.print("getFlashChipSize:");
Serial.println(ESP.getFlashChipSize());
delay(100);
Serial.print("getFlashChipSpeed:");
Serial.println(ESP.getFlashChipSpeed());
delay(100);
Serial.print("getCycleCount:");
Serial.println(ESP.getCycleCount());
delay(100);
Serial.print("getVcc:");
Serial.println(ESP.getVcc());
delay(100);

}

/*
APIs related to deep sleep and watchdog timer are available in the ESP object, only available in Alpha version.

ESP.deepSleep(microseconds, mode) will put the chip into deep sleep. mode is one of WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED. (GPIO16 needs to be tied to RST to wake from deepSleep.)
ESP.restart() restarts the CPU.
ESP.getFreeHeap() returns the free heap size.
ESP.getChipId() returns the ESP8266 chip ID as a 32-bit integer.
Several APIs may be used to get flash chip info:
ESP.getFlashChipId() returns the flash chip ID as a 32-bit integer.
ESP.getFlashChipSize() returns the flash chip size, in bytes, as seen by the SDK (may be less than actual size).
ESP.getFlashChipSpeed(void) returns the flash chip frequency, in Hz.
ESP.getCycleCount() returns the cpu instruction cycle count since start as an unsigned 32-bit. This is useful for accurate timing of very short actions like bit banging.
ESP.getVcc() may be used to measure supply voltage. ESP needs to reconfigure the ADC at startup in order for this feature to be available. Add the following line to the top of your sketch to use getVcc:
ADC_MODE(ADC_VCC);
TOUT pin has to be disconnected in this mode.
Note that by default ADC is configured to read from TOUT pin using analogRead(A0), and ESP.getVCC() is not available.
*/
