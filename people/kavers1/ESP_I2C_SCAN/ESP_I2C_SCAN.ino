//#define brzo

#ifdef brzo
#include <brzo_i2c.h>
#else
#include <Wire.h>
#endif
/* 
 ESP8266 BlinkWithoutDelay by Simon Peter
 Blink the blue LED on the ESP-01 module
 Based on the Arduino Blink without Delay example
 This example code is in the public domain
 
 The blue LED on the ESP-01 module is connected to GPIO1 
 (which is also the TXD pin; so we cannot use Serial.print() at the same time)
 
 Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/

///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
#define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
#define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
#define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
#define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
#define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
#define I2C_GPS_COMMAND__1            0x01      
#define I2C_GPS_COMMAND__2            0x02      
#define I2C_GPS_COMMAND__3            0x03      
#define I2C_GPS_COMMAND__4            0x04      
#define I2C_GPS_COMMAND__5            0x05      
#define I2C_GPS_COMMAND__6            0x06      
#define I2C_GPS_COMMAND__7            0x07
#define I2C_GPS_COMMAND__8            0x08      
#define I2C_GPS_COMMAND__9            0x09
#define I2C_GPS_COMMAND__a            0x0a
#define I2C_GPS_COMMAND__b            0x0b
#define I2C_GPS_COMMAND__c            0x0c
#define I2C_GPS_COMMAND__d            0x0d
#define I2C_GPS_COMMAND__e            0x0e
#define I2C_GPS_COMMAND__f            0x0f


#define I2C_GPS_REG_VERSION                          3   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_REG_RES2                             4   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES3                             5   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES4                             6   // reserved for future use (uint8_t)

#define I2C_GPS_DAY                                  7   // day
#define I2C_GPS_MONTH                                8   // month
#define I2C_GPS_YEAR                                 9   // year (last 2 digits)

#define I2C_GPS_TIME                                10   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_LOCATION                            14   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_LAT                                 14   // 
#define I2C_GPS_LON                                 18   // 
#define I2C_GPS_GROUND_SPEED                        22   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                            24   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE                       26   // GPS ground course (uint16_t)
#define I2C_GPS_FIX_AGE                             28   // GPS fix age 0.001 sec resolution
#define I2C_GPS_RES1                                30   // reserved for future use (uint16_t)


typedef struct {
  uint8_t    new_data : 1;
  uint8_t    gps2dfix : 1;
  uint8_t    gps3dfix : 1;
  uint8_t    numsats : 4;
} STATUS_REGISTER;

typedef struct {
  int32_t      lat;            //degree*10 000 000
  int32_t      lon;            //degree*10 000 000
} GPS_COORDINATES;

typedef struct {

  //Status and command registers
  STATUS_REGISTER       status;                   // 00  status register
    uint8_t               res1;
    uint8_t               res2;
  uint8_t               sw_version;               // 03  Version of the I2C_GPS sw
  uint8_t               res3;                     // 04  reserved for future use
  uint8_t               res4;                     // 05  reserved for future use
  uint8_t               res5;                     // 06  reserved for future use
  uint8_t               day;                      // 07
  uint8_t               month;                    // 08
  uint8_t               year;                     // 09
  uint16_t              last_receive;                     // 30 reserved for future use
  uint32_t              time;                     // 10 UTC Time from GPS

  //GPS & navigation data
  GPS_COORDINATES       gps_loc;                  // 14 current location (8 byte) lat,lon
//  int32_t               nav_lat;                  // 15 The desired bank towards North (Positive) or South (Negative)      1 deg = 100 max 30deg (3000)
//  int32_t               nav_lon;                  // 19 The desired bank towards East (Positive) or West (Negative)        1 deg = 100 max 30deg (3000)
  uint16_t              ground_speed;             // 22 ground speed from gps m/s*100
  int16_t               altitude;                 // 24 gps altitude
  uint16_t              ground_course;            // 26 GPS ground course
  uint16_t              fix_age;                  // 28 GPS fix age 0.001 sec resolution
 

} I2C_REGISTERS;

// ****************************************************************************** end include file

int ledState = LOW;     
int nDevices = 0;
int address = 0;
int error = 0;
int tmp = 0;
int cnt = 0;
uint8_t buffer[3];
I2C_REGISTERS regs;


unsigned long previousMillis = 0;
const long interval = 100;
// pin  2 = SDA
// pin 14 = SCL
#define SDA_PIN 2
#define SCL_PIN 14

void setup() {
  Serial.begin(115200);
#ifdef brzo  
  // Setup i2c with clock stretching timeout of 2000 usec
  brzo_i2c_setup(SDA_PIN, SCL_PIN, 2000);
#else
  Wire.begin();
  Wire.setClockStretchLimit(50000);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("size of registers =");
  Serial.println(sizeof(regs));
  Serial.println((uint32_t)&regs.status - (uint32_t)&regs);
  Serial.println((uint32_t)&regs.sw_version - (uint32_t)&regs);
  Serial.println((uint32_t)&regs.year - (uint32_t)&regs);
  Serial.println((uint32_t)&regs.time - (uint32_t)&regs);
  Serial.println((uint32_t)&regs.gps_loc.lat - (uint32_t)&regs);
  Serial.println((uint32_t)&regs.last_receive - (uint32_t)&regs);
}

void loop()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;   
    
    if (ledState == LOW)
      ledState = HIGH;  // Note that this switches the LED *off*
    else
      ledState = LOW;   // Note that this switches the LED *on*
    digitalWrite(LED_BUILTIN, ledState);

    tmp = 0;
    address++ ;
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
#ifdef brzo
    brzo_i2c_start_transaction(address, 1500);
    
    error = brzo_i2c_end_transaction();
#else
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
#endif

    if (error == 0)
    {
      Serial.print(millis());
      Serial.print(" : ");
      Serial.print("I2C - 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
      if (address == 0x20)
      {
        Serial.println("  GPS I2C slave found");
        Serial.println("  read version number");
#ifdef brzo        
        brzo_i2c_start_transaction(address, 100);
        buffer[0] = 0x03; // select temperature register
        brzo_i2c_write(buffer, 1, true); // Write one byte WITH repeated start
        buffer[0] = 0x00; // select temperature register
        brzo_i2c_read(buffer, 1, false); // Read temperature, two bytes
        error = brzo_i2c_end_transaction();
        Serial.print(" version returned = ");
        Serial.println(buffer[0]);
        Serial.print(" return code = ");
        Serial.println(error);
#else
        Wire.beginTransmission(address);
        Wire.write(3);
        Wire.endTransmission();
        
        Wire.requestFrom(address, 32);
//        Wire.endTransmission();
        // Wait for data to become available
        while (cnt = Wire.available() < 1)
        ;
        cnt = Wire.available();
        Serial.print(cnt);
        Serial.print(" bytes received [");
        uint8_t * ptr = (uint8_t *)&regs;
        ptr = ptr + 3;
        while (Wire.available() > 0)
        {
           tmp = Wire.read();
           *ptr++ = tmp;
           Serial.print(tmp);
           if (cnt>1) Serial.print(",");
           delay(10);
        }
        Serial.println("]");

       Serial.print("Year ");Serial.println(regs.year) ;
       Serial.print("month ");Serial.println(regs.month);
       Serial.print("day" );Serial.println( regs.day);
       Serial.print("time");Serial.println( regs.time);
       Serial.print("location");Serial.print( regs.gps_loc.lat); Serial.print(" ");Serial.println( regs.gps_loc.lon);
       Serial.print("satellites");Serial.println( regs.status.numsats);
       Serial.print("altitude");Serial.println( regs.altitude);
       Serial.print("last received :");Serial.println( regs.last_receive);
    

#endif        
      }
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print(millis());
      Serial.print(" : ");
      Serial.print("Unknown error at 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    } 
    if (address == 127)  
    {
      if (nDevices == 0) 
      {
        Serial.println("No I2C devices found");
      }
      else
      {
        Serial.println("done");
      }
      address = 0;
      nDevices = 0;
    }
  }
}
