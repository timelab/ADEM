//#define brzo

#include <arduino_mpu.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <Adafruit_NeoPixel.h>



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
 (which is also the TXD pin; so we cannot use __LOG() at the same time)
 
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

#define SERIAL_BAUD 115200
#define DEBUG
#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGHEX(msg) Serial.print("0x");Serial.print(msg,HEX);Serial.print(" ")
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGHEX(msg)
#define __LOGLN(msg)
#endif

#define DEFAULT_MPU_HZ 4
#define INTERRUPT_PIN 4

volatile bool dataReady = false;
volatile uint8_t tap_direction;
volatile uint8_t tap_count;
void interrupt(){
  dataReady = true;
}
volatile bool tapped = false;
void tap_cb(unsigned char in1,unsigned char in2){
  __LOGLN("Tap call back called");
  __LOG(" IN1 : ");__LOG(in1);
  __LOG(" IN2 : ");__LOG(in2);
  __LOGLN("");
  tap_direction = in1;
  tap_count = in2;
  tapped = true;
}



unsigned short gyro_rate, gyro_fsr;
unsigned char accel_fsr;
float gyro_sens;
unsigned short accel_sens;

short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
unsigned long sensor_timestamp;

// ****************************************************************************** end tapdetection


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            0

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS   8

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// ****************************************************************************** end neopixels


int ledState = LOW;     
int nDevices = 0;
int address = 0;
int error = 0;
int tmp = 0;
int cnt = 0;
uint8_t buffer[3];
I2C_REGISTERS regs;

void SetTapColor(){

  switch (tap_direction){
  case 1:
      pixels.setPixelColor(7,128 - tap_count * 16,0,0);
      break;
  case 2:
      pixels.setPixelColor(7,128 + tap_count * 16,0,0);
      break ;
  case 3:
      pixels.setPixelColor(7,0,128 - tap_count * 16,0);
      break;
  case 4:
      pixels.setPixelColor(7,0,128 + tap_count * 16,0);
      break;
  case 5:
      pixels.setPixelColor(7,0,0,128 - tap_count * 16);
      break;
  case 6:
      pixels.setPixelColor(7,0,0,128 + tap_count * 16);
      break;
  default:
    ;
  }
  pixels.show();
  tap_direction = 0;
  tap_count = 0;
}

unsigned long previousMillis = 0;
const long interval = 100;
// pin  2 = SDA
// pin 14 = SCL
#define SDA_PIN 2
#define SCL_PIN 14

void setup() {
  Serial.begin(SERIAL_BAUD);
  pixels.begin();
  //pixels.setBrightness(31);
  pixels.setPixelColor(0,128,0,0);
  pixels.show();
  
#ifdef brzo  
  // Setup i2c with clock stretching timeout of 2000 usec
  brzo_i2c_setup(SDA_PIN, SCL_PIN, 2000);
#else
  Wire.begin();
  Wire.setClockStretchLimit(50000);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  __LOG("size of registers =");
  __LOGLN(sizeof(regs));
  __LOGLN((uint32_t)&regs.status - (uint32_t)&regs);
  __LOGLN((uint32_t)&regs.sw_version - (uint32_t)&regs);
  __LOGLN((uint32_t)&regs.year - (uint32_t)&regs);
  __LOGLN((uint32_t)&regs.time - (uint32_t)&regs);
  __LOGLN((uint32_t)&regs.gps_loc.lat - (uint32_t)&regs);
  __LOGLN((uint32_t)&regs.last_receive - (uint32_t)&regs);


  struct int_param_s params;
  params.pin = INTERRUPT_PIN;
  params.cb = interrupt;
  __LOGLN(mpu_init(&params));
  __LOGLN(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  __LOGLN(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  __LOGLN(mpu_set_sample_rate(DEFAULT_MPU_HZ));
  
  mpu_get_sample_rate(&gyro_rate); __LOG("FIFOrate: ");__LOG(gyro_rate);__LOGLN("Hz");
  mpu_get_gyro_fsr(&gyro_fsr); __LOG("Gyro FSR: +/- ");__LOG(gyro_fsr);__LOGLN("DPS");
  mpu_get_accel_fsr(&accel_fsr); __LOG("Accel FSR: +/- ");__LOG(accel_fsr);__LOGLN("G");

  dmp_register_tap_cb(tap_cb);
  dmp_set_tap_axes(7); // all axis X,Y,Z
  dmp_set_tap_thresh(1,10);
  dmp_set_tap_thresh(2,10);
  dmp_set_tap_thresh(3,10);
  
  __LOGLN(dmp_load_motion_driver_firmware());
  __LOGLN(dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL));
  __LOGLN(dmp_set_fifo_rate(DEFAULT_MPU_HZ));
  __LOGLN(mpu_set_dmp_state(1));

  dmp_set_interrupt_mode(DMP_INT_GESTURE);

  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);

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
      __LOG(millis());
      __LOG(" : ");
      __LOG("I2C - ");
      __LOGHEX(address);__LOGLN();
      if (address == 0x20)
      {
        __LOGLN("  GPS I2C slave found");
        __LOGLN("  read version number");
        pixels.setPixelColor(1,128,0,0);
#ifdef brzo        
        brzo_i2c_start_transaction(address, 100);
        buffer[0] = 0x03; // select temperature register
        brzo_i2c_write(buffer, 1, true); // Write one byte WITH repeated start
        buffer[0] = 0x00; // select temperature register
        brzo_i2c_read(buffer, 1, false); // Read temperature, two bytes
        error = brzo_i2c_end_transaction();
        __LOG(" version returned = ");
        __LOGLN(buffer[0]);
        __LOG(" return code = ");
        __LOGLN(error);
#else
        Wire.beginTransmission(address);
        Wire.write(0);
        Wire.endTransmission();
        __LOGLN("ready to receive version number. ");
//        delay(2);

        __LOG("I2C status : ");
        __LOGLN(twi_status());
        int bcnt = Wire.requestFrom(address, 32);
        __LOG("I2C requestfrom : ");
        __LOG(bcnt);
        if (bcnt != 32)
        {
          __LOGLN("timed out !!!!!!!!!!!!!");
          pixels.setPixelColor(6,128,128,0);
        }
        else          
        {
          __LOGLN();
//        Wire.endTransmission();
          // Wait for data to become available
 /*         int timeout = 0;
          delay(10);
          while ((cnt = Wire.available() < 1) or (timeout++ > 10))
            delay(1);
            
          cnt = Wire.available();
          if (cnt < 1 )
            __LOG("timed out !!!!!!!!!!!!!");
          else          
            __LOG(cnt);*/
          __LOG(" bytes received [");
          uint8_t * ptr = (uint8_t *)&regs;
          ptr = ptr;
          cnt = 1;
          while (Wire.available() > 0)
          {
             tmp = Wire.read();
             *ptr++ = tmp;
             __LOG(tmp);
             if (++cnt>1) __LOG(",");
////             delay(10);
          }
          __LOGLN("]");
  
         __LOG("Year ");__LOGLN(regs.year) ;
         __LOG("month ");__LOGLN(regs.month);
         __LOG("day " );__LOGLN( regs.day);
         __LOG("time ");__LOGLN( regs.time);
         __LOG("location ");__LOG( regs.gps_loc.lat); __LOG(" ");__LOGLN( regs.gps_loc.lon);
         __LOG("satellites ");__LOGLN( regs.status.numsats);
         __LOG("altitude ");__LOGLN( regs.altitude);
         __LOG("last received : ");__LOGLN( regs.last_receive);
         pixels.setPixelColor(0,2 << regs.status.numsats ,0,128 * regs.status.gps2dfix);
        }
#endif        
      }
      nDevices++;
    }
    else if (error==4) 
    {
      __LOG(millis());
      __LOG(" : ");
      __LOGHEX(address);__LOGLN();
    } 
    if (address == 127)  
    {
      if (nDevices == 0) 
      {
        __LOGLN("No I2C devices found");
      }
      else
      {
        __LOGLN("done");
      }
 //     for(int i = nDevices;i>0;i--)
 //       pixels.setPixelColor(i, pixels.Color(0,0,0));
      address = 0;
      nDevices = 0;
    }
  }
  if (nDevices >0)
    pixels.setPixelColor(nDevices, pixels.Color(0,150,0)); // Moderately bright green color.
    pixels.setPixelColor(nDevices +1, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel col
    if(dataReady){
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if(!more) dataReady=false;
      __LOGLN("tap detected");
      if (tapped) {
        SetTapColor();
        tapped = false;
      }
//    if(sensors & INV_XYZ_GYRO)
//      __LOG("Gyro: ");__LOG(gyro[0]/gyro_sens);__LOG(" ");__LOG(gyro[1]/gyro_sens);__LOG(" ");__LOGLN(gyro[2]/gyro_sens);
//    if(sensors & INV_XYZ_ACCEL)
//      __LOG("Acce: ");__LOG(accel[0]/(float)accel_sens);__LOG(" ");__LOG(accel[1]/(float)accel_sens);__LOG(" ");__LOGLN(accel[2]/(float)accel_sens);
//      __LOGLN();
    }
}
