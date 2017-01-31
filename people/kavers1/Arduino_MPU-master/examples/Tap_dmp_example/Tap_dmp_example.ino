#include <Adafruit_NeoPixel.h>

#include <arduino_mpu.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <Wire.h>
#include <I2Cdev.h>

#define __LOGLN(x) Serial.println(x)
#define __LOG(x) Serial.print(x)

#define DEFAULT_MPU_HZ 4

volatile bool dataReady = false;
volatile int16_t inter = 0;
void interrupt(){
  dataReady = true;
}

short state = 0;
bool standstill = false;

void tap_cb(unsigned char i, unsigned char k) {
    __LOG("MPU: ");__LOGLN("==============================================================");
    __LOG("MPU: ");__LOG(">>>>>>>>>>>>>tap detected ");__LOG(i);__LOG("........."); __LOG(k); __LOGLN(" keep on going<<<<<<<<<");
    __LOG("MPU: ");__LOGLN("==============================================================");
    state = 3;
    standstill = false;
}

#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      8

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

unsigned short gyro_rate, gyro_fsr;
unsigned char accel_fsr;
float gyro_sens;
unsigned short accel_sens;
int nDevices = 0;
int error;

void setu__LOG() {
  Serial.begin(115200);
  while(!Serial);
  struct int_param_s params;
  params.pin = 3;
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
  dmp_set_tap_thresh(2,20);
  dmp_set_tap_thresh(3,30);

  __LOGLN(dmp_load_motion_driver_firmware());
  __LOGLN(dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL));
  dmp_set_tap_thresh(TAP_XYZ, 250);
  dmp_set_tap_axes(TAP_XYZ);
  dmp_set_tap_count(2);
  dmp_set_tap_time(100);
  dmp_set_tap_time_multi(500);

  __LOGLN(dmp_set_fifo_rate(DEFAULT_MPU_HZ));
  __LOGLN(mpu_set_dmp_state(1));

  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);

  pixels.begin(); // This initializes the NeoPixel library.

  if (nDevices == 0)
  {
  Wire.begin();
  for(int address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C - 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) 
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
  }

}



short gyro[3], accel[3], sensors,intstatus;
short accl[3];
unsigned char more;
long quat[4];
long lastmillis;
short threshold = 150;
unsigned long sensor_timestamp;
float anorm;

void loo__LOG() {
  
  if(dataReady){
//    mpu_get_int_status(&intstatus);
    Serial.print("interupt status : ");Serial.println(intstatus);
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if(!more) dataReady=false;
    if(sensors & INV_XYZ_GYRO)
      Serial.print("Gyro: ");Serial.print(gyro[0]/gyro_sens);Serial.print(" ");Serial.print(gyro[1]/gyro_sens);Serial.print(" ");Serial.println(gyro[2]/gyro_sens);
    if(sensors & INV_XYZ_ACCEL)
      Serial.print("Acce: ");Serial.print(accel[0]);Serial.print(" ");Serial.print(accel[1]);Serial.print(" ");Serial.println(accel[2]);
    Serial.print("Delta:");Serial.print(accel[0]-accl[0]);Serial.print(" ");Serial.print(accel[1]-accl[1]);Serial.print(" ");Serial.println(accel[2]-accl[2]);
    uint32_t lon = 0;
    uint32_t lat = 0;
    uint32_t time= 0;
    long tmo = 0;
    I2Cdev::readBytes(32,19,4,(uint8_t *)&lon);
    I2Cdev::readBytes(32,15,4,(uint8_t *)&lat);
    I2Cdev::readBytes(32,11,4,(uint8_t *)&time);
    Serial.print(lat);Serial.print("  ");Serial.print(lon);Serial.print(" ");Serial.print(time);
    if (state != 3)
      {
        if ((abs(accel[0]-accl[0]) < threshold) && (abs(accel[0]-accl[0]) < threshold) & (abs(accel[0]-accl[0]) < threshold) )
        {  
          if (! standstill)
          {
            Serial.println("standstill detection activated");
            standstill = true;
            state = 1;
          }
          else
            if (millis() - lastmillis > 10000)
            {
              Serial.println("!!!!!!!!!!!!!!!! we came to a halt !!!!!!!!!!!!!!!!!");
              dmp_enable_feature(DMP_FEATURE_TAP);
              state = 2;
              for(int address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C - 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) 
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
            }
        }
        else
        {
          standstill = false;
          if (state == 1) state = 0;
          lastmillis = millis();
        }  
      }
      accl[0] = accel[0];
      accl[1] = accel[1];
      accl[2] = accel[2];
      Serial.print(" ");Serial.println(state);
      __LOGLN();
      }
      for(int i=0;i<1;i++){
        switch (state){
        case 0 :
            pixels.setPixelColor(i, pixels.Color(150,0,0)); // Moderately bright green color.
            break;
        case 1 :
            pixels.setPixelColor(i, pixels.Color(100,100,0)); // Moderately bright yellow color.
            break;
        case 2 :
            pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright red color.
            break;
        case 3 :
            pixels.setPixelColor(i, pixels.Color(0,0,150)); // Moderately bright green color.
            pixels.show();
            dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
            delay(5000);
            state = 0;
            standstill = false;
            break;
        }            
        pixels.show(); // This sends the updated pixel color to the hardware.
        
  }
}
