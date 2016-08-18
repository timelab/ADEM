#include <arduino_mpu.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#define PL(x) Serial.println(x)
#define P(x) Serial.print(x)

#define DEFAULT_MPU_HZ 4

volatile bool dataReady = false;
void interrupt(){
  dataReady = true;
}

volatile bool tapped = false;
void tap_cb(unsigned char direction,unsigned char count){
  tapped = true;
  PL("Tap call back called");
  P("dir : ");P(dir);
  switch (dir){
  case TAP_X_UP   : P(" X_UP"); break;
  case TAP_X_DOWN : P(" X_DOWN"); break;
  case TAP_Y_UP   : P(" Y_UP"); break;
  case TAP_Y_DOWN : P(" Y_DOWN"); break; 
  case TAP_Z_UP   : P(" Z_UP"); break;
  case TAP_Z_DOWN : P(" Z_DOWN"); break;
  }
  P(" count : ");P(count);
  PL("");
}

unsigned short gyro_rate, gyro_fsr;
unsigned char accel_fsr;
float gyro_sens;
unsigned short accel_sens;
void setup() {
  Serial.begin(115200);
  while(!Serial);
  struct int_param_s params;
  params.pin = 2;
  params.cb = interrupt;
  PL(mpu_init(&params));
  PL(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  PL(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  PL(mpu_set_sample_rate(DEFAULT_MPU_HZ));
  
  mpu_get_sample_rate(&gyro_rate); P("FIFOrate: ");P(gyro_rate);PL("Hz");
  mpu_get_gyro_fsr(&gyro_fsr); P("Gyro FSR: +/- ");P(gyro_fsr);PL("DPS");
  mpu_get_accel_fsr(&accel_fsr); P("Accel FSR: +/- ");P(accel_fsr);PL("G");

  dmp_register_tap_cb(tap_cb);
  dmp_set_tap_axes(7); // all axis X,Y,Z
  dmp_set_tap_thresh(1,10);
  dmp_set_tap_thresh(2,20);
  dmp_set_tap_thresh(3,30);
  
  PL(dmp_load_motion_driver_firmware());
  PL(dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL));
  PL(dmp_set_fifo_rate(DEFAULT_MPU_HZ));
  PL(mpu_set_dmp_state(1));


  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);
}

short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
unsigned long sensor_timestamp;

void loop() {
  if(dataReady){
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if(!more) dataReady=false;
    if(sensors & INV_XYZ_GYRO)
      Serial.print("Gyro: ");Serial.print(gyro[0]/gyro_sens);Serial.print(" ");Serial.print(gyro[1]/gyro_sens);Serial.print(" ");Serial.println(gyro[2]/gyro_sens);
    if(sensors & INV_XYZ_ACCEL)
      Serial.print("Acce: ");Serial.print(accel[0]/(float)accel_sens);Serial.print(" ");Serial.print(accel[1]/(float)accel_sens);Serial.print(" ");Serial.println(accel[2]/(float)accel_sens);
      PL();
  }
}
