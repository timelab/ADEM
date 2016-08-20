#ifndef I2GPS_H
#define I2GPS_H
//#include "Arduino"
#include "Wire.h"

const uint8_t I2GPS_I2C_ADDR = 88; // TODO: get from header
const uint8_t N_DATA_BYTE = 0x68;
const uint8_t I2GPS_SLAVE_SELECT = 10;

const uint8_t I2GPS_DATETIME_ADDR = 0x00;
const uint8_t I2GPS_LAT_ADDR = 0x04;
const uint8_t I2GPS_LON_ADDR = 0x08;
const uint8_t I2GPS_ALT_ADDR = 0x0C;
const uint8_t I2GPS_SPEED_ADDR = 0x10;
const uint8_t I2GPS_COURSE_ADDR = 0x14;
const uint8_t I2GPS_FIX_AGE_ADDR = 0x18;
const uint8_t I2GPS_YEAR_ADDR = 0x1C;
const uint8_t I2GPS_MONTH_ADDR = 0x1D;
const uint8_t I2GPS_DAY_ADDR = 0x1E;
const uint8_t I2GPS_HOUR_ADDR = 0x1F;
const uint8_t I2GPS_MINUTE_ADDR = 0x20;
const uint8_t I2GPS_SECOND_ADDR = 0x21;
const uint8_t I2GPS_LOGRATE_ADDR = 0x22;
const uint8_t I2GPS_ANALOG_LOG_ADDR = 0x24;
const uint8_t I2GPS_DIGITAL_DIR_ADDR = 0x26;
const uint8_t I2GPS_DIGITAL_RW_ADDR = 0x27;
const uint8_t I2GPS_DIGITAL_LOG_ADDR = 0x28;
const uint8_t I2GPS_D3_PWM_ADDR = 0x2A;
const uint8_t I2GPS_D5_PWM_ADDR = 0x2B;
const uint8_t I2GPS_D6_PWM_ADDR = 0x2C;
const uint8_t I2GPS_D9_PWM_ADDR = 0x2D;
const uint8_t I2GPS_D10_PWM_ADDR = 0x2E;
const uint8_t I2GPS_FILENAME_ADDR = 0x2F;
const uint8_t I2GPS_FILESTAT_ADDR = 0x41;
const uint8_t I2GPS_SEEK_ADDR = 0x42;
const uint8_t I2GPS_TELL_ADDR = 0x42;
const uint8_t I2GPS_FILE_DATA_ADDR = 0x46;
const uint8_t I2GPS_ERROR_ADDR = 0x67;
const uint8_t I2GPS_FILE_ENABLE = (1 << 1);
const uint8_t I2GPS_OPEN_WRITE = 0b00000011;
const uint8_t I2GPS_OPEN_READ = 0b00000010;
const uint8_t I2GPS_FILE_CLOSE = 0;

const uint8_t I2GPS_FILE_ERROR = 0x1;

/* 
 * Read n_byte from slave starting from offset address addr.  
 * Store result stored in dest (which must be at least n_byte long).
 * Return true if successful.
 * 
 * Must be preceded with call to Wire.begin()
 */void gps_raw_write(uint8_t addr,
		   uint8_t n_byte,
		   uint8_t *source){
  Wire.beginTransmission(I2GPS_I2C_ADDR);
  Wire.write(addr);
  for(uint8_t i=0; i < n_byte && i < 32; i++){
    Wire.write(source[i]);
  }
  Wire.endTransmission();
  delay(100); // wait for server to do stuff
}
void gps_raw_write1(uint8_t addr,
		   uint8_t data_byte){
  gps_raw_write(addr, 1, &data_byte);
}

bool gps_raw_read(uint8_t addr,
		  uint8_t n_byte,
		  uint8_t *dest){
  
  bool out = false;
  Wire.beginTransmission(I2GPS_I2C_ADDR); 
  Wire.write(addr); 
  Wire.endTransmission();
  Wire.requestFrom((int)I2GPS_I2C_ADDR, (int)n_byte); // request n_byte bytes 
  if(Wire.available()){
    for(uint8_t i = 0; i < n_byte; i++){
      dest[i] = Wire.read();
    }
    out = true;
  }
  return out;
}
int my_delay(unsigned long ms){
  int val = 0;
  for(int i=0; i < ms; i++){
    for(int j=0; j < 100; j++){
      val += j + i;
    }
  }
  return val;
}
#endif