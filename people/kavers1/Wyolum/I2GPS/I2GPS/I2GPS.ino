


/*
  I2GPS Slave code handles all of the I2C events.  Load this code
  directly onto the I2GPS device.  

  see also http://wyolum.com/docs/I2GPS/I2GPS_V1datasheet%20(2).pdf
  
  This is slave code.
  For client code, see
  
 */

#include <inttypes.h>
#include <SD.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Time.h>
#include "I2GPS.h"
/*
#define N_DATA_BYTE 32
#define I2GPS_SLAVE_SELECT 6
#define I2GPS_I2C_ADDR 48

#define I2GPS_LAT_ADDR 0x04
#define I2GPS_LON_ADDR 0x08
#define I2GPS_ALT_ADDR 0x0c 
#define I2GPS_SPEED_ADDR 0x10
#define I2GPS_COURSE_ADDR 0x14
#define I2GPS_FIX_AGE_ADDR 0x18
#define I2GPS_YEAR_ADDR 0x1c
#define I2GPS_MONTH_ADDR 0x1d
#define I2GPS_DAY_ADDR 0x1e
#define I2GPS_HOUR_ADDR 0x1f
#define I2GPS_MINUTE_ADDR 0x20
#define I2GPS_SECOND_ADDR 0x21

#define I2GPS_FILESTAT_ADDR 0x41
#define I2GPS_FILE_ERROR 0x67
#define I2GPS_TELL_ADDR 0x42
#define I2GPS_SEEK_ADDR 0x42
#define I2GPS_DIGITAL_DIR_ADDR 0x26
#define I2GPS_DIGITAL_RW_ADDR 0x27
#define I2GPS_FILE_DATA_ADDR 0x2f
#define I2GPS_ERROR_ADDR 0x67
#define I2GPS_FILENAME_ADDR 0x2f
#define I2GPS_FILE_ENABLE 0x02
*/

// globals
const int  LED1 = 4;
const int  LED2 = 7; 
const int SQW_PIN = 3;
const int PPS_PIN = 2;
volatile boolean pps_led_state = true;
union converter_t {
  long long_dat; 
  unsigned long ulong_dat; 
  uint8_t byte_dat[4];
};
converter_t converter;
time_t next_log_time = 0;

File file;

uint8_t address = 0;
uint8_t gps_data[N_DATA_BYTE];
unsigned int *log_interval_p = (unsigned int *)gps_data + 0x22;

TinyGPS gps;
SoftwareSerial sws(6, A7);
void I2GPS_onReceive(int n_byte);
void I2GPS_onRequest(void);

bool feedgps();

void setup(){
  Serial.begin(115200);
  Serial.println("I2GPS Slave v1.0");
  Serial.println("Copyright WyoLum, LLC 2012");
  sws.begin(9600);
  pinMode(I2GPS_SLAVE_SELECT, OUTPUT);
  if(!SD.begin(I2GPS_SLAVE_SELECT)){
    Serial.println("Cannot open SD card");
  }
  else{
    Serial.println("SD card open");
  }
  if(!SD.open("00N.BIN")){
    Serial.println("Cannot open SD file 00N.BIN");
  }
  Wire.begin(I2GPS_I2C_ADDR);
  Wire.onReceive(I2GPS_onReceive);
  Wire.onRequest(I2GPS_onRequest);

  attachInterrupt(PPS_PIN - 2, pps_interrupt, RISING);

  for(uint8_t i=0; i < N_DATA_BYTE; i++){
    gps_data[i] = 255;
  }
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  for(int i=0; i < 3; i++){
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    delay(100);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED1, LOW);
    delay(50);
  }
  digitalWrite(LED2, LOW);
  digitalWrite(LED1, LOW);
}

//----------------------------------
// log requested data to open file
//----------------------------------
void log(){
  int Year;
  uint8_t Month, Day, Hour, Minute, Second, Hundredths;
  tmElements_t tm_ele;
  unsigned long gps_time, speed, course, age;
  long lat, lon, alt;

  gps.crack_datetime(&Year, &tm_ele.Month, &tm_ele.Day, 
         &tm_ele.Hour, &tm_ele.Minute, &tm_ele.Second, 
         &Hundredths, &age);
  tm_ele.Year = CalendarYrToTm(Year);
  gps_time = makeTime(tm_ele);
  
}

void loop(){
  int Year;
  uint8_t Month, Day, Hour, Minute, Second, Hundredths;
  tmElements_t tm_ele;
  unsigned long gps_time, speed, course, age;
  long lat, lon, alt;

  if(feedgps()){
    // fill up databuffer with GPS data
    gps.crack_datetime(&Year, &tm_ele.Month, &tm_ele.Day, 
           &tm_ele.Hour, &tm_ele.Minute, &tm_ele.Second, 
           &Hundredths, &age);
    tm_ele.Year = CalendarYrToTm(Year);
    gps_time = makeTime(tm_ele);
    serialize_ulong(gps_time, gps_data + 0x00);

    gps.get_position(&lat, &lon, &age);
    serialize_long(lat, gps_data + I2GPS_LAT_ADDR);
    serialize_long(lon, gps_data + I2GPS_LON_ADDR);

    alt = gps.altitude();
    serialize_long(alt, gps_data + I2GPS_ALT_ADDR);

    speed = gps.speed();
    serialize_ulong(speed, gps_data + I2GPS_SPEED_ADDR);

    course = gps.course();
    serialize_ulong(course, gps_data + I2GPS_COURSE_ADDR);
    
    gps_data[I2GPS_YEAR_ADDR] = (uint8_t)(Year % 100);
    gps_data[I2GPS_MONTH_ADDR] = tm_ele.Month;
    gps_data[I2GPS_DAY_ADDR] = tm_ele.Day;
    gps_data[I2GPS_HOUR_ADDR] = tm_ele.Hour;
    gps_data[I2GPS_MINUTE_ADDR] = tm_ele.Minute;
    gps_data[I2GPS_SECOND_ADDR] = tm_ele.Second;
  }
  else{
    age = gps.age();
  }
  serialize_ulong(age, gps_data + I2GPS_FIX_AGE_ADDR);
  gps_data[I2GPS_DIGITAL_RW_ADDR] = PIND; // write to digital pins

  if((*log_interval_p > 0) && 
     (now() >= next_log_time)){
    log_data();
  }
}
//----------------------------------
// log additional data into SD file 
//----------------------------------
void log_data(){
  if(file_enabled() && (get_filemode() == FILE_WRITE)){
    // GPS date
    // GPS fix
    // Digital pins
    // Analog pins
  }
  next_log_time = now() + *log_interval_p;
}

//----------------------------------
// is logging enabled in the I2C memory map 
//----------------------------------
bool file_enabled(){
  return gps_data[I2GPS_FILESTAT_ADDR] & I2GPS_FILE_ENABLE;
}
//----------------------------------
// copy the file name from the I2C memory map to the out parameter
//----------------------------------

void get_filename(char* out){
  uint8_t i = 0;
  while(gps_data[I2GPS_FILENAME_ADDR + i] && (i < 12)){
    out[i] = gps_data[I2GPS_FILENAME_ADDR + i];
    i++;
  }
  out[i] = 0;
}

//----------------------------------
// what is the filemode READ or WRITE
//----------------------------------
uint8_t get_filemode(){
  uint8_t out;

  if((gps_data[I2GPS_FILESTAT_ADDR]) & 1){
    out = FILE_WRITE;
  }
  else{
    out = FILE_READ;
  }
  return out;
}

//----------------------------------
// read 32 bytes from the SD file and store them in the I2C fire data area
// keep track of the file posistion in the I2C register
//----------------------------------
void read_filedata(){
  uint8_t i = 0;
  if(file_enabled){
    file.read(gps_data + I2GPS_FILE_DATA_ADDR, 32);
    *((unsigned long*)(gps_data + I2GPS_TELL_ADDR)) = file.position(); // tell data
  }
}

// read as much data as is available.
//----------------------------------
// read the I2C received data and process it
//----------------------------------

void I2GPS_onReceive(int n_byte){
  address = Wire.read();
  uint8_t data_idx = 0;
  uint8_t filemode;
  char filename[13];
  filename[12] = 0;
#ifndef DBG
  Serial.print("R: <");
  Serial.print(address, HEX);
  Serial.print("> ");
  Serial.println(Wire.available());
#endif  
  while(Wire.available() && (address + data_idx < N_DATA_BYTE)){
    gps_data[address + data_idx] = Wire.read();
    data_idx++;
  }
  if((address == I2GPS_FILESTAT_ADDR)){
    if(file_enabled()){
      // This is the trigger to open the file
      filemode = get_filemode();
      get_filename(filename);
      if(filemode == FILE_READ){
  if(SD.exists(filename)){
    file = SD.open(filename, FILE_READ);
    read_filedata();
  }
  else{
    gps_data[I2GPS_ERROR_ADDR] = I2GPS_FILE_ERROR;
    Serial.println("FILE DOES NOT EXIST");
    Serial.println(filename);
  }
      }
      else{
  file = SD.open(filename, FILE_WRITE);
      }
      if(!file){
  Serial.println("BAD");
  Serial.println(filename);
  Serial.print("read? ");
  Serial.println(filemode==FILE_READ);
  error(I2GPS_FILE_ERROR);
      }
    }
    else{
      file.close();
    }
  }
  else if((address == I2GPS_FILE_DATA_ADDR) && get_filemode() == FILE_WRITE){
    file.write(gps_data + I2GPS_FILE_DATA_ADDR, data_idx);
#ifdef DBG
    for(int ii=0; ii < data_idx; ii++){
      Serial.print((char)gps_data[I2GPS_FILE_DATA_ADDR + ii]);
    }
#endif
    *((unsigned long*)(gps_data + I2GPS_TELL_ADDR)) = file.position(); // tell data
  }
  else if(((address == I2GPS_SEEK_ADDR) && (data_idx == 4))){
    for(int ii=0; ii<4; ii++){
      converter.byte_dat[ii] = gps_data[I2GPS_SEEK_ADDR + ii];
    }
    file.seek(converter.ulong_dat);
    if(get_filemode() == FILE_READ){
      read_filedata();
    }
  }
  else if((address == I2GPS_DIGITAL_DIR_ADDR) && (data_idx == 1)){
    // reconfigure digital pins
    uint8_t dir = gps_data[I2GPS_DIGITAL_DIR_ADDR];
    for(int ii=0; ii<8; ii++){
      if((dir >> ii) && 1){
  pinMode(ii, OUTPUT);
      }
      else{
  pinMode(ii, INPUT);
      }
    }
  }
  else if((address == I2GPS_DIGITAL_RW_ADDR) && (data_idx == 1)){
    // write digital pins
    uint8_t pd = 0;
    // TODO: Must be faster way!
    for(int ii=0; ii<8; ii++){
      if((gps_data[I2GPS_DIGITAL_RW_ADDR] >> ii) & 1){
  digitalWrite(ii, HIGH);
      }
      else{
  digitalWrite(ii, LOW);
      }
    }
  }
}

void I2GPS_onRequest(){
  int n_byte = 32;
  if(N_DATA_BYTE - address < 32){
    n_byte = N_DATA_BYTE - address;
  }
  Wire.write(gps_data + address, n_byte);
  if((address == I2GPS_FILE_DATA_ADDR)){
    if(file_enabled() && (get_filemode() == FILE_READ)){
      read_filedata(); // reload!
    }
  }
}

bool feedgps(){
  while (sws.available())
  {
    if (gps.encode(sws.read())){
      delay(100);
      return true;
    }
  }
  return false;
}
//----------------------------------
// serialize short to databuffer
//----------------------------------
void serialize_short(int16_t val, uint8_t* dest){
  char *dat_p = (char*)&val;
  for(uint8_t i=0; i < 2; i++){
    dest[i] = dat_p[i];
  }
}
//----------------------------------
// serialize long to databuffer
//----------------------------------
void serialize_long(long val, uint8_t* dest){
  char *dat_p = (char*)&val;
  for(uint8_t i=0; i < 4; i++){
    dest[i] = dat_p[i];
  }
}
//----------------------------------
// serialize usigned short to databuffer
//----------------------------------
void serialize_ushort(uint16_t val, uint8_t* dest){
  char *dat_p = (char*)&val;
  for(uint8_t i=0; i < 2; i++){
    dest[i] = dat_p[i];
  }
}
//----------------------------------
// serialize usigned long to databuffer
//----------------------------------
void serialize_ulong(unsigned long val, uint8_t* dest){
  char *dat_p = (char*)&val;
  for(uint8_t i=0; i < 4; i++){
    dest[i] = dat_p[i];
  }
}

//----------------------------------
//
//----------------------------------
void pps_interrupt(){
  unsigned long  now_us = micros();
  pps_led_state = !pps_led_state;
  digitalWrite(LED2, pps_led_state);
}

//----------------------------------
// write error code in I2C memory map
//----------------------------------
void error(byte code){
  gps_data[I2GPS_ERROR_ADDR] = code;
}
