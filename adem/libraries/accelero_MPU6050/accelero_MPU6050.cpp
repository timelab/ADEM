/*
 * This file is part of the ADEM project.
 *
 * ADEM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,·
 * (at your option) any later version.
 *
 * ADEM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ADEM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 Lieven Blancke, Koen Verstringe
 *
 */

/*
  MPU6050Sensor.cpp - Skeleton library for ADEM MPU6050Sensors.
*/

#include "accelero_MPU6050.h"
#define DEBUG_MPU6050 
#ifdef DEBUG_MPU6050
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

// static member initialisation
volatile bool MPU6050Sensor::_dataReady = false;


MPU6050Sensor::MPU6050Sensor() {
	_measured = false;
}
MPU6050Sensor::~MPU6050Sensor(){
}

// eventueel overloading, bv met INPUT pins, OUTPUT pins...

void MPU6050Sensor::begin () {
    __LOG(" initializing accelero :");
    if (! _initialized)
        configdmp();
  /*struct int_param_s params;
  params.pin = INTERRUPT_PIN;
  params.cb =  mpu_interrupt;
  mpu_init(&params);
  // check on return code for success !!!!
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  
  dmp_register_tap_cb(tap_cb);

  dmp_set_tap_axes(7); // all axis X,Y,Z
  dmp_set_tap_thresh(1,10);
  dmp_set_tap_thresh(2,10);
  dmp_set_tap_thresh(3,10);
  dmp_load_motion_driver_firmware();*/
  dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);

  dmp_set_interrupt_mode(DMP_INT_CONTINUOUS); //DMP_INT_GESTURE
  
  mpu_get_gyro_sens(&gyro_sens);
  __LOG("MPU: ");__LOG("gyro sens ");__LOG(gyro_sens);
  mpu_get_accel_sens(&accel_sens);
  __LOG(" accel sens ");__LOGLN(accel_sens);
}

void MPU6050Sensor::configdmp () {
    __LOG(" send config to accelero :");
  struct int_param_s params;
  params.pin = INTERRUPT_PIN;
  params.cb =  mpu_interrupt;
  mpu_init(&params);
  // check on return code for success !!!!
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  
  dmp_register_tap_cb(tap_cb);

  dmp_set_tap_axes(7); // all axis X,Y,Z
  dmp_set_tap_thresh(1,10);
  dmp_set_tap_thresh(2,10);
  dmp_set_tap_thresh(3,10);
  dmp_load_motion_driver_firmware();
  
  mpu_get_gyro_sens(&gyro_sens);
  __LOG("MPU: ");__LOG("gyro sens ");__LOG(gyro_sens);
  mpu_get_accel_sens(&accel_sens);
  __LOG("accel sens ");__LOGLN(accel_sens);
  _initialized = true;
}

void MPU6050Sensor::end () {
}

void MPU6050Sensor::read() {
 
  if(_dataReady){
    __LOG("MPU: ");__LOG("before dmp_read ");
    dmp_read_fifo(gyro, accel, quat, &_sensorTimestamp, &sensors, &more);
    __LOGLN(_sensorTimestamp);
    __LOG("MPU: ");__LOG(" sensors ");
    __LOGLN(sensors);
    if(sensors & INV_XYZ_ACCEL)
    {
        measuredData._accel_X = (float)accel[0]/accel_sens;
        measuredData._accel_Y = (float)accel[1]/accel_sens;
        measuredData._accel_Z = (float)accel[2]/accel_sens;
        __LOG("MPU: accel data 1: ");__LOG(accel[0]);
        __LOG(" 2: ");__LOG(accel[1]);
        __LOG(" 3: ");__LOGLN(accel[2]);
        
        // calculate the total acceleration. 
        // No square root taken for speed reason
        // We only need this value to compare to a threshold so we can live with the square value
        measuredData._accel_T = (measuredData._accel_X * measuredData._accel_X) + 
                                (measuredData._accel_Y * measuredData._accel_Y) + 
                                (measuredData._accel_Z * measuredData._accel_Z) ;
        _measured = true;
    }
    else
    {
        measuredData._accel_X = 0;
        measuredData._accel_Y = 0;
        measuredData._accel_Z = 0;
        measuredData._accel_T = 0;
    }
    
    if(sensors & INV_XYZ_GYRO)
    {
        measuredData._gyro_X = gyro[0]/gyro_sens;
        measuredData._gyro_Y = gyro[1]/gyro_sens;
        measuredData._gyro_Z = gyro[2]/gyro_sens;
        __LOG("MPU: gyro data 1: ");__LOG(gyro[0]);
        __LOG(" 2: ");__LOG(gyro[1]);
        __LOG(" 3: ");__LOGLN(gyro[2]);
        _measured = true;
    }
    else
    {
        measuredData._gyro_X = 0;
        measuredData._gyro_Y = 0;
        measuredData._gyro_Z = 0;
    }
    measuredData._timestamp = _sensorTimestamp;
    // check if we are moving
     __LOG("MPU: accel total ");__LOG(measuredData._accel_T);__LOG(" >? ");__LOGLN(movingThreshold);
    if (measuredData._accel_T > movingThreshold) {
        __LOGLN("MPU: we are moving ");
        _movingTimestamp = _sensorTimestamp;
    }
    _dataReady = more;
    
  }
}

bool MPU6050Sensor::isMoving()
{
    long now = measuredData._timestamp;
    if ((now - _movingTimestamp) > notmovingDelay) 
        return false;
    else
        if (now > _movingTimestamp)
            return true;
        else // we have wrapped around (can it happen ?)
        {
            // quick and dirty fix. 
            // if we wrapped around assume lastmoving = now
            _movingTimestamp = now;
        }
}

bool MPU6050Sensor::hasData()
{ 
    return _dataReady; 
}

void MPU6050Sensor::write() {
}

void MPU6050Sensor::mpu_interrupt() {
 _dataReady = true;
}

void MPU6050Sensor::tap_cb(unsigned char i, unsigned char k) {
    __LOGLN("");
    __LOG("MPU: ");__LOGLN("==============================================================");
    __LOG("MPU: ");__LOGLN(">>>>>>>>>>>>>tap detected ............. keep on going<<<<<<<<<");
    __LOG("MPU: ");__LOG(">>>>>>>>>>>>>  "); __LOG(i); __LOG(" "); __LOGLN (k );
    __LOG("MPU: ");__LOGLN("==============================================================");
}
void MPU6050Sensor::process() {
}

void MPU6050Sensor::enableLowPower(bool status) {
// just enable the tap detection and no additional data reading
// this should actually go to the LP mode of the MPU but we leave it for now/
/// TODO : Implement the low power mode of the MPU generating an interrupt on tap detection to revive the system
    __LOGLN(">>>>>============= ENTER LOW POWER ==================<<<<<");
    dmp_enable_feature(DMP_FEATURE_TAP );
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);// DMP_INT_GESTURE
}

String MPU6050Sensor::report()  {
     if (!_measured)
        read();
    _measured = false;
    return buildReport(&measuredData);
 }

String MPU6050Sensor::buildReport(MPU6050SensorData *sData){
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    MPU6050SensorData *tmpData = reinterpret_cast <MPU6050SensorData*>(sData);
    root["Sensor"] = "MPU6050/MPU6050";
    root["Timestamp"] = tmpData->_timestamp;
    root["Accel_X"] = (float)tmpData->_accel_X;
    root["Accel_Y"] = (float)tmpData->_accel_Y;
    root["Accel_Z"] = (float)tmpData->_accel_Z;
    root["Gyro_X"] = (float)tmpData->_gyro_X;
    root["Gyro_Y"] = (float)tmpData->_gyro_Y;
    root["Gyro_Z"] = (float)tmpData->_gyro_Z;
    
    root.printTo(response, sizeof(response));
    
    return response;
}
   
size_t MPU6050Sensor::dataBufferSize() {
    return sizeof(measuredData);
}

uint8_t * MPU6050Sensor::dataToBuffer(){
    return (uint8_t *) & measuredData;
};

String MPU6050Sensor::bufferedReport(uint8_t * bufferedData){
    MPU6050SensorData tmpData;
    memcpy(&tmpData,bufferedData,sizeof(measuredData));
    buildReport(&tmpData);
}

