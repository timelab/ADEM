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

#include "Accelero_MPU6050.h"

// static member initialisation
volatile bool MPU6050Sensor::_dataReady = false;


MPU6050Sensor::MPU6050Sensor() {
	_measured = false;
}
MPU6050Sensor::~MPU6050Sensor(){
}

// eventueel overloading, bv met INPUT pins, OUTPUT pins...

void MPU6050Sensor::begin () {
  struct int_param_s params;
  params.pin = INTERRUPT_PIN;
  params.cb =  mpu_interrupt;
  mpu_init(&params);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  
  //dmp_register_tap_cb(tap_cb);

  dmp_set_tap_axes(7); // all axis X,Y,Z
  dmp_set_tap_thresh(1,10);
  dmp_set_tap_thresh(2,20);
  dmp_set_tap_thresh(3,30);
  
  dmp_load_motion_driver_firmware();
  dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);
  
  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);
}

void MPU6050Sensor::end () {
}

void MPU6050Sensor::read() {
  if(_dataReady){
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if(sensors & INV_XYZ_ACCEL)
    {
        measuredData._accel_X = accel[1]/accel_sens;
        measuredData._accel_Y = accel[1]/accel_sens;
        measuredData._accel_Z = accel[1]/accel_sens;
    }
    else
    {
        measuredData._accel_X = 0;
        measuredData._accel_Y = 0;
        measuredData._accel_Z = 0;
    }
    
    if(sensors & INV_XYZ_GYRO)
    {
        measuredData._gyro_X = gyro[1]/gyro_sens;
        measuredData._gyro_Y = gyro[1]/gyro_sens;
        measuredData._gyro_Z = gyro[1]/gyro_sens;
    }
    else
    {
        measuredData._gyro_X = 0;
        measuredData._gyro_Y = 0;
        measuredData._gyro_Z = 0;
    }
    measuredData._timestamp = sensor_timestamp;
    
    _dataReady = more;
    _measured = true;
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

void MPU6050Sensor::process() {
}

void MPU6050Sensor::enableLowPower(bool status) {
// just enable the tap detection and no additional data reading
// this should actually go to the LP mode of the MPU but we leave it for now/
/// TODO : Implement the low power mode of the MPU generating an interrupt on tap detection to revive the system

    dmp_enable_feature(DMP_FEATURE_TAP );
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

