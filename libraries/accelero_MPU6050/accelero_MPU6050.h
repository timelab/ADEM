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
  MPU6050Sensor.h - Skeleton Library for ADEM MPU6050Sensors.

  Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/
#ifndef Accelero_MPU6050_class_h
#define Accelero_MPU6050_class_h

#define ESP8266

#include <Arduino.h>

#include <Sensor.h>
#include <ArduinoJson.h>
#include <arduino_mpu.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#define PL(x) Serial.println(x)
#define P(x) Serial.print(x)

#define DEFAULT_MPU_HZ 4

#define INTERRUPT_PIN 5

struct MPU6050SensorData : sensorData {
    float _accel_X;
    float _accel_Y;
    float _accel_Z;
    float _accel_T;
    float _gyro_X;
    float _gyro_Y;
    float _gyro_Z;
    long _timestamp;
};


//abstract class MPU6050Sensor
class MPU6050Sensor : public Sensor{

 
  public:
    MPU6050Sensor();
    ~MPU6050Sensor();
  //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    uint8_t* bufferData();
    String bufferedReport(uint8_t* buffer);
    virtual String report();
    virtual String buildReport(MPU6050SensorData *sData);
    size_t dataBufferSize();
    uint8_t * dataToBuffer();
    bool hasData();
    void enableLowPower(bool status);
    bool isMoving();
    bool shaken = false;
    float movingThreshold = 1.001;
    long  notmovingDelay = 20000L;
    //MPU6050Sensor ();
  protected:
    
    bool _measured = false;
    unsigned char more;
    MPU6050SensorData measuredData;
    short gyro[3], accel[3], sensors;
    float gyro_sens;
    unsigned short accel_sens;
    long quat[4];
    unsigned long _sensorTimestamp;
    unsigned long _movingTimestamp;
    
     // Static methods and variables for the interrupt functions
    static void mpu_interrupt();
    static void tap_cb(unsigned char, unsigned char);
    static volatile bool _dataReady;
};

#endif
