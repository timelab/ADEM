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
  Sensor.cpp - Skeleton library for ADEM sensors.
*/

#include "Sensor.h"


Sensor::Sensor() {
	_measured = false;
}
Sensor::~Sensor(){
}

// eventueel overloading, bv met INPUT pins, OUTPUT pins...

void Sensor::begin () {
}

void Sensor::end () {
}

void Sensor::read() {
}

void Sensor::write() {
}

//void Sensor::interrupt() {
//}

void Sensor::process() {
}

String Sensor::report()  {
}

String Sensor::buildReport(sensorData *sData){

}
   
size_t Sensor::dataBufferSize() {
    return sizeof(measuredData);
}

uint8_t * Sensor::dataToBuffer(){
    return (uint8_t *) & measuredData;
};

String Sensor::bufferedReport(uint8_t * bufferedData){
    sensorData tmpData;
    memcpy(&tmpData,bufferedData,sizeof(measuredData));
    buildReport(&tmpData);
}

