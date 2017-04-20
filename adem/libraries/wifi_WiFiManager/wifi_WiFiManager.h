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
 * Copyright 2016 Dag Wieers, Marco Stolle
 *
 */

#ifndef _WiFiManagerWifi_h
#define _WiFiManagerWifi_h

#include <Arduino.h>
#include <myWiFiManager.h>

class WiFiManagerWifi {
public:
    WiFiManagerWifi();
    ~WiFiManagerWifi();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();

    virtual void start_ap(char[20]);
    virtual void start_client();
    virtual void sleep();

    bool connected = false;
    bool connected_once = false;

private:
    myWiFiManager wifimanager;
    bool _initialized;
    char SSID[20];
};

#endif