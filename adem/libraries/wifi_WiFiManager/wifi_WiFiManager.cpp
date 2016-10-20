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

#include "wifi_WiFiManager.h"

WiFiManagerWifi::WiFiManagerWifi() {
    WiFiManager wifimanager;
}

WiFiManagerWifi::~WiFiManagerWifi() {
    delete &wifimanager;
}

void WiFiManagerWifi::begin() {
    Serial.print("Initializing WiFiManager... ");
    // Set ConfigPortal timeout as AP
    wifimanager.setConfigPortalTimeout(300);
    // Set connect timeout for WiFiManager client
    wifimanager.setConnectTimeout(15);
    // Only consider APs with signal over 15%
    wifimanager.setMinimumSignalQuality(15);
    Serial.println("OK");
}

void WiFiManagerWifi::end() {
}

void WiFiManagerWifi::read() {
}

void WiFiManagerWifi::write() {
}

void WiFiManagerWifi::process() {
}

void WiFiManagerWifi::sleep() {
    Serial.print("Turning WiFi off... ");

    // Disable WIFI
    //wifi_set_sleep_type(LIGHT_SLEEP_T);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

    Serial.println("OK");
}

void WiFiManagerWifi::start_ap(char SSID[20]) {
    // Wake WIFI
    WiFi.forceSleepWake();

    // Start ConfigPortal and wait for submit or timeout
    if (wifimanager.startConfigPortal(SSID)) {
        connected_once = true;
        // Log that wifi-connection worked
    } else {
        // Log that wifi-connection failed
    }
}

void WiFiManagerWifi::start_client() {
    // Wake WIFI
    WiFi.forceSleepWake();

    // Start ConfigPortal and wait for submit or timeout
    if (wifimanager.autoConnect()) {
        connected = true;
        // Log that wifi-connection worked
    } else {
        // Log that wifi-connection failed
        connected = false;
    }
}

String WiFiManagerWifi::report()  {
    return "Nothing to report.";
}