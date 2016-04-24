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
 * Copyright 2016 Dag Wieers
 *
 */

#include <buzzer_passive.h>

#define BUZZER_PIN 5
#define SERIAL_BAUD 74880

PassiveBuzzer buzzer;

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println("Serial communication... OK");

    buzzer.begin();

    Serial.println();
    Serial.println("Press \"a\" for alarm sound.");
    Serial.println("Press \"c\" for collect sound.");
    Serial.println("Press \"C\" for config sound.");
    Serial.println("Press \"s\" for start sound.");
    Serial.println("Press \"w\" for warning sound.");
    Serial.println("Press 1-8 for notes.");
    Serial.println();
}

void loop() {

    delay(10);

    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c != '\n' and c != '\r') {
            Serial.print("Key "); Serial.print(c); Serial.println(" is pressed. ");
            switch (c) {
                case 'a': buzzer.alarm_sound(); break;
                case 'c': buzzer.collect_sound(); break;
                case 'C': buzzer.config_sound(); break;
                case 's': buzzer.start_sound(); break;
                case 'w': buzzer.warning_sound(); break;
                case '1': buzzer.playnote(NOTE_C, 100); break;
                case '2': buzzer.playnote(NOTE_D, 100); break;
                case '3': buzzer.playnote(NOTE_E, 100); break;
                case '4': buzzer.playnote(NOTE_F, 100); break;
                case '5': buzzer.playnote(NOTE_G, 100); break;
                case '6': buzzer.playnote(NOTE_A, 100); break;
                case '7': buzzer.playnote(NOTE_B, 100); break;
                case '8': buzzer.playnote(NOTE_CC, 100); break;
                default: Serial.println("No action.");
            }
        }
    }
}