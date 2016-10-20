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

#ifndef _PASSIVEBUZZER_h
#define _PASSIVEBUZZER_h

#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <Arduino.h>

#define BUZZER_PIN 5

enum note_t {
    NOTE_C,
    NOTE_D,
    NOTE_E,
    NOTE_F,
    NOTE_G,
    NOTE_A,
    NOTE_B,
    NOTE_CC,
};

//abstract class Sensor
class PassiveBuzzer {
public:
    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();
    //Sensor ();
    PassiveBuzzer();
    void start_sound();
    void config_sound();
    void collect_sound();
    void warning_sound();
    void alarm_sound();

    virtual void playnote(note_t note, int duration);
    virtual void playtone(int tone, int duration);

private:
    int tones[8] {
        1915,
        1700,
        1519,
        1432,
        1275,
        1136,
        1014,
        956,
    };

};

#endif
