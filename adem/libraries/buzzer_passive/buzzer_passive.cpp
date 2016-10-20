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

#include "buzzer_passive.h"

PassiveBuzzer::PassiveBuzzer() {

}

// eventueel overloading, bv met INPUT pins, OUTPUT pins...

void PassiveBuzzer::begin() {
    Serial.print("Initializing passive buzzer... ");
    pinMode(BUZZER_PIN, OUTPUT);
    Serial.println("OK");
}

void PassiveBuzzer::end () {
}

void PassiveBuzzer::read() {
}

void PassiveBuzzer::write() {
}
/*
void PassiveBuzzer::interrupt() {
}
*/
void PassiveBuzzer::process() {
}

String PassiveBuzzer::report()  {
    return "Nothing to report.";
}

void PassiveBuzzer::playtone(int tone, int duration) {
    for (long i = 0; i < duration * 1000L; i += tone * 2) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(tone);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(tone);
    }
}

void PassiveBuzzer::playnote(note_t note, int duration) {
    playtone(PassiveBuzzer::tones[note], duration);
}

void PassiveBuzzer::start_sound() {
    playnote(NOTE_C, 150);
}

void PassiveBuzzer::collect_sound() {
    playnote(NOTE_D, 150);
}

void PassiveBuzzer::config_sound() {
    playnote(NOTE_F, 150);
}

void PassiveBuzzer::warning_sound() {
    playnote(NOTE_B, 150);
    playnote(NOTE_A, 150);
    playnote(NOTE_B, 150);
    playnote(NOTE_A, 150);
}

void PassiveBuzzer::alarm_sound() {
    playnote(NOTE_CC, 300);
    playnote(NOTE_B, 300);
    playnote(NOTE_CC, 300);
    playnote(NOTE_B, 300);
}
