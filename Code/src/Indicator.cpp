#include "pins.h"
#include "Indicator.h"


//Define LED class methods

void LED::begin() {
    pinMode(_pin, OUTPUT);
    write(false);
}

void LED::write(bool state) {
    digitalWrite(_pin, state ? HIGH : LOW);
    _state = state;
}

void LED::on() {
    _interval = 0;
    write(true);
}

void LED::off() {
    _interval = 0;
    write(false);
}

void LED::blink(unsigned long interval) {
    _interval = interval;
}

void LED::update() {
    if (_interval == 0) {
        return;
    }
    unsigned long now = millis();
    if (now - _lastUpdate >= _interval / 2) { // half the period on, half off
        _lastUpdate = now;
        write(!_state);
    }
}

//Define Buzzer class methods

void Buzzer::begin() {
    pinMode(_pin, OUTPUT);
    off();
}

void Buzzer::on() {
    _offAt = 0;
    digitalWrite(_pin, HIGH);
    _state = true;
}

void Buzzer::off() {
    _offAt = 0;
    digitalWrite(_pin, LOW);
    _state = false;
}

void Buzzer::beep(unsigned long duration) {
    digitalWrite(_pin, HIGH);
    _state = true;
    _offAt = millis() + duration;
    if (_offAt == 0) {
        _offAt = 1; // 0 means "no timer"
    }
}

void Buzzer::update() {
    if (_offAt != 0 && (long)(millis() - _offAt) >= 0) {
        off();
    }
}
