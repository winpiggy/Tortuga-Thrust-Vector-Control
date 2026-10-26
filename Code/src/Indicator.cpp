#include <Arduino.h>
#include "Indicator.h"

void Indicator::begin() {
    pinMode(_pin, OUTPUT);
    off();
}

void Indicator::on() {
    _blinking = false;
    write(true);
}

void Indicator::off() {
    _blinking = false;
    write(false);
}

void Indicator::blink(unsigned long intervalMs) {
    if (_blinking && _interval == intervalMs) {
        return;
    }
    _blinking = true;
    _interval = intervalMs;
    _lastUpdate = millis();
    write(true);
}

void Indicator::update() {
    if (_blinking && millis() - _lastUpdate >= _interval) {
        _lastUpdate = millis();
        write(!_state);
    }
}

void Indicator::write(bool state) {
    _state = state;
    digitalWrite(_pin, state ? HIGH : LOW);
}
