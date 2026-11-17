#include "pins.h"
#include "Indicator.h"


//Define LED class methods

LED::LED() {
    _pin = -1;
    _lastUpdate = 0;
    _state = false;
}

LED::LED(int pin) {
    _pin = pin;
    _lastUpdate = 0;
    _state = false;
}

void LED::on(int &LEDPin) {
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, HIGH);
    _state = true;
}

void LED::off(int &LEDPin) {
    digitalWrite(LEDPin, LOW);
    _state = false;
}

void LED::blink(int &LEDPin, int interval) {
    if (_state) {
        off(LEDPin);
    } else {
        on(LEDPin);
        delay(interval);
        on(LEDPin);
    }
}

//Define Buzzer class methods

Buzzer::Buzzer() {
    _pin = BuzzerPin;
    _lastUpdate = 0;
    _state = false;
}

void Buzzer::on() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH);
    _state = true;
}
void Buzzer::off() {
    digitalWrite(_pin, LOW);
    _state = false;
}
void Buzzer::beep() {
    on();
    delay(interval);
    off();
}