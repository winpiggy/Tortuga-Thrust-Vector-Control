#ifndef INDICATOR_H
#define INDICATOR_H
#include <Arduino.h>
#include "pins.h"

// Have a blink, turn on, and turn off functionality for each LED
// The LED class encapsulates the behavior of an individual LED, including its pin assignment, current state, and timing for blinking.
// You assign the pin for the LED when creating an instance of this class.

class LED {

    public:

    LED();

    int pinBlue = BlueLED;
    int pinWhite = WhiteLED;

    LED(int pin);
    void on(int &LEDPin);
    void off(int &LEDPin);
    void blink(int &LEDPin, int interval = 500);

private:
    int _pin;
    unsigned long _lastUpdate;
    bool _state;
    int interval = 500; // Interval for blinking in milliseconds
};

class Buzzer {

    public:

    Buzzer();
    int interval = 500; // Interval for beeping in milliseconds
    Buzzer(int pin);
    void on();
    void off();
    void beep();

private:
    int _pin;
    unsigned long _lastUpdate;
    bool _state;

};

#endif // INDICATOR_H