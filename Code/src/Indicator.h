#ifndef INDICATOR_H
#define INDICATOR_H

#include "pins.h"

// Have a blink, turn on, and turn off functionality for each LED
// The LED class encapsulates the behavior of an individual LED, including its pin assignment, current state, and timing for blinking.
// You assign the pin for the LED when creating an instance of this class.

class LED {

    public:

    LED();

    LED(int pin);
    void on();
    void off();
    void blink();
    void update();

private:
    int _pin;
    unsigned long _lastUpdate;
    bool _state;
};

class Buzzer {

    public:

    Buzzer();

    Buzzer(int pin);
    void on();
    void off();
    void beep();
    void update();

private:
    int _pin;
    unsigned long _lastUpdate;
    bool _state;

};

#endif // INDICATOR_H