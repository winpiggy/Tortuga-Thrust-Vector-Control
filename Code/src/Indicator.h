#ifndef INDICATOR_H
#define INDICATOR_H
#include <Arduino.h>
#include "pins.h"

// LED and Buzzer are non-blocking: blink()/beep() never call delay(), so they
// can't stall the control loop. Call update() once per loop to drive them.

class LED {

    public:

    explicit LED(int pin) : _pin(pin) {}

    void begin();
    void on();
    void off();

    // Blink with the given period (ms); keeps blinking until on() or off()
    void blink(unsigned long interval = 500);
    void update();

private:
    int _pin;
    unsigned long _lastUpdate = 0;
    unsigned long _interval = 0; // 0 = not blinking
    bool _state = false;

    void write(bool state);
};

// Written for an active buzzer (makes its own tone when the pin is HIGH).
class Buzzer {

    public:

    explicit Buzzer(int pin = BuzzerPin) : _pin(pin) {}

    void begin();
    void on();
    void off();

    // Sound for `duration` ms, then turn off automatically in update()
    void beep(unsigned long duration = 500);
    void update();

private:
    int _pin;
    unsigned long _offAt = 0;
    bool _state = false;

};

#endif // INDICATOR_H
