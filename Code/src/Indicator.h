#ifndef INDICATOR_H
#define INDICATOR_H

// On/off/blink for anything driven by a single digital pin.
// Blinking is non-blocking: call update() every loop.
class Indicator {

    public:

    explicit Indicator(int pin) : _pin(pin) {}

    void begin();
    void on();
    void off();
    void blink(unsigned long intervalMs);   // safe to call every loop, only restarts if the interval changes
    void update();

private:
    void write(bool state);

    int _pin;
    unsigned long _lastUpdate = 0;
    unsigned long _interval = 0;
    bool _state = false;
    bool _blinking = false;
};

// You assign the pin when creating an instance, e.g. LED blue(BlueLED);
class LED : public Indicator {

    public:

    using Indicator::Indicator;
};

// Assumes an active buzzer (makes its own tone when the pin is high).
class Buzzer : public Indicator {

    public:

    using Indicator::Indicator;
    void beep(unsigned long intervalMs) { blink(intervalMs); }
};

#endif // INDICATOR_H
