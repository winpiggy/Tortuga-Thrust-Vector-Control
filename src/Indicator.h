#ifndef INDICATOR_H
#define INDICATOR_H

#include "pins.h"


class LED {

    public:

    LED();

    
    void on();
    void off();
    void blink();
    void update();

private:
    int _pin;
    unsigned long _lastUpdate;
    bool _state;
};

#endif // INDICATOR_H