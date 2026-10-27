#include "pins.h"
#include "gimbal.h"

Gimbal::Gimbal(int pitchPin, int yawPin) {
    this->pitchPin = pitchPin;
    this->yawPin = yawPin;
}

