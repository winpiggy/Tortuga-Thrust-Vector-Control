#ifndef GIMBAL_H
#define GIMBAL_H

#include "pins.h"

class Gimbal {

    private:
        int pitchPin = PitchPin;
        int yawPin = YawPin;

    public:

        Gimbal(int pitchPin, int yawPin);
        void setPitch(int angle);
        void setYaw(int angle);
};

#endif // GIMBAL_H