#ifndef TVC_CONTROLLER_H
#define TVC_CONTROLLER_H
#include <Arduino.h>
#include "pins.h"
#include "setup.h"
#include <Servo.h>

class ServoController {


    private:
    
        int Pitch = PitchPin;
        int Yaw = YawPin;

    public: 

        
        void getPitchPin(int PitchPin);
        void getYawPin(int YawPin);

        ServoController();
        void setPitch(float angle);
        void setYaw(float angle);

        void setOffset(float pitchOffset, float yawOffset);
        void setLinkageRatio(float pitchRatio, float yawRatio);
        void setGimbalLimits(float minPitch, float maxPitch, float minYaw, float maxYaw);

        void centerServos(float centerPitch, float centerYaw);
        void testServos(float testPitch, float testYaw, int delayTime);


};



#endif // TVC_CONTROLLER_H