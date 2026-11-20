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

        Servo servoPitch;
        Servo servoYaw;

        float pitchOffset = pitch_offset;
        float yawOffset = yaw_offset;
        float pitchRatio = pitch_ratio;
        float yawRatio = yaw_ratio;

        float minPitch = min_pitch;
        float maxPitch = max_pitch;
        float minYaw = min_yaw;
        float maxYaw = max_yaw;

        float centerPitch = 0.0f;
        float centerYaw = 0.0f;

    public:

        ServoController() = default;

        // Call before begin() to override the default pins
        void setPitchPin(int pitchPin);
        void setYawPin(int yawPin);

        // Attach the servos; call from setup()
        void begin();

        // Angles are gimbal angles in degrees; limits, ratio and offset are applied
        void setPitch(float angle);
        void setYaw(float angle);

        void setOffset(float pitchOffset, float yawOffset);
        void setLinkageRatio(float pitchRatio, float yawRatio);
        void setGimbalLimits(float minPitch, float maxPitch, float minYaw, float maxYaw);

        void centerServos(float centerPitch, float centerYaw);
        void testServos(float testPitch, float testYaw, int delayTime);


};



#endif // TVC_CONTROLLER_H
