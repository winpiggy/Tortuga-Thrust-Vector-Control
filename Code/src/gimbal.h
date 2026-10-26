#ifndef GIMBAL_H
#define GIMBAL_H

#include <Servo.h>

// Moves the motor mount. Angles are degrees of motor deflection from centre,
// clamped to +/- maxGimbalAngle; the servo centre, direction and linkage ratio
// are in setup.h.
class Gimbal {

    private:
        int pitchPin;
        int yawPin;
        Servo pitchServo;
        Servo yawServo;
        float pitchAngle = 0.0f;
        float yawAngle = 0.0f;

    public:

        Gimbal(int pitchPin, int yawPin) : pitchPin(pitchPin), yawPin(yawPin) {}

        // Attaches the servos and centres them.
        void begin();
        void setPitch(float angle);
        void setYaw(float angle);
        void center();

        float getPitch() const { return pitchAngle; }
        float getYaw() const { return yawAngle; }
};

#endif // GIMBAL_H
