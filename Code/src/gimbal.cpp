#include <Arduino.h>
#include "gimbal.h"
#include "setup.h"

// Gimbal degrees -> servo pulse width. writeMicroseconds keeps sub-degree resolution
// that Servo::write(int degrees) would throw away.
static int toMicroseconds(int centerUs, int direction, float gimbalAngle) {
    return centerUs + lroundf(direction * gimbalAngle * servoDegPerGimbalDeg * servoUsPerDeg);
}

void Gimbal::begin() {
    pitchServo.attach(pitchPin);
    yawServo.attach(yawPin);
    center();
}

void Gimbal::setPitch(float angle) {
    pitchAngle = constrain(angle, -maxGimbalAngle, maxGimbalAngle);
    pitchServo.writeMicroseconds(toMicroseconds(pitchServoCenterUs, pitchDirection, pitchAngle));
}

void Gimbal::setYaw(float angle) {
    yawAngle = constrain(angle, -maxGimbalAngle, maxGimbalAngle);
    yawServo.writeMicroseconds(toMicroseconds(yawServoCenterUs, yawDirection, yawAngle));
}

void Gimbal::center() {
    setPitch(0.0f);
    setYaw(0.0f);
}
