#include "pins.h"
#include "setup.h"
#include "TVC_Controller.h"

void ServoController::setPitchPin(int pitchPin) {
    Pitch = pitchPin;
}

void ServoController::setYawPin(int yawPin) {
    Yaw = yawPin;
}

void ServoController::begin() {
    servoPitch.attach(Pitch);
    servoYaw.attach(Yaw);
}

// Servo.write() only takes whole degrees. Sending the pulse width directly
// gives about 0.1 degree steps instead. 544-2400 us is the Servo library's
// default 0-180 degree range.
static int servoDegreesToMicros(float servoDegrees) {
    servoDegrees = constrain(servoDegrees, 0.0f, 180.0f);
    return (int)(544.0f + servoDegrees * (2400.0f - 544.0f) / 180.0f + 0.5f);
}

// angle = gimbal deflection from straight (degrees). Clamp it to what the
// gimbal can physically reach, convert to servo degrees, then add the trim.
void ServoController::setPitch(float angle) {
    angle = constrain(angle, minPitch, maxPitch);
    servoPitch.writeMicroseconds(servoDegreesToMicros(servo_center + angle * pitchRatio + pitchOffset));
}

void ServoController::setYaw(float angle) {
    angle = constrain(angle, minYaw, maxYaw);
    servoYaw.writeMicroseconds(servoDegreesToMicros(servo_center + angle * yawRatio + yawOffset));
}

void ServoController::setOffset(float pitchOffset, float yawOffset) {
    this->pitchOffset = pitchOffset;
    this->yawOffset = yawOffset;
}

void ServoController::setLinkageRatio(float pitchRatio, float yawRatio) {
    this->pitchRatio = pitchRatio;
    this->yawRatio = yawRatio;
}

void ServoController::setGimbalLimits(float minPitch, float maxPitch, float minYaw, float maxYaw) {
    this->minPitch = minPitch;
    this->maxPitch = maxPitch;
    this->minYaw = minYaw;
    this->maxYaw = maxYaw;
}

void ServoController::centerServos(float centerPitch, float centerYaw) {
    this->centerPitch = centerPitch;
    this->centerYaw = centerYaw;
    setPitch(centerPitch);
    setYaw(centerYaw);
}

void ServoController::testServos(float testPitch, float testYaw, int delayTime) {
    setPitch(testPitch);
    setYaw(testYaw);
    delay(delayTime);
    setPitch(centerPitch);
    setYaw(centerYaw);
}
