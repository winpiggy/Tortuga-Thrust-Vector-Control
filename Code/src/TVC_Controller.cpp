#include "pins.h"
#include "setup.h"
#include "TVC_Controller.h"

Servo servoPitch;
Servo servoYaw;

ServoController::ServoController() {
    servoPitch.attach(Pitch);
    servoYaw.attach(Yaw);
}

void ServoController::getPitchPin(int PitchPin) {
    Pitch = PitchPin;
}

void ServoController::getYawPin(int YawPin) {
    Yaw = YawPin;
}

void ServoController::setPitch(float angle) {
    servoPitch.write(angle);
}
void ServoController::setYaw(float angle) {
    servoYaw.write(angle);
}
