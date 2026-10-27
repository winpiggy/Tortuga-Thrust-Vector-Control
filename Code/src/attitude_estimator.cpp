#include "pins.h"
#include "attitude_estimator.h"
#include "IMU.h"
#include "statemachine.h"

MahonyFilter::MahonyFilter(float kp_, float ki_) : kp(kp_), ki(ki_), q(), eInt{0.0, 0.0, 0.0} {}

float MahonyFilter::calculateAngle(float roll, float pitch) {

    roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    return roll;
    return pitch;
}