#include "pins.h"
#include "statemachine.h"
#include "IMU.h"
#include "setup.h"
#include <math.h>

const char* stateName(State state) {
    switch (state) {
        case State::IDLE:    return "IDLE";
        case State::ARMED:   return "ARMED";
        case State::LAUNCH:  return "LAUNCH";
        case State::FLIGHT:  return "FLIGHT";
        case State::DESCENT: return "DESCENT";
        case State::LANDED:  return "LANDED";
    }
    return "UNKNOWN";
}

void StateMachine::update(float altitude) {
    const unsigned long now = millis();

    switch(currentState) {
        case State::IDLE:
            if (isUprightDetected(now)) {
                currentState = State::ARMED;
            }
            break;

        case State::ARMED:
            if (isLaunchDetected(now)) {
                currentState = State::LAUNCH;
            }
            break;

        case State::LAUNCH:
            if (altitude > altitudeThreshold) {
                maxAltitude = altitude;
                currentState = State::FLIGHT;
            }
            break;

        case State::FLIGHT:
            if (isApogeeDetected(altitude)) {
                currentState = State::DESCENT;
            }
            break;

        case State::DESCENT:
            if (isLandedDetected(altitude, now)) {
                currentState = State::LANDED;
            }
            break;

        case State::LANDED:
            // Remain in LANDED state until reset
            break;
    }
}

bool StateMachine::isUprightDetected(unsigned long now) {
    float ax, ay, az;
    imu.getAcceleration(ax, ay, az);

    // Angle between the z axis (up the rocket) and the measured gravity vector
    float aNorm = sqrtf(ax * ax + ay * ay + az * az);
    bool upright = false;
    if (aNorm > 0.0f && az > 0.0f) {
        float tiltDeg = acosf(az / aNorm) * 180.0f / PI;
        upright = tiltDeg < uprightToleranceDeg;
    }

    if (!upright) {
        uprightSince = 0;
        return false;
    }
    if (uprightSince == 0) {
        uprightSince = now;
    }
    return now - uprightSince >= uprightHoldMs;
}

bool StateMachine::isLaunchDetected(unsigned long now) {
    float ax, ay, az;
    imu.getAcceleration(ax, ay, az);

    if (az <= launchThreshold) {
        launchSince = 0;
        return false;
    }
    if (launchSince == 0) {
        launchSince = now;
    }
    return now - launchSince >= launchHoldMs;
}

bool StateMachine::isApogeeDetected(float altitude) {
    if (altitude > maxAltitude) {
        maxAltitude = altitude;
    }
    return altitude < maxAltitude - apogeeDrop;
}

bool StateMachine::isLandedDetected(float altitude, unsigned long now) {
    // Landed = altitude has stopped changing. Under a parachute the rocket
    // falls several meters per second, so it can't stay in a 1 m band for 3 s.
    if (landedSince == 0 || fabsf(altitude - landedRefAltitude) > landedAltitudeBand) {
        landedSince = now;
        landedRefAltitude = altitude;
        return false;
    }
    return now - landedSince >= landedHoldMs;
}
