#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "IMU.h"
#include "pins.h"
#include <Arduino.h>

enum class State {
    IDLE,
    ARMED,
    LAUNCH,
    FLIGHT,
    DESCENT,
    LANDED
};

const char* stateName(State state);


class StateMachine {

    private:

    IMU &imu;

    State currentState = State::IDLE;

    // Timers for conditions that must hold for a while (0 = not currently true)
    unsigned long uprightSince = 0;
    unsigned long launchSince = 0;
    unsigned long landedSince = 0;

    bool uprightReady = false;     // upright long enough that arming is allowed

    float maxAltitude = 0.0f;     // highest altitude seen, for apogee detection
    float landedRefAltitude = 0.0f; // altitude at the start of the landed check

    // Each returns true once its condition has held for the configured time
    bool isUprightDetected(unsigned long now);
    bool isLaunchDetected(unsigned long now);
    bool isApogeeDetected(float altitude);
    bool isLandedDetected(float altitude, unsigned long now);

    public:

    explicit StateMachine(IMU &imuRef) : imu(imuRef) {}

    State getState() const { return currentState; }

    // Call once per loop after imu.update(). altitude = meters above the pad.
    void update(float altitude);

    // Arming is manual (e.g. a Bluetooth "ARM" command). arm() only succeeds
    // from IDLE after the rocket has been upright for uprightHoldMs.
    bool arm();
    // Back to IDLE. Only allowed while ARMED (never once launched).
    bool disarm();
    bool isUprightReady() const { return uprightReady; }

};

#endif // STATEMACHINE_H
