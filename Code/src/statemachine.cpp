#include <math.h>
#include "statemachine.h"
#include "setup.h"

// Returns true once `condition` has been true for `required` calls in a row.
static bool sustained(bool condition, int &count, int required) {
    count = condition ? count + 1 : 0;
    return count >= required;
}

State StateMachine::update(const FlightData &data) {

    if (!groundSet) {
        groundAltitude = data.altitude;
        groundSet = true;
    }

    // While sitting on the pad, slowly average the altitude to get a ground reference
    // (absolute baro altitude drifts with weather and isn't 0 at the launch site).
    if (currentState == State::IDLE || currentState == State::ARMED) {
        groundAltitude += groundAltitudeSmoothing * (data.altitude - groundAltitude);
    }

    relativeAltitude = data.altitude - groundAltitude;

    if (currentState == State::IDLE || currentState == State::ARMED) {
        maxAltitude = relativeAltitude;
    } else if (relativeAltitude > maxAltitude) {
        maxAltitude = relativeAltitude;
    }

    // Update the state based on the current conditions
    switch(currentState) {
        case State::IDLE:
            if (isUprightDetected(data)) {
                currentState = State::ARMED;
            }
            break;

        case State::ARMED:
            if (isLaunchDetected(data)) {
                currentState = State::LAUNCH;
            }
            break;

        case State::LAUNCH:
            if (relativeAltitude > altitudeThreshold) {
                currentState = State::FLIGHT;
            }
            break;

        case State::FLIGHT:
            if (isApogeeDetected()) {
                currentState = State::DESCENT;
            }
            break;

        case State::DESCENT:
            if (isLandedDetected(data)) {
                currentState = State::LANDED;
            }
            break;

        case State::LANDED:
            // Remain in LANDED state until reset
            break;
    }

    return currentState;
}

bool StateMachine::isUprightDetected(const FlightData &data) {
    // Uses the estimator's tilt, which on the pad comes from the accelerometer
    bool upright = fabsf(data.pitch) < uprightTolerance && fabsf(data.yaw) < uprightTolerance;
    return sustained(upright, uprightCount, uprightSamples);
}

bool StateMachine::isLaunchDetected(const FlightData &data) {
    // Assuming z-axis points out the nose
    return sustained(data.az > launchThreshold, launchCount, launchSamples);
}

bool StateMachine::isApogeeDetected() {
    return sustained(relativeAltitude < maxAltitude - apogeeDrop, apogeeCount, apogeeSamples);
}

bool StateMachine::isLandedDetected(const FlightData &data) {
    float accelMagnitude = sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    bool still = fabsf(accelMagnitude - gravity) < landedAccelTolerance;
    bool low = relativeAltitude < landedAltitude;
    return sustained(still && low, landedCount, landedSamples);
}
