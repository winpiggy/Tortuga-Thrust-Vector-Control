#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "flight_data.h"

// Decides the flight phase from processed sensor data. It never touches hardware,
// so it can be tested by feeding it FlightData by hand.
class StateMachine {

    private:

    State currentState = State::IDLE;

    float groundAltitude = 0.0f;    // pad altitude, tracked while IDLE/ARMED, frozen at launch
    bool groundSet = false;
    float relativeAltitude = 0.0f;  // m above the pad
    float maxAltitude = 0.0f;       // highest relativeAltitude since launch

    // Consecutive-sample counters so one noisy reading can't change state
    int uprightCount = 0;
    int launchCount = 0;
    int apogeeCount = 0;
    int landedCount = 0;

    bool isUprightDetected(const FlightData &data);
    bool isLaunchDetected(const FlightData &data);
    bool isApogeeDetected();
    bool isLandedDetected(const FlightData &data);

    public:

    StateMachine() = default;

    // Call once per loop. Returns the (possibly new) state.
    State update(const FlightData &data);

    State getState() const { return currentState; }
    float getRelativeAltitude() const { return relativeAltitude; }

};

#endif // STATEMACHINE_H
