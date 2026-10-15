#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "baro.h"
#include "IMU.h"
#include "pins.h"

enum State {
    IDLE,
    ARMED,
    LAUNCH,
    FLIGHT,
    DESCENT,
    LANDED
};


class StateMachine {

    private:

    IMU &imu;

    State currentState = State::IDLE;

   
    bool LandedDetected = false;
    bool UprightDetected = false;


    public:

    const float altitudeThreshold = 10.0f; // Altitude threshold for state transitions (in meters)

    explicit StateMachine(IMU &imuRef) : imu(imuRef) {}

    StateMachine() = default;

    void updateState(float altitude, float pitch, float yaw, bool launchDetected, bool landedDetected, bool uprightDetected);
    
    bool isLaunchDetected();

    
};

#endif // STATEMACHINE_H