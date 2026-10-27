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

   

    public:



    explicit StateMachine(IMU &imuRef) : imu(imuRef) {}

    StateMachine() = default;

    void updateState(float altitude, float pitch, float yaw, bool launchDetected, bool landedDetected, bool uprightDetected);
    
    bool isLaunchDetected();
    bool isLandedDetected();
    bool isUprightDetected();


    
};

#endif // STATEMACHINE_H