#include "pins.h"
#include "statemachine.h"
#include "IMU.h"
#include "setup.h"

void StateMachine::updateState(float altitude, float pitch, float yaw, bool launchDetected, bool landedDetected, bool uprightDetected) {
    // Update the state based on the current conditions
    switch(currentState) {
        case State::IDLE:
            if (uprightDetected) {
                currentState = State::ARMED;
            }
            break;

        case State::ARMED:
            if (launchDetected) {
                currentState = State::LAUNCH;
            }
            break;

        case State::LAUNCH:
            if (altitude > altitudeThreshold) {
                currentState = State::FLIGHT;
            }
            break;

        case State::FLIGHT:
            if (landedDetected) {
                currentState = State::DESCENT;
            }
            break;

        case State::DESCENT:
            if (landedDetected) {
                currentState = State::LANDED;
            }
            break;

        case State::LANDED:
            // Remain in LANDED state until reset or new launch
            break;
    }
}

bool StateMachine::isLaunchDetected() {
    float ax, ay, az;
    imu.getAcceleration(ax, ay, az);   // fills ax/ay/az
    if (currentState == State::ARMED && az > launchThreshold) {
        return true;
    } else {
        return false;
    }
};

bool StateMachine::isUprightDetected() {
    float ax, ay, az;
    imu.getAcceleration(ax, ay, az);  
    // Check if the rocket is upright based on the accelerometer readings
    if (currentState == State::IDLE && ax < 0.1f && ax > -0.1f && ay < 0.1f && ay > -0.1f) { // Assuming z-axis points upwards
        return true;
    } else {
        return false;
    }
};

