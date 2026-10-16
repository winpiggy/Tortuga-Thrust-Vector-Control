#ifndef PID_H
#define PID_H
#include <iostream>
#include <algorithm>
#include <Arduino.h>
#include "pins.h"
#include <time.h>
#include "setup.h"

class PID{

    private:

    float kp = 0.0f;
    float ki = 0.0f;
    float  kd = 0.0f;          // PID coefficients
    float saturation = 0.0f;   // max magnitude of the output (+/-)

    float setpoint = 0.0f;          // target value the PID drives the input toward
    float error = 0.0f;         
    float deltaTime = 0.0f;         // seconds since last update
    float errorIntegral = 0.0f;     // accumulated error over time (I term state)
    float errorDerivative = 0.0f;   // rate of change of the input (D term)

    float lastInput = 0.0f;         // previous measurement, for the derivative
    float lastOutput = 0.0f;        // previous output, reused if deltaTime == 0

    unsigned long currentTime = 0;  // millis() timestamps
    unsigned long lastTime = 0;

    bool initialized = false;       // false until the first update() call

    public:

    float getkp() const { return kp; }
    float getki() const { return ki; }
    float getkd() const { return kd; }
    float getsaturation() const { return saturation; }

    float getSetpoint() const { return setpoint; }

    

    float getErrorIntegral() const { return errorIntegral; }
    float getErrorDerivative() const { return errorDerivative; }

    PID( float kp, float ki, float kd, float saturation);
    float update(float input);
    void setTunings(float kp, float ki, float kd, float saturation);


};



#endif // PID_H