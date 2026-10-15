#ifndef PID_H
#define PID_H
#include <iostream>
#include <algorithm>
#include <Arduino.h>
#include "pins.h"
#include <time.h>
#include "Setup.h"

class PID{

    private:
    
    static const float kp = 2.0f;
    static const float ki = 0.5f;
    static const float kd = 1.0f;
    static const float saturation = 45.0f;

    float setpoint = 0.0f;          // target value the PID drives the input toward
    float error = 0.0f;             // setpoint - input
    float saturation = 0.0f;        // output is limited to +/- this value
    float deltaTime = 0.0f;         // seconds since last update
    float errorIntegral;     // accumulated error over time (I term state)
    float errorDerivative;   // rate of change of the input (D term)

    float lastInput = 0.0f;         // previous measurement, for the derivative
    float lastOutput = 0.0f;        // previous output, reused if deltaTime == 0

    unsigned long currentTime = 0;  // millis() timestamps
    unsigned long lastTime = 0;

    bool initialized = false;       // false until the first update() call

    public:

    PID(float kp, float ki, float kd, float saturation);
    float update(float input);


};



#endif // PID_H