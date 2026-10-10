#ifndef PID_H
#define PID_H
#include <iostream>
#include <algorithm>
#include <Arduino.h>
#include "pins.h"
#include <time.h>

class PID{

    private:

    // Tunable gains

    float kp, ki, kd;

    // Servo constraints

    float min_pitch = 0.0f;
    float max_pitch = 45.0f;
    float min_yaw = 0.0f;
    float max_yaw = 45.0f;

    // More variables

    float setpoint = 0.0f;          // target value the PID drives the input toward
    float error = 0.0f;             // setpoint - input
    float saturation = 0.0f;        // output is limited to +/- this value
    float deltaTime = 0.0f;         // seconds since last update
    float errorIntegral = 0.0f;     // accumulated error over time (I term state)
    float errorDerivative = 0.0f;   // rate of change of the input (D term)

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