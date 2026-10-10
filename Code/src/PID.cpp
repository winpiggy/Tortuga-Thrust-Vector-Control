#include "PID.h"
#include "TVC_Controller.h"

PID::PID(float kp, float ki, float kd, float saturation) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->saturation = saturation;   // max magnitude of the output (+/-)
}

// Call this once per control loop with the latest measurement (e.g. pitch angle).
// Returns the correction to send to the actuator, limited to +/- saturation.
float PID::update(float input) {

    currentTime = millis();

    // First call: we have no previous timestamp or measurement, so we can't
    // compute a time step or a derivative yet. Just record the starting values
    // and output nothing. Without this, deltaTime would be the entire uptime
    // and the integral/derivative would spike.
    if (!initialized) {
        lastTime = currentTime;
        lastInput = input;
        initialized = true;
        return 0.0f;
    }

    // Time since the last update, in seconds.
    deltaTime = (currentTime - lastTime) / 1000.0f;

    // If called twice within the same millisecond, deltaTime is 0 and the
    // derivative would divide by zero. Reuse the previous output instead.
    if (deltaTime <= 0.0f) {
        return lastOutput;
    }
    lastTime = currentTime;

    // Error = how far the measurement is from where we want it.
    error = setpoint - input;

    // I term: accumulate error over time (area under the error curve).
    // This removes steady offsets that P alone can't fix.
    errorIntegral += error * deltaTime;

    // Anti-windup: if the output is stuck at its limit, the integral would keep
    // growing and cause a big overshoot later. Limit it so the I term alone
    // (ki * errorIntegral) can never exceed the output limit.
    // The ki > 0 check avoids dividing by zero.
    if (ki > 0.0f) {
        errorIntegral = constrain(errorIntegral, -saturation / ki, saturation / ki);
    }

    // D term: rate of change of the MEASUREMENT (not the error). The minus sign
    // makes it oppose motion, same as differentiating error when setpoint is
    // constant. Using the measurement avoids a spike when the setpoint changes.
    // Must be computed BEFORE lastInput is updated.
    errorDerivative = -(input - lastInput) / deltaTime;
    lastInput = input;

    // Limit the D term so a noisy sensor spike can't dominate the output.
    if (kd > 0.0f) {
        errorDerivative = constrain(errorDerivative, -saturation / kd, saturation / kd);
    }

    // Combine the three terms:
    //   P: reacts to the current error
    //   I: reacts to accumulated past error
    //   D: reacts to how fast the measurement is changing (damping)
    float output = kp * error + ki * errorIntegral + kd * errorDerivative;

    // Final clamp to the actuator's allowed range; remember it for the
    // deltaTime == 0 case above.
    lastOutput = constrain(output, -saturation, saturation);
    return lastOutput;
}
