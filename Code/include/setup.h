#ifndef SETUP_H
#define SETUP_H

// Setup configs for full system

// Constants for the BME280 sensor
static const float seaLevelPressure = 1013.25f; // hPa

// Constants for the state machine

static const float launchThreshold = 2 * 9.81f; // m/s^2, threshold for detecting launch
static const float altitudeThreshold = 10.0f; // m, threshold for transitioning to FLIGHT

// Servo constraints
    static const float min_pitch = 0.0f;
    static const float max_pitch = 45.0f;
    static const float min_yaw = 0.0f;
    static const float max_yaw = 45.0f;

#endif // SETUP_H