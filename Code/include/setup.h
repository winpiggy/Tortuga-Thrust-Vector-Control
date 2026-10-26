#ifndef SETUP_H
#define SETUP_H

// Setup configs for full system
// constexpr at namespace scope has internal linkage, so these are safe to include anywhere.

constexpr float gravity = 9.81f; // m/s^2

// ---------- Main loop ----------
constexpr unsigned long loopPeriodUs = 5000;   // 200 Hz control loop
constexpr unsigned long logFlushMs = 500;      // how often buffered log data is written to the card
constexpr unsigned long debugPrintMs = 100;    // serial debug output rate (TVC_DEBUG builds)

// ---------- Barometer (BME280) ----------
constexpr float seaLevelPressure = 1013.25f;   // hPa
constexpr float groundAltitudeSmoothing = 0.01f; // low-pass factor for the ground reference while on the pad

// ---------- State machine ----------
// Sample counts assume the 200 Hz loop above.
constexpr float launchThreshold = 2 * gravity; // m/s^2 along body z, threshold for detecting launch
constexpr int   launchSamples = 10;            // 50 ms above threshold
constexpr float altitudeThreshold = 10.0f;     // m above ground, LAUNCH -> FLIGHT
constexpr float apogeeDrop = 2.0f;             // m below max altitude, FLIGHT -> DESCENT
constexpr int   apogeeSamples = 20;            // 100 ms below (max - apogeeDrop)
constexpr float landedAltitude = 5.0f;         // m above ground
constexpr float landedAccelTolerance = 0.5f;   // m/s^2 away from 1 g
constexpr int   landedSamples = 400;           // 2 s still and near the ground
constexpr float uprightTolerance = 5.0f;       // deg of tilt allowed to arm
constexpr int   uprightSamples = 200;          // 1 s upright

// ---------- Attitude estimator (Mahony) ----------
constexpr float mahonyKp = 1.0f;
constexpr float mahonyKi = 0.005f;
constexpr int   gyroCalibrationSamples = 500;  // ~1 s, rocket must be still

// ---------- TVC PID (placeholder gains, tune on the bench) ----------
constexpr float pitchKp = 0.5f;
constexpr float pitchKi = 0.05f;
constexpr float pitchKd = 0.1f;
constexpr float yawKp = 0.5f;
constexpr float yawKi = 0.05f;
constexpr float yawKd = 0.1f;

// ---------- Gimbal / servos ----------
constexpr float maxGimbalAngle = 5.0f;         // deg of motor deflection either side of centre
constexpr float servoDegPerGimbalDeg = 1.0f;   // linkage ratio, measure on the hardware
constexpr float servoUsPerDeg = 10.0f;         // pulse width change per servo degree
constexpr int   pitchServoCenterUs = 1500;     // pulse width with the motor straight, calibrate
constexpr int   yawServoCenterUs = 1500;
constexpr int   pitchDirection = 1;            // flip to -1 if the servo moves the wrong way
constexpr int   yawDirection = 1;

#endif // SETUP_H
