#ifndef SETUP_H
#define SETUP_H

// Setup configs for full system

// ---------- Main loop ----------
static const unsigned long loopPeriodMicros = 10000; // 100 Hz control loop

// ---------- Barometer (BME280) ----------
static const float seaLevelPressure = 1013.25f; // hPa
static const float groundAltitudeFilter = 0.05f; // 0..1, how fast the ground reference tracks while on the pad

// ---------- IMU ----------
static const int gyroCalibrationSamples = 500; // rocket must be still during startup calibration

// ---------- Attitude filter (Mahony) ----------
// On the pad the accelerometer only sees gravity, so it is used to correct gyro drift.
// After launch it sees thrust and drag, so the correction is switched off (gains = 0).
static const float mahonyKpGround = 2.0f;
static const float mahonyKiGround = 0.0f;
static const float mahonyKpFlight = 0.0f;
static const float mahonyKiFlight = 0.0f;

// ---------- State machine ----------
static const float launchThreshold = 2 * 9.81f;     // m/s^2 on the z axis, threshold for detecting launch
static const unsigned long launchHoldMs = 50;       // launch accel must last this long (rejects bumps)
static const float altitudeThreshold = 10.0f;       // m above pad, LAUNCH -> FLIGHT
static const float uprightToleranceDeg = 5.0f;      // max tilt from vertical to count as upright
static const unsigned long uprightHoldMs = 2000;    // must stay upright this long to arm
static const float apogeeDrop = 2.0f;               // m below max altitude, FLIGHT -> DESCENT
static const float landedAltitudeBand = 1.0f;       // m, altitude must stay within this band...
static const unsigned long landedHoldMs = 3000;     // ...for this long, DESCENT -> LANDED

// ---------- Servo / gimbal ----------
// Gimbal angles are deflections from straight, in degrees.
// PLACEHOLDER limits: measure how far your gimbal can move before it binds.
static const float min_pitch = -10.0f;
static const float max_pitch = 10.0f;
static const float min_yaw = -10.0f;
static const float max_yaw = 10.0f;
static const float servo_center = 90.0f;  // servo command (deg) that puts the gimbal straight
static const float pitch_offset = 0.0f;   // degrees, trim for pitch servo
static const float yaw_offset = 0.0f;     // degrees, trim for yaw servo
static const float pitch_ratio = 1.0f;    // servo degrees per gimbal degree (negative flips direction)
static const float yaw_ratio = 1.0f;      // servo degrees per gimbal degree (negative flips direction)
// Linkage geometry (not used yet)
static const float linkage_length = 0.05f;    // meters, linkage from servo horn to pivot point
static const float servo_horn_radius = 0.02f; // meters, radius of the servo horn

// ---------- PID ----------
// PLACEHOLDER gains: these are untuned guesses. Tune them in simulation before flying.
static const float pitch_kp = 0.6f;
static const float pitch_ki = 0.1f;
static const float pitch_kd = 0.15f;
static const float yaw_kp = 0.6f;
static const float yaw_ki = 0.1f;
static const float yaw_kd = 0.15f;

// ---------- Logging ----------
static const unsigned long logFlushMs = 1000; // write buffered log data to the card this often

#endif // SETUP_H
