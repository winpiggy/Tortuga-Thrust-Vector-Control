#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

// Shared data passed between the modules each loop.
// Body axes: z points out the nose (up on the pad), x and y are the gimbal axes.

enum class State {
    IDLE,
    ARMED,
    LAUNCH,
    FLIGHT,
    DESCENT,
    LANDED
};

inline const char* stateName(State state) {
    switch (state) {
        case State::IDLE:    return "IDLE";
        case State::ARMED:   return "ARMED";
        case State::LAUNCH:  return "LAUNCH";
        case State::FLIGHT:  return "FLIGHT";
        case State::DESCENT: return "DESCENT";
        case State::LANDED:  return "LANDED";
    }
    return "?";
}

struct FlightData {
    unsigned long timeUs = 0;

    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // m/s^2, body frame
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // rad/s, gyro bias removed

    float pitch = 0.0f;   // deg, rotation about body y
    float yaw = 0.0f;     // deg, rotation about body x
    float roll = 0.0f;    // deg, rotation about body z (not controllable by the gimbal)

    float temperature = 0.0f;       // deg C
    float pressure = 0.0f;          // Pa
    float altitude = 0.0f;          // m, relative to seaLevelPressure
    float relativeAltitude = 0.0f;  // m above the pad

    float pitchCommand = 0.0f;      // deg of gimbal deflection
    float yawCommand = 0.0f;

    State state = State::IDLE;
};

#endif // FLIGHT_DATA_H
