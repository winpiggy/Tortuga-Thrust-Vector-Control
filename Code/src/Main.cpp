#include "pins.h"
#include "setup.h"
#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include "baro.h"
#include "attitude_estimator.h"
#include "statemachine.h"
#include "PID.h"
#include "TVC_Controller.h"
#include "SDlogger.h"
#include "Indicator.h"

// ---------- Hardware objects ----------
// These are global so they exist for the whole program. (Objects created
// inside setup() are destroyed when setup() returns, so loop() can't use them.)
IMU imu;
BME baro;
MahonyFilter attitude(imu);
StateMachine stateMachine(imu);
ServoController servos;
SDlogger logger;
LED blueLED(BlueLED);
LED whiteLED(WhiteLED);
Buzzer buzzer(BuzzerPin);

// PID output is a gimbal angle, so its limit is the gimbal's travel
PID pitchPID(pitch_kp, pitch_ki, pitch_kd, max_pitch);
PID yawPID(yaw_kp, yaw_ki, yaw_kd, max_yaw);

// ---------- Loop state ----------
unsigned long lastLoopMicros = 0;
unsigned long lastLandedBeep = 0;
float groundAltitude = 0.0f;   // pad altitude (m above sea level), tracked until launch
State lastState = State::IDLE;

// Startup failed: beep and blink forever so the problem is obvious on the pad
void fail(const char* message) {
    Serial.println(message);
    servos.centerServos(0.0f, 0.0f);
    while (true) {
        digitalWrite(WhiteLED, HIGH);
        digitalWrite(BuzzerPin, HIGH);
        delay(100);
        digitalWrite(WhiteLED, LOW);
        digitalWrite(BuzzerPin, LOW);
        delay(100);
    }
}

// Indicator pattern for each state
void showState(State state) {
    switch (state) {
        case State::IDLE:    blueLED.blink(1000); break;
        case State::ARMED:   blueLED.blink(200); buzzer.beep(300); break;
        case State::LAUNCH:
        case State::FLIGHT:
        case State::DESCENT: blueLED.on(); break;
        case State::LANDED:  blueLED.blink(1000); break;
    }
}

void setup() {

    // Initialize serial
    Serial.begin(115200);

    // Indicators first, so they can report any failure below
    blueLED.begin();
    whiteLED.begin();
    buzzer.begin();
    whiteLED.on(); // white = starting up

    // Servos straight before anything else moves
    servos.begin();
    servos.centerServos(0.0f, 0.0f);

    // I2C (both sensors share the bus); 400 kHz keeps sensor reads short
    Wire.begin();
    Wire.setClock(400000);

    if (!imu.begin()) {
        fail("Failed to initialize IMU :(");
    }
    if (!baro.begin()) {
        fail("Failed to initialize BME280 :(");
    }
    if (!logger.begin() || logger.createLogFile("FLIGHT") != 0) {
        fail("Failed to start SD logging :(");
    }

    // Rocket must be sitting still for this (about 1 second)
    imu.calibrateGyro(gyroCalibrationSamples);
    attitude.setGains(mahonyKpGround, mahonyKiGround);

    // Starting pad altitude: average a few barometer readings
    delay(100); // let the BME280 produce its first samples
    float temperature, pressure, altitude, sum = 0.0f;
    for (int i = 0; i < 10; i++) {
        baro.read(temperature, pressure, altitude);
        sum += altitude;
        delay(20);
    }
    groundAltitude = sum / 10.0f;

#ifdef BENCH_MODE
    // Sweep the gimbal to its limits so you can check direction and travel
    servos.testServos(max_pitch, max_yaw, 500);
    servos.testServos(min_pitch, min_yaw, 500);
#endif

    logger.logData("time_ms,state,alt_agl_m,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,pitch_cmd_deg,yaw_cmd_deg");

    whiteLED.off();
    showState(stateMachine.getState());
    buzzer.beep(100);
    lastLoopMicros = micros();
}

void loop(){

    // ---------- Fixed-rate timing ----------
    unsigned long now = micros();
    if (now - lastLoopMicros < loopPeriodMicros) {
        blueLED.update();
        buzzer.update();
        return;
    }
    float dt = (now - lastLoopMicros) / 1000000.0f;
    lastLoopMicros = now;

    // ---------- Sensors ----------
    imu.update();
    float temperature, pressure, altitude;
    baro.read(temperature, pressure, altitude);

    // Track the pad altitude until launch so slow weather/pressure drift
    // while waiting on the pad doesn't look like a climb or fall
    State state = stateMachine.getState();
    if (state == State::IDLE || state == State::ARMED) {
        groundAltitude += groundAltitudeFilter * (altitude - groundAltitude);
    }
    float altitudeAGL = altitude - groundAltitude;

    // ---------- State machine ----------
    stateMachine.update(altitudeAGL);
    state = stateMachine.getState();

    if (state != lastState) {
        if (state == State::LAUNCH) {
            // Under thrust the accelerometer no longer measures gravity,
            // so stop using it to correct the attitude estimate
            attitude.setGains(mahonyKpFlight, mahonyKiFlight);
            pitchPID.reset();
            yawPID.reset();
        }
        showState(state);
        logger.flush(); // make sure every state change reaches the card
        lastState = state;
    }

    // ---------- Attitude ----------
    attitude.update(dt);
    float roll, pitch, yaw;
    attitude.getEulers(roll, pitch, yaw);

    // ---------- Gimbal control ----------
    // Steer only while the motor can be burning. The pitch servo corrects tilt
    // about the y axis and the yaw servo corrects tilt about the x axis.
    // CHECK ON THE BENCH: which servo moves for which tilt, and the direction,
    // depends on how the IMU and servos are mounted (flip with pitch_ratio/yaw_ratio).
    bool controlActive = (state == State::LAUNCH || state == State::FLIGHT);
#ifdef BENCH_MODE
    controlActive = true; // tilt the rocket by hand and watch the gimbal respond
#endif

    float pitchCmd = 0.0f;
    float yawCmd = 0.0f;
    if (controlActive) {
        pitchCmd = pitchPID.update(pitch);
        yawCmd = yawPID.update(roll);
    }
    servos.setPitch(pitchCmd);
    servos.setYaw(yawCmd);

    // ---------- Logging ----------
    float ax, ay, az, gx, gy, gz;
    imu.getAcceleration(ax, ay, az);
    imu.getGyroscope(gx, gy, gz);

    char line[200];
    snprintf(line, sizeof(line),
             "%lu,%s,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f",
             millis(), stateName(state), altitudeAGL, ax, ay, az, gx, gy, gz,
             roll, pitch, yaw, pitchCmd, yawCmd);
    logger.logData(line);

#ifdef TVC_DEBUG
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.println(line);
    }
#endif

    // ---------- Indicators ----------
    // Once landed, chirp every 2 s to help find the rocket
    if (state == State::LANDED && millis() - lastLandedBeep >= 2000) {
        lastLandedBeep = millis();
        buzzer.beep(200);
    }
    blueLED.update();
    buzzer.update();
}
