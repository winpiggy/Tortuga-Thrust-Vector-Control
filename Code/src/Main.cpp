#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "setup.h"
#include "flight_data.h"
#include "IMU.h"
#include "baro.h"
#include "attitude_estimator.h"
#include "statemachine.h"
#include "gimbal.h"
#include "TVC_Controller.h"
#include "SDlogger.h"
#include "Indicator.h"


// One instance of each module. Constructors only store settings;
// all hardware is started in setup() through begin().
IMU imu;
BME baro;
MahonyFilter estimator(mahonyKp, mahonyKi);
StateMachine stateMachine;
Gimbal gimbal(PitchPin, YawPin);
TVC_Controller controller(gimbal);
SDlogger logger(BUILTIN_SDCARD);
LED blueLED(BlueLED);
LED whiteLED(WhiteLED);
Buzzer buzzer(BuzzerPin);

FlightData data;
unsigned long lastLoopUs = 0;
unsigned long lastDebugMs = 0;


// A sensor needed for flight is missing: flash and beep forever.
static void fatalError(const char* message) {
    Serial.println(message);
    whiteLED.blink(100);
    buzzer.beep(100);
    while (true) {
        whiteLED.update();
        buzzer.update();
    }
}

static void onStateChange(State newState) {
    Serial.print("State -> ");
    Serial.println(stateName(newState));

    switch (newState) {
        case State::ARMED:
            blueLED.blink(250);
            break;

        case State::LAUNCH:
            // Under thrust the accelerometer no longer measures gravity: gyro only from here
            estimator.setAccelCorrection(false);
            controller.reset();
            blueLED.on();
            break;

        case State::DESCENT:
            controller.reset();   // centres the gimbal
            blueLED.blink(100);
            logger.flush();
            break;

        case State::LANDED:
            blueLED.off();
            buzzer.beep(500);     // recovery beacon
            logger.flush();
            break;

        default:
            break;
    }
}


void setup() {

    // Initialize serial
    Serial.begin(115200);

    // Initialize indicators
    blueLED.begin();
    whiteLED.begin();
    buzzer.begin();

    // Initialize I2C (pins 18/19 on Teensy 4.1, see pins.h). Shared by IMU and barometer.
    Wire.begin();
    Wire.setClock(400000);

    if (!imu.begin()) {
        fatalError("Failed to initialize IMU :(");
    }

    if (!baro.begin()) {
        fatalError("Failed to initialize BME280 :(");
    }

    gimbal.begin();

    // Flying without a log is allowed, but warn about it
    if (!logger.begin() || !logger.createLogFile()) {
        Serial.println("WARNING: logging disabled");
    } else {
        Serial.print("Logging to ");
        Serial.println(logger.getFileName());
    }

    // Rocket must be still while the gyro bias is measured
    whiteLED.on();
    imu.calibrateGyro(gyroCalibrationSamples);
    imu.update();
    imu.getAcceleration(data.ax, data.ay, data.az);
    estimator.initFromAccel(data.ax, data.ay, data.az);
    whiteLED.off();

    lastLoopUs = micros();
}

void loop(){

    blueLED.update();
    whiteLED.update();
    buzzer.update();

    // Run the control loop at a fixed rate
    unsigned long now = micros();
    if (now - lastLoopUs < loopPeriodUs) {
        return;
    }
    float dt = (now - lastLoopUs) / 1000000.0f;
    lastLoopUs = now;
    data.timeUs = now;

    // Read sensors
    imu.update();
    imu.getAcceleration(data.ax, data.ay, data.az);
    imu.getGyroscope(data.gx, data.gy, data.gz);
    baro.read(data.temperature, data.pressure, data.altitude);

    // Estimate attitude
    estimator.update(data.gx, data.gy, data.gz, data.ax, data.ay, data.az, dt);
    estimator.getAngles(data.pitch, data.yaw, data.roll);

    // Update state machine
    State previousState = stateMachine.getState();
    data.state = stateMachine.update(data);
    data.relativeAltitude = stateMachine.getRelativeAltitude();
    if (data.state != previousState) {
        onStateChange(data.state);
    }

    // Control actuators: only steer while the motor is burning / rocket is climbing
    if (data.state == State::LAUNCH || data.state == State::FLIGHT) {
        controller.update(data.pitch, data.yaw);
    }
    data.pitchCommand = controller.getPitchCommand();
    data.yawCommand = controller.getYawCommand();

    // Log to SD
    logger.logFlightData(data);

#ifdef TVC_DEBUG
    if (millis() - lastDebugMs >= debugPrintMs) {
        lastDebugMs = millis();
        Serial.printf("%-7s pitch %6.2f yaw %6.2f roll %7.2f | alt %6.2f | cmd %5.2f %5.2f\n",
            stateName(data.state), data.pitch, data.yaw, data.roll,
            data.relativeAltitude, data.pitchCommand, data.yawCommand);
    }
#endif

}
