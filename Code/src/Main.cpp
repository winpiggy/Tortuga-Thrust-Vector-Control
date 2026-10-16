#include "pins.h"
#include "setup.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include "IMU.h"
#include "baro.h"


void setup() {

    // Initialize serial
    Serial.begin(115200);

    // Initialize pins
    pinMode(BlueLED, OUTPUT);
    pinMode(WhiteLED, OUTPUT);
    pinMode(BuzzerPin, OUTPUT);

    pinMode(PitchPin, OUTPUT);
    pinMode(YawPin, OUTPUT);

    pinMode(SCLpin, OUTPUT);
    pinMode(SDApin, OUTPUT);

    //Initialize I2C
    Wire.begin();

    //Initialize IMU
    Adafruit_MPU6050 imu;  // Create an instance of the IMU

    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU :(");
        while (1) {
            // Do nothing
        }
    }

     Adafruit_BME280 bme;  // Create an instance of the BME280 sensor

    //Initialize Barometer
    if (!bme.begin()) {
        Serial.println("Failed to initialize BME280 :(");
        while (1) {
            // Do nothing
        }
    }

}

void loop(){

    // Main code loop
    // Read sensor data, update state machine, control actuators, etc.
    
}