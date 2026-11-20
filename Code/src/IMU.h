#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Arduino.h>
#include "pins.h"
#include <Adafruit_MPU6050.h>



class IMU {

    private:

    Adafruit_MPU6050 imu;
    sensors_event_t a, g, temp;

    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;


    public:

    IMU() = default;

    bool begin();

    // Averages the gyro while the rocket is still and subtracts it from later readings
    void calibrateGyro(int samples);

    // Reads the sensor once and caches accel/gyro/temp
    void update();
    void getAcceleration(float &x, float &y, float &z);   // m/s^2
    void getGyroscope(float &x, float &y, float &z);      // rad/s, bias removed
    float getTemperature();                               // deg C


};


#endif // IMU_H
