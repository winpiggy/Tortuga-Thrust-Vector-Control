#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Arduino.h>
#include "pins.h"
#include <Adafruit_MPU6050.h>



class IMU {


    public:
    IMU() = default;

    bool begin();

    // Reads the sensor once and caches accel/gyro/temp
    void update();
    void getAcceleration(float &x, float &y, float &z);   // m/s^2
    void getGyroscope(float &x, float &y, float &z);      // rad/s
    float getTemperature();                               // deg C

    private:

    Adafruit_MPU6050 imu;
    sensors_event_t a, g, temp; 

};


#endif // IMU_H
