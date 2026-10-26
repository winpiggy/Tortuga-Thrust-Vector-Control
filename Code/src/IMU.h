#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>



class IMU {

    private:

    Adafruit_MPU6050 imu;
    sensors_event_t a, g, temp;

    float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;  // rad/s


    public:

    IMU() = default;

    // Wire.begin() must be called first (done once in setup()).
    bool begin();

    // Averages the gyro at rest to find its zero offset. Rocket must be still.
    void calibrateGyro(int samples);

    // Reads the sensor once and caches accel/gyro/temp
    void update();
    void getAcceleration(float &x, float &y, float &z) const;   // m/s^2
    void getGyroscope(float &x, float &y, float &z) const;      // rad/s, bias removed
    float getTemperature() const;                               // deg C

};


#endif // IMU_H
