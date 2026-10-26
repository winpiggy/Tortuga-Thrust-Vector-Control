#include "IMU.h"



bool IMU::begin(){

    // 0x68 if AD0 is low, 0x69 if AD0 is high
    if (!imu.begin(0x68, &Wire)) {
        Serial.println("Failed to initialize IMU");
        return false;
    }

    imu.setAccelerometerRange(MPU6050_RANGE_16_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    return true;

}


void IMU::calibrateGyro(int samples){

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < samples; i++) {
        imu.getEvent(&a, &g, &temp);
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(2);
    }

    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;
}


void IMU::update(){
    imu.getEvent(&a, &g, &temp);
}


void IMU::getAcceleration(float &x, float &y, float &z) const {
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;
}


void IMU::getGyroscope(float &x, float &y, float &z) const {
    x = g.gyro.x - gyroBiasX;
    y = g.gyro.y - gyroBiasY;
    z = g.gyro.z - gyroBiasZ;
}


float IMU::getTemperature() const {
    return temp.temperature;
}
