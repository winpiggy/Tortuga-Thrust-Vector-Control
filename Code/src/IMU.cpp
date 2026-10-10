#include "IMU.h"



bool IMU::begin(){

    Wire.begin();

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


void IMU::update(){
    imu.getEvent(&a, &g, &temp);
}


void IMU::getAcceleration(float &x, float &y, float &z){
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;
}


void IMU::getGyroscope(float &x, float &y, float &z){
    x = g.gyro.x;
    y = g.gyro.y;
    z = g.gyro.z;
}


float IMU::getTemperature(){
    return temp.temperature;
}
