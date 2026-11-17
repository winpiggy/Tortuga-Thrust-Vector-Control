#include "pins.h"
#include "attitude_estimator.h"
#include "IMU.h"
#include "statemachine.h"

MahonyFilter::MahonyFilter(IMU& imu) : imu(imu) {
    // Initialize the filter state
    q0 = 1.0f; // Quaternion representing no rotation
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void MahonyFilter::getEulers(float& roll, float& pitch, float& yaw) {
    // Calculate the angle based on roll and pitch
    float ax, ay, az;
    imu.getAcceleration(ax, ay, az);


constexpr float deg2rad = 180.0f / 3.14159265358979323846f;
    roll = std::atan2(ay, az) * deg2rad;
    pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * deg2rad;
    yaw = 0.0f; // Placeholder - implement yaw calculation if needed
}