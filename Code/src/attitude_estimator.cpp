#include "pins.h"
#include "attitude_estimator.h"
#include "IMU.h"
#include <cmath>

static constexpr float rad2deg = 180.0f / 3.14159265358979323846f;

MahonyFilter::MahonyFilter(IMU& imu) : imu(imu) {
    // q defaults to the identity quaternion (no rotation); eInt starts at zero
}

void MahonyFilter::setGains(float kp, float ki) {
    this->kp = kp;
    this->ki = ki;
    if (ki == 0.0f) {
        eInt[0] = eInt[1] = eInt[2] = 0.0f;
    }
}

void MahonyFilter::update(float dt) {
    float ax, ay, az, gx, gy, gz;
    imu.getAcceleration(ax, ay, az);
    imu.getGyroscope(gx, gy, gz);

    float w = q.getscalar();
    float x = q.getq1();
    float y = q.getq2();
    float z = q.getq3();

    // Accelerometer correction. Only trusted when it reads close to 1 g, i.e.
    // it is measuring gravity and not thrust or a bump.
    float aNorm = std::sqrt(ax * ax + ay * ay + az * az);
    bool accelIsGravity = std::fabs(aNorm - 9.81f) < 1.0f;
    if ((kp > 0.0f || ki > 0.0f) && accelIsGravity) {
        ax /= aNorm;
        ay /= aNorm;
        az /= aNorm;

        // Which way the current estimate thinks "up" is, in the sensor frame
        float vx = 2.0f * (x * z - w * y);
        float vy = 2.0f * (w * x + y * z);
        float vz = w * w - x * x - y * y + z * z;

        // Error = rotation needed to line the estimate up with the measured gravity
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        if (ki > 0.0f) {
            eInt[0] += ex * dt;
            eInt[1] += ey * dt;
            eInt[2] += ez * dt;
            gx += ki * eInt[0];
            gy += ki * eInt[1];
            gz += ki * eInt[2];
        }

        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;
    }

    // Integrate the rotation rate: q_dot = 0.5 * q * (0, gx, gy, gz)
    float dw = 0.5f * (-x * gx - y * gy - z * gz);
    float dx = 0.5f * ( w * gx + y * gz - z * gy);
    float dy = 0.5f * ( w * gy - x * gz + z * gx);
    float dz = 0.5f * ( w * gz + x * gy - y * gx);

    w += dw * dt;
    x += dx * dt;
    y += dy * dt;
    z += dz * dt;

    // Keep it a unit quaternion so rounding errors don't build up
    float n = std::sqrt(w * w + x * x + y * y + z * z);
    q = Quaternion(w / n, x / n, y / n, z / n);
}

void MahonyFilter::getEulers(float& roll, float& pitch, float& yaw) const {
    const float w = q.getscalar();
    const float x = q.getq1();
    const float y = q.getq2();
    const float z = q.getq3();

    // Clamp so rounding can't push asin outside [-1, 1] and return NaN
    float sinp = 2 * (w * y - z * x);
    sinp = sinp > 1.0f ? 1.0f : (sinp < -1.0f ? -1.0f : sinp);

    roll  = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * rad2deg;
    pitch = std::asin(sinp) * rad2deg;
    yaw   = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * rad2deg;
}
