#include <Arduino.h>
#include "attitude_estimator.h"
#include "setup.h"

void MahonyFilter::initFromAccel(float ax, float ay, float az) {

    // Tilt from gravity alone (only valid while stationary). Rotation about z can't be
    // seen by the accelerometer, so it starts at 0.
    float yawX = atan2f(ay, az);
    float pitchY = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cx = cosf(yawX / 2.0f), sx = sinf(yawX / 2.0f);
    float cy = cosf(pitchY / 2.0f), sy = sinf(pitchY / 2.0f);

    q = Quaternion(cx * cy, sx * cy, cx * sy, -sx * sy).normalized();
    eInt[0] = eInt[1] = eInt[2] = 0.0f;
}

void MahonyFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {

    float norm = sqrtf(ax * ax + ay * ay + az * az);

    // Only trust the accelerometer as a gravity reference when it reads close to 1 g.
    if (accelCorrection && norm > 0.85f * gravity && norm < 1.15f * gravity) {

        ax /= norm;
        ay /= norm;
        az /= norm;

        float w = q.getscalar(), x = q.getq1(), y = q.getq2(), z = q.getq3();

        // Direction of "up" predicted by the current estimate, in the body frame
        float vx = 2.0f * (x * z - w * y);
        float vy = 2.0f * (w * x + y * z);
        float vz = w * w - x * x - y * y + z * z;

        // Error = cross product of measured and predicted up (axis * sin(angle) between them)
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
    Quaternion qDot = q * Quaternion(0.0f, gx, gy, gz) * 0.5f;
    q = (q + qDot * dt).normalized();
}

void MahonyFilter::getAngles(float &pitch, float &yaw, float &roll) const {

    float w = q.getscalar(), x = q.getq1(), y = q.getq2(), z = q.getq3();

    yaw = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
    pitch = asinf(constrain(2.0f * (w * y - z * x), -1.0f, 1.0f)) * RAD_TO_DEG;
    roll = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
}
