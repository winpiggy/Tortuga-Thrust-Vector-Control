#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "pins.h"
#include "IMU.h"



class Quaternion {

    private:

    float scalar; // constant term
    float q1, q2, q3;

    public:

    Quaternion(): scalar(1.0), q1(0.0), q2(0.0), q3(0.0) {}
    Quaternion(float w, float x, float y, float z) : scalar(w), q1(x), q2(y), q3(z) {}

    float getscalar() const {
        return scalar;
    }

    float getMagnitudeSquared() const {
        return scalar * scalar + q1 * q1 + q2 * q2 + q3 * q3;
    }

    float getq1() const {
        return q1;
    }

    float getq2() const {
        return q2;
    }

    float getq3() const {
        return q3;
    }

    Quaternion conjugate() const {
        return Quaternion(scalar, -q1, -q2, -q3);
    }


};

// Mahony filter: integrates the gyro to track orientation, and (when kp/ki > 0)
// nudges the estimate toward the gravity direction seen by the accelerometer
// to cancel gyro drift. Assumes the IMU z axis points up the rocket.
class MahonyFilter {

    private:

    IMU& imu; // reference to the IMU instance

    float kp = 0.0f; // proportional gain
    float ki = 0.0f; // integral gain
    Quaternion q; // estimated orientation
    float eInt[3] = {0.0f, 0.0f, 0.0f}; // integral error

    public:

    explicit MahonyFilter(IMU& imu);

    // Ground: accelerometer correction on. Flight: set both to 0.
    void setGains(float kp, float ki);

    // Call once per loop after imu.update(); dt in seconds
    void update(float dt);

    Quaternion getQuaternion() const {
        return q;
    }

    // Degrees. roll = rotation about x, pitch = rotation about y (these two are
    // the rocket's tilt), yaw = rotation about z (the rocket spinning on its axis).
    void getEulers(float& roll, float& pitch, float& yaw) const;

};


#endif // ATTITUDE_ESTIMATOR_H
