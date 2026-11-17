#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "pins.h"
#include "IMU.h"
#include "statemachine.h"



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

class MahonyFilter {

    private:

    float kp; // proportional gain
    float ki; // integral gain
    Quaternion q; // estimated orientation
    float eInt[3]; // integral error

    public:


    IMU& imu; // reference to the IMU instance

     Quaternion getQuaternion() const {
        return q;
    }

    // Find the angle from the quaternion
    float roll, pitch, yaw;

    float getEulers(float roll, float pitch);


    roll  = atan2(2 * (q.getscalar()*q.getq1() + q.getq2()*q.getq3()), 1 - 2 * (q.getq1()*q.getq1() + q.getq2()*q.getq2()));
    pitch = asin (2 * (q.getscalar()*q.getq2() - q.getq3()*q.getq1()));
    yaw   = atan2(2 * (q.getscalar()*q.getq3() + q.getq1()*q.getq2()), 1 - 2 * (q.getq2()*q.getq2() + q.getq3()*q.getq3()));

};


#endif // ATTITUDE_ESTIMATOR_H