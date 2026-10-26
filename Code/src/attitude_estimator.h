#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <math.h>



class Quaternion {

    private:

    float scalar; // constant term
    float q1, q2, q3;

    public:

    Quaternion(): scalar(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    Quaternion(float w, float x, float y, float z) : scalar(w), q1(x), q2(y), q3(z) {}

    float getscalar() const {
        return scalar;
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

    float getMagnitudeSquared() const {
        return scalar * scalar + q1 * q1 + q2 * q2 + q3 * q3;
    }

    Quaternion conjugate() const {
        return Quaternion(scalar, -q1, -q2, -q3);
    }

    Quaternion normalized() const {
        float magnitude = sqrtf(getMagnitudeSquared());
        if (magnitude <= 0.0f) {
            return Quaternion();
        }
        return *this * (1.0f / magnitude);
    }

    Quaternion operator+(const Quaternion &other) const {
        return Quaternion(scalar + other.scalar, q1 + other.q1, q2 + other.q2, q3 + other.q3);
    }

    Quaternion operator*(float k) const {
        return Quaternion(scalar * k, q1 * k, q2 * k, q3 * k);
    }

    // Hamilton product: applies rotation `other` after this one (in the body frame)
    Quaternion operator*(const Quaternion &other) const {
        return Quaternion(
            scalar * other.scalar - q1 * other.q1 - q2 * other.q2 - q3 * other.q3,
            scalar * other.q1 + q1 * other.scalar + q2 * other.q3 - q3 * other.q2,
            scalar * other.q2 - q1 * other.q3 + q2 * other.scalar + q3 * other.q1,
            scalar * other.q3 + q1 * other.q2 - q2 * other.q1 + q3 * other.scalar);
    }

};

// Fuses gyro and accelerometer into an orientation estimate.
// The gyro is integrated every update; the accelerometer slowly pulls the estimate
// back toward gravity, but only while it reads ~1 g. Under thrust the accelerometer
// measures the motor, not gravity, so call setAccelCorrection(false) at launch.
class MahonyFilter {

    private:

    float kp; // proportional gain
    float ki; // integral gain
    Quaternion q; // estimated orientation
    float eInt[3]; // integral error
    bool accelCorrection = true;

    public:

    MahonyFilter(float kp_, float ki_) : kp(kp_), ki(ki_), q(), eInt{0.0f, 0.0f, 0.0f} {}

    // Start from the tilt the accelerometer sees, so the filter doesn't need time to converge.
    void initFromAccel(float ax, float ay, float az);

    // Gyro in rad/s (bias removed), accel in m/s^2, dt in seconds.
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // Angles in degrees. pitch = about body y, yaw = about body x, roll = about body z.
    void getAngles(float &pitch, float &yaw, float &roll) const;

    Quaternion getQuaternion() const { return q; }

    void setAccelCorrection(bool enabled) { accelCorrection = enabled; }
    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; }

};


#endif // ATTITUDE_ESTIMATOR_H
