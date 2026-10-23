#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "pins.h"
#include "IMU.h"
#include "statemachine.h"



class AttitudeEstimator {
    
    private:
        IMU imu;
        StateMachine stateMachine;

        public:
        AttitudeEstimator() : imu(), stateMachine(imu) {}
        void update(float altitude, float pitch, float yaw, bool launchDetected, bool landedDetected, bool uprightDetected) {
            stateMachine.updateState(altitude, pitch, yaw, launchDetected, landedDetected, uprightDetected);
        }

        void getQuaternion(float &scalar, float &q1, float &q2, float &q3) {
            float x, y, z;
            float roll, pitch, yaw;
            imu.getAcceleration(x, y, z);
            imu.getGyroscope(roll, pitch, yaw);

        }

        void updateQuaternion(float &scalar, float &q1, float &q2, float &q3) {
            getQuaternion(scalar, q1, q2, q3);
        }


};



#endif // ATTITUDE_ESTIMATOR_H