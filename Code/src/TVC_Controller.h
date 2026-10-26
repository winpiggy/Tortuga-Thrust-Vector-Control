#ifndef TVC_CONTROLLER_H
#define TVC_CONTROLLER_H

#include "PID.h"
#include "gimbal.h"

// Two PIDs (pitch and yaw) driving the measured tilt toward 0 by deflecting the gimbal.
class TVC_Controller {

    private:

    PID pitchPID;
    PID yawPID;
    Gimbal &gimbal;

    public:

    explicit TVC_Controller(Gimbal &gimbalRef);

    // Measured angles in degrees, from the attitude estimator.
    void update(float pitch, float yaw);

    // Centres the gimbal and clears the PID state.
    void reset();

    float getPitchCommand() const { return gimbal.getPitch(); }
    float getYawCommand() const { return gimbal.getYaw(); }

};

#endif // TVC_CONTROLLER_H
