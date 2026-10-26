#include "TVC_Controller.h"
#include "setup.h"

TVC_Controller::TVC_Controller(Gimbal &gimbalRef)
    : pitchPID(pitchKp, pitchKi, pitchKd, maxGimbalAngle),
      yawPID(yawKp, yawKi, yawKd, maxGimbalAngle),
      gimbal(gimbalRef) {}

void TVC_Controller::update(float pitch, float yaw) {
    // Setpoints are 0 (straight up). PID outputs are already limited to +/- maxGimbalAngle.
    gimbal.setPitch(pitchPID.update(pitch));
    gimbal.setYaw(yawPID.update(yaw));
}

void TVC_Controller::reset() {
    pitchPID.reset();
    yawPID.reset();
    gimbal.center();
}
