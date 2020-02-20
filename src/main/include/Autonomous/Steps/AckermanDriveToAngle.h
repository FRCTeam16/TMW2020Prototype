#pragma once

#include "Autonomous/Step.h"
#include <units/units.h>

class AckermanDriveToAngle : public Step {
 public:
  AckermanDriveToAngle(double steerAngle, double speed, double ackermanA, double target_angle, 
    double allowedError = 3.0, units::second_t timeout = 5.0_s)
  : steerAngle(steerAngle), speed(speed), ackermanA(ackermanA), target(target_angle), allowedError(allowedError), timeout(timeout)
  {
        manualDriveControl = true;  // we are controlling drive
  }
  bool Run(std::shared_ptr<World> world) override;
 private:
  const double steerAngle;
  const double speed;
  const double ackermanA;
  const double target;
  const double allowedError;
  const units::second_t timeout;
  units::second_t startTime{-1};
};
