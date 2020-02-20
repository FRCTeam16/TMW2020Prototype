#pragma once

#include "Autonomous/Step.h"
#include <units/units.h>

class AckermanDriveToAngle : public Step {
 public:
  AckermanDriveToAngle(units::degree_t steerAngle, double speed, double ackermanA, units::degree_t target_angle, 
    units::degree_t allowedError = 3.0_s, units::second_t timeout = 5.0_s)
  : steerAngle(steerAngle), speed(speed), ackermanA(ackermanA), target(target_angle), allowedError(allowedError), timeout(timeout)
  {
        manualDriveControl = true;  // we are controlling drive
  }
  bool Run(std::shared_ptr<World> world) override;
 private:
  const units::degree_t steerAngle;
  const double speed;
  const double ackermanA;
  const units::degree_t target;
  const units::degree_t allowedError;
  const units::second_t timeout;
  units::second_t startTime{-1};
};
