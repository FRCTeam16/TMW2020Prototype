#pragma once

#include "Autonomous/Step.h"

class DriveToTarget : public Step {
public:
  DriveToTarget(double angle, double yspeed, double targetArea, double timeout, double maxTargetArea=100.0);
  bool Run(std::shared_ptr<World> world) override;
  void SetFieldCentric(bool fieldCentric_);
private:
  double startTime = -1;
  const double angle;
  const double yspeed;
  const double targetArea;
  const double maxTargetArea;
  const double timeout;
  bool fieldCentric = false;
};
