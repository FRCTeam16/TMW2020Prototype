#pragma once

#include "Autonomous/Step.h"

class AlignToTarget : public Step {
 public:
  AlignToTarget(double _angle, double _target, double _timeout, int _scansToHold=0);
  bool Run(std::shared_ptr<World> world) override;
 private:
  const double angle;
  const double target;
  const double timeout;
  const double scansToHold;
  double startTime = -1;
  double scanCount = 0;
};
