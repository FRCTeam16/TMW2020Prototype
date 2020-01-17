#pragma once
#include "Autonomous/Step.h"
#include "Robot.h"

class SetVisionOutputRange : public Step {
 public:
  SetVisionOutputRange(double _range) : range(_range) {}

  bool Run(std::shared_ptr<World> world) {
    std::cout << "SetVisionOutputRange(" << range << ")\n";
    Robot::visionSystem->SetMaxOutputRange(range);
    return true;
  }
 private:
  const double range;
};
