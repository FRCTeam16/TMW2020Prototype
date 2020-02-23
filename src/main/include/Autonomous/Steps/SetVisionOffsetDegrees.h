#pragma once
#include "Autonomous/Step.h"
#include "Robot.h"

class SetVisionOffsetDegrees : public Step {
 public:
  SetVisionOffsetDegrees(double offset) : offset(offset) {}

  bool Run(std::shared_ptr<World> world) {
    std::cout << "SetVisionOffsetDegrees(" << offset << ")\n";
    Robot::visionSystem->SetOffsetDegrees(offset);
    return true;
  }
 private:
  const double offset;
};
