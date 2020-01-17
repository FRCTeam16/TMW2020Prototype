#pragma once

#include "Autonomous/Step.h"
#include "Robot.h"
#include <iostream>

class SelectVisionPipeline : public Step {
 public:
  SelectVisionPipeline(int pipeline_) : pipeline(pipeline_)  {}

  bool Run(std::shared_ptr<World> world) override {
    if (!firstRun) {
      firstRun = true;
      Robot::visionSystem->GetLimelight()->SelectPipeline(pipeline);
      std::cout << "SelectVisionPipeline( " << pipeline << " )\n"; 
    }
    return true;
  }
 private:
  bool firstRun = false;
  const int pipeline;
};
