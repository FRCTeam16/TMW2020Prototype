#pragma once
#include "Autonomous/Step.h"
#include "Robot.h"

class SetVisionLight : public Step {
 public:
  SetVisionLight(bool state) : lightState(state) {}

  bool Run(std::shared_ptr<World> world) {
    Limelight::CameraMode cameraMode;
    if (lightState) {
      cameraMode = Limelight::CameraMode::ImageProcessing;
    } else {
      cameraMode = Limelight::CameraMode::DriverCamera;
    }
    Robot::visionSystem->GetLimelight()->SetCameraMode(cameraMode);
    return true;
  }
private:
  const bool lightState;
};
