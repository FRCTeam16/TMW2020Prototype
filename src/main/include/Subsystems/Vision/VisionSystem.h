#pragma once

#include "Subsystems/SubsystemManager.h"
#include <frc/controller/PIDController.h>
#include "Limelight.h"
#include "Util/UtilityFunctions.h"
#include <iostream>


struct VisionInfo {
public:
  bool hasTarget = false;
  bool inThreshold = false;
  double xOffset = 0.0;
  double xSpeed = 0.0;
  double targetArea = 0.0;
};

/**
 * Vision-assisted alignment and driving
 */
class VisionSystem : public SubsystemManager {
 public:
  VisionSystem();
  void Run() override;
  void Instrument() override;
  void ToggleCameraMode();
  std::shared_ptr<VisionInfo> GetLastVisionInfo();
  std::shared_ptr<Limelight> GetLimelight() { return limelight; }
  void SetMaxOutputRange(double _range);
  void ResetMaxOutputRange();
  void EnableVisionTracking();
  void DisableVisionTracking();
  bool IsVisionTrackingEnabled();
  void SetOffsetDegrees(double offset);
private:
  std::shared_ptr<Limelight> limelight;
  std::unique_ptr<frc2::PIDController> xoffPID;
  std::shared_ptr<VisionInfo> currentVisionInfo;
  double outputRange = 0.3;
  double offsetDegrees = 0.0;
};
