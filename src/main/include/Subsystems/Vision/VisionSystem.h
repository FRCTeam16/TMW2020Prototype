#pragma once

#include "Subsystems/SubsystemManager.h"
#include <frc/PIDController.h>  // TODO: Deprecated, need to move to frc2::PIDController
#include "Limelight.h"
#include "XOffsetController.h"
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
  std::unique_ptr<XOffsetController> xoffsetController;
  std::unique_ptr<frc::PIDController> xoffPID;
  std::shared_ptr<VisionInfo> currentVisionInfo;
  double offsetDegrees = 0.0;
};
