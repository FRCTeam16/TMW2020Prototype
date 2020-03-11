#pragma once
#include "networktables/NetworkTable.h"

struct SceneInfo {
  bool hasTarget = false;
  double xOffset = 0.0;
  double yOffset = 0.0;
  double targetArea = 0.0;
  double skew = 0.0;
  double latency = 0.0;
};

/**
 * System for handling interaction with vision system.
 * 
 * @see http://docs.limelightvision.io/en/latest/networktables_api.html
 */
class Limelight {
 public:
 enum class CameraMode { Unknown = -1, ImageProcessing = 0, DriverCamera = 1 };
 enum class StreamMode { SideBySide = 0, LimelightMain = 1, USBMain = 2};

  Limelight();

  SceneInfo GetScene() const;
  
  int GetCurrentPipeline() const;
  void SelectPipeline(int pipelineNumber);
  
  CameraMode GetCameraMode() const;
  void SetCameraMode(CameraMode visionMode);
  CameraMode ToggleCameraMode();
  void SetStreamMode(StreamMode streamMode);

  /** Uses angle to target object to determine distance **/
  double CalculateDistance(double heightToCamera, double heightToTarget) const;

private:
  enum class LedMode { kPipeline = 0, kOff = 1, kBlink = 2, kOn = 3 };
  void SetLedMode(LedMode ledMode);
  std::shared_ptr<NetworkTable> dataTable;
};
