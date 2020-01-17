#pragma once

#include "Autonomous/Step.h"
#include "Subsystems/Drive/FrontTwoAveragingDriveEncoderPIDSource.h"

class OpenDriveToDistance : public Step {
public:
  explicit OpenDriveToDistance(double _angle, double y, double x, double distance, double distanceThreshold,
    double rampUpTime = -1, double rampDownDist = -1,  double timeOut = -1);
  bool Run(std::shared_ptr<World> world) override;
private:
  const double angle;
  const double ySpeed;
  const double xSpeed;
  const double rampUpTime;
  const double rampDownDist;
  const double timeOut;
  const double distance;
  const double distanceThreshold;
  double startTime = -1;

  std::unique_ptr<FrontTwoAveragingDriveEncoderPIDSource> encoder;
};
