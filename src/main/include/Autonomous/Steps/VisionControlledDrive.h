#pragma once

#include "Autonomous/DriveUnit.h"
#include "Autonomous/Step.h"


class VisionControlledDrive : public Step {
 public:
  VisionControlledDrive(double _angle, double _speed, double _x, double _y,
					double _threshold, DriveUnit::Units _units,
					double _timeout,
					double _rampUp, double _rampDown);

  VisionControlledDrive(double _angle, double y, double x, 
          double driveTime, bool _useTwist = true);

  bool Run(std::shared_ptr<World> world) override;
private:
  std::unique_ptr<Step> drive;
};
