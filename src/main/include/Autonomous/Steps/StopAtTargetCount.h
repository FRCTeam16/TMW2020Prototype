#pragma once

#include "Autonomous/Step.h"
#include "Autonomous/Steps/TimedDrive.h"

/**
 * Wraps a step (usually drive) and performs target counting.  Once count reached the wrapped step ceases to be run, 
 * and this step completes.
 * 
 * TODO: Make a template
 */
class StopAtTargetCount : public Step {
public:
  explicit StopAtTargetCount(TimedDrive *step, int numberOfTargets, bool movingRight, double ignoreTime, double timeOutTime);
  // ~StopAtTarget();
  bool Run(std::shared_ptr<World> world) override;
  virtual const CrabInfo* GetCrabInfo() override { return step->GetCrabInfo(); }
	bool IsManualDriveControl() const override { return step->IsManualDriveControl(); }
  void SetMinTx(double _minTx) { kMinTx = _minTx; }
  void SetStepDownYSpeed(double _speed) { stepDownSpeedEnabled = true; stepDownSpeed = _speed; }

private:
  const std::unique_ptr<TimedDrive> step;
  const int numberOfTargets;
  const bool movingRight;
  const double ignoreTime;
  const double timeOutTime;

  double startTime = -1;
  int targetsFound = 0;
  bool timedOut;

  double kMinTx = 20.0;           // we have to be within this range to meet threshold
  const double kXTargetJump = 5.0;      // jump amount to look for to see next target 
  const double kXDirectionCheck = -10.0; // amount to make sure we are past to avoid vision jumping to previous target

  double currentTx = 0.0;
  double lastTx = 0.0;

  bool stepDownSpeedEnabled = false;
  bool stepDownSpeedSet = false;
  double stepDownSpeed = 0;

};
