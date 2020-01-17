#pragma once

#include "Autonomous/Step.h"

class StopAtTarget : public Step {
public:
  explicit StopAtTarget(Step *step, double xThreshold, int numScans, double ignoreTime, double timeOutTime);
  // ~StopAtTarget();
  bool Run(std::shared_ptr<World> world) override;
  virtual const CrabInfo* GetCrabInfo() override { return step->GetCrabInfo(); }
	bool IsManualDriveControl() const override { return step->IsManualDriveControl(); }
private:
  const std::unique_ptr<Step> step;
  double startTime = -1;
  const double xThreshold;
  const int numScans;
  const double ignoreTime;
  const double timeOutTime;
  bool timedOut;
  int scanCount = 0;
};
