#pragma once

#include "Autonomous/Strategy.h"
#include <units/units.h>


class GoalSideSweepStrategy : public StepStrategy {
public:
    enum Mode { kOffset, kCenter };
	GoalSideSweepStrategy(std::shared_ptr<World> world, Mode mode);
	virtual ~GoalSideSweepStrategy() {}
	virtual void Init(std::shared_ptr<World> world) {}

private:
	void Offset(std::shared_ptr<World> world);
	void Center(std::shared_ptr<World> world);
};
