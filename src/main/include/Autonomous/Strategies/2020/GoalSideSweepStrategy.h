#pragma once

#include "Autonomous/Strategy.h"
#include <units/units.h>


class GoalSideSweepStrategy : public StepStrategy {
public:
    enum Mode { Standard };
	GoalSideSweepStrategy(std::shared_ptr<World> world, Mode mode);
	virtual ~GoalSideSweepStrategy() {}
	virtual void Init(std::shared_ptr<World> world) {}

private:
	void OldWork(std::shared_ptr<World> world, Mode mode);
};
