#pragma once

#include "Autonomous/Strategy.h"
#include <units/units.h>


class GoalSideSweepStrategy : public StepStrategy {
public:
    enum Mode { EightBall };
	GoalSideSweepStrategy(std::shared_ptr<World> world, Mode mode);
	virtual ~GoalSideSweepStrategy() {}
	virtual void Init(std::shared_ptr<World> world) {}

private:
	// void SweepTwo();
	// void SweepFive();
    void SweepEight();

    void TrenchRun(double speed, units::inch_t xdist, units::inch_t ydist);
};
