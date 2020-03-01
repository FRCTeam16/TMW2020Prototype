#pragma once

#include "Autonomous/Strategy.h"

class SnatchAndScoot : public StepStrategy {
public:
    SnatchAndScoot(std::shared_ptr<World> world);
	virtual ~SnatchAndScoot() {}
	virtual void Init(std::shared_ptr<World> world) {}

private:
};