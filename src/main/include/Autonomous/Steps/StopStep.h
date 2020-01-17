#ifndef SRC_AUTONOMOUS_STEPS_STOPSTEP_H_
#define SRC_AUTONOMOUS_STEPS_STOPSTEP_H_

#include <vector>
#include "Autonomous/Step.h"
#include "Autonomous/World.h"
#include "Robot.h"


class StopStep : public Step {
public:
	StopStep() {}
	virtual ~StopStep() {}

	bool Run(std::shared_ptr<World> world) override {
		Robot::driveBase->HaltUsingClosedLoop();
		return true;
	}
};

#endif /* SRC_AUTONOMOUS_STEPS_STOPSTEP_H_ */
