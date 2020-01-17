#ifndef SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_
#define SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_

#include <vector>
#include "Autonomous/Step.h"
#include "Autonomous/World.h"

class WrappedStep;

class ConcurrentStep: public Step {
public:
	explicit ConcurrentStep(std::initializer_list<Step*> stepList, bool _haltOnEnd = false);
	virtual ~ConcurrentStep();
	bool Run(std::shared_ptr<World> world) override;
	const CrabInfo* GetCrabInfo() override;
private:
	std::vector<WrappedStep*> steps;
	const std::unique_ptr<CrabInfo> STOP { new CrabInfo() };
	bool haltOnEnd = true;
	const bool debug = true;
};

class WrappedStep {
public:
	WrappedStep(Step* _step) : step(_step) {}
	virtual ~WrappedStep();
	bool Run(std::shared_ptr<World> world);
	bool IsFinished();
	Step* GetStep() { return step; }
private:
	Step* step;
	bool finished = false;
};

#endif /* SRC_AUTONOMOUS_STEPS_CONCURRENTSTEP_H_ */
