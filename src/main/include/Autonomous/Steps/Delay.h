#ifndef SRC_AUTONOMOUS_STEPS_DELAY_H_
#define SRC_AUTONOMOUS_STEPS_DELAY_H_

#include "Autonomous/World.h"
#include "Autonomous/Step.h"

class Delay : public Step {
private:
	const double delay;
	double startTime = -1;
public:
	Delay(double _delay) : delay(_delay) {}
	virtual ~Delay() {}
	bool Run(std::shared_ptr<World> world) override;
};

#endif /* SRC_AUTONOMOUS_STEPS_DELAY_H_ */
