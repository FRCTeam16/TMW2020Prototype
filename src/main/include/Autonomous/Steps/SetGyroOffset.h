#ifndef SRC_AUTONOMOUS_STEPS_SETGYROOFFSET_H_
#define SRC_AUTONOMOUS_STEPS_SETGYROOFFSET_H_

#include "Autonomous/Step.h"
#include "Robot.h"

class SetGyroOffset : public Step {
public:
	SetGyroOffset(float _offset) : offset(_offset) {}
	virtual ~SetGyroOffset() {}
	bool Run(std::shared_ptr<World> world) override;
private:
	const float offset;
};

#endif /* SRC_AUTONOMOUS_STEPS_SETGYROOFFSET_H_ */
