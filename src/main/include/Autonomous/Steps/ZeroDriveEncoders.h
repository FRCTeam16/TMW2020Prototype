#ifndef SRC_AUTONOMOUS_STEPS_ZERODRIVEENCODERS_H_
#define SRC_AUTONOMOUS_STEPS_ZERODRIVEENCODERS_H_

#include "Autonomous/Step.h"

class ZeroDriveEncoders: public Step {
public:
	ZeroDriveEncoders();
	virtual ~ZeroDriveEncoders();
	bool Run(std::shared_ptr<World> world) override;
private:
	bool firstRun = true;
	double startTime;
	int lastEncoderPosition = 0;
};

#endif /* SRC_AUTONOMOUS_STEPS_ZERODRIVEENCODERS_H_ */
