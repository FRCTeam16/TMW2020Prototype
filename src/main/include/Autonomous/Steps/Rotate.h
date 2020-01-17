#ifndef SRC_AUTONOMOUS_STEPS_ROTATE_H_
#define SRC_AUTONOMOUS_STEPS_ROTATE_H_

#include "Autonomous/Step.h"

class Rotate : public Step {
private:
	double startTime = -1;
	const double angle = -1;
	const double THRESHOLD;
	const double TIMEOUT = 5.0;

	const int scansToHold;
	int heldScans = 0;
	bool continueOnTimeout = false;
public:
	Rotate(double _angle, double _threshold = 5.0, double _timeout = 5.0, int _scansToHold = 0) :
		angle(_angle),
		THRESHOLD(_threshold),
		TIMEOUT(_timeout),
		scansToHold(_scansToHold) {}
	virtual ~Rotate() {}
	bool Run(std::shared_ptr<World> world);
	void SetContinueOnTimeout(bool _continue) { continueOnTimeout = _continue; }
};

#endif /* SRC_AUTONOMOUS_STEPS_ROTATE_H_ */
