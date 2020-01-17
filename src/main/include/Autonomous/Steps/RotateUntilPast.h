#ifndef SRC_AUTONOMOUS_STEPS_ROTATEUNTILPAST_H_
#define SRC_AUTONOMOUS_STEPS_ROTATEUNTILPAST_H_

#include "Autonomous/Step.h"
#include "Autonomous/World.h"


class RotateUntilPast: public Step {
public:
	RotateUntilPast(bool _rightTurn, double _angle, double _thresholdAngle) :
		rightTurn(_rightTurn), angle(_angle), thresholdAngle(_thresholdAngle)
	{}

	virtual ~RotateUntilPast() {}
	bool Run(std::shared_ptr<World> world);

private:
	const bool rightTurn;
	const double angle;
	const double thresholdAngle;
	double startTime = -1;
	const double TIMEOUT = 3.0;
};

#endif /* SRC_AUTONOMOUS_STEPS_ROTATEUNTILPAST_H_ */
