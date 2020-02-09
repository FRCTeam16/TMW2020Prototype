#pragma once

#include "Autonomous/Step.h"
#include "Robot.h"

class SetGyroOffset : public Step {
public:
	SetGyroOffset(float _offset) : offset(_offset) {}
	
	virtual ~SetGyroOffset() {}
	
	bool Run(std::shared_ptr<World> world) override {
		std::cout << "Setting gyro offset to " << offset << "\n";
		RobotMap::gyro->SetOffset(offset);
		return true;
	}
private:
	const float offset;
};
