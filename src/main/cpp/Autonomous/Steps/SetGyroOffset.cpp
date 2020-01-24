#include <Autonomous/Steps/SetGyroOffset.h>

bool SetGyroOffset::Run(std::shared_ptr<World> world) {
	std::cout << "Setting gyro offset to " << offset << "\n";
	RobotMap::gyro->SetOffset(offset);
	return true;
}


