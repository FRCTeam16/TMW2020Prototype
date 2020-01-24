#include <Autonomous/Steps/RotateUntilPast.h>
#include <Robot.h>

bool RotateUntilPast::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		Robot::driveBase->SetTargetAngle(angle);
		startTime = currentTime;
	}
	if ((currentTime - startTime) > TIMEOUT) {
		std::cerr << "*** RotateUntilPast Timed out turning\n";
		crab->Stop();
		return false;
	}

	const float yaw = RobotMap::gyro->GetYaw();
	const double yawError = Robot::driveBase->GetTwistControlError();
	double twistOutput = Robot::driveBase->GetCrabTwistOutput();	// GetCrabControlOutput


	bool finished = false;
	if (rightTurn) {
		finished = (yaw > thresholdAngle);
		twistOutput = fabs(twistOutput);
	} else {
		finished = (yaw < thresholdAngle);
		twistOutput = -(fabs(twistOutput));
	}

	std::cout << "RotateUntilPast(speed= " << twistOutput
			<< " | setpoint= " << angle
			<< " | yaw = " << yaw << " | threshold= " << thresholdAngle
			<< " | error = " << yawError
			<< " | finished = " << finished << ")\n";


	if (finished) {
		std::cout << "RotateUntilPast: Exiting Turn\n";
		return true;
	} else {
		crab->Update(twistOutput, 0.0, 0.0, true);
		return false;
	}
}

