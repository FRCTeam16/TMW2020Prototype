#include <Autonomous/Steps/Rotate.h>
#include <Robot.h>


bool Rotate::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		Robot::driveBase->SetTargetAngle(angle);
		startTime = currentTime;
	}

	const float yaw = RobotMap::gyro->GetYaw();
	const double yawError = Robot::driveBase->GetTwistControlError();
	std::cout << "Rotate(setpoint = " << angle
			             << " yaw = " << yaw
						 << " error = " << yawError
						 << " scansHeld = " << heldScans << " / " << scansToHold
						 << ")\n";

	if ((currentTime - startTime) > TIMEOUT) {
		std::cerr << "*** Rotate -> Timed out turning *** \n";
		crab->Stop();
		return continueOnTimeout;
	}

	if (fabs(yawError) <= THRESHOLD || (fabs(yawError) >= (360 - THRESHOLD))) {
		heldScans++;
		if (heldScans > scansToHold) {
			crab->Update(0.0, 0.0, 0.0, true);
			std::cout << "Exiting Turn\n";
			return true;
		}
	} else {
		heldScans = 0;
	}

	crab->Update(Robot::driveBase->GetTwistControlOutput(), 0.0, 0.0, true);
	return false;

}
