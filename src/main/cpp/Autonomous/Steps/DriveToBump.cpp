#include "Autonomous/Steps/DriveToBump.h"
#include "Robot.h"
#include "Util/RampUtil.h"

bool DriveToBump::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->SetTargetAngle(angle);
	}
	const double elapsed = (currentTime - startTime);

	if (!collisionDetected) {
		collisionDetected = collisionDetector->Detect();
	}
	if (collisionDetected && (elapsed > delayCheckTime)) {
		std::cout << "DriveToBump detected collision\n";
		crab->Stop();
		return true;
	}

	if (elapsed > maxTimeToDrive) {
		std::cerr << "ERROR: DriveToBump timed out\n";
		crab->Stop();
		return true;
	} else {
		std::cout << "DriveToBump -> updating driveBase\n";

		double modY = ySpeed;
		double modX = xSpeed;

		if (rampTime >= 0) {
			modX = RampUtil::RampUp(xSpeed, (currentTime-startTime), rampTime, 0.0);
			modY = RampUtil::RampUp(ySpeed, (currentTime-startTime), rampTime, 0.0);
		}

		crab->Update(
				(float) Robot::driveBase->GetTwistControlOutput(),
				(float) modY,
				(float) modX,
				true);
		return false;
	}
}
