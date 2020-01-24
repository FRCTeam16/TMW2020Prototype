#include "Autonomous/Steps/TimedDrive.h"
#include "Robot.h"
#include "Util/RampUtil.h"

bool TimedDrive::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->UseOpenLoopDrive();
		Robot::driveBase->SetTargetAngle(angle);
		std::cout << "TimedDrive(" 
			<< angle << ", "
			<< ySpeed << ", "
			<< xSpeed << ", "
			<< timeToDrive << ", "
			<< useGyro << ")\n";
	}
	const double elapsed = currentTime - startTime;
	if (elapsed > timeToDrive) {
		return true;
	} else {
		// const double twist = (useTwist) ? Robot::driveBase->GetTwistControlOutput() : 0.0;
		double y = ySpeed;
		if (rampUpTime > 0) {
			y = RampUtil::RampUp(ySpeed, elapsed, rampUpTime);
		}
		crab->Update(
				(float) Robot::driveBase->GetTwistControlOutput(),
				(float) y,
				(float) xSpeed,
				useGyro);
		return false;
	}
}
