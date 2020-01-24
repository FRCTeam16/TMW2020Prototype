/*
 * ZeroDriveEncoders.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */
#include <Robot.h>
#include <Autonomous/Steps/ZeroDriveEncoders.h>

ZeroDriveEncoders::ZeroDriveEncoders() {
}

ZeroDriveEncoders::~ZeroDriveEncoders() {
}

bool ZeroDriveEncoders::Run(std::shared_ptr<World> world) {
	if (firstRun) {
		firstRun = false;
		startTime = world->GetClock();
		Robot::driveBase->ZeroDriveEncoders();
	}

	// TODO: Access encoder value to verify zero
	// Just return after 10 ms right now
	return ((world->GetClock() - startTime) > 0.01);

	/*
	lastEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition()
	std::cout << "Waiting for encoder position to be zero: " << lastEncoderPosition << "\n";
	return (lastEncoderPosition == 0);
	*/

}
