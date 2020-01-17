/*
 * Strategy.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#include "Robot.h"
#include "Autonomous/Strategy.h"
#include "Autonomous/World.h"
#include <iomanip>

/*****************************************************************************/


StepStrategy::~StepStrategy() {
	for (auto it = steps.begin(); it != steps.end(); ++it) {
		delete *it;
	}
	steps.clear();
}

void StepStrategy::Init(std::shared_ptr<World> world) {
}

bool StepStrategy::Run(std::shared_ptr<World> world) {
	if (currentStep >= steps.size()) {
		if (!finished) {
			std::cout << "!!! StepStrategy -> Finished all auto steps !!!\n";
			finished = true;
		}
		RunDrives(STOP.get(), false);
		return true;
	}

	std::cout << "\n[" << world->GetClock() << "] StepStrategy::Run -> " << "currentStep = " << currentStep << "\n";
	Step* step = steps[currentStep];
	const bool stepComplete = step->Run(world);

	if (!step->IsManualDriveControl()) {
		RunDrives(step->GetCrabInfo(), true);
	}

	if (stepComplete) {
		currentStep++;
	}
	return false;
}

void StepStrategy::RunDrives(const CrabInfo *crab, bool showMessage) {
	if (showMessage) {
		std::cout << "RunDrives -> ";
		std::cout << "\ttwist: " << std::setw(5) << crab->twist
				  << "  yspeed: " << std::setw(5) << crab->yspeed
				  << "  xspeed: " << std::setw(5) << crab->xspeed
				  << "\n";
	}
	Robot::driveBase->Crab(crab->twist, crab->yspeed, crab->xspeed, crab->gyro);

}
