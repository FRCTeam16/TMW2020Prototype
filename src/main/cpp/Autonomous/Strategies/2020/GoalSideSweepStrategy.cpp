#include "Autonomous/Strategies/2020/GoalSideSweepStrategy.h"

#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/DriveToDistance.h"

#include "Autonomous/Steps/2020/SetTurretPosition.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"

GoalSideSweepStrategy::GoalSideSweepStrategy(std::shared_ptr<World> world, Mode mode) {
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(0.0, 0.3, 0_in, 6_in),
		new SetTurretPosition(-180),
		new EnableIntake(true),     // TODO: Pass in desired speed
		new EnableShooter(true),
	}));
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(0.0, 0.3, 0_in, 54_in),
		new EnableVisionTracking(true)
	}));
	
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(0.0, 0.3, 0_in, 80_in),
		new EnableFeeder(true)
	}));
	steps.push_back(new EnableFeeder(false));

	// Turn and drive to bar
	const double initialBarAngle = 60.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(initialBarAngle),
		new SetTurretPosition(0)
	}));

	auto driveToBar = new DriveToDistance(initialBarAngle, 0.2, -2_in, 32_in);
	driveToBar->SetUseGyro(false);
	steps.push_back(new ConcurrentStep({
		driveToBar,
		new EnableVisionTracking(true)
	}));
	steps.push_back(driveToBar);

	// Backup, turn and setup sweep angle
	auto driveBack = new DriveToDistance(initialBarAngle, -0.2, 0_in, 16_in);
	driveBack->SetUseGyro(false);
	steps.push_back(driveBack);

	steps.push_back(new ConcurrentStep({
		new Rotate(120.0),
		new SetTurretPosition(70.0)
	}));
	
}

void GoalSideSweepStrategy::OldWork(std::shared_ptr<World> world, Mode mode) {

    const double firstAngle = 45.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(firstAngle),
		new SetTurretPosition(-111),
		new EnableIntake(true),     // TODO: Pass in desired speed
		new EnableShooter(false)
	}));
	steps.push_back(new DriveToDistance(firstAngle, 0.3, 0_in, 81_in));
	auto driveToBar = new DriveToDistance(firstAngle, 0.2, 0_in, 6_in);
	driveToBar->SetUseGyro(false);
	steps.push_back(new ConcurrentStep({
		driveToBar,
		new EnableVisionTracking(true),
		new EnableShooter(true)
	}));
	steps.push_back(driveToBar);

	const double secondAngle = 30.0;
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(secondAngle, 0.2, -50_in, 110_in),
		new EnableFeeder(true)			
	}));
}
