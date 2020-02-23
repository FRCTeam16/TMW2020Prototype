#include "Autonomous/Strategies/2020/GoalSideSweepStrategy.h"

#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/DriveToDistance.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/SetVisionOffsetDegrees.h"

#include "Autonomous/Steps/2020/SetTurretPosition.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"

GoalSideSweepStrategy::GoalSideSweepStrategy(std::shared_ptr<World> world, Mode mode) {
	std::cout << "GoalSideSweepStrategy::GoalSideSweepStrategy : " << mode << "\n";
	if (Mode::kOffset == mode) {
		Offset(world);
	} else if (Mode::kCenter == mode) {
		Center(world);
	}
}

void GoalSideSweepStrategy::Offset(std::shared_ptr<World> world) {
	const double firstAngle = 180.0;
	steps.push_back(new SetGyroOffset(180.0));
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(firstAngle, 0.3, 0_in, -6_in),
		new SetTurretPosition(-180),
		new EnableIntake(true),     // TODO: Pass in desired speed?
		new EnableShooter(true),
	}));
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(firstAngle, 0.3, 0_in, -54_in),
		new SetVisionOffsetDegrees(3.0),
		new EnableVisionTracking(true)
	}));
	
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(firstAngle, 0.3, 0_in, -78_in),
		new EnableFeeder(true)
	}));
	steps.push_back(new Delay(1.0));
	steps.push_back(new EnableFeeder(false));

	// Turn and drive to bar
	const double initialBarAngle = -120.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(initialBarAngle),
		new SetTurretPosition(100)
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

	double sweepAngle = -75.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(sweepAngle),
		new SetTurretPosition(100.0),
		new SetVisionOffsetDegrees(3.0),
	}));

	steps.push_back(new DriveToDistance(sweepAngle, 0.2, -20_in, 0_in));
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(sweepAngle, 0.2, -56_in, 110_in),
		new EnableFeeder(true)
	}));
}

/**
 * Strategy for starting centered with goal, sweeping up bar
 * then rushing back to trench
 */
void GoalSideSweepStrategy::Center(std::shared_ptr<World> world) {

	steps.push_back(new SetGyroOffset(180.0));

	const double firstAngle = -135.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(firstAngle),
		new SetTurretPosition(-111),
		new EnableIntake(true),     // TODO: Pass in desired speed
		new EnableShooter(false)
	}));
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(firstAngle, 0.3, 0_in, -81_in),
		new EnableVisionTracking(true)}));
	
	// Approach bar
	auto driveToBar = new DriveToDistance(firstAngle, 0.2, 0_in, 6_in);
	driveToBar->SetUseGyro(false);
	steps.push_back(new ConcurrentStep({
		driveToBar,
		new EnableShooter(true)
	}));

	// Sweep up bar
	const double secondAngle = -120.0;
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(secondAngle, 0.2, 56_in, -110_in),
		new EnableFeeder(true)			
	}));

	// Drive back to far end of trench
	steps.push_back(new DriveToDistance(-180.0, 0.3, 0_in, 110_in));
	
	// Jog over to pickup location
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(-180.0, 0.2, 24_in, -6_in),
		new SetVisionOffsetDegrees(3.0)
	}));
	steps.push_back(new DriveToDistance(-180.0, 0.3, 0_in, -110_in));

}
