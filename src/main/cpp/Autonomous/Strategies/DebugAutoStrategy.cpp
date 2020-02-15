#include "Autonomous/Strategies/DebugAutoStrategy.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/DriveToBump.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/RotateUntilPast.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/VisionControlledDrive.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/OpenDriveToDistance.h"
#include "Autonomous/Steps/SetVisionLight.h"
#include "Autonomous/Steps/StopAtTargetCount.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/AlignToTarget.h"

#include "Autonomous/Steps/DriveToDistance.h"

#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"
#include "Autonomous/Steps/2020/SetTurretPosition.h"

#include <units/units.h>

DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	// DebugSimple();
	Measure();
	std::cout << "--- DEBUG Autonomous ---\n";
}
	

void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	AutoStartPosition startPosition = world->GetStartPosition();
}

void DebugAutoStrategy::DebugSimple() {
	// steps.push_back(new SetVisionLight(true));

	const double startAngle = 0.0;
	const double driveSpeed = 0.4;
	const double driveX = 0.0;
	const double driveY = -24.0;
	auto drive = new ClosedLoopDrive2(startAngle, driveSpeed, driveX, driveY, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6);
	// auto drive = new TimedDrive(startAngle, -0.2, 0.2, 2.0, 1.0);

	// steps.push_back(drive);
	// steps.push_back(new ClosedLoopDrive2(startAngle, driveSpeed, 24.0, 0, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6));
	// steps.push_back(new ClosedLoopDrive2(startAngle, driveSpeed, 0.0, 24, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6));
	// steps.push_back(new ClosedLoopDrive2(startAngle, driveSpeed, -24.0, 0, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6));

	steps.push_back(new EnableIntake(true));
	steps.push_back(new ClosedLoopDrive2(startAngle, driveSpeed, 0, 60.0, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6));
	steps.push_back(new ClosedLoopDrive2(startAngle, driveSpeed, 0, -60.0, -1, DriveUnit::Units::kInches, 5.0, 1.0, 6));
	steps.push_back(new EnableIntake(false));
}

void DebugAutoStrategy::Measure() {
	const double firstAngle = 45.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(firstAngle),
		new SetTurretPosition(97),
		new EnableIntake(true),
		new EnableShooter(false)
	}));
	steps.push_back(new DriveToDistance(firstAngle, 0.3, 0_in, 75_in));
	auto driveToBar = new DriveToDistance(firstAngle, 0.2, 0_in, 25_in);
	driveToBar->SetUseGyro(false);
	steps.push_back(new ConcurrentStep({
		driveToBar,
		new EnableVisionTracking(true),
		new EnableShooter(true)
	}));
	steps.push_back(driveToBar);
	const double secondAngle = 30.0;
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(secondAngle, 0.2, -65_in, 130_in),
		new EnableFeeder(true)
	}));
}
