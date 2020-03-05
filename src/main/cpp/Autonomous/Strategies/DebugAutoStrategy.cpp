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
#include "Autonomous/Steps/AckermanDriveToAngle.h"

#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"
#include "Autonomous/Steps/2020/SetFeederArmPosition.h"
#include "Autonomous/Steps/2020/SetTurretPosition.h"

#include <units/units.h>

DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	// DebugSimple();
	// Measure();
	std::cout << "--- DEBUG Autonomous ---\n";

	steps.push_back(new SetTurretPosition(-180, 0.2_s));

	// const double firstAngle = 180.0;
	// steps.push_back(new ConcurrentStep({
	// 	new SetGyroOffset(180.0),
	// 	new SetFeederArmPosition(FeederArm::Position::kVertical, 0.25_s),
	// 	new SetTurretPosition(-180, 0.2_s),
	// 	new Delay(0.5)
	// }));

	// auto pushOff = new DriveToDistance(firstAngle, 0.5, 0_in, -60_in);
	// pushOff->SetRampUpTime(0.25_s);
	// steps.push_back(new ConcurrentStep({
	// 	pushOff,
	// 	new SetFeederArmPosition(FeederArm::Position::kZero),
	// 	new EnableShooter(true)
	// })); 

/*
	const double firstAngle = 180.0;
	auto scootBack = new DriveToDistance(firstAngle, 0.6, 0_in, 12_in);
    steps.push_back(scootBack);


	auto crabOver = new DriveToDistance(firstAngle, 0.4, 100_in, -100_in);
    crabOver->SetRampDownDistance(18_in);
    steps.push_back(crabOver);
	*/
}
	

void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	// AutoStartPosition startPosition = world->GetStartPosition();
}

void DebugAutoStrategy::DebugSimple() {
	const double firstAngle = 45.0;
	steps.push_back(new ConcurrentStep({
		new Rotate(firstAngle),
		new SetTurretPosition(-111),
		new EnableIntake(true),
		new EnableShooter(false)
	}));
	steps.push_back(new DriveToDistance(firstAngle, 0.3, 0_in, 81_in));
	auto driveToBar = new DriveToDistance(firstAngle, 0.2, 0_in, 7_in);
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

void DebugAutoStrategy::Measure() {
	steps.push_back(new DriveToDistance(0.0, 0.3, 0_in, 60_in));

	// const units::degree_t zero(0);
	// const units::degree_t fortyFive(45.0);

	// auto step = new AckermanDriveToAngle(units::degree_t(0), 0.2, 1.0, units::degree_t(45),units::degree_t(3.0), units::second_t(5.0));
	// auto step = new AckermanDriveToAngle(0, 0.2, 1, 45);
	// steps.push_back(step);

	
}
