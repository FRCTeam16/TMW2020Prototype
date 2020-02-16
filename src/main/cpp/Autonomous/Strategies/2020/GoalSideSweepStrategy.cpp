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
	
    
    switch(mode) {
        case Mode::EightBall:
        default:
            SweepEight();
            break;
    }
}

void GoalSideSweepStrategy::SweepEight() {
    const double secondAngle = 30.0;
	steps.push_back(new ConcurrentStep({
		new DriveToDistance(secondAngle, 0.2, -50_in, 110_in),
		new EnableFeeder(true)			
	}));

    TrenchRun(0.2, -24_in, -120_in);
}

void GoalSideSweepStrategy::TrenchRun(double speed, units::inch_t xdist, units::inch_t ydist) {
    const double angle = 30.0;
    steps.push_back(new DriveToDistance(angle, speed, xdist, ydist));
}