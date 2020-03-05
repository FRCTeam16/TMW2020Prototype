#include "Autonomous/Strategies/2020/SnatchAndScoot.h"

#include <iostream>
#include <units/units.h>

#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/DriveToDistance.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/SetVisionOffsetDegrees.h"
#include "Autonomous/Steps/TimedDrive.h"

#include "Autonomous/Steps/2020/SetTurretPosition.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"
#include "Autonomous/Steps/2020/SelectShootingProfile.h"
#include "Autonomous/Steps/2020/SetFeederArmPosition.h"
#include "Autonomous/Steps/2020/SetFeederArmOpenLoop.h"


SnatchAndScoot::SnatchAndScoot(std::shared_ptr<World> world)
{
    std::cout << "SnatchAndScoot::SnatchAndScoot\n";

    const double firstAngle = 180.0;
    steps.push_back(new ConcurrentStep({
		new SetGyroOffset(firstAngle),
        new SetFeederArmPosition(FeederArm::Position::kVertical, 0.25_s),
        new SetTurretPosition(-451, 0.2_s),
		new Delay(0.75)
	}));

    // Run and grab balls
    auto grabOppoBalls = new DriveToDistance(firstAngle, 0.6, 0_in, -104_in);
    grabOppoBalls->SetRampDownDistance(6_in);
    steps.push_back(new ConcurrentStep({
        grabOppoBalls,
        new SetFeederArmPosition(FeederArm::Position::kZero, 0.25_s),
        new EnableIntake(true),
        new SelectVisionPipeline(2),
        new SetVisionOffsetDegrees(1),
        new SelectShootingProfile(ShootingProfile::kMedium)
    }));


    // Scoot back from the ball pickup
    auto scootBack = new DriveToDistance(firstAngle, 0.6, 0_in, 12_in);
    steps.push_back(new ConcurrentStep({
        scootBack,
        new SetFeederArmOpenLoop(0.0)
    }));


    auto crabOver = new DriveToDistance(firstAngle, 0.4, 80_in, -12_in);
    crabOver->SetRampDownDistance(3_in);
    steps.push_back(new ConcurrentStep({
        crabOver
    }));


    const double sweepAngle = 115.0;
    steps.push_back(new ConcurrentStep({
        new Rotate(sweepAngle),
        new SetFeederArmPosition(FeederArm::Position::kPlayerStation),
        new EnableShooter(true),
    }));
    steps.push_back(new ConcurrentStep({
        new EnableVisionTracking(true),
        new Delay(1.0)
    }));
    steps.push_back(new ConcurrentStep({
        new EnableFeeder(true),
        new Delay(1.5)
    }));

    auto driveToBar = new DriveToDistance(sweepAngle, 0.15, 18_in, -19.5_in);
    steps.push_back(new ConcurrentStep({
        driveToBar,
        new EnableFeeder(false),
        new SetFeederArmOpenLoop(0.0)
    }));

    // sweep bar
    auto sweep = new DriveToDistance(sweepAngle, 0.15, 54_in, 27_in);
    sweep->SetTimeOut(2.5_s);
    steps.push_back(new ConcurrentStep({
        sweep
    }));;

    // coast atan2(xdist, ydist)
    auto coastSweep = new DriveToDistance(sweepAngle, 0.00001, 44_in, 22_in);
    coastSweep->SetTimeOut(0.25_s);
    steps.push_back(coastSweep);

    // raise arm, then start shooting
    steps.push_back(new SetFeederArmPosition(FeederArm::Position::kPlayerStation));
    steps.push_back(new EnableFeeder(true));
}