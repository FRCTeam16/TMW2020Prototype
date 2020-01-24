#include "Autonomous/Steps/OpenDriveToDistance.h"
#include <iostream>
#include "Robot.h"
#include "Util/RampUtil.h"
#include "Autonomous/DriveUnit.h"

OpenDriveToDistance::OpenDriveToDistance(double _angle, double y, double x, double distance,
    double distanceThreshold, double rampUpTime, double rampDownDist,  double timeOut) 
  : angle(_angle), ySpeed(y), xSpeed(x),
    distance(DriveUnit::ToPulses(distance, DriveUnit::kInches)),
    distanceThreshold(DriveUnit::ToPulses(distanceThreshold, DriveUnit::kInches)),
    rampUpTime(rampUpTime), 
    rampDownDist(DriveUnit::ToPulses(rampDownDist, DriveUnit::Units::kInches)), 
    timeOut(timeOut)
{
    auto wheels = Robot::driveBase->GetWheels();
    encoder.reset(new FrontTwoAveragingDriveEncoderPIDSource(wheels));
}


bool OpenDriveToDistance::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
        Robot::driveBase->UseOpenLoopDrive();
        Robot::driveBase->SetTargetAngle(angle);
        encoder->SetInitialEncoderValue();
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;


    if ((timeOut != -1) && (elapsed > timeOut)) {
        std::cout << "* OpenDriveToDistance timed out after: " << elapsed << "\n";        
    }

    const double error = encoder->PIDGet();
    std::cout << "OpenDriveToDistance error: " << error
              << " delta = " << (distance-error) 
              << " | thresh: " << distanceThreshold << "\n";
    if ((distance - error) < distanceThreshold ) {
        std::cout << " OpenDriveToDistance Met Distance!\n";
        return true;
    }

    double xSpd = xSpeed;
    double ySpd = ySpeed;
    if (rampUpTime > 0) {
        xSpd = RampUtil::RampUp(xSpeed, elapsed, rampUpTime, 0.0);
        ySpd = RampUtil::RampUp(ySpeed, elapsed, rampUpTime, 0.0);
    }

    if (rampDownDist > 0) {
        xSpd = RampUtil::RampDown(xSpeed, error, distance, rampDownDist);
        ySpd = RampUtil::RampDown(ySpeed, error, distance, rampDownDist);
    }

    const double twistOutput = Robot::driveBase->GetTwistControlOutput();
    crab->Update(
            (float) twistOutput,
            (float) ySpd,
            (float) xSpd,
            true);
    return false;
}