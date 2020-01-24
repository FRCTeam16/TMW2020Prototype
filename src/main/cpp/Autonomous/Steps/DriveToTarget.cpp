#include "Autonomous/Steps/DriveToTarget.h"
#include <iostream>
#include "Robot.h"
#include "Util/RampUtil.h"

DriveToTarget::DriveToTarget(double _angle, double _yspeed, double _targetArea, double _timeout, double _maxTargetArea) 
  : angle(_angle), yspeed(_yspeed), targetArea(_targetArea), timeout(_timeout), maxTargetArea(_maxTargetArea)
{
}

bool DriveToTarget::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
        Robot::driveBase->SetTargetAngle(angle);
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    const auto vision = Robot::visionSystem->GetLastVisionInfo();
    const bool visionThresholdMet = (vision->targetArea >= targetArea) && 
                                    (vision->targetArea <= maxTargetArea);

    double xspeed = 0.0;

    std::cout << "DriveToTarget: tgt? " << vision->hasTarget
                  << " | xspeed: " << vision->xSpeed
                  << " | tx: " << vision->xOffset
                  << " | ta: " << vision->targetArea
                  << " | ta met? " << visionThresholdMet
                  << "\n";

    double adjustedSpeed = yspeed;
    if (vision->hasTarget) {
        xspeed = vision->xSpeed;
        if (vision->targetArea > (targetArea * 0.75)) {
            adjustedSpeed = yspeed * 0.75;
        }
    }
    
    crab->Update(
        Robot::driveBase->GetTwistControlOutput(), 
        adjustedSpeed,
        xspeed, 
        fieldCentric);

    const bool timedOut = (elapsed > timeout);
    if (timedOut) {
        std::cout << "*** DriveToTarget timedout after " << timeout << " secs ***\n";
    }

    return visionThresholdMet || timedOut;  
}

void DriveToTarget::SetFieldCentric(bool fieldCentric_) {
    fieldCentric = fieldCentric_;
}
