#include <iostream>
#include "Autonomous/Steps/AlignToTarget.h"
#include "Robot.h"

AlignToTarget::AlignToTarget(double _angle, double _target, double _timeout, int _scansToHold)
  : angle(_angle), target(_target), timeout(_timeout), scansToHold(_scansToHold) {}

bool AlignToTarget::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
        Robot::driveBase->SetTargetAngle(angle);
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    const auto vision = Robot::visionSystem->GetLastVisionInfo();
    const bool thresholdMet = vision->hasTarget && (fabs(vision->xOffset) < target);
    const double xspeed = vision->hasTarget ? vision->xSpeed : 0.0; // TODO: Search direction speed?

    std::cout << "AlignToTarget: seeTarget? " << vision->hasTarget
              << " | xspeed: " << vision->xSpeed
              << " | tx: " << vision->xOffset
              << " | targetError: " << target
              << " | thresholdMet?: " << thresholdMet
              << "\n";

    if (thresholdMet) {
        scanCount++;
        std::cout << "*** AlignToTarget met target threshold | scans = " << scanCount << " / " << scansToHold << " ***\n";
        if (scanCount >= scansToHold) {
            return true;
        }
    } else {
        scanCount = 0;
    }
    
    if (elapsed > timeout) {
        std::cout << "!!! AlignToTarget TIMED OUT !!!\n";
        return true;
    } 

    crab->Update(
        Robot::driveBase->GetTwistControlOutput(), 
        0.0,
        xspeed, 
        false);
    return false;
    
}