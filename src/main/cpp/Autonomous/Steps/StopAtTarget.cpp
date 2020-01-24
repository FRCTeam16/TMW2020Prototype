#include "Autonomous/Steps/StopAtTarget.h"
#include "Robot.h"

StopAtTarget::StopAtTarget(Step *step, double xThreshold, int numScans, double _ignoreTime, double _timeOutTime) 
  : step(step), xThreshold(xThreshold), numScans(numScans), ignoreTime(_ignoreTime), timeOutTime(_timeOutTime) {}

bool StopAtTarget::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    timedOut = (elapsed > timeOutTime);
    if (timedOut) {
        std::cout << "StopAtTarget: TIMEOUT\n";
        return true;
    }

    bool visionThresholdMet = false;
    if (elapsed >= ignoreTime) {
        const auto vision = Robot::visionSystem->GetLastVisionInfo();
        const bool hasTarget = vision->hasTarget;
        
        if (hasTarget) {
            if (fabs(vision->xOffset) <= xThreshold) {
                if (++scanCount >= numScans) {
                    visionThresholdMet = true;
                }
            } // outside of threshold
        } else {
            scanCount = 0;
        }

        std::cout << "StopAtTarget(scans = " << scanCount << "):" 
                    << " | tgt? " << vision->hasTarget
                    << " | xspeed: " << vision->xSpeed
                    << " | ta: " << vision->targetArea
                    << " | tx: " << vision->xOffset
                    << " | ta met? " << visionThresholdMet
                    << "\n"; 
    } else {
        std::cout << "StopAtTarget(Waiting " << elapsed << " / " << ignoreTime << ")\n";
    }

    
    if (visionThresholdMet) {
        std::cout << "StopAtTarget: Exiting due to vision threshold met\n";
        return true;
    } else {
        return step->Run(world);
    }
}
