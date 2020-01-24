#include "Autonomous/Steps/StopAtTargetCount.h"

#include "Robot.h"

StopAtTargetCount::StopAtTargetCount(TimedDrive *step, int numTargets_, bool moveRight_, double _ignoreTime, double _timeOutTime) 
  : step(step),  numberOfTargets(numTargets_), movingRight(moveRight_), ignoreTime(_ignoreTime), timeOutTime(_timeOutTime) {}

bool StopAtTargetCount::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    timedOut = (elapsed > timeOutTime);
    if (timedOut) {
        std::cout << "StopAtTargetCount: TIMEOUT\n";
        return true;
    }

    bool visionThresholdMet = false;
    if (elapsed >= ignoreTime) {
        const auto vision = Robot::visionSystem->GetLastVisionInfo();
        const bool hasTarget = vision->hasTarget;
        const double dir = movingRight ? 1.0 : -1.0;
        
        if (hasTarget) {
            currentTx = vision->xOffset;

            const double deltaX = currentTx - lastTx;

            if ((deltaX * dir) > kXTargetJump) {
                targetsFound++;
            }
            if ((deltaX * dir) > kXDirectionCheck) {
                lastTx = currentTx;
            }

            visionThresholdMet = (targetsFound >= numberOfTargets);
           
            std::cout << "StopAtTargetCount(targetsFound = " << targetsFound << "):" 
                    << " | xspeed: " << vision->xSpeed
                    << " | ta: " << vision->targetArea
                    << " | tx: " << vision->xOffset
                    << " | deltax: " << deltaX
                    << " | ta met? " << visionThresholdMet
                    << "\n"; 

            // Step down speed
            if (stepDownSpeedEnabled && (targetsFound == (numberOfTargets - 2))) {
                if (!stepDownSpeedSet) {
                    std::cout << "StopAtTargetCount - setting speed\n";
                    step->OverrideYSpeed(stepDownSpeed);
                }
                // else step down speed already configured in wrapped step
            }
        } else {
            std::cout << "StopAtTarget(targetsFound = " << targetsFound << "): no target\n";
        }
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