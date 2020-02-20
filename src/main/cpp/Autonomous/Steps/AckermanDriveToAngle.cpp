#include "Autonomous/Steps/AckermanDriveToAngle.h"

#include <frc/Timer.h>
#include <cmath>
#include "Robot.h"

bool AckermanDriveToAngle::Run(std::shared_ptr<World> world)
{
    const units::second_t now = units::second_t(frc::Timer::GetFPGATimestamp());
    if (startTime < units::second_t(0)) {
        startTime = now;
        std::cout << "AckermanDriveToAngle starting at " << now << " seconds\n";
    }

    // Exit conditions
    const double currentAngle = (RobotMap::gyro->GetYaw());
    if (std::fabs(target - currentAngle) < allowedError) {
        std::cout << "AckermanDriveToAngle hit target angle of " << currentAngle
                  << " [Target: " << target << "] "
                  << " [Error: " << allowedError << "]\n";
        return true;
    }
    if ((now - startTime) > timeout) {
        std::cout << "*** AckermanDriveToAngle Timed Out after " << timeout << " seconds\n";
        return true;
    }

    // Robot::driveBase->SetTargetAngle(angle);
    // Robot::driveBase->GetTwistControlOutput();
    double radians = steerAngle * M_PI / 180.0;
    Robot::driveBase->Steer(radians, speed, ackermanA);
    return false;
}