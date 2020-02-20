#include "Autonomous/Steps/AckermanDriveToAngle.h"

#include <frc/Timer.h>
#include "Robot.h"

bool AckermanDriveToAngle::Run(std::shared_ptr<World> world)
{
    const units::second_t now = units::second_t(frc::Timer::GetFPGATimestamp());
    if (startTime < units::second_t(0)) {
        startTime = now;
        std::cout << "AckermanDriveToAngle starting at " << now << " seconds\n";
    }

    // Exit conditions
    const units::degree_t currentAngle = units::degree_t(RobotMap::gyro->GetYaw());
    if (units::math::fabs(steerAngle - currentAngle) < allowedError) {
        std::cout << "AckermanDriveToAngle hit target angle of " << currentAngle
                  << " [Target: " << steerAngle << "]\n";
        return true;
    }
    if ((now - startTime) > timeout) {
        std::cout << "*** AckermanDriveToAngle Timed Out after " << timeout << " seconds\n";
        return true;
    }

    units::radian_t radians = steerAngle;
    Robot::driveBase->Steer(radians.to<double>(), speed, ackermanA);
    return false;
}