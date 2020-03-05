#include "Autonomous/Steps/DriveToDistance.h"
#include "Robot.h"
#include "Util/RampUtil.h"
#include "Autonomous/DriveUnit.h"

bool DriveToDistance::Run(std::shared_ptr<World> world) {
    const units::second_t now {world->GetClock()};
    
    if (startTime < 0_s) {
        startTime = now;

		Robot::driveBase->SetTargetAngle(angle);

		std::cout << "DriveToDistance: Setting Target Drive Distance:" << targetSetpoint << "| Speed:" << speed << "\n";
		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
    }

    const units::second_t elapsedTimeSecs = now - startTime;
    const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();

    if (debug) {
        frc::SmartDashboard::PutNumber("Auto.Step.DriveToDistance.Encoder", currentEncoderPosition);
    }

    std::cout << "DriveToDistance: " << currentEncoderPosition << " => " << targetSetpoint << "\n";

    //*****************
    // HALT CONDITIONS
    //*****************
    if (currentEncoderPosition > targetSetpoint) {
        std::cout << "DriveToDistance: target reached in " << elapsedTimeSecs << " secs\n";
        // crab->Stop();
        return true;
    }

    if (haltedDueToJam || (stopDriveOnJam && Robot::feederArm->IsIntakeJamDetected())) {
        haltedDueToJam = true;
        std::cout << "**** DriveToDistance: EMERGENCY STOP DUE TO INTAKE JAM ****\n";
        Robot::feederArm->StopIntake();
        crab->Stop();
        return false;
    }

    if (elapsedTimeSecs > stepTimeOut) {
        std::cerr << "**** DriveToDistance: EMERGENCY STOP DUE TO TIMEOUT ****\n";
        crab->Stop();
        return true;
    }  
    

    //*****************
    // Drive Control
    //*****************
    double adjSpeed = speed;

    if (rampUpTime >= 0_s) {
        adjSpeed = RampUtil::RampUp(adjSpeed, elapsedTimeSecs.to<double>(), rampUpTime.to<double>());
    }

    if (rampDownDistance > -1_in) {
        double pulses = rampDownDistance.to<double>();
        adjSpeed = RampUtil::RampDown(adjSpeed, currentEncoderPosition, targetSetpoint, pulses);
    }

    const auto yspeed = std::copysign(adjSpeed * units::math::cos(angleRadians), ydist.to<double>());
    const auto xspeed = std::copysign(adjSpeed * units::math::sin(angleRadians), xdist.to<double>());
    const auto twistOutput = Robot::driveBase->GetTwistControlOutput();

    crab->Update(twistOutput, yspeed, xspeed, useGyro);
    return false;
}
