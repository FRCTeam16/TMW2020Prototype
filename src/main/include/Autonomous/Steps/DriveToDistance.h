#pragma once

#include "Autonomous/Step.h"
#include <units/units.h>
#include "Autonomous/DriveUnit.h"
#include "Robot.h"

class DriveToDistance : public Step {
public:
    DriveToDistance(double angle, double speed, units::inch_t xdist, units::inch_t ydist)
        : angle(angle), speed(speed), xdist(xdist), ydist(ydist)
    {
        angleRadians = units::math::atan2(xdist, ydist);
        auto hypotenuse = units::math::sqrt((xdist * xdist) + (ydist * ydist));
        targetSetpoint = /*Robot::driveBase->GetDriveControlEncoderPosition() +*/ DriveUnit::ToPulses(hypotenuse.to<double>(), DriveUnit::kInches);
    }

	bool Run(std::shared_ptr<World> world) override;

    void SetUseGyro(bool useGyro) { this->useGyro = useGyro; }

    void SetDebug(bool debug) { this->debug = debug; }

    void SetRampUpTime(units::second_t time) { this->rampUpTime = time; }

    void SetRampDownDistance(units::inch_t distance) { this->rampDownDistance = distance; }

    void SetTimeOut(units::second_t timeout) { this->stepTimeOut = timeout; }

    void SetStopDriveOnJam(bool stopDrive) { this->stopDriveOnJam = stopDrive; }

private:
    const double angle;
    const double speed;
    const units::inch_t xdist;
    const units::inch_t ydist;
    double targetSetpoint;
    units::radian_t angleRadians;
    
    bool useGyro = true;
    bool debug = false;
    units::second_t startTime = -1_s;
    units::second_t stepTimeOut = 10.0_s;

    units::second_t rampUpTime = -1_s;
    units::inch_t rampDownDistance = -1_in;

    bool stopDriveOnJam = false;
    bool haltedDueToJam = false;
};