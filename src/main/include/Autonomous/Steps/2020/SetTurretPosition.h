#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"
#include <units/units.h>

class SetTurretPosition : public Step {
public:
    explicit SetTurretPosition(double setpoint, units::second_t timeout=1.0_s)
    : setpoint(setpoint), timeout(timeout) {
    }

    bool Run(std::shared_ptr<World> world) override {
        auto now = units::second_t{world->GetClock()};
        
        if (startTime < 0_s) {
            std::cout << "SetTurretPosition(" << setpoint << ")\n";
            startTime = now;
            Robot::turret->GetTurretRotation().SetTurretSetpoint(setpoint);
        }
        auto elapsed = now - startTime;
        bool timedOut = (elapsed > timeout);
        if (timedOut) {
            std::cout << "!!! SetTurretPosition TIMED OUT !!!\n";
        }
        
        bool inPosition = false;
        if (elapsed > 0.5_s)
        {
            inPosition = Robot::turret->GetTurretRotation().IsTurretInPosition();
            std::cout << "SetTurretPosition IN POSITION\n";
        }
        return (timedOut || inPosition);
    }
private:
    units::second_t startTime = -1_s;
    double setpoint;
    units::second_t timeout;
};