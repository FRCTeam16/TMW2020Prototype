#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"
#include <units/units.h>

class SetTurretPosition : public Step {
public:
    explicit SetTurretPosition(double setpoint, units::second_t timeout=1.0_s)
    : setpoint(setpoint), timeout(timeout), useSetpoint(true) {
    }

    explicit SetTurretPosition(TurretRotation::Position position, units::second_t timeout=1.0_s)
    : position(position), timeout(timeout), useSetpoint(false) {
    }

    bool Run(std::shared_ptr<World> world) override {
        auto now = units::second_t{world->GetClock()};
        
        if (startTime < 0_s) {
            startTime = now;
            if (useSetpoint) {
                std::cout << "SetTurretPosition(setpoint = " << setpoint << ")\n";
                Robot::turret->GetTurretRotation().SetTurretSetpoint(setpoint);
            } else {
                std::cout << "SetTurretPosition(position = " << setpoint << ")\n";
                Robot::turret->GetTurretRotation().SetTurretPosition(position);
            }  
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
            std::cout << "SetTurretPosition in position? " << inPosition << "\n";
        }
        return (timedOut || inPosition);
    }
private:
    units::second_t startTime = -1_s;
    double setpoint;
    TurretRotation::Position position;
    units::second_t timeout;
    const bool useSetpoint;
};