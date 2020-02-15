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
            Robot::turret->SetTurretPositionControl(true);  // FIXME: why no turret move
            Robot::turret->SetTurretSetpoint(setpoint);
        }
        auto elapsed = now - startTime;
        return ((elapsed > timeout) || Robot::turret->IsTurretInPosition());
    }
private:
    units::second_t startTime = -1_s;
    double setpoint;
    units::second_t timeout;
};