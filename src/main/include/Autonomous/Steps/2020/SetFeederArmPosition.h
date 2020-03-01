#pragma once

#include "Robot.h"
#include "Subsystems/FeederArm/FeederArm.h"
#include "Autonomous/Step.h"
#include <units/units.h>

class SetFeederArmPosition : public Step {
public:
    explicit SetFeederArmPosition(FeederArm::Position position, units::second_t timeout = 1.0_s)
    : position(position), timeout(timeout)
    {
    }

    bool Run(std::shared_ptr<World> world) override
    {
        auto now = units::second_t{world->GetClock()};
        if (startTime < 0_s) {
            std::cout << "SetFeederArmPosition(" << position << ")\n";
            startTime = now;
            Robot::feederArm->SetArmPosition(position);
        }

        auto elapsed = now - startTime;
        bool timedOut = (elapsed > timeout);
        if (timedOut) {
            std::cout << "!!! SetFeederArmPosition TIMED OUT !!!\n";
        }

        bool inPosition = false;
        if (elapsed > 0.5_s)
        {
            inPosition = Robot::feederArm->IsArmInPosition();
            std::cout << "SetFeederArmPosition in position? " << inPosition << "\n";
        }
        return (timedOut || inPosition);
    }

private:
    const FeederArm::Position position;
    const units::second_t timeout;
    units::second_t startTime = -1_s;
};