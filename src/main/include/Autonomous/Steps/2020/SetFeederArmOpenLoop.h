#pragma once

#include "Robot.h"
#include "Subsystems/FeederArm/FeederArm.h"
#include "Autonomous/Step.h"
#include <units/units.h>

class SetFeederArmOpenLoop : public Step {
public:
    SetFeederArmOpenLoop(double speed) : speed(speed) {}

    bool Run(std::shared_ptr<World> world) override
    {
        std::cout << "SetFeederArmOpenLoop(" << speed << ")\n";
        Robot::feederArm->RunArm(speed);
        return true;
    }

private:
    const double speed;
};